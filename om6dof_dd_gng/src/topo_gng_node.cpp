// topo_gng_node: environment topology for the TopoVLA <-> om6dof_dd_gng
// integration (M2 + M3 of the integration task -- see the task notes for the
// full milestone list; M4's robot-body graph and self-body mask are not in
// this file yet).
//
// Pipeline, once per captured depth frame:
//   RealSense D405 depth + color, aligned to the depth stream
//     -> pixel_step-strided grid, rs2_deproject_pixel_to_point (camera-frame
//        metres, using the device's own live intrinsics, never hardcoded)
//     -> self-body mask (placeholder here; M4 fills this in with the robot's
//        capsule graph)
//     -> tf2 lookup(world_frame, camera_frame, frame timestamp), applied to
//        every surviving point, so the graph is learned in the WORLD frame
//        and stays put as the wrist (and camera with it) moves
//     -> DynamicGrowingNeuralGas::partialFit (vendored core, unmodified)
//     -> visualization_msgs/MarkerArray on ~/environment_graph
//
// In parallel, once per period (default 0.5 s, off the capture thread):
//   color frame -> YOLOX (OpenCV DNN) -> detections, filtered to target_classes
//     -> per-box depth (median + MAD over the box interior, current frame)
//     -> every GNG node re-projected into the CURRENT camera pose (world node
//        -> camera-frame via the inverse of the same tf2 transform, so a node
//        made from a past camera pose is checked against what the camera
//        sees *now*, which is what rejects stale/occluded nodes) and matched
//        against boxes by depth + pixel-in-box + centre distance
//     -> per-node class held via a stable-nodeId temporal map (DD-GNG reuses
//        array slots on removal, so nodeId, not array index, is the only
//        thing a label can be safely attached to across frames)
//     -> ~/labels (JSON) and node colours in ~/environment_graph
//
// This is a from-scratch reimplementation of TopoVLA's labelling algorithm
// (native_depth_yolo/src/main.cpp, DepthYoloProcessor::labelGraph/enrichDepth),
// not a copy of that file -- ported here because that file also contains
// ONNX Runtime, GDI/Win32 viewer, and RealSense capture code that must NOT be
// vendored (ONNX Runtime is unavailable on this Jetson; this node uses OpenCV
// DNN instead, and owns capture itself). Two simplifications from the
// original, both specific to this hardware/design and documented where they
// apply below: (1) color is aligned to depth (rs2::align), so one shared
// pixel grid and one shared intrinsics struct cover both, instead of
// TopoVLA's separate depth/color projections; (2) node re-projection always
// starts from a world-frame point (this integration's nodes are learned in
// world, not camera, frame).

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include <librealsense2/rs.hpp>
#include <opencv2/opencv.hpp>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/color_rgba.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "tf2/exceptions.h"
#include "tf2/LinearMath/Transform.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "om6dof_dd_gng/ddgng.hpp"
#include "om6dof_dd_gng/yolox_detector.hpp"
#include "om6dof_dd_gng/async_yolo.hpp"

using namespace std::chrono_literals;
using om6dof_dd_gng::AsyncYolo;
using om6dof_dd_gng::YoloDetection;
using om6dof_dd_gng::YoloXDetector;

namespace
{

// Robot topology: constant, unlike the environment graph -- these are the
// URDF link names whose TF origins become body-graph nodes (per the
// integration's design decision), read fresh every frame. Order matches the
// kinematic chain from link1 down to the four things hanging off link7.
struct BodyLinkSpec
{
  const char * frame_id;
  const char * radius_param;
};

constexpr std::array<BodyLinkSpec, 11> kBodyLinks = {{
  {"link1", "body_radius.link1"},
  {"link2", "body_radius.link2"},
  {"link3", "body_radius.link3"},
  {"link4", "body_radius.link4"},
  {"link5", "body_radius.link5"},
  {"link6", "body_radius.link6"},
  {"link7", "body_radius.link7"},
  {"end_effector_link", "body_radius.end_effector_link"},
  {"gripper_left_link", "body_radius.gripper_left_link"},
  {"gripper_right_link", "body_radius.gripper_right_link"},
  {"d405_payload_link", "body_radius.d405_payload_link"},
}};

// Kinematic edges, by index into kBodyLinks: link1-2-3-4-5-6-7, then link7's
// four children (end effector, both fingers, the camera payload).
constexpr std::array<std::pair<int, int>, 10> kBodyEdges = {{
  {0, 1}, {1, 2}, {2, 3}, {3, 4}, {4, 5}, {5, 6},
  {6, 7}, {6, 8}, {6, 9}, {6, 10},
}};

struct BodySegment
{
  tf2::Vector3 a;
  tf2::Vector3 b;
  float radius_a = 0.0F;
  float radius_b = 0.0F;
};

// Distance from p to the capsule swept by segment [a, b] with radius
// linearly tapered from radius_a to radius_b -- used both to mask depth
// points near the robot's own body (M4) and, via the same segments, to draw
// the robot graph in RViz, so what the user sees IS what is being masked.
float capsuleClearance(const tf2::Vector3 & p, const BodySegment & seg)
{
  const tf2::Vector3 ab = seg.b - seg.a;
  const double len2 = ab.length2();
  double t = len2 > 1e-12 ? (p - seg.a).dot(ab) / len2 : 0.0;
  t = std::clamp(t, 0.0, 1.0);
  const tf2::Vector3 closest = seg.a + ab * t;
  const float radius_at_t = static_cast<float>(seg.radius_a + t * (seg.radius_b - seg.radius_a));
  return static_cast<float>((p - closest).length()) - radius_at_t;
}

std::vector<std::string> splitCommaList(const std::string & csv)
{
  std::vector<std::string> result;
  std::stringstream ss(csv);
  std::string item;
  while (std::getline(ss, item, ',')) {
    const size_t start = item.find_first_not_of(" \t");
    const size_t end = item.find_last_not_of(" \t");
    if (start != std::string::npos) {
      result.push_back(item.substr(start, end - start + 1));
    }
  }
  return result;
}

std::string expandHome(const std::string & path)
{
  if (path.size() >= 2 && path[0] == '~' && path[1] == '/') {
    const char * home = std::getenv("HOME");
    if (home != nullptr) {
      return std::string(home) + path.substr(1);
    }
  }
  return path;
}

// Deterministic class-id -> colour so a given COCO class always renders the
// same hue in RViz across runs. Golden-angle hue stepping keeps adjacent
// class ids visually distinct instead of drifting through a smooth rainbow.
std_msgs::msg::ColorRGBA classColor(int class_id)
{
  const float hue = std::fmod(static_cast<float>(class_id) * 137.508F, 360.0F);
  cv::Mat hsv(1, 1, CV_32FC3, cv::Scalar(hue, 0.85F, 1.0F));
  cv::Mat rgb;
  cv::cvtColor(hsv, rgb, cv::COLOR_HSV2RGB);
  const cv::Vec3f & c = rgb.at<cv::Vec3f>(0, 0);
  std_msgs::msg::ColorRGBA color;
  color.r = c[0];
  color.g = c[1];
  color.b = c[2];
  color.a = 1.0F;
  return color;
}

// Minimal hand-rolled JSON array serialisation: the only strings involved
// are COCO class names from om6dof_dd_gng::kCocoClasses (fixed, alnum +
// space, no escaping needed), so this avoids pulling in a JSON dependency
// for a message shape this simple.
struct LabeledNode
{
  int index = 0;
  uint32_t node_id = 0;
  std::string class_name;
  float confidence = 0.0F;
  float x = 0.0F;
  float y = 0.0F;
  float z = 0.0F;
};

std::string toJson(const std::vector<LabeledNode> & labeled)
{
  std::ostringstream out;
  out << '[';
  for (size_t i = 0; i < labeled.size(); ++i) {
    const auto & n = labeled[i];
    if (i > 0) {
      out << ',';
    }
    out << "{\"index\":" << n.index
        << ",\"node_id\":" << n.node_id
        << ",\"class\":\"" << n.class_name << "\""
        << ",\"confidence\":" << n.confidence
        << ",\"x\":" << n.x << ",\"y\":" << n.y << ",\"z\":" << n.z << "}";
  }
  out << ']';
  return out.str();
}

}  // namespace

class TopoGngNode : public rclcpp::Node
{
public:
  TopoGngNode()
  : rclcpp::Node("topo_gng_node")
  {
    declareParameters();
    loadParameters();

    gng_ = std::make_unique<DynamicGrowingNeuralGas>(max_nodes_);

    auto detector = std::make_shared<YoloXDetector>(
      expandHome(yolo_model_path_), static_cast<float>(yolo_confidence_),
      static_cast<float>(yolo_nms_threshold_));
    std::unordered_set<std::string> allowed(target_classes_.begin(), target_classes_.end());
    async_yolo_ = std::make_unique<AsyncYolo>(detector, yolo_period_sec_, std::move(allowed));

    env_graph_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      environment_graph_topic_, rclcpp::QoS(2));
    labels_pub_ = create_publisher<std_msgs::msg::String>(labels_topic_, rclcpp::QoS(2));
    robot_graph_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      robot_graph_topic_, rclcpp::QoS(2));

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);

    startCamera();

    running_ = true;
    capture_thread_ = std::thread(&TopoGngNode::captureLoop, this);
  }

  ~TopoGngNode() override
  {
    running_ = false;
    if (capture_thread_.joinable()) {
      capture_thread_.join();
    }
    try {
      pipe_.stop();
    } catch (const std::exception &) {
      // already stopped, or never started -- fine on the way out
    }
  }

private:
  struct BoxDepth
  {
    float depth = 0.0F;        // median over the box interior, metres
    float depth_mad = 0.0F;    // median absolute deviation, metres
    int depth_samples = 0;
  };

  struct TemporalNodeLabel
  {
    int class_id = -1;
    float evidence = 0.0F;
    int missed_frames = 0;
  };

  void declareParameters()
  {
    declare_parameter<int>("pixel_step", 6);
    declare_parameter<int>("max_nodes", 500);
    declare_parameter<int>("updates", 300);
    declare_parameter<double>("z_min", 0.2);
    declare_parameter<double>("z_max", 4.0);
    declare_parameter<int>("width", 640);
    declare_parameter<int>("height", 480);
    declare_parameter<int>("fps", 30);
    declare_parameter<std::string>("world_frame", "world");
    declare_parameter<std::string>("camera_frame", "d405_depth_optical_frame");
    declare_parameter<std::string>("environment_graph_topic", "/om6dof_topo_gng/environment_graph");
    declare_parameter<double>("tf_timeout_sec", 0.1);
    declare_parameter<double>("node_marker_scale", 0.02);
    declare_parameter<double>("edge_marker_width", 0.004);

    declare_parameter<std::string>("yolo_model_path", "~/.cache/om6dof_perception/yolox_s.onnx");
    declare_parameter<double>("yolo_confidence", 0.35);
    declare_parameter<double>("yolo_nms_threshold", 0.5);
    declare_parameter<double>("yolo_period_sec", 0.5);
    // Comma-separated, not a string array parameter: an empty array in a ROS
    // params YAML ("target_classes: []") has no type rclcpp can infer, and
    // declare_parameter() then fails at startup ("No parameter value set")
    // when the empty-array default collides with that untyped override.
    declare_parameter<std::string>("target_classes", "");
    declare_parameter<double>("label_confidence", 0.35);
    declare_parameter<int>("min_label_nodes", 3);
    declare_parameter<std::string>("labels_topic", "/om6dof_topo_gng/labels");

    declare_parameter<std::string>("robot_graph_topic", "/om6dof_topo_gng/robot_graph");
    declare_parameter<double>("body_segment_spacing", 0.05);
    declare_parameter<double>("body_mask_margin", 0.01);
    // Capsule radii, one per kBodyLinks entry -- rough estimates (half the
    // median of each mesh's own bounding-box extents, meshes/chain_link*.stl
    // and d405_wrist_cam.stl), not a CAD fit. Deliberately on the generous
    // side: for the self-body mask (M4's actual point), erring toward
    // over-masking near the robot's own body costs a few discarded
    // environment points close to the arm, while under-masking leaves the
    // wrist camera's own gripper fingers as phantom obstacle nodes -- the
    // exact failure this mask exists to prevent. Tune per-link from here if
    // the M4 verification (no GNG node stuck to a visible finger) fails.
    declare_parameter<double>("body_radius.link1", 0.020);
    declare_parameter<double>("body_radius.link2", 0.021);
    declare_parameter<double>("body_radius.link3", 0.022);
    declare_parameter<double>("body_radius.link4", 0.023);
    declare_parameter<double>("body_radius.link5", 0.021);
    declare_parameter<double>("body_radius.link6", 0.016);
    declare_parameter<double>("body_radius.link7", 0.040);
    declare_parameter<double>("body_radius.end_effector_link", 0.015);
    declare_parameter<double>("body_radius.gripper_left_link", 0.029);
    declare_parameter<double>("body_radius.gripper_right_link", 0.029);
    declare_parameter<double>("body_radius.d405_payload_link", 0.024);
  }

  void loadParameters()
  {
    pixel_step_ = static_cast<int>(get_parameter("pixel_step").as_int());
    max_nodes_ = static_cast<int>(get_parameter("max_nodes").as_int());
    updates_ = static_cast<int>(get_parameter("updates").as_int());
    z_min_ = get_parameter("z_min").as_double();
    z_max_ = get_parameter("z_max").as_double();
    width_ = static_cast<int>(get_parameter("width").as_int());
    height_ = static_cast<int>(get_parameter("height").as_int());
    fps_ = static_cast<int>(get_parameter("fps").as_int());
    world_frame_ = get_parameter("world_frame").as_string();
    camera_frame_ = get_parameter("camera_frame").as_string();
    environment_graph_topic_ = get_parameter("environment_graph_topic").as_string();
    tf_timeout_sec_ = get_parameter("tf_timeout_sec").as_double();
    node_marker_scale_ = get_parameter("node_marker_scale").as_double();
    edge_marker_width_ = get_parameter("edge_marker_width").as_double();

    yolo_model_path_ = get_parameter("yolo_model_path").as_string();
    yolo_confidence_ = get_parameter("yolo_confidence").as_double();
    yolo_nms_threshold_ = get_parameter("yolo_nms_threshold").as_double();
    yolo_period_sec_ = get_parameter("yolo_period_sec").as_double();
    target_classes_ = splitCommaList(get_parameter("target_classes").as_string());
    label_confidence_ = get_parameter("label_confidence").as_double();
    min_label_nodes_ = static_cast<int>(get_parameter("min_label_nodes").as_int());
    labels_topic_ = get_parameter("labels_topic").as_string();

    robot_graph_topic_ = get_parameter("robot_graph_topic").as_string();
    body_segment_spacing_ = get_parameter("body_segment_spacing").as_double();
    body_mask_margin_ = get_parameter("body_mask_margin").as_double();
    for (const auto & link : kBodyLinks) {
      body_radius_[link.frame_id] = get_parameter(link.radius_param).as_double();
    }

    if (pixel_step_ < 1) {
      throw std::runtime_error("pixel_step must be >= 1");
    }
    if (max_nodes_ < 2) {
      throw std::runtime_error("max_nodes must be >= 2");
    }
  }

  void startCamera()
  {
    rs2::config cfg;
    cfg.enable_stream(RS2_STREAM_DEPTH, width_, height_, RS2_FORMAT_Z16, fps_);
    cfg.enable_stream(RS2_STREAM_COLOR, width_, height_, RS2_FORMAT_BGR8, fps_);
    try {
      profile_ = pipe_.start(cfg);
    } catch (const rs2::error & e) {
      RCLCPP_FATAL(
        get_logger(),
        "Failed to start the RealSense D405 (%s). If another process owns it, stop the "
        "conflicting service first: "
        "systemctl --user stop om6dof-dd-gng.service om6dof-perception.service",
        e.what());
      throw;
    }
    auto depth_stream = profile_.get_stream(RS2_STREAM_DEPTH).as<rs2::video_stream_profile>();
    depth_intrinsics_ = depth_stream.get_intrinsics();
    RCLCPP_INFO(
      get_logger(),
      "D405 depth+color started: %dx%d @ %d fps, fx=%.2f fy=%.2f ppx=%.2f ppy=%.2f "
      "(color aligned to depth)",
      width_, height_, fps_,
      depth_intrinsics_.fx, depth_intrinsics_.fy,
      depth_intrinsics_.ppx, depth_intrinsics_.ppy);
  }

  void captureLoop()
  {
    rs2::align align_to_depth(RS2_STREAM_DEPTH);
    while (running_ && rclcpp::ok()) {
      rs2::frameset frames;
      try {
        // Bounded wait so shutdown (running_ flipping false) is noticed
        // promptly instead of blocking indefinitely on a stalled camera.
        if (!pipe_.try_wait_for_frames(&frames, 1000)) {
          continue;
        }
      } catch (const rs2::error & e) {
        RCLCPP_ERROR_THROTTLE(
          get_logger(), *get_clock(), 5000, "RealSense wait_for_frames failed: %s", e.what());
        continue;
      }

      frames = align_to_depth.process(frames);
      rs2::depth_frame depth = frames.get_depth_frame();
      rs2::video_frame color = frames.get_color_frame();
      if (!depth || !color) {
        continue;
      }

      processFrame(depth, color);
    }
  }

  void processFrame(const rs2::depth_frame & depth, const rs2::video_frame & color)
  {
    // depth.get_timestamp() is milliseconds in RS2_TIMESTAMP_DOMAIN_SYSTEM_TIME
    // for D400 depth frames by default -- i.e. already host wall-clock time,
    // the same domain rclcpp::Clock(RCL_ROS_TIME) uses when not running with
    // use_sim_time. That is what lets us look tf2 up "at the frame's own
    // timestamp" per the integration's design decision, without a separate
    // hardware time-sync step. If this camera is ever reconfigured into
    // hardware- or global-timestamp mode, this assumption breaks and the tf2
    // lookup below will start hitting the extrapolation fallback constantly.
    const rclcpp::Time frame_stamp(
      static_cast<int64_t>(depth.get_timestamp() * 1.0e6), RCL_ROS_TIME);

    geometry_msgs::msg::TransformStamped tf_msg;
    if (!lookupCameraToWorld(frame_stamp, tf_msg)) {
      return;
    }
    tf2::Transform camera_to_world;
    tf2::fromMsg(tf_msg.transform, camera_to_world);
    const tf2::Transform world_to_camera = camera_to_world.inverse();

    computeBodySegments();
    publishRobotGraph();

    std::vector<GngPoint3f> points;
    points.reserve(
      static_cast<size_t>((width_ / pixel_step_) + 1) *
      static_cast<size_t>((height_ / pixel_step_) + 1));

    for (int v = 0; v < height_; v += pixel_step_) {
      for (int u = 0; u < width_; u += pixel_step_) {
        const float d = depth.get_distance(u, v);
        if (d <= static_cast<float>(z_min_) || d >= static_cast<float>(z_max_)) {
          continue;
        }
        const float pixel[2] = {static_cast<float>(u), static_cast<float>(v)};
        float camera_point[3];
        rs2_deproject_pixel_to_point(camera_point, &depth_intrinsics_, pixel, d);

        const tf2::Vector3 p_cam(camera_point[0], camera_point[1], camera_point[2]);
        const tf2::Vector3 p_world = camera_to_world * p_cam;
        GngPoint3f world_point{
          static_cast<float>(p_world.x()),
          static_cast<float>(p_world.y()),
          static_cast<float>(p_world.z())};

        if (isMaskedBySelfBody(world_point)) {
          continue;
        }
        points.push_back(world_point);
      }
    }

    if (points.size() >= 2) {
      gng_->partialFit(points, updates_);
    }

    // YOLO: submit the aligned color frame (non-blocking; the worker may
    // still be busy from the previous submission, or the period may not
    // have elapsed -- either way this returns immediately either way) and
    // read back whatever the last completed run produced.
    cv::Mat color_mat(cv::Size(width_, height_), CV_8UC3,
      const_cast<void *>(color.get_data()), cv::Mat::AUTO_STEP);
    async_yolo_->submit(color_mat);
    std::vector<YoloDetection> detections = async_yolo_->snapshot();

    std::vector<GngPoint3f> nodes;
    std::vector<uint32_t> node_ids;
    std::vector<std::pair<uint16_t, uint16_t>> edges;
    gng_->copyGraph(nodes, node_ids, edges);

    std::vector<int16_t> node_class_id;
    std::vector<float> node_confidence;
    labelGraph(depth, detections, world_to_camera, nodes, node_ids, node_class_id, node_confidence);

    publishEnvironmentGraph(nodes, edges, node_class_id);
    publishLabels(nodes, node_ids, node_class_id, node_confidence);
  }

  // Depth over the box interior, current frame, in metres. Mirrors
  // TopoVLA's enrichDepth: inset the box 20% on each side (the label match
  // below needs the depth *inside* the object, not at its silhouette edge,
  // where background/foreground mixing is worst), subsample so a huge box
  // doesn't scan every pixel, then take the median and the median absolute
  // deviation of the surviving in-range samples.
  BoxDepth boxDepth(const rs2::depth_frame & depth, const YoloDetection & det) const
  {
    const float margin_x = det.w * 0.20F;
    const float margin_y = det.h * 0.20F;
    const int left = std::clamp(
      static_cast<int>(std::ceil(det.x + margin_x)), 0, width_ - 1);
    const int right = std::clamp(
      static_cast<int>(std::floor(det.x + det.w - margin_x)), left, width_ - 1);
    const int top = std::clamp(
      static_cast<int>(std::ceil(det.y + margin_y)), 0, height_ - 1);
    const int bottom = std::clamp(
      static_cast<int>(std::floor(det.y + det.h - margin_y)), top, height_ - 1);
    const int area = std::max(1, (right - left + 1) * (bottom - top + 1));
    const int step = std::max(1, static_cast<int>(std::sqrt(area / 2048.0)));

    std::vector<float> samples;
    for (int y = top; y <= bottom; y += step) {
      for (int x = left; x <= right; x += step) {
        const float z = depth.get_distance(x, y);
        if (z > static_cast<float>(z_min_) && z < static_cast<float>(z_max_)) {
          samples.push_back(z);
        }
      }
    }
    BoxDepth result;
    if (samples.empty()) {
      return result;
    }
    result.depth_samples = static_cast<int>(samples.size());
    auto middle = samples.begin() + static_cast<std::ptrdiff_t>(samples.size() / 2);
    std::nth_element(samples.begin(), middle, samples.end());
    result.depth = *middle;
    for (float & s : samples) {
      s = std::abs(s - result.depth);
    }
    middle = samples.begin() + static_cast<std::ptrdiff_t>(samples.size() / 2);
    std::nth_element(samples.begin(), middle, samples.end());
    result.depth_mad = *middle;
    return result;
  }

  // Median depth over the 3x3 neighbourhood around (u, v) in the current
  // frame. Used to check whether a node (possibly made from a much earlier,
  // different camera pose) is still consistent with what the camera sees
  // right now -- a stale or occluded node fails this and never gets or
  // keeps a label, matching TopoVLA's observedDepth().
  float observedDepth(const rs2::depth_frame & depth, int u, int v) const
  {
    std::array<float, 9> values{};
    size_t count = 0;
    for (int dy = -1; dy <= 1; ++dy) {
      const int y = v + dy;
      if (y < 0 || y >= height_) {
        continue;
      }
      for (int dx = -1; dx <= 1; ++dx) {
        const int x = u + dx;
        if (x < 0 || x >= width_) {
          continue;
        }
        const float z = depth.get_distance(x, y);
        if (z >= static_cast<float>(z_min_) && z <= static_cast<float>(z_max_)) {
          values[count++] = z;
        }
      }
    }
    if (count == 0) {
      return 0.0F;
    }
    auto middle = values.begin() + static_cast<std::ptrdiff_t>(count / 2);
    std::nth_element(values.begin(), middle, values.begin() + static_cast<std::ptrdiff_t>(count));
    return *middle;
  }

  void labelGraph(
    const rs2::depth_frame & depth,
    const std::vector<YoloDetection> & detections,
    const tf2::Transform & world_to_camera,
    const std::vector<GngPoint3f> & nodes,
    const std::vector<uint32_t> & node_ids,
    std::vector<int16_t> & node_class_id,
    std::vector<float> & node_confidence)
  {
    const size_t node_count = nodes.size();
    node_class_id.assign(node_count, int16_t{-1});
    node_confidence.assign(node_count, 0.0F);

    if (node_count == 0) {
      temporal_labels_.clear();
      return;
    }
    if (detections.empty()) {
      applyTemporalHold(node_ids, node_class_id, node_confidence, /*any_observed=*/false);
      pruneTemporalLabels(node_ids);
      return;
    }

    std::vector<BoxDepth> box_depths(detections.size());
    std::vector<float> gates(detections.size(), 0.0F);
    std::vector<uint8_t> usable(detections.size(), 0);
    for (size_t i = 0; i < detections.size(); ++i) {
      const YoloDetection & det = detections[i];
      if (det.score < static_cast<float>(label_confidence_)) {
        continue;
      }
      box_depths[i] = boxDepth(depth, det);
      if (box_depths[i].depth_samples < 16 || box_depths[i].depth <= 0.0F) {
        continue;
      }
      // 1.4826x MAD approximates a 1-sigma robust spread for a normal
      // distribution; 2.5 sigma, floored at 12 cm and 8% of range, floored
      // again but this time capped at 60 cm, keeps the gate sane for both
      // very clean and very noisy boxes. Matches TopoVLA's detectionGates.
      const float robust_sigma = 1.4826F * box_depths[i].depth_mad;
      gates[i] = std::clamp(
        std::max({0.12F, 2.5F * robust_sigma, 0.08F * box_depths[i].depth}), 0.12F, 0.60F);
      usable[i] = 1;
    }

    struct Accumulator
    {
      int count = 0;
      double score_sum = 0.0;
    };
    std::vector<Accumulator> accumulators(detections.size());
    std::vector<int> node_best_detection(node_count, -1);
    std::vector<float> node_best_score(node_count, 0.0F);
    bool any_node_matched = false;

    for (size_t node_index = 0; node_index < node_count; ++node_index) {
      const GngPoint3f & node = nodes[node_index];
      if (!(node.x == node.x) || !(node.y == node.y) || !(node.z == node.z)) {
        continue;  // NaN guard
      }
      const tf2::Vector3 p_world(node.x, node.y, node.z);
      const tf2::Vector3 p_cam = world_to_camera * p_world;
      if (!(p_cam.z() > 1e-6)) {
        continue;  // behind or at the camera right now
      }
      const float point[3] = {
        static_cast<float>(p_cam.x()), static_cast<float>(p_cam.y()),
        static_cast<float>(p_cam.z())};
      float pixel[2]{};
      rs2_project_point_to_pixel(pixel, &depth_intrinsics_, point);
      if (!std::isfinite(pixel[0]) || !std::isfinite(pixel[1])) {
        continue;
      }
      const int u = static_cast<int>(std::lround(pixel[0]));
      const int v = static_cast<int>(std::lround(pixel[1]));
      if (u < 0 || u >= width_ || v < 0 || v >= height_) {
        continue;
      }

      const float current_depth = observedDepth(depth, u, v);
      if (current_depth <= 0.0F) {
        continue;
      }
      const float visibility_gate = std::max(0.10F, 0.05F * current_depth);
      const float visible_difference = std::abs(point[2] - current_depth);
      if (visible_difference > visibility_gate) {
        continue;  // occluded or stale: what the camera sees here now disagrees
      }

      int best_detection = -1;
      int best_class = -1;
      float best_score = 0.0F;
      float competing_score = 0.0F;
      int competing_class = -1;
      for (size_t i = 0; i < detections.size(); ++i) {
        if (!usable[i]) {
          continue;
        }
        const YoloDetection & det = detections[i];
        if (pixel[0] < det.x || pixel[0] > det.x + det.w ||
          pixel[1] < det.y || pixel[1] > det.y + det.h)
        {
          continue;
        }
        const float depth_difference = std::abs(point[2] - box_depths[i].depth);
        const float gate = gates[i];
        if (depth_difference > gate) {
          continue;
        }
        const float half_w = std::max(1.0F, 0.5F * det.w);
        const float half_h = std::max(1.0F, 0.5F * det.h);
        const float center_u = det.x + 0.5F * det.w;
        const float center_v = det.y + 0.5F * det.h;
        const float center_distance = std::min(
          1.0F, std::max(std::abs(pixel[0] - center_u) / half_w,
            std::abs(pixel[1] - center_v) / half_h));
        const float depth_quality = std::exp(
          -0.5F * (depth_difference / gate) * (depth_difference / gate));
        const float visibility_quality = std::exp(
          -0.5F * (visible_difference / visibility_gate) *
          (visible_difference / visibility_gate));
        const float center_quality = 0.7F + 0.3F * (1.0F - center_distance);
        const float score = det.score * depth_quality * visibility_quality * center_quality;

        if (det.class_id == best_class) {
          if (score > best_score) {
            best_score = score;
            best_detection = static_cast<int>(i);
          }
        } else if (score > best_score) {
          competing_score = best_score;
          competing_class = best_class;
          best_score = score;
          best_detection = static_cast<int>(i);
          best_class = det.class_id;
        } else if (det.class_id == competing_class) {
          competing_score = std::max(competing_score, score);
        } else if (score > competing_score) {
          competing_score = score;
          competing_class = det.class_id;
        }
      }
      if (best_detection < 0) {
        continue;
      }
      // Two different classes scored within 20% of each other: too close to
      // call, becomes UNKNOWN (i.e. this node just doesn't get a label this
      // frame) rather than silently picking the marginal winner.
      if (competing_score > 0.0F && competing_class != best_class &&
        best_score < 1.20F * competing_score)
      {
        continue;
      }

      node_best_detection[node_index] = best_detection;
      node_best_score[node_index] = best_score;
      any_node_matched = true;
      Accumulator & acc = accumulators[static_cast<size_t>(best_detection)];
      ++acc.count;
      acc.score_sum += best_score;
    }

    std::vector<uint8_t> accepted(detections.size(), 0);
    for (size_t i = 0; i < detections.size(); ++i) {
      if (accumulators[i].count >= min_label_nodes_ && accumulators[i].score_sum >= 0.5) {
        accepted[i] = 1;
      }
    }

    for (size_t node_index = 0; node_index < node_count; ++node_index) {
      const int detection_index = node_best_detection[node_index];
      const bool matched = detection_index >= 0 && accepted[static_cast<size_t>(detection_index)];

      const uint32_t node_id = node_ids[node_index];
      TemporalNodeLabel & state = temporal_labels_[node_id];
      if (matched) {
        const int observed_class = detections[static_cast<size_t>(detection_index)].class_id;
        const float observed_evidence = node_best_score[node_index];
        if (state.class_id == observed_class) {
          state.evidence = std::clamp(0.65F * state.evidence + 0.75F * observed_evidence, 0.0F, 1.0F);
        } else {
          state.class_id = observed_class;
          state.evidence = observed_evidence;
        }
        state.missed_frames = 0;
        node_class_id[node_index] = static_cast<int16_t>(observed_class);
        node_confidence[node_index] = observed_evidence;
      } else {
        state.evidence *= 0.72F;
        ++state.missed_frames;
        // Hold the label for up to 3 missed frames while evidence remains
        // (brief occlusion/re-detection gaps), matching TopoVLA.
        if (state.class_id >= 0 && state.evidence >= 0.05F && state.missed_frames <= 3) {
          node_class_id[node_index] = static_cast<int16_t>(state.class_id);
          node_confidence[node_index] = state.evidence;
        }
      }
    }
    (void)any_node_matched;
    pruneTemporalLabels(node_ids);
  }

  void applyTemporalHold(
    const std::vector<uint32_t> & node_ids, std::vector<int16_t> & node_class_id,
    std::vector<float> & node_confidence, bool /*any_observed*/)
  {
    for (size_t node_index = 0; node_index < node_ids.size(); ++node_index) {
      const uint32_t node_id = node_ids[node_index];
      auto it = temporal_labels_.find(node_id);
      if (it == temporal_labels_.end()) {
        continue;
      }
      TemporalNodeLabel & state = it->second;
      state.evidence *= 0.72F;
      ++state.missed_frames;
      if (state.class_id >= 0 && state.evidence >= 0.05F && state.missed_frames <= 3) {
        node_class_id[node_index] = static_cast<int16_t>(state.class_id);
        node_confidence[node_index] = state.evidence;
      }
    }
  }

  // DD-GNG removes nodes by swapping the last slot into the removed one, so
  // a nodeId that no longer appears anywhere in the current graph is gone
  // for good (not just temporarily off-screen); keep the map from growing
  // forever with labels for nodes that no longer exist.
  void pruneTemporalLabels(const std::vector<uint32_t> & node_ids)
  {
    std::unordered_set<uint32_t> present(node_ids.begin(), node_ids.end());
    for (auto it = temporal_labels_.begin(); it != temporal_labels_.end(); ) {
      if (present.count(it->first) == 0) {
        it = temporal_labels_.erase(it);
      } else {
        ++it;
      }
    }
  }

  bool lookupCameraToWorld(const rclcpp::Time & frame_stamp, geometry_msgs::msg::TransformStamped & out)
  {
    const auto timeout = tf2::durationFromSec(tf_timeout_sec_);
    try {
      out = tf_buffer_->lookupTransform(world_frame_, camera_frame_, frame_stamp, timeout);
      return true;
    } catch (const tf2::TransformException & ex) {
      // Frame-exact lookup failed (buffer not filled that far back/forward
      // yet, e.g. right after startup). Fall back to the latest available
      // transform rather than dropping every frame until timestamps line up
      // exactly; this trades a little pose lag for not stalling the graph.
      try {
        out = tf_buffer_->lookupTransform(world_frame_, camera_frame_, tf2::TimePointZero);
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "tf2 lookup at frame timestamp failed (%s); used latest transform instead.",
          ex.what());
        return true;
      } catch (const tf2::TransformException & ex2) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "No transform %s -> %s available yet (%s); dropping this frame. Is "
          "robot_state_publisher running?",
          world_frame_.c_str(), camera_frame_.c_str(), ex2.what());
        return false;
      }
    }
  }

  // Reads the 11 body-link TF frames (constant topology, positions only
  // change as the arm moves) and rebuilds body_segments_. Latest-available
  // TF is fine here (unlike the camera lookup, this isn't gated on matching
  // a captured frame's exact timestamp) -- a few milliseconds of staleness
  // on a several-centimetre-radius capsule doesn't matter.
  void computeBodySegments()
  {
    std::unordered_map<std::string, tf2::Vector3> positions;
    positions.reserve(kBodyLinks.size());
    for (const auto & link : kBodyLinks) {
      geometry_msgs::msg::TransformStamped tf_msg;
      try {
        tf_msg = tf_buffer_->lookupTransform(world_frame_, link.frame_id, tf2::TimePointZero);
      } catch (const tf2::TransformException & ex) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "No transform %s -> %s yet (%s); robot graph/self-body mask skipped this frame.",
          world_frame_.c_str(), link.frame_id, ex.what());
        body_segments_.clear();
        return;
      }
      positions.emplace(
        link.frame_id,
        tf2::Vector3(
          tf_msg.transform.translation.x, tf_msg.transform.translation.y,
          tf_msg.transform.translation.z));
    }

    body_segments_.clear();
    body_segments_.reserve(kBodyEdges.size());
    for (const auto & [ia, ib] : kBodyEdges) {
      const BodyLinkSpec & a = kBodyLinks[static_cast<size_t>(ia)];
      const BodyLinkSpec & b = kBodyLinks[static_cast<size_t>(ib)];
      BodySegment seg;
      seg.a = positions.at(a.frame_id);
      seg.b = positions.at(b.frame_id);
      seg.radius_a = static_cast<float>(body_radius_.at(a.frame_id));
      seg.radius_b = static_cast<float>(body_radius_.at(b.frame_id));
      body_segments_.push_back(seg);
    }
  }

  // M4: the wrist camera always sees its own gripper fingers, so any depth
  // point within body_mask_margin_ of the robot's own capsule graph is
  // dropped before it can seed or feed a GNG node -- otherwise the fingers
  // show up as ordinary (and, worse, moving) obstacle nodes.
  bool isMaskedBySelfBody(const GngPoint3f & world_point) const
  {
    const tf2::Vector3 p(world_point.x, world_point.y, world_point.z);
    for (const BodySegment & seg : body_segments_) {
      if (capsuleClearance(p, seg) < static_cast<float>(body_mask_margin_)) {
        return true;
      }
    }
    return false;
  }

  void publishRobotGraph()
  {
    const auto stamp = now();

    visualization_msgs::msg::Marker node_marker;
    node_marker.header.frame_id = world_frame_;
    node_marker.header.stamp = stamp;
    node_marker.ns = "robot_graph_nodes";
    node_marker.id = 0;
    node_marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
    node_marker.action = visualization_msgs::msg::Marker::ADD;
    node_marker.pose.orientation.w = 1.0;
    node_marker.scale.x = node_marker_scale_ * 1.5;
    node_marker.scale.y = node_marker_scale_ * 1.5;
    node_marker.scale.z = node_marker_scale_ * 1.5;
    node_marker.color.r = 0.15F;
    node_marker.color.g = 0.35F;
    node_marker.color.b = 0.95F;
    node_marker.color.a = 1.0F;

    visualization_msgs::msg::Marker edge_marker;
    edge_marker.header.frame_id = world_frame_;
    edge_marker.header.stamp = stamp;
    edge_marker.ns = "robot_graph_edges";
    edge_marker.id = 1;
    edge_marker.type = visualization_msgs::msg::Marker::LINE_LIST;
    edge_marker.action = visualization_msgs::msg::Marker::ADD;
    edge_marker.pose.orientation.w = 1.0;
    edge_marker.scale.x = edge_marker_width_ * 1.5;
    edge_marker.color.r = 0.15F;
    edge_marker.color.g = 0.35F;
    edge_marker.color.b = 0.95F;
    edge_marker.color.a = 0.9F;

    auto pushPoint = [](visualization_msgs::msg::Marker & marker, const tf2::Vector3 & v) {
        geometry_msgs::msg::Point p;
        p.x = v.x();
        p.y = v.y();
        p.z = v.z();
        marker.points.push_back(p);
      };

    for (const BodySegment & seg : body_segments_) {
      pushPoint(edge_marker, seg.a);
      pushPoint(edge_marker, seg.b);

      const double length = (seg.b - seg.a).length();
      // Interpolated nodes at ~body_segment_spacing_ along a long edge (e.g.
      // link3-link4, link7-end_effector_link): purely for graph density
      // (matching the environment graph's node+edge look, and giving a
      // denser set of labelled points along the arm for later use), not for
      // the mask itself -- capsuleClearance() already covers the whole
      // segment continuously regardless of how many nodes are drawn here.
      const int extra = std::max(0, static_cast<int>(length / body_segment_spacing_) - 1);
      pushPoint(node_marker, seg.a);
      for (int i = 1; i <= extra; ++i) {
        const double t = static_cast<double>(i) / static_cast<double>(extra + 1);
        pushPoint(node_marker, seg.a + (seg.b - seg.a) * t);
      }
    }
    if (!body_segments_.empty()) {
      pushPoint(node_marker, body_segments_.back().b);
    }

    visualization_msgs::msg::MarkerArray array;
    array.markers.push_back(node_marker);
    array.markers.push_back(edge_marker);
    robot_graph_pub_->publish(array);
  }

  void publishEnvironmentGraph(
    const std::vector<GngPoint3f> & nodes,
    const std::vector<std::pair<uint16_t, uint16_t>> & edges,
    const std::vector<int16_t> & node_class_id)
  {
    const auto stamp = now();

    visualization_msgs::msg::Marker node_marker;
    node_marker.header.frame_id = world_frame_;
    node_marker.header.stamp = stamp;
    node_marker.ns = "environment_graph_nodes";
    node_marker.id = 0;
    node_marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
    node_marker.action = visualization_msgs::msg::Marker::ADD;
    node_marker.pose.orientation.w = 1.0;
    node_marker.scale.x = node_marker_scale_;
    node_marker.scale.y = node_marker_scale_;
    node_marker.scale.z = node_marker_scale_;
    node_marker.color.r = 0.6F;
    node_marker.color.g = 0.6F;
    node_marker.color.b = 0.6F;
    node_marker.color.a = 1.0F;
    node_marker.points.reserve(nodes.size());
    node_marker.colors.reserve(nodes.size());
    for (size_t i = 0; i < nodes.size(); ++i) {
      geometry_msgs::msg::Point p;
      p.x = nodes[i].x;
      p.y = nodes[i].y;
      p.z = nodes[i].z;
      node_marker.points.push_back(p);
      if (node_class_id[i] >= 0) {
        node_marker.colors.push_back(classColor(node_class_id[i]));
      } else {
        std_msgs::msg::ColorRGBA grey;
        grey.r = 0.6F;
        grey.g = 0.6F;
        grey.b = 0.6F;
        grey.a = 1.0F;
        node_marker.colors.push_back(grey);
      }
    }

    visualization_msgs::msg::Marker edge_marker;
    edge_marker.header.frame_id = world_frame_;
    edge_marker.header.stamp = stamp;
    edge_marker.ns = "environment_graph_edges";
    edge_marker.id = 1;
    edge_marker.type = visualization_msgs::msg::Marker::LINE_LIST;
    edge_marker.action = visualization_msgs::msg::Marker::ADD;
    edge_marker.pose.orientation.w = 1.0;
    edge_marker.scale.x = edge_marker_width_;
    edge_marker.color.r = 0.6F;
    edge_marker.color.g = 0.6F;
    edge_marker.color.b = 0.6F;
    edge_marker.color.a = 0.8F;
    edge_marker.points.reserve(edges.size() * 2);
    for (const auto & [a, b] : edges) {
      geometry_msgs::msg::Point pa;
      pa.x = nodes[a].x;
      pa.y = nodes[a].y;
      pa.z = nodes[a].z;
      geometry_msgs::msg::Point pb;
      pb.x = nodes[b].x;
      pb.y = nodes[b].y;
      pb.z = nodes[b].z;
      edge_marker.points.push_back(pa);
      edge_marker.points.push_back(pb);
    }

    visualization_msgs::msg::MarkerArray array;
    array.markers.push_back(node_marker);
    array.markers.push_back(edge_marker);
    env_graph_pub_->publish(array);
  }

  void publishLabels(
    const std::vector<GngPoint3f> & nodes,
    const std::vector<uint32_t> & node_ids,
    const std::vector<int16_t> & node_class_id,
    const std::vector<float> & node_confidence)
  {
    std::vector<LabeledNode> labeled;
    for (size_t i = 0; i < nodes.size(); ++i) {
      if (node_class_id[i] < 0) {
        continue;
      }
      LabeledNode entry;
      entry.index = static_cast<int>(i);
      entry.node_id = node_ids[i];
      entry.class_name = (node_class_id[i] < static_cast<int16_t>(om6dof_dd_gng::kCocoClasses.size()))
        ? om6dof_dd_gng::kCocoClasses[static_cast<size_t>(node_class_id[i])] : "unknown";
      entry.confidence = node_confidence[i];
      entry.x = nodes[i].x;
      entry.y = nodes[i].y;
      entry.z = nodes[i].z;
      labeled.push_back(entry);
    }
    std_msgs::msg::String msg;
    msg.data = toJson(labeled);
    labels_pub_->publish(msg);
  }

  // Parameters
  int pixel_step_ = 6;
  int max_nodes_ = 500;
  int updates_ = 300;
  double z_min_ = 0.2;
  double z_max_ = 4.0;
  int width_ = 640;
  int height_ = 480;
  int fps_ = 30;
  std::string world_frame_;
  std::string camera_frame_;
  std::string environment_graph_topic_;
  double tf_timeout_sec_ = 0.1;
  double node_marker_scale_ = 0.02;
  double edge_marker_width_ = 0.004;

  std::string yolo_model_path_;
  double yolo_confidence_ = 0.35;
  double yolo_nms_threshold_ = 0.5;
  double yolo_period_sec_ = 0.5;
  std::vector<std::string> target_classes_;
  double label_confidence_ = 0.35;
  int min_label_nodes_ = 3;
  std::string labels_topic_;

  std::string robot_graph_topic_;
  double body_segment_spacing_ = 0.05;
  double body_mask_margin_ = 0.01;
  std::unordered_map<std::string, double> body_radius_;
  std::vector<BodySegment> body_segments_;

  // RealSense
  rs2::pipeline pipe_;
  rs2::pipeline_profile profile_;
  rs2_intrinsics depth_intrinsics_{};

  // GNG core
  std::unique_ptr<DynamicGrowingNeuralGas> gng_;

  // YOLO
  std::unique_ptr<AsyncYolo> async_yolo_;
  std::unordered_map<uint32_t, TemporalNodeLabel> temporal_labels_;

  // tf2
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;

  // ROS I/O
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr env_graph_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr labels_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr robot_graph_pub_;

  // Capture thread: rs2::pipeline::wait_for_frames blocks for up to the
  // camera's frame interval, which would starve a SingleThreadedExecutor if
  // it ran on a timer callback, so it gets its own thread instead.
  std::thread capture_thread_;
  std::atomic<bool> running_{false};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  int ret = 0;
  try {
    auto node = std::make_shared<TopoGngNode>();
    rclcpp::spin(node);
  } catch (const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger("topo_gng_node"), "Fatal error: %s", e.what());
    ret = 1;
  }
  rclcpp::shutdown();
  return ret;
}
