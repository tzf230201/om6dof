// topo_gng_node: environment topology for the TopoVLA <-> om6dof_dd_gng
// integration: world-fixed environment DD-GNG, YOLO semantic labels, typed
// environment graph output, and the current-body capsule graph/self-mask.
//
// Pipeline, once per captured depth frame:
//   RealSense D405 (V1) / D435(i) (V2), color aligned to native depth
//     -> pixel_step-strided grid, rs2_deproject_pixel_to_point (camera-frame
//        metres, using the device's own live intrinsics, never hardcoded)
//     -> self-body mask from the robot's live TF capsule graph
//     -> tf2 lookup(world_frame, camera_frame, frame timestamp), applied to
//        every surviving point, so the graph is learned in the WORLD frame
//        and stays put as the wrist (and camera with it) moves
//     -> DynamicDensityGrowingNeuralGas::partialFit (semantic DD-GNG)
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
//     -> fresh, depth-supported matches set WORLD attention regions for the
//        next DD-GNG update; duplicate inference results never extend their TTL.
// Reachability remains a separate ordinary-GNG/kNN implementation.
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
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <deque>
#include <iomanip>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include <librealsense2/rs.hpp>
#include <opencv2/opencv.hpp>

#include "rclcpp/rclcpp.hpp"
#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "std_msgs/msg/color_rgba.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "tf2/exceptions.h"
#include "tf2/LinearMath/Transform.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "om6dof_dd_gng/dynamic_density_gng.hpp"
#include "om6dof_dd_gng/semantic_density_attention.hpp"
#include "om6dof_dd_gng/yolox_detector.hpp"
#include "om6dof_dd_gng/async_yolo.hpp"
#include "om6dof_dd_gng/camera_profile.hpp"
#include "om6dof_dd_gng/msg/environment_graph.hpp"

using namespace std::chrono_literals;
using om6dof_dd_gng::AsyncYolo;
using om6dof_dd_gng::YoloDetection;
using om6dof_dd_gng::YoloXDetector;

namespace
{

using SteadyClock = std::chrono::steady_clock;

double steadySeconds(const SteadyClock::time_point & time = SteadyClock::now())
{
  return std::chrono::duration<double>(time.time_since_epoch()).count();
}

struct YoloSourceFrame
{
  uint64_t sequence;
  SteadyClock::time_point receipt_time;
  tf2::Transform camera_to_world;
};

double elapsedMs(
  const SteadyClock::time_point & start,
  const SteadyClock::time_point & end = SteadyClock::now())
{
  return std::chrono::duration<double, std::milli>(end - start).count();
}

const char * timestampDomainName(rs2_timestamp_domain domain)
{
  switch (domain) {
    case RS2_TIMESTAMP_DOMAIN_HARDWARE_CLOCK:
      return "hardware_clock";
    case RS2_TIMESTAMP_DOMAIN_SYSTEM_TIME:
      return "system_time";
    case RS2_TIMESTAMP_DOMAIN_GLOBAL_TIME:
      return "global_time";
    default:
      return "unknown";
  }
}

const char * submitOutcomeName(AsyncYolo::SubmitOutcome outcome)
{
  switch (outcome) {
    case AsyncYolo::SubmitOutcome::Started:
      return "started";
    case AsyncYolo::SubmitOutcome::Busy:
      return "busy";
    case AsyncYolo::SubmitOutcome::RateLimited:
      return "rate_limited";
    default:
      return "unknown";
  }
}

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

    om6dof_dd_gng::DynamicDensityGngParameters density_parameters;
    density_parameters.max_nodes = max_nodes_;
    density_parameters.attention_sample_fraction = static_cast<float>(
      get_parameter("ddgng_attention_sample_fraction").as_double());
    gng_ = std::make_unique<om6dof_dd_gng::DynamicDensityGrowingNeuralGas>(density_parameters);
    om6dof_dd_gng::SemanticDensityAttentionParameters attention_parameters;
    attention_parameters.max_source_age_sec = semantic_max_source_age_sec_;
    attention_parameters.radius_m = static_cast<float>(
      get_parameter("ddgng_attention_radius_m").as_double());
    attention_parameters.max_strength = static_cast<float>(
      get_parameter("ddgng_max_strength").as_double());
    density_attention_ = std::make_unique<om6dof_dd_gng::SemanticDensityAttention>(
      attention_parameters);
    RCLCPP_INFO(get_logger(),
      "Environment algorithm=semantic_dd_gng_v1; semantic attention + whole-cloud sampling. "
      "Robot reachability algorithm is independent and unchanged.");

    auto detector = std::make_shared<YoloXDetector>(
      expandHome(yolo_model_path_), static_cast<float>(yolo_confidence_),
      static_cast<float>(yolo_nms_threshold_));
    std::unordered_set<std::string> allowed(target_classes_.begin(), target_classes_.end());
    async_yolo_ = std::make_unique<AsyncYolo>(detector, yolo_period_sec_, std::move(allowed));

    env_graph_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      environment_graph_topic_, rclcpp::QoS(2));
    env_graph_data_pub_ = create_publisher<om6dof_dd_gng::msg::EnvironmentGraph>(
      environment_graph_data_topic_, rclcpp::QoS(2).reliable());
    labels_pub_ = create_publisher<std_msgs::msg::String>(labels_topic_, rclcpp::QoS(2));
    robot_graph_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      robot_graph_topic_, rclcpp::QoS(2));
    status_pub_ = create_publisher<std_msgs::msg::String>(
      get_parameter("status_topic").as_string(), rclcpp::QoS(1).reliable().transient_local());
    perception_metrics_pub_ = create_publisher<std_msgs::msg::String>(
      perception_metrics_topic_, rclcpp::QoS(rclcpp::KeepLast(32)).best_effort());
    if (!debug_image_topic_.empty()) {
      debug_image_pub_ = create_publisher<sensor_msgs::msg::CompressedImage>(
        debug_image_topic_, rclcpp::SensorDataQoS().keep_last(2));
    }

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    // Use this node's remaps, including the isolated V2 TF topics. A default
    // internally-created listener node would lose the launch's per-node remaps.
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_, this, true);

    startCamera();
    publishStatus("waiting_for_frames", false);

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

  // Per-capture telemetry is intentionally carried as one local value through
  // the capture thread.  This keeps the instrumentation out of the mapping
  // state and avoids adding locks to the latency-sensitive path.
  struct FrameMetrics
  {
    uint64_t sequence = 0;
    bool frame_received = false;
    bool accepted = false;
    std::string drop_reason;

    int64_t ros_host_receipt_ns = 0;
    double realsense_timestamp_ms = -1.0;
    std::string realsense_timestamp_domain = "unavailable";
    double realsense_color_timestamp_ms = -1.0;
    std::string realsense_color_timestamp_domain = "unavailable";
    double depth_color_timestamp_delta_ms = -1.0;
    uint64_t depth_frame_number = 0;
    uint64_t color_frame_number = 0;
    uint64_t device_frame_gap = 0;
    bool device_frame_number_reset = false;
    std::string tf_mode = "not_attempted";

    double capture_wait_ms = -1.0;
    double alignment_ms = -1.0;
    double camera_tf_ms = -1.0;
    double body_tf_ms = -1.0;
    double robot_graph_publish_ms = -1.0;
    double deprojection_tf_mask_ms = -1.0;
    double gng_update_ms = -1.0;
    double graph_snapshot_ms = -1.0;
    double yolo_submit_ms = -1.0;
    double yolo_snapshot_ms = -1.0;
    double semantic_fusion_ms = -1.0;
    double publication_ms = -1.0;
    double debug_image_ms = -1.0;
    double processing_total_ms = -1.0;
    double host_receipt_to_pipeline_complete_ms = -1.0;
    double capture_cycle_total_ms = -1.0;

    uint64_t sampled_pixels = 0;
    uint64_t depth_rejected = 0;
    uint64_t deprojection_rejected = 0;
    uint64_t self_masked = 0;
    uint64_t accepted_points = 0;
    uint64_t body_segments = 0;
    uint64_t graph_nodes = 0;
    uint64_t graph_edges = 0;
    uint64_t detections = 0;
    uint64_t labeled_nodes = 0;

    std::string yolo_submit_outcome = "not_attempted";
    uint64_t yolo_result_input_frame_sequence = 0;
    uint64_t yolo_result_sequence = 0;
    uint64_t yolo_started_count = 0;
    uint64_t yolo_busy_skip_count = 0;
    uint64_t yolo_rate_limited_count = 0;
    uint64_t yolo_completed_count = 0;
    double yolo_inference_ms = -1.0;
    double yolo_result_age_ms = -1.0;
    double yolo_source_age_ms = -1.0;
    bool yolo_geometry_usable = false;
    uint64_t density_attention_regions = 0;
    uint64_t density_focused_nodes = 0;
    uint64_t density_focused_samples = 0;
    uint64_t density_uniform_samples = 0;
    uint64_t density_regions_next_frame = 0;
    bool yolo_busy = false;
    bool debug_image_published = false;

    SteadyClock::time_point cycle_started{};
    SteadyClock::time_point host_receipt_steady{};
  };

  struct DropCounters
  {
    uint64_t capture_attempts = 0;
    uint64_t received_frames = 0;
    uint64_t published_frames = 0;
    uint64_t dropped_received_frames = 0;
    uint64_t capture_failures = 0;
    uint64_t camera_timeouts = 0;
    uint64_t camera_errors = 0;
    uint64_t invalid_frames = 0;
    uint64_t stale_before_tf = 0;
    uint64_t camera_tf_unavailable = 0;
    uint64_t body_tf_unavailable = 0;
    uint64_t stale_before_deprojection = 0;
    uint64_t stale_before_publish = 0;
    uint64_t processing_errors = 0;
    uint64_t device_frame_gap_total = 0;
    uint64_t device_frame_number_resets = 0;
  };

  void declareParameters()
  {
    rcl_interfaces::msg::ParameterDescriptor startup_only;
    startup_only.read_only = true;
    startup_only.description = "Restart DD-GNG when changing robot/camera geometry";
    declare_parameter<std::string>("model_version", "v1", startup_only);
    declare_parameter<std::string>("camera_model", "auto", startup_only);
    declare_parameter<std::string>("camera_serial", "", startup_only);
    declare_parameter<std::string>("status_topic", "/om6dof_topo_gng/status");
    declare_parameter<std::string>(
      "perception_metrics_topic", "/om6dof_topo_gng/perception_metrics");
    // Empty by default to preserve the previous bandwidth/CPU behaviour.  The
    // V2 config opts in explicitly to a rate-limited compressed debug stream.
    declare_parameter<std::string>("debug_image_topic", "");
    declare_parameter<double>("debug_image_period_sec", 0.5);
    declare_parameter<int>("debug_jpeg_quality", 80);
    declare_parameter<double>("max_frame_age_sec", 0.5, startup_only);
    declare_parameter<int>("pixel_step", 6);
    declare_parameter<int>("max_nodes", 500);
    declare_parameter<int>("updates", 300);
    declare_parameter<std::string>("environment_graph_method", "dd_gng", startup_only);
    declare_parameter<double>("ddgng_attention_sample_fraction", 0.5, startup_only);
    declare_parameter<double>("ddgng_attention_radius_m", 0.05, startup_only);
    declare_parameter<double>("ddgng_max_strength", 3.0, startup_only);
    declare_parameter<double>("semantic_max_source_age_sec", 1.0, startup_only);
    declare_parameter<double>("semantic_max_camera_translation_m", 0.02, startup_only);
    declare_parameter<double>("semantic_max_camera_rotation_rad", 0.10, startup_only);
    declare_parameter<double>("z_min", 0.2);
    declare_parameter<double>("z_max", 4.0);
    declare_parameter<int>("width", 640);
    declare_parameter<int>("height", 480);
    declare_parameter<int>("fps", 30);
    declare_parameter<std::string>("world_frame", "world");
    declare_parameter<std::string>("camera_frame", "auto", startup_only);
    declare_parameter<std::string>("environment_graph_topic", "/om6dof_topo_gng/environment_graph");
    declare_parameter<std::string>(
      "environment_graph_data_topic", "/om6dof_topo_gng/environment_graph_data");
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
    if (get_parameter("environment_graph_method").as_string() != "dd_gng") {
      throw std::runtime_error("Environment supports dd_gng; reachability graph_method is separate");
    }
    semantic_max_source_age_sec_ = get_parameter("semantic_max_source_age_sec").as_double();
    semantic_max_camera_translation_m_ = get_parameter("semantic_max_camera_translation_m").as_double();
    semantic_max_camera_rotation_rad_ = get_parameter("semantic_max_camera_rotation_rad").as_double();
    if (!std::isfinite(semantic_max_source_age_sec_) || semantic_max_source_age_sec_ <= 0.0 ||
      !std::isfinite(semantic_max_camera_translation_m_) || semantic_max_camera_translation_m_ < 0.0 ||
      !std::isfinite(semantic_max_camera_rotation_rad_) || semantic_max_camera_rotation_rad_ < 0.0 ||
      updates_ < 1)
    {
      throw std::runtime_error("Invalid semantic source-age/camera-motion limits or DD-GNG updates");
    }
    z_min_ = get_parameter("z_min").as_double();
    z_max_ = get_parameter("z_max").as_double();
    width_ = static_cast<int>(get_parameter("width").as_int());
    height_ = static_cast<int>(get_parameter("height").as_int());
    fps_ = static_cast<int>(get_parameter("fps").as_int());
    world_frame_ = get_parameter("world_frame").as_string();
    camera_profile_ = om6dof_dd_gng::cameraProfile(
      get_parameter("model_version").as_string(), get_parameter("camera_model").as_string(),
      get_parameter("camera_frame").as_string());
    camera_frame_ = camera_profile_.depth_frame;
    camera_serial_ = get_parameter("camera_serial").as_string();
    max_frame_age_sec_ = get_parameter("max_frame_age_sec").as_double();
    if (width_ <= 0 || height_ <= 0 || fps_ <= 0 ||
      !std::isfinite(max_frame_age_sec_) || max_frame_age_sec_ <= 0)
    {
      throw std::runtime_error("Camera dimensions, FPS and max_frame_age_sec must be positive");
    }
    environment_graph_topic_ = get_parameter("environment_graph_topic").as_string();
    environment_graph_data_topic_ = get_parameter("environment_graph_data_topic").as_string();
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
    perception_metrics_topic_ = get_parameter("perception_metrics_topic").as_string();
    debug_image_topic_ = get_parameter("debug_image_topic").as_string();
    debug_image_period_sec_ = get_parameter("debug_image_period_sec").as_double();
    debug_jpeg_quality_ = static_cast<int>(get_parameter("debug_jpeg_quality").as_int());

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
    if (perception_metrics_topic_.empty()) {
      throw std::runtime_error("perception_metrics_topic must not be empty");
    }
    if (!std::isfinite(debug_image_period_sec_) || debug_image_period_sec_ <= 0.0) {
      throw std::runtime_error("debug_image_period_sec must be positive");
    }
    if (debug_jpeg_quality_ < 1 || debug_jpeg_quality_ > 100) {
      throw std::runtime_error("debug_jpeg_quality must be in [1, 100]");
    }
  }

  void startCamera()
  {
    rs2::context context;
    std::vector<om6dof_dd_gng::CameraIdentity> devices;
    for (const auto & device : context.query_devices()) {
      if (device.supports(RS2_CAMERA_INFO_NAME) && device.supports(RS2_CAMERA_INFO_SERIAL_NUMBER)) {
        devices.push_back({device.get_info(RS2_CAMERA_INFO_NAME),
          device.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER)});
      }
    }
    camera_serial_ = om6dof_dd_gng::selectCameraSerial(devices, camera_profile_, camera_serial_);
    rs2::config cfg;
    cfg.enable_device(camera_serial_);
    cfg.enable_stream(RS2_STREAM_DEPTH, width_, height_, RS2_FORMAT_Z16, fps_);
    cfg.enable_stream(RS2_STREAM_COLOR, width_, height_, RS2_FORMAT_BGR8, fps_);
    try {
      profile_ = pipe_.start(cfg);
    } catch (const rs2::error & e) {
      RCLCPP_FATAL(
        get_logger(),
        "Failed to start the RealSense %s (%s). If another process owns it, stop the "
        "conflicting service first: "
        "systemctl --user stop om6dof-dd-gng.service om6dof-perception.service",
        camera_profile_.model.c_str(), e.what());
      throw;
    }
    auto depth_stream = profile_.get_stream(RS2_STREAM_DEPTH).as<rs2::video_stream_profile>();
    depth_intrinsics_ = depth_stream.get_intrinsics();
    RCLCPP_INFO(
      get_logger(),
      "%s/%s depth+color started: %dx%d @ %d fps, fx=%.2f fy=%.2f ppx=%.2f ppy=%.2f "
      "(color aligned to depth)",
      camera_profile_.version.c_str(), camera_profile_.model.c_str(), width_, height_, fps_,
      depth_intrinsics_.fx, depth_intrinsics_.fy,
      depth_intrinsics_.ppx, depth_intrinsics_.ppy);
    if (camera_profile_.version == "v2") {
      RCLCPP_WARN(get_logger(),
        "V2 world graph uses nominal D435/D435i URDF mounting, not verified hand-eye calibration. "
        "Read-only mapping/preview only; camera and body TF must both come from V2.");
    }
  }

  void publishStatus(const std::string & state, bool accepted)
  {
    std::ostringstream json;
    json << "{\"model_version\":" << std::quoted(camera_profile_.version)
         << ",\"camera_model\":" << std::quoted(camera_profile_.model)
         << ",\"camera_serial\":" << std::quoted(camera_serial_)
         << ",\"camera_frame\":" << std::quoted(camera_frame_)
         << ",\"world_frame\":" << std::quoted(world_frame_)
         << ",\"state\":" << std::quoted(state)
         << ",\"accepted\":" << (accepted ? "true" : "false")
         << ",\"environment_algorithm\":\"semantic_dd_gng_v1\""
         << ",\"calibration_verified\":false,\"transform_source\":\"urdf_nominal\""
         << ",\"timestamp_source\":\"ros_host_frame_receipt\",\"hardware_time_synchronized\":false}";
    std_msgs::msg::String msg;
    msg.data = json.str();
    status_pub_->publish(msg);
  }

  void markDropped(FrameMetrics & metrics, const std::string & reason)
  {
    if (!metrics.drop_reason.empty()) {
      return;
    }
    metrics.drop_reason = reason;
    if (metrics.frame_received) {
      ++drop_counters_.dropped_received_frames;
    } else {
      ++drop_counters_.capture_failures;
    }
    if (reason == "camera_timeout") {
      ++drop_counters_.camera_timeouts;
    } else if (reason == "camera_error") {
      ++drop_counters_.camera_errors;
    } else if (reason == "invalid_frames") {
      ++drop_counters_.invalid_frames;
    } else if (reason == "stale_before_tf") {
      ++drop_counters_.stale_before_tf;
    } else if (reason == "camera_tf_unavailable") {
      ++drop_counters_.camera_tf_unavailable;
    } else if (reason == "body_tf_unavailable") {
      ++drop_counters_.body_tf_unavailable;
    } else if (reason == "stale_before_deprojection") {
      ++drop_counters_.stale_before_deprojection;
    } else if (reason == "stale_before_publish") {
      ++drop_counters_.stale_before_publish;
    } else if (reason == "frame_processing_error") {
      ++drop_counters_.processing_errors;
    }
  }

  void completeAndPublishMetrics(FrameMetrics & metrics)
  {
    const auto completed = SteadyClock::now();
    metrics.capture_cycle_total_ms = elapsedMs(metrics.cycle_started, completed);
    if (metrics.host_receipt_steady.time_since_epoch().count() != 0) {
      metrics.host_receipt_to_pipeline_complete_ms =
        elapsedMs(metrics.host_receipt_steady, completed);
      if (metrics.processing_total_ms < 0.0) {
        metrics.processing_total_ms = metrics.host_receipt_to_pipeline_complete_ms;
      }
    }

    std::ostringstream json;
    json << std::fixed << std::setprecision(3)
         << "{\"schema\":\"om6dof.perception_metrics.v1\""
         << ",\"capture_sequence\":" << metrics.sequence
         << ",\"frame_received\":" << (metrics.frame_received ? "true" : "false")
         << ",\"accepted\":" << (metrics.accepted ? "true" : "false")
         << ",\"drop_reason\":" << std::quoted(metrics.drop_reason)
         << ",\"provenance\":{"
         << "\"ros_host_receipt_ns\":" << metrics.ros_host_receipt_ns
         << ",\"ros_timestamp_source\":\"host_receipt_before_alignment\""
         << ",\"hardware_time_synchronized\":false"
         << ",\"realsense_depth_timestamp_ms\":" << metrics.realsense_timestamp_ms
         << ",\"realsense_depth_timestamp_domain\":"
         << std::quoted(metrics.realsense_timestamp_domain)
         << ",\"realsense_color_timestamp_ms\":"
         << metrics.realsense_color_timestamp_ms
         << ",\"realsense_color_timestamp_domain\":"
         << std::quoted(metrics.realsense_color_timestamp_domain)
         << ",\"depth_color_timestamp_delta_ms\":"
         << metrics.depth_color_timestamp_delta_ms
         << ",\"depth_frame_number\":" << metrics.depth_frame_number
         << ",\"color_frame_number\":" << metrics.color_frame_number
         << ",\"device_frame_gap\":" << metrics.device_frame_gap
         << ",\"device_frame_number_reset\":"
         << (metrics.device_frame_number_reset ? "true" : "false")
         << ",\"world_frame\":" << std::quoted(world_frame_)
         << ",\"camera_frame\":" << std::quoted(camera_frame_)
         << ",\"tf_mode\":" << std::quoted(metrics.tf_mode) << "}"
         << ",\"stages_ms\":{"
         << "\"capture_wait\":" << metrics.capture_wait_ms
         << ",\"alignment\":" << metrics.alignment_ms
         << ",\"camera_tf_lookup\":" << metrics.camera_tf_ms
         << ",\"body_tf_lookup\":" << metrics.body_tf_ms
         << ",\"robot_graph_publication\":" << metrics.robot_graph_publish_ms
         << ",\"deprojection_world_tf_self_mask\":" << metrics.deprojection_tf_mask_ms
         << ",\"dd_gng_update\":" << metrics.gng_update_ms
         << ",\"graph_snapshot\":" << metrics.graph_snapshot_ms
         << ",\"yolo_submit\":" << metrics.yolo_submit_ms
         << ",\"yolo_snapshot\":" << metrics.yolo_snapshot_ms
         << ",\"semantic_fusion\":" << metrics.semantic_fusion_ms
         << ",\"core_publication\":" << metrics.publication_ms
         << ",\"debug_image\":" << metrics.debug_image_ms
         << ",\"processing_total\":" << metrics.processing_total_ms
         << ",\"host_receipt_to_pipeline_complete\":"
         << metrics.host_receipt_to_pipeline_complete_ms
         << ",\"capture_cycle_total\":" << metrics.capture_cycle_total_ms << "}"
         << ",\"counts\":{"
         << "\"sampled_pixels\":" << metrics.sampled_pixels
         << ",\"depth_rejected\":" << metrics.depth_rejected
         << ",\"deprojection_rejected\":" << metrics.deprojection_rejected
         << ",\"self_masked\":" << metrics.self_masked
         << ",\"accepted_points\":" << metrics.accepted_points
         << ",\"body_segments\":" << metrics.body_segments
         << ",\"graph_nodes\":" << metrics.graph_nodes
         << ",\"graph_edges\":" << metrics.graph_edges
         << ",\"detections\":" << metrics.detections
         << ",\"labeled_nodes\":" << metrics.labeled_nodes << "}"
         << ",\"yolo\":{"
         << "\"submit_outcome\":" << std::quoted(metrics.yolo_submit_outcome)
         << ",\"result_input_frame_sequence\":"
         << metrics.yolo_result_input_frame_sequence
         << ",\"result_sequence\":" << metrics.yolo_result_sequence
         << ",\"result_available\":"
         << (metrics.yolo_result_sequence > 0 ? "true" : "false")
         << ",\"result_matches_current_frame\":"
         << (metrics.yolo_result_input_frame_sequence == metrics.sequence &&
      metrics.yolo_result_sequence > 0 ? "true" : "false")
         << ",\"started_count\":" << metrics.yolo_started_count
         << ",\"busy_skip_count\":" << metrics.yolo_busy_skip_count
         << ",\"rate_limited_count\":" << metrics.yolo_rate_limited_count
         << ",\"completed_count\":" << metrics.yolo_completed_count
         << ",\"inference_ms\":" << metrics.yolo_inference_ms
         << ",\"result_age_ms\":" << metrics.yolo_result_age_ms
         << ",\"source_age_ms\":" << metrics.yolo_source_age_ms
         << ",\"geometry_usable\":" << (metrics.yolo_geometry_usable ? "true" : "false")
         << ",\"busy\":" << (metrics.yolo_busy ? "true" : "false") << "}"
         << ",\"density\":{\"algorithm\":\"semantic_dd_gng_v1\""
         << ",\"active_attention_regions\":" << metrics.density_attention_regions
         << ",\"focused_nodes\":" << metrics.density_focused_nodes
         << ",\"focused_samples_cumulative\":" << metrics.density_focused_samples
         << ",\"uniform_samples_cumulative\":" << metrics.density_uniform_samples
         << ",\"attention_regions_next_frame\":" << metrics.density_regions_next_frame << "}"
         << ",\"drops_cumulative\":{"
         << "\"capture_attempts\":" << drop_counters_.capture_attempts
         << ",\"received_frames\":" << drop_counters_.received_frames
         << ",\"published_frames\":" << drop_counters_.published_frames
         << ",\"dropped_received_frames\":" << drop_counters_.dropped_received_frames
         << ",\"capture_failures\":" << drop_counters_.capture_failures
         << ",\"camera_timeouts\":" << drop_counters_.camera_timeouts
         << ",\"camera_errors\":" << drop_counters_.camera_errors
         << ",\"invalid_frames\":" << drop_counters_.invalid_frames
         << ",\"stale_before_tf\":" << drop_counters_.stale_before_tf
         << ",\"camera_tf_unavailable\":" << drop_counters_.camera_tf_unavailable
         << ",\"body_tf_unavailable\":" << drop_counters_.body_tf_unavailable
         << ",\"stale_before_deprojection\":" << drop_counters_.stale_before_deprojection
         << ",\"stale_before_publish\":" << drop_counters_.stale_before_publish
         << ",\"processing_errors\":" << drop_counters_.processing_errors
         << ",\"device_frame_gap_total\":" << drop_counters_.device_frame_gap_total
         << ",\"device_frame_number_resets\":"
         << drop_counters_.device_frame_number_resets << "}"
         << ",\"debug_image\":{"
         << "\"enabled\":" << (debug_image_pub_ ? "true" : "false")
         << ",\"published_this_frame\":"
         << (metrics.debug_image_published ? "true" : "false") << "}}";
    std_msgs::msg::String message;
    message.data = json.str();
    perception_metrics_pub_->publish(message);
  }

  void captureLoop()
  {
    rs2::align align_to_depth(RS2_STREAM_DEPTH);
    while (running_ && rclcpp::ok()) {
      FrameMetrics metrics;
      metrics.sequence = ++metrics_sequence_;
      metrics.cycle_started = SteadyClock::now();
      ++drop_counters_.capture_attempts;

      rs2::frameset frames;
      const auto capture_wait_started = SteadyClock::now();
      try {
        // Bounded wait so shutdown (running_ flipping false) is noticed
        // promptly instead of blocking indefinitely on a stalled camera.
        if (!pipe_.try_wait_for_frames(&frames, 1000)) {
          metrics.capture_wait_ms = elapsedMs(capture_wait_started);
          markDropped(metrics, "camera_timeout");
          publishStatus("camera_timeout", false);
          completeAndPublishMetrics(metrics);
          continue;
        }
      } catch (const rs2::error & e) {
        metrics.capture_wait_ms = elapsedMs(capture_wait_started);
        markDropped(metrics, "camera_error");
        RCLCPP_ERROR_THROTTLE(
          get_logger(), *get_clock(), 5000, "RealSense wait_for_frames failed: %s", e.what());
        publishStatus("camera_error", false);
        completeAndPublishMetrics(metrics);
        continue;
      }
      metrics.capture_wait_ms = elapsedMs(capture_wait_started);
      metrics.frame_received = true;
      ++drop_counters_.received_frames;

      // SDK hardware-clock timestamps are not ROS epoch timestamps. Use an
      // explicitly labelled host receipt time before registration/inference.
      const auto frame_stamp = now();
      metrics.ros_host_receipt_ns = frame_stamp.nanoseconds();
      metrics.host_receipt_steady = SteadyClock::now();
      try {
        const rs2::depth_frame raw_depth = frames.get_depth_frame();
        const rs2::video_frame raw_color = frames.get_color_frame();
        if (raw_depth) {
          metrics.realsense_timestamp_ms = raw_depth.get_timestamp();
          metrics.realsense_timestamp_domain =
            timestampDomainName(raw_depth.get_frame_timestamp_domain());
          metrics.depth_frame_number = raw_depth.get_frame_number();
          if (previous_depth_frame_number_ > 0) {
            if (metrics.depth_frame_number < previous_depth_frame_number_) {
              metrics.device_frame_number_reset = true;
              ++drop_counters_.device_frame_number_resets;
            } else if (metrics.depth_frame_number > previous_depth_frame_number_ + 1) {
              metrics.device_frame_gap =
                metrics.depth_frame_number - previous_depth_frame_number_ - 1;
              drop_counters_.device_frame_gap_total += metrics.device_frame_gap;
            }
          }
          previous_depth_frame_number_ = metrics.depth_frame_number;
        }
        if (raw_color) {
          metrics.realsense_color_timestamp_ms = raw_color.get_timestamp();
          metrics.realsense_color_timestamp_domain =
            timestampDomainName(raw_color.get_frame_timestamp_domain());
          metrics.color_frame_number = raw_color.get_frame_number();
          if (metrics.realsense_timestamp_ms >= 0.0 &&
            metrics.realsense_color_timestamp_domain == metrics.realsense_timestamp_domain)
          {
            metrics.depth_color_timestamp_delta_ms =
              metrics.realsense_color_timestamp_ms - metrics.realsense_timestamp_ms;
          }
        }

        const auto alignment_started = SteadyClock::now();
        frames = align_to_depth.process(frames);
        metrics.alignment_ms = elapsedMs(alignment_started);
        rs2::depth_frame depth = frames.get_depth_frame();
        rs2::video_frame color = frames.get_color_frame();
        if (!running_) {
          break;
        }
        if (!depth || !color) {
          markDropped(metrics, "invalid_frames");
          publishStatus("invalid_frames", false);
        } else {
          processFrame(depth, color, frame_stamp, metrics);
        }
      } catch (const std::exception & ex) {
        markDropped(metrics, "frame_processing_error");
        RCLCPP_ERROR_THROTTLE(
          get_logger(), *get_clock(), 5000, "RealSense frame processing failed: %s", ex.what());
        publishStatus("frame_processing_error", false);
      }
      completeAndPublishMetrics(metrics);
    }
  }

  bool processFrame(
    const rs2::depth_frame & depth, const rs2::video_frame & color,
    const rclcpp::Time & frame_stamp, FrameMetrics & metrics)
  {
    const auto processing_started = SteadyClock::now();
    const auto drop = [this, &metrics, &processing_started](
      const std::string & reason, const std::string & status) -> bool
      {
        markDropped(metrics, reason);
        publishStatus(status, false);
        metrics.processing_total_ms = elapsedMs(processing_started);
        return false;
      };

    if (!om6dof_dd_gng::freshCapture(frame_stamp.nanoseconds(), now().nanoseconds(), max_frame_age_sec_)) {
      return drop("stale_before_tf", "stale_capture");
    }
    geometry_msgs::msg::TransformStamped tf_msg;
    const auto camera_tf_started = SteadyClock::now();
    if (!lookupCameraToWorld(frame_stamp, tf_msg, metrics.tf_mode)) {
      metrics.camera_tf_ms = elapsedMs(camera_tf_started);
      return drop("camera_tf_unavailable", "camera_tf_unavailable");
    }
    metrics.camera_tf_ms = elapsedMs(camera_tf_started);
    tf2::Transform camera_to_world;
    tf2::fromMsg(tf_msg.transform, camera_to_world);
    const tf2::Transform world_to_camera = camera_to_world.inverse();

    const auto body_tf_started = SteadyClock::now();
    const bool body_tf_available = computeBodySegments(frame_stamp);
    metrics.body_tf_ms = elapsedMs(body_tf_started);
    metrics.body_segments = static_cast<uint64_t>(body_segments_.size());
    if (!body_tf_available && camera_profile_.version == "v2") {
      return drop("body_tf_unavailable", "body_tf_unavailable");
    }
    if (!om6dof_dd_gng::freshCapture(frame_stamp.nanoseconds(), now().nanoseconds(), max_frame_age_sec_)) {
      return drop("stale_before_deprojection", "stale_capture");
    }
    last_frame_stamp_ = frame_stamp;
    const auto robot_graph_publish_started = SteadyClock::now();
    publishRobotGraph();
    metrics.robot_graph_publish_ms = elapsedMs(robot_graph_publish_started);

    std::vector<GngPoint3f> points;
    points.reserve(
      static_cast<size_t>((width_ / pixel_step_) + 1) *
      static_cast<size_t>((height_ / pixel_step_) + 1));

    const auto deprojection_started = SteadyClock::now();
    for (int v = 0; v < height_; v += pixel_step_) {
      for (int u = 0; u < width_; u += pixel_step_) {
        ++metrics.sampled_pixels;
        const float d = depth.get_distance(u, v);
        if (!std::isfinite(d) || d <= static_cast<float>(z_min_) || d >= static_cast<float>(z_max_)) {
          ++metrics.depth_rejected;
          continue;
        }
        const float pixel[2] = {static_cast<float>(u), static_cast<float>(v)};
        float camera_point[3];
        rs2_deproject_pixel_to_point(camera_point, &depth_intrinsics_, pixel, d);
        if (!std::isfinite(camera_point[0]) || !std::isfinite(camera_point[1]) || !std::isfinite(camera_point[2])) {
          ++metrics.deprojection_rejected;
          continue;
        }

        const tf2::Vector3 p_cam(camera_point[0], camera_point[1], camera_point[2]);
        const tf2::Vector3 p_world = camera_to_world * p_cam;
        GngPoint3f world_point{
          static_cast<float>(p_world.x()),
          static_cast<float>(p_world.y()),
          static_cast<float>(p_world.z())};

        if (isMaskedBySelfBody(world_point)) {
          ++metrics.self_masked;
          continue;
        }
        points.push_back(world_point);
      }
    }
    metrics.deprojection_tf_mask_ms = elapsedMs(deprojection_started);
    metrics.accepted_points = static_cast<uint64_t>(points.size());

    const auto gng_update_started = SteadyClock::now();
    // ROI expiry uses the originating image receipt time, not repeated uses
    // of the same detector output. Clear expired attention before any learning.
    gng_->setAttentionRegions(density_attention_->regions(steadySeconds()));
    if (points.size() >= 2) {
      gng_->partialFit(points, updates_);
    }
    metrics.gng_update_ms = elapsedMs(gng_update_started);
    const auto density_stats = gng_->stats();
    metrics.density_attention_regions = density_stats.active_attention_regions;
    metrics.density_focused_nodes = density_stats.focused_nodes;
    metrics.density_focused_samples = density_stats.focused_samples;
    metrics.density_uniform_samples = density_stats.uniform_samples;

    // YOLO: submit the aligned color frame (non-blocking; the worker may
    // still be busy from the previous submission, or the period may not
    // have elapsed -- either way this returns immediately either way) and
    // read back whatever the last completed run produced.
    cv::Mat color_mat(cv::Size(width_, height_), CV_8UC3,
      const_cast<void *>(color.get_data()), cv::Mat::AUTO_STEP);
    const auto yolo_submit_started = SteadyClock::now();
    const AsyncYolo::SubmitOutcome yolo_submit_outcome =
      async_yolo_->submit(color_mat, metrics.sequence);
    if (yolo_submit_outcome == AsyncYolo::SubmitOutcome::Started) {
      yolo_source_frames_.push_back({metrics.sequence, metrics.host_receipt_steady, camera_to_world});
      while (yolo_source_frames_.size() > 16) {
        yolo_source_frames_.pop_front();
      }
    }
    metrics.yolo_submit_ms = elapsedMs(yolo_submit_started);
    metrics.yolo_submit_outcome = submitOutcomeName(yolo_submit_outcome);

    const auto yolo_snapshot_started = SteadyClock::now();
    AsyncYolo::Snapshot yolo_snapshot = async_yolo_->snapshot();
    metrics.yolo_snapshot_ms = elapsedMs(yolo_snapshot_started);
    metrics.yolo_result_input_frame_sequence = yolo_snapshot.input_frame_sequence;
    metrics.yolo_result_sequence = yolo_snapshot.result_sequence;
    metrics.yolo_started_count = yolo_snapshot.started_count;
    metrics.yolo_busy_skip_count = yolo_snapshot.busy_skip_count;
    metrics.yolo_rate_limited_count = yolo_snapshot.rate_limited_count;
    metrics.yolo_completed_count = yolo_snapshot.completed_count;
    metrics.yolo_inference_ms = yolo_snapshot.inference_ms;
    metrics.yolo_result_age_ms = yolo_snapshot.result_age_ms;
    metrics.yolo_busy = yolo_snapshot.busy;
    std::vector<YoloDetection> detections = std::move(yolo_snapshot.detections);
    metrics.detections = static_cast<uint64_t>(detections.size());
    const auto source_frame = std::find_if(yolo_source_frames_.begin(), yolo_source_frames_.end(),
      [&yolo_snapshot](const YoloSourceFrame & source) {
        return source.sequence == yolo_snapshot.input_frame_sequence;
      });
    double source_time_sec = -1.0;
    if (source_frame != yolo_source_frames_.end() && yolo_snapshot.result_sequence > 0) {
      source_time_sec = steadySeconds(source_frame->receipt_time);
      metrics.yolo_source_age_ms = (steadySeconds() - source_time_sec) * 1000.0;
      const double translation = (source_frame->camera_to_world.getOrigin() -
        camera_to_world.getOrigin()).length();
      const auto previous_rotation = source_frame->camera_to_world.getRotation();
      const auto current_rotation = camera_to_world.getRotation();
      const double quaternion_dot = previous_rotation.x() * current_rotation.x() +
        previous_rotation.y() * current_rotation.y() + previous_rotation.z() * current_rotation.z() +
        previous_rotation.w() * current_rotation.w();
      const double rotation = 2.0 * std::acos(std::clamp(std::abs(quaternion_dot), 0.0, 1.0));
      metrics.yolo_geometry_usable = std::isfinite(translation) && std::isfinite(rotation) &&
        metrics.yolo_source_age_ms >= 0.0 &&
        metrics.yolo_source_age_ms <= 1000.0 * semantic_max_source_age_sec_ &&
        translation <= semantic_max_camera_translation_m_ &&
        rotation <= semantic_max_camera_rotation_rad_;
    }
    if (!metrics.yolo_geometry_usable) {
      detections.clear();
    }

    std::vector<GngPoint3f> nodes;
    std::vector<uint32_t> node_ids;
    std::vector<std::pair<uint16_t, uint16_t>> edges;
    const auto graph_snapshot_started = SteadyClock::now();
    gng_->copyGraph(nodes, node_ids, edges);
    metrics.graph_snapshot_ms = elapsedMs(graph_snapshot_started);
    metrics.graph_nodes = static_cast<uint64_t>(nodes.size());
    metrics.graph_edges = static_cast<uint64_t>(edges.size());

    std::vector<int16_t> node_class_id;
    std::vector<float> node_confidence;
    std::vector<float> observed_scores;
    const auto semantic_fusion_started = SteadyClock::now();
    labelGraph(depth, detections, world_to_camera, nodes, node_ids, node_class_id, node_confidence,
      &observed_scores);
    density_attention_->observe(yolo_snapshot.result_sequence, source_time_sec, steadySeconds(),
      metrics.yolo_geometry_usable, nodes, observed_scores);
    metrics.density_regions_next_frame = density_attention_->regions(steadySeconds()).size();
    metrics.semantic_fusion_ms = elapsedMs(semantic_fusion_started);
    metrics.labeled_nodes = static_cast<uint64_t>(std::count_if(
      node_class_id.begin(), node_class_id.end(), [](int16_t value) {return value >= 0;}));

    if (!om6dof_dd_gng::freshCapture(frame_stamp.nanoseconds(), now().nanoseconds(), max_frame_age_sec_)) {
      return drop("stale_before_publish", "stale_capture");
    }
    const auto publication_started = SteadyClock::now();
    publishEnvironmentGraph(nodes, edges, node_class_id);
    publishEnvironmentGraphData(
      nodes, node_ids, edges, node_class_id, node_confidence);
    publishLabels(nodes, node_ids, node_class_id, node_confidence);
    publishStatus("mapping", true);
    metrics.publication_ms = elapsedMs(publication_started);

    metrics.accepted = true;
    ++drop_counters_.published_frames;
    const auto debug_image_started = SteadyClock::now();
    metrics.debug_image_published = publishDebugImage(
      color_mat, detections, world_to_camera, nodes, node_class_id, frame_stamp, metrics);
    metrics.debug_image_ms = elapsedMs(debug_image_started);
    metrics.processing_total_ms = elapsedMs(processing_started);
    return true;
  }

  bool publishDebugImage(
    const cv::Mat & color,
    const std::vector<YoloDetection> & detections,
    const tf2::Transform & world_to_camera,
    const std::vector<GngPoint3f> & nodes,
    const std::vector<int16_t> & node_class_id,
    const rclcpp::Time & frame_stamp,
    const FrameMetrics & metrics)
  {
    if (!debug_image_pub_) {
      return false;
    }
    const auto now_steady = SteadyClock::now();
    if (last_debug_image_publish_.time_since_epoch().count() != 0 &&
      std::chrono::duration<double>(now_steady - last_debug_image_publish_).count() <
      debug_image_period_sec_)
    {
      return false;
    }

    try {
      cv::Mat annotated = color.clone();
      for (const auto & detection : detections) {
        const int left = std::clamp(
          static_cast<int>(std::lround(detection.x)), 0, annotated.cols - 1);
        const int top = std::clamp(
          static_cast<int>(std::lround(detection.y)), 0, annotated.rows - 1);
        const int right = std::clamp(
          static_cast<int>(std::lround(detection.x + detection.w)), 0, annotated.cols - 1);
        const int bottom = std::clamp(
          static_cast<int>(std::lround(detection.y + detection.h)), 0, annotated.rows - 1);
        if (right <= left || bottom <= top) {
          continue;
        }
        const auto rgb = classColor(detection.class_id);
        const cv::Scalar bgr(255.0 * rgb.b, 255.0 * rgb.g, 255.0 * rgb.r);
        cv::rectangle(annotated, cv::Point(left, top), cv::Point(right, bottom), bgr, 2);
        std::ostringstream label;
        label << detection.className() << ' ' << std::fixed << std::setprecision(2)
              << detection.score;
        cv::putText(
          annotated, label.str(), cv::Point(left, std::max(14, top - 5)),
          cv::FONT_HERSHEY_SIMPLEX, 0.45, bgr, 1, cv::LINE_AA);
      }

      // Reproject the current world-frame semantic topology into the sensor
      // POV. This makes camera orientation/TF errors visible in one image.
      for (size_t i = 0; i < nodes.size(); ++i) {
        const tf2::Vector3 p_camera = world_to_camera * tf2::Vector3(
          nodes[i].x, nodes[i].y, nodes[i].z);
        if (!(p_camera.z() > 1e-6)) {
          continue;
        }
        const float point[3] = {
          static_cast<float>(p_camera.x()), static_cast<float>(p_camera.y()),
          static_cast<float>(p_camera.z())};
        float pixel[2]{};
        rs2_project_point_to_pixel(pixel, &depth_intrinsics_, point);
        if (!std::isfinite(pixel[0]) || !std::isfinite(pixel[1])) {
          continue;
        }
        const int u = static_cast<int>(std::lround(pixel[0]));
        const int v = static_cast<int>(std::lround(pixel[1]));
        if (u < 0 || u >= annotated.cols || v < 0 || v >= annotated.rows) {
          continue;
        }
        cv::Scalar bgr(180, 180, 180);
        if (i < node_class_id.size() && node_class_id[i] >= 0) {
          const auto rgb = classColor(node_class_id[i]);
          bgr = cv::Scalar(255.0 * rgb.b, 255.0 * rgb.g, 255.0 * rgb.r);
        }
        cv::circle(annotated, cv::Point(u, v), 2, bgr, cv::FILLED, cv::LINE_AA);
      }

      std::ostringstream hud;
      hud << "frame " << metrics.sequence << " | nodes " << metrics.graph_nodes
          << " | labeled " << metrics.labeled_nodes << " | yolo result "
          << metrics.yolo_result_sequence << " age " << std::fixed << std::setprecision(0)
          << metrics.yolo_result_age_ms << " ms";
      cv::putText(
        annotated, hud.str(), cv::Point(8, annotated.rows - 10),
        cv::FONT_HERSHEY_SIMPLEX, 0.45, cv::Scalar(255, 255, 255), 1, cv::LINE_AA);

      std::vector<unsigned char> encoded;
      const std::vector<int> jpeg_parameters{
        cv::IMWRITE_JPEG_QUALITY, debug_jpeg_quality_};
      if (!cv::imencode(".jpg", annotated, encoded, jpeg_parameters)) {
        return false;
      }
      sensor_msgs::msg::CompressedImage message;
      message.header.stamp = frame_stamp;
      message.header.frame_id = camera_frame_;
      message.format = "jpeg";
      message.data = std::move(encoded);
      debug_image_pub_->publish(message);
      last_debug_image_publish_ = now_steady;
      return true;
    } catch (const std::exception & ex) {
      // Debug output must never suppress the graph outputs for this frame.
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000, "Debug image publication failed: %s", ex.what());
      return false;
    }
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
    std::vector<float> & node_confidence, std::vector<float> * observed_scores = nullptr)
  {
    const size_t node_count = nodes.size();
    node_class_id.assign(node_count, int16_t{-1});
    node_confidence.assign(node_count, 0.0F);
    if (observed_scores) {
      observed_scores->assign(node_count, 0.0F);
    }

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
        if (observed_scores) {
          (*observed_scores)[node_index] = observed_evidence;
        }
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

  bool lookupCameraToWorld(
    const rclcpp::Time & frame_stamp, geometry_msgs::msg::TransformStamped & out,
    std::string & tf_mode)
  {
    const auto timeout = tf2::durationFromSec(tf_timeout_sec_);
    try {
      out = tf_buffer_->lookupTransform(world_frame_, camera_frame_, frame_stamp, timeout);
      tf_mode = "exact_frame_stamp";
      return true;
    } catch (const tf2::TransformException & ex) {
      if (camera_profile_.version == "v2") {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
          "V2 capture-time TF %s -> %s unavailable (%s); dropping frame, no latest-TF fallback.",
          world_frame_.c_str(), camera_frame_.c_str(), ex.what());
        tf_mode = "unavailable";
        return false;
      }
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
        tf_mode = "latest_fallback";
        return true;
      } catch (const tf2::TransformException & ex2) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "No transform %s -> %s available yet (%s); dropping this frame. Is "
          "robot_state_publisher running?",
          world_frame_.c_str(), camera_frame_.c_str(), ex2.what());
        tf_mode = "unavailable";
        return false;
      }
    }
  }

  // V2 self-masking uses the same capture-time V2 chain as camera projection.
  // Keep the legacy V1 latest-body behaviour for existing deployments only.
  bool computeBodySegments(const rclcpp::Time & frame_stamp)
  {
    std::unordered_map<std::string, tf2::Vector3> positions;
    positions.reserve(kBodyLinks.size());
    for (const auto & link : kBodyLinks) {
      geometry_msgs::msg::TransformStamped tf_msg;
      try {
        if (camera_profile_.version == "v2") {
          tf_msg = tf_buffer_->lookupTransform(world_frame_, link.frame_id, frame_stamp);
        } else {
          tf_msg = tf_buffer_->lookupTransform(world_frame_, link.frame_id, tf2::TimePointZero);
        }
      } catch (const tf2::TransformException & ex) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "No transform %s -> %s yet (%s); robot graph/self-body mask skipped this frame.",
          world_frame_.c_str(), link.frame_id, ex.what());
        body_segments_.clear();
        return false;
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
    return true;
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
    const auto stamp = last_frame_stamp_;

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
    const auto stamp = last_frame_stamp_;

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

  // Typed planning interface. MarkerArray remains the RViz representation,
  // while this message preserves stable DD-GNG node IDs, semantic state and
  // edge identity for the independent reachability planner.
  void publishEnvironmentGraphData(
    const std::vector<GngPoint3f> & nodes,
    const std::vector<uint32_t> & node_ids,
    const std::vector<std::pair<uint16_t, uint16_t>> & edges,
    const std::vector<int16_t> & node_class_id,
    const std::vector<float> & node_confidence)
  {
    om6dof_dd_gng::msg::EnvironmentGraph message;
    message.header.frame_id = world_frame_;
    message.header.stamp = last_frame_stamp_;
    message.nodes.reserve(nodes.size());
    for (size_t index = 0; index < nodes.size(); ++index) {
      om6dof_dd_gng::msg::EnvironmentNode output;
      output.id = node_ids[index];
      output.position.x = nodes[index].x;
      output.position.y = nodes[index].y;
      output.position.z = nodes[index].z;
      output.class_id = node_class_id[index];
      output.confidence = node_confidence[index];
      message.nodes.push_back(output);
    }
    message.edges.reserve(edges.size());
    for (const auto & [source_index, target_index] : edges) {
      if (source_index >= nodes.size() || target_index >= nodes.size()) {
        continue;
      }
      om6dof_dd_gng::msg::TopologyEdge output;
      output.source_id = node_ids[source_index];
      output.target_id = node_ids[target_index];
      const double dx = static_cast<double>(nodes[source_index].x - nodes[target_index].x);
      const double dy = static_cast<double>(nodes[source_index].y - nodes[target_index].y);
      const double dz = static_cast<double>(nodes[source_index].z - nodes[target_index].z);
      output.cost = std::sqrt(dx * dx + dy * dy + dz * dz);
      message.edges.push_back(output);
    }
    env_graph_data_pub_->publish(message);
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
  om6dof_dd_gng::CameraProfile camera_profile_;
  std::string camera_serial_;
  double max_frame_age_sec_ = 0.5;
  rclcpp::Time last_frame_stamp_{0, 0, RCL_ROS_TIME};
  std::string environment_graph_topic_;
  std::string environment_graph_data_topic_;
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
  double semantic_max_source_age_sec_ = 1.0;
  double semantic_max_camera_translation_m_ = 0.02;
  double semantic_max_camera_rotation_rad_ = 0.10;
  std::string labels_topic_;
  std::string perception_metrics_topic_;
  std::string debug_image_topic_;
  double debug_image_period_sec_ = 0.5;
  int debug_jpeg_quality_ = 80;

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
  std::unique_ptr<om6dof_dd_gng::DynamicDensityGrowingNeuralGas> gng_;
  std::unique_ptr<om6dof_dd_gng::SemanticDensityAttention> density_attention_;

  // YOLO
  std::unique_ptr<AsyncYolo> async_yolo_;
  std::deque<YoloSourceFrame> yolo_source_frames_;
  std::unordered_map<uint32_t, TemporalNodeLabel> temporal_labels_;

  // tf2
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;

  // ROS I/O
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr env_graph_pub_;
  rclcpp::Publisher<om6dof_dd_gng::msg::EnvironmentGraph>::SharedPtr env_graph_data_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr labels_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr robot_graph_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr perception_metrics_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr debug_image_pub_;

  // Written only by capture_thread_, so these cumulative values do not need
  // atomics. They are emitted in every metrics sample for loss-aware bagging.
  uint64_t metrics_sequence_ = 0;
  uint64_t previous_depth_frame_number_ = 0;
  DropCounters drop_counters_;
  SteadyClock::time_point last_debug_image_publish_{};

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
