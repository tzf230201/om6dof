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
//   RGB color frame -> YOLOX (TensorRT or OpenCV) -> 2D detections
//     -> per-box depth (median + MAD from the same retained source frame)
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
#include <condition_variable>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <cstdlib>
#include <deque>
#include <functional>
#include <iomanip>
#include <memory>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <tuple>
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
#include "om6dof_dd_gng/semantic_cluster_propagation.hpp"
#include "om6dof_dd_gng/semantic_density_attention.hpp"
#include "om6dof_dd_gng/tensorrt_yolox_detector.hpp"
#include "om6dof_dd_gng/yolox_detector.hpp"
#include "om6dof_dd_gng/async_yolo.hpp"
#include "om6dof_dd_gng/camera_profile.hpp"
#include "om6dof_dd_gng/msg/environment_graph.hpp"

using namespace std::chrono_literals;
using om6dof_dd_gng::AsyncYolo;
using om6dof_dd_gng::YoloDetection;
using om6dof_dd_gng::YoloDetector;
using om6dof_dd_gng::YoloXDetector;
using om6dof_dd_gng::TensorRtYoloXDetector;

namespace
{

using SteadyClock = std::chrono::steady_clock;

double steadySeconds(const SteadyClock::time_point & time = SteadyClock::now())
{
  return std::chrono::duration<double>(time.time_since_epoch()).count();
}

struct DepthImage
{
  DepthImage(const rs2::depth_frame & frame, int image_width, int image_height)
  : width(image_width), height(image_height), units(frame.get_units()),
    pixels(static_cast<size_t>(image_width) * static_cast<size_t>(image_height))
  {
    std::memcpy(pixels.data(), frame.get_data(), pixels.size() * sizeof(uint16_t));
  }

  float get_distance(int x, int y) const
  {
    if (x < 0 || x >= width || y < 0 || y >= height) {
      return 0.0F;
    }
    return units * static_cast<float>(
      pixels[static_cast<size_t>(y) * static_cast<size_t>(width) + static_cast<size_t>(x)]);
  }

  int width;
  int height;
  float units;
  std::vector<uint16_t> pixels;
};

struct YoloSourceFrame
{
  YoloSourceFrame(
    uint64_t source_sequence, const SteadyClock::time_point & source_receipt_time,
    const rclcpp::Time & source_stamp, DepthImage source_depth,
    cv::Mat source_color)
  : sequence(source_sequence), receipt_time(source_receipt_time), stamp(source_stamp),
    depth(std::move(source_depth)), color_bgr(std::move(source_color))
  {
    camera_to_world.setIdentity();
  }

  uint64_t sequence;
  SteadyClock::time_point receipt_time;
  rclcpp::Time stamp;
  DepthImage depth;
  cv::Mat color_bgr;
  bool pose_valid = false;
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

std::string normalizeTargetClass(std::string value)
{
  const size_t start = value.find_first_not_of(" \t\r\n");
  const size_t end = value.find_last_not_of(" \t\r\n");
  if (start == std::string::npos) {
    return "";
  }
  value = value.substr(start, end - start + 1);
  std::transform(value.begin(), value.end(), value.begin(),
    [](unsigned char c) {return static_cast<char>(std::tolower(c));});
  return value;
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

std_msgs::msg::ColorRGBA semanticNodeColor(int class_id, float alpha = 1.0F)
{
  if (class_id >= 0) {
    auto color = classColor(class_id);
    color.a = alpha;
    return color;
  }
  std_msgs::msg::ColorRGBA grey;
  grey.r = 0.6F;
  grey.g = 0.6F;
  grey.b = 0.6F;
  grey.a = alpha;
  return grey;
}

// A class color identifies semantics; an instance color identifies one YOLO
// box within that class. The same pair is used for its DD-GNG member nodes,
// centroid, text, and membership lines in RViz.
std_msgs::msg::ColorRGBA objectClusterColor(int class_id, int detection_index, float alpha = 1.0F)
{
  const unsigned int seed = static_cast<unsigned int>(class_id + 1) * 2654435761U ^
    static_cast<unsigned int>(detection_index + 1) * 2246822519U;
  const float hue = static_cast<float>(seed % 360U);
  cv::Mat hsv(1, 1, CV_32FC3, cv::Scalar(hue, 0.90F, 1.0F));
  cv::Mat rgb;
  cv::cvtColor(hsv, rgb, cv::COLOR_HSV2RGB);
  const cv::Vec3f & c = rgb.at<cv::Vec3f>(0, 0);
  std_msgs::msg::ColorRGBA color;
  color.r = c[0];
  color.g = c[1];
  color.b = c[2];
  color.a = alpha;
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

struct ObjectCluster
{
  int detection_index = -1;
  uint64_t track_id = 0U;
  int class_id = -1;
  std::string class_name;
  float confidence = 0.0F;
  float x = 0.0F;
  float y = 0.0F;
  float z = 0.0F;
  float tracked_x = 0.0F;
  float tracked_y = 0.0F;
  float tracked_z = 0.0F;
  float box_x = 0.0F;
  float box_y = 0.0F;
  float box_w = 0.0F;
  float box_h = 0.0F;
  std::vector<uint32_t> node_ids;
  std::vector<GngPoint3f> member_points;
};

struct ObjectTrack
{
  uint64_t id = 0U;
  int class_id = -1;
  float x = 0.0F;
  float y = 0.0F;
  float z = 0.0F;
  uint64_t last_seen_frame = 0U;
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

std::string clustersToJson(const std::vector<ObjectCluster> & clusters)
{
  std::ostringstream out;
  out << '[';
  for (size_t i = 0; i < clusters.size(); ++i) {
    const auto & cluster = clusters[i];
    if (i > 0) {out << ',';}
    out << "{\"instance\":" << cluster.detection_index
        << ",\"track_id\":" << cluster.track_id
        << ",\"class_id\":" << cluster.class_id
        << ",\"class\":\"" << cluster.class_name << "\""
        << ",\"confidence\":" << cluster.confidence
        << ",\"centroid\":{\"x\":" << cluster.x << ",\"y\":" << cluster.y
        << ",\"z\":" << cluster.z << "}"
        << ",\"tracked_centroid\":{\"x\":" << cluster.tracked_x
        << ",\"y\":" << cluster.tracked_y << ",\"z\":" << cluster.tracked_z << "}"
        << ",\"box\":{\"x\":" << cluster.box_x << ",\"y\":" << cluster.box_y
        << ",\"w\":" << cluster.box_w << ",\"h\":" << cluster.box_h << "}"
        << ",\"node_ids\":[";
    for (size_t node = 0; node < cluster.node_ids.size(); ++node) {
      if (node > 0) {out << ',';}
      out << cluster.node_ids[node];
    }
    out << "]}";
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

    std::shared_ptr<YoloDetector> detector;
    if (yolo_backend_ == "tensorrt") {
      detector = std::make_shared<TensorRtYoloXDetector>(
        expandHome(yolo_engine_path_), static_cast<float>(yolo_confidence_),
        static_cast<float>(yolo_nms_threshold_));
    } else if (yolo_backend_ == "opencv") {
      detector = std::make_shared<YoloXDetector>(
        expandHome(yolo_model_path_), static_cast<float>(yolo_confidence_),
        static_cast<float>(yolo_nms_threshold_));
    } else {
      throw std::runtime_error("yolo_backend must be 'opencv' or 'tensorrt'");
    }
    RCLCPP_INFO(get_logger(), "YOLOX backend=%s", detector->backendName());
    // Detection and display always keep every COCO class. The selected target
    // is applied later to DD-GNG attention and the planning representation,
    // after all detections have been drawn and fused into semantic node colors.
    async_yolo_ = std::make_unique<AsyncYolo>(
      detector, yolo_period_sec_, std::unordered_set<std::string>{});

    env_graph_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      environment_graph_topic_, rclcpp::QoS(2));
    env_graph_data_pub_ = create_publisher<om6dof_dd_gng::msg::EnvironmentGraph>(
      environment_graph_data_topic_, rclcpp::QoS(2).reliable());
    labels_pub_ = create_publisher<std_msgs::msg::String>(labels_topic_, rclcpp::QoS(2));
    object_clusters_pub_ = create_publisher<std_msgs::msg::String>(
      object_clusters_topic_, rclcpp::QoS(2));
    object_clusters_marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      object_clusters_marker_topic_, rclcpp::QoS(2));
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
    if (!rgb_image_topic_.empty()) {
      rgb_image_pub_ = create_publisher<sensor_msgs::msg::CompressedImage>(
        rgb_image_topic_, rclcpp::SensorDataQoS().keep_last(2));
    }
    target_classes_sub_ = create_subscription<std_msgs::msg::String>(
      target_classes_topic_, rclcpp::QoS(10),
      std::bind(&TopoGngNode::onTargetClassSelection, this, std::placeholders::_1));

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    // Use this node's remaps, including the isolated V2 TF topics. A default
    // internally-created listener node would lose the launch's per-node remaps.
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_, this, true);

    startCamera();
    publishStatus("waiting_for_frames", false);

    running_ = true;
    processing_thread_ = std::thread(&TopoGngNode::processingLoop, this);
    capture_thread_ = std::thread(&TopoGngNode::captureLoop, this);
  }

  ~TopoGngNode() override
  {
    running_ = false;
    frame_queue_cv_.notify_all();
    if (capture_thread_.joinable()) {
      capture_thread_.join();
    }
    if (processing_thread_.joinable()) {
      processing_thread_.join();
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

  // Per-capture telemetry travels with its retained frameset from the camera
  // thread to the mapping thread. The one-frame queue keeps camera display
  // latency bounded while DD-GNG processes at its own sustainable rate.
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
    // `detections` is the raw completed worker output. `fusion_detections`
    // is the subset that passed the source age and camera-pose gates.
    uint64_t detections = 0;
    uint64_t fusion_detections = 0;
    uint64_t labeled_nodes = 0;
    uint64_t inherited_cluster_nodes = 0;
    uint64_t planning_target_nodes = 0;

    std::string yolo_submit_outcome = "not_attempted";
    uint64_t yolo_result_input_frame_sequence = 0;
    uint64_t yolo_result_sequence = 0;
    uint64_t yolo_started_count = 0;
    uint64_t yolo_busy_skip_count = 0;
    uint64_t yolo_rate_limited_count = 0;
    uint64_t yolo_completed_count = 0;
    uint64_t yolo_failed_count = 0;
    double yolo_inference_ms = -1.0;
    double yolo_result_age_ms = -1.0;
    double yolo_source_age_ms = -1.0;
    double yolo_camera_translation_m = -1.0;
    double yolo_camera_rotation_rad = -1.0;
    std::string yolo_geometry_rejection_reason = "no_result";
    std::string yolo_last_error;
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
    uint64_t processing_backlog_drops = 0;
    uint64_t device_frame_gap_total = 0;
    uint64_t device_frame_number_resets = 0;
  };

  struct PendingFrame
  {
    DepthImage depth;
    DepthImage yolo_depth;
    cv::Mat yolo_color_bgr;
    rclcpp::Time frame_stamp;
    FrameMetrics metrics;
  };

  void declareParameters()
  {
    rcl_interfaces::msg::ParameterDescriptor startup_only;
    startup_only.read_only = true;
    startup_only.description = "Restart DD-GNG when changing robot/camera geometry";
    declare_parameter<std::string>("model_version", "v1", startup_only);
    declare_parameter<std::string>("camera_model", "auto", startup_only);
    declare_parameter<std::string>("camera_serial", "", startup_only);
    declare_parameter<std::string>("camera_calibration_file", "", startup_only);
    declare_parameter<std::string>("camera_calibration_sha256", "", startup_only);
    declare_parameter<std::string>("camera_calibration_schema", "", startup_only);
    declare_parameter<std::string>("camera_calibration_model", "", startup_only);
    declare_parameter<std::string>("camera_calibration_serial", "", startup_only);
    declare_parameter<std::string>("camera_calibration_parent_frame", "", startup_only);
    declare_parameter<std::vector<double>>(
      "camera_calibration_color_xyz", std::vector<double>{}, startup_only);
    declare_parameter<std::vector<double>>(
      "camera_calibration_color_quaternion_xyzw", std::vector<double>{}, startup_only);
    declare_parameter<std::string>("camera_calibration_method", "", startup_only);
    declare_parameter<int>("camera_calibration_samples", 0, startup_only);
    declare_parameter<double>("camera_calibration_translation_rmse_m", -1.0, startup_only);
    declare_parameter<double>("camera_calibration_rotation_rmse_rad", -1.0, startup_only);
    declare_parameter<double>("nominal_world_z_offset_m", 0.0, startup_only);
    declare_parameter<std::string>("status_topic", "/om6dof_topo_gng/status");
    declare_parameter<std::string>(
      "perception_metrics_topic", "/om6dof_topo_gng/perception_metrics");
    // Empty by default to preserve the previous bandwidth/CPU behaviour.  The
    // V2 config opts in explicitly to a rate-limited compressed debug stream.
    declare_parameter<std::string>("debug_image_topic", "");
    declare_parameter<double>("debug_image_period_sec", 0.5);
    declare_parameter<int>("debug_jpeg_quality", 80);
    declare_parameter<std::string>("rgb_image_topic", "");
    declare_parameter<double>("rgb_image_period_sec", 1.0 / 30.0);
    declare_parameter<int>("rgb_jpeg_quality", 80);
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
    declare_parameter<std::string>("yolo_backend", "opencv", startup_only);
    declare_parameter<std::string>(
      "yolo_engine_path", "~/.cache/om6dof_perception/yolox_s_fp16.engine", startup_only);
    declare_parameter<double>("yolo_confidence", 0.35);
    declare_parameter<double>("yolo_nms_threshold", 0.5);
    declare_parameter<double>("yolo_period_sec", 0.5);
    // Comma-separated, not a string array parameter: an empty array in a ROS
    // params YAML ("target_classes: []") has no type rclcpp can infer, and
    // declare_parameter() then fails at startup ("No parameter value set")
    // when the empty-array default collides with that untyped override.
    declare_parameter<std::string>("target_classes", "");
    declare_parameter<std::string>(
      "target_classes_topic", "/om6dof_topo_gng/set_target_classes");
    declare_parameter<double>("label_confidence", 0.35);
    declare_parameter<int>("min_label_nodes", 3);
    // Visual completion only: an UNKNOWN node needs multiple same-class
    // DD-GNG neighbours, no competing class, and short 3D edges.
    declare_parameter<bool>("semantic_cluster_inherit_enabled", true);
    declare_parameter<double>("semantic_cluster_inherit_max_edge_m", 0.055);
    declare_parameter<int>("semantic_cluster_inherit_min_neighbours", 2);
    declare_parameter<double>("semantic_cluster_inherit_min_confidence", 0.15);
    declare_parameter<std::string>("labels_topic", "/om6dof_topo_gng/labels");
    declare_parameter<std::string>(
      "object_clusters_topic", "/om6dof_topo_gng/object_clusters");
    declare_parameter<std::string>(
      "object_clusters_marker_topic", "/om6dof_topo_gng/object_clusters_markers");
    declare_parameter<double>("object_track_max_distance_m", 0.08);
    declare_parameter<double>("object_track_smoothing_alpha", 0.35);
    declare_parameter<int>("object_track_hold_frames", 30);

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
    color_frame_ = camera_profile_.version == "v2" ?
      "d435_color_optical_frame" : "d405_color_optical_frame";
    camera_serial_ = get_parameter("camera_serial").as_string();
    camera_calibration_file_ = get_parameter("camera_calibration_file").as_string();
    camera_calibration_sha256_ = get_parameter("camera_calibration_sha256").as_string();
    camera_calibration_schema_ = get_parameter("camera_calibration_schema").as_string();
    camera_calibration_model_ = get_parameter("camera_calibration_model").as_string();
    camera_calibration_serial_ = get_parameter("camera_calibration_serial").as_string();
    camera_calibration_parent_frame_ =
      get_parameter("camera_calibration_parent_frame").as_string();
    camera_calibration_color_xyz_ =
      get_parameter("camera_calibration_color_xyz").as_double_array();
    camera_calibration_color_quaternion_xyzw_ =
      get_parameter("camera_calibration_color_quaternion_xyzw").as_double_array();
    camera_calibration_method_ = get_parameter("camera_calibration_method").as_string();
    camera_calibration_samples_ =
      static_cast<int>(get_parameter("camera_calibration_samples").as_int());
    camera_calibration_translation_rmse_m_ =
      get_parameter("camera_calibration_translation_rmse_m").as_double();
    camera_calibration_rotation_rmse_rad_ =
      get_parameter("camera_calibration_rotation_rmse_rad").as_double();
    nominal_world_z_offset_m_ = get_parameter("nominal_world_z_offset_m").as_double();
    max_frame_age_sec_ = get_parameter("max_frame_age_sec").as_double();
    if (width_ <= 0 || height_ <= 0 || fps_ <= 0 ||
      !std::isfinite(max_frame_age_sec_) || max_frame_age_sec_ <= 0 ||
      !std::isfinite(nominal_world_z_offset_m_) ||
      std::abs(nominal_world_z_offset_m_) > 0.10)
    {
      throw std::runtime_error(
              "Camera dimensions/FPS/frame age must be positive and nominal Z offset bounded");
    }
    environment_graph_topic_ = get_parameter("environment_graph_topic").as_string();
    environment_graph_data_topic_ = get_parameter("environment_graph_data_topic").as_string();
    tf_timeout_sec_ = get_parameter("tf_timeout_sec").as_double();
    node_marker_scale_ = get_parameter("node_marker_scale").as_double();
    edge_marker_width_ = get_parameter("edge_marker_width").as_double();

    yolo_model_path_ = get_parameter("yolo_model_path").as_string();
    yolo_backend_ = get_parameter("yolo_backend").as_string();
    yolo_engine_path_ = get_parameter("yolo_engine_path").as_string();
    yolo_confidence_ = get_parameter("yolo_confidence").as_double();
    yolo_nms_threshold_ = get_parameter("yolo_nms_threshold").as_double();
    yolo_period_sec_ = get_parameter("yolo_period_sec").as_double();
    target_classes_ = splitCommaList(get_parameter("target_classes").as_string());
    target_classes_topic_ = get_parameter("target_classes_topic").as_string();
    label_confidence_ = get_parameter("label_confidence").as_double();
    min_label_nodes_ = static_cast<int>(get_parameter("min_label_nodes").as_int());
    semantic_cluster_propagation_.enabled =
      get_parameter("semantic_cluster_inherit_enabled").as_bool();
    semantic_cluster_propagation_.max_edge_length_m = static_cast<float>(
      get_parameter("semantic_cluster_inherit_max_edge_m").as_double());
    semantic_cluster_propagation_.min_same_class_neighbours = static_cast<size_t>(
      get_parameter("semantic_cluster_inherit_min_neighbours").as_int());
    semantic_cluster_propagation_.min_neighbour_confidence = static_cast<float>(
      get_parameter("semantic_cluster_inherit_min_confidence").as_double());
    if (!std::isfinite(semantic_cluster_propagation_.max_edge_length_m) ||
      semantic_cluster_propagation_.max_edge_length_m <= 0.0F ||
      semantic_cluster_propagation_.min_same_class_neighbours == 0U ||
      !std::isfinite(semantic_cluster_propagation_.min_neighbour_confidence) ||
      semantic_cluster_propagation_.min_neighbour_confidence < 0.0F)
    {
      throw std::runtime_error("Invalid semantic cluster label propagation parameters");
    }
    labels_topic_ = get_parameter("labels_topic").as_string();
    object_clusters_topic_ = get_parameter("object_clusters_topic").as_string();
    object_clusters_marker_topic_ = get_parameter("object_clusters_marker_topic").as_string();
    object_track_max_distance_m_ = get_parameter("object_track_max_distance_m").as_double();
    object_track_smoothing_alpha_ = get_parameter("object_track_smoothing_alpha").as_double();
    object_track_hold_frames_ = static_cast<int>(get_parameter("object_track_hold_frames").as_int());
    perception_metrics_topic_ = get_parameter("perception_metrics_topic").as_string();
    debug_image_topic_ = get_parameter("debug_image_topic").as_string();
    debug_image_period_sec_ = get_parameter("debug_image_period_sec").as_double();
    debug_jpeg_quality_ = static_cast<int>(get_parameter("debug_jpeg_quality").as_int());
    rgb_image_topic_ = get_parameter("rgb_image_topic").as_string();
    rgb_image_period_sec_ = get_parameter("rgb_image_period_sec").as_double();
    rgb_jpeg_quality_ = static_cast<int>(get_parameter("rgb_jpeg_quality").as_int());

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
    if (target_classes_topic_.empty()) {
      throw std::runtime_error("target_classes_topic must not be empty");
    }
    if (!std::isfinite(object_track_max_distance_m_) || object_track_max_distance_m_ <= 0.0 ||
      !std::isfinite(object_track_smoothing_alpha_) || object_track_smoothing_alpha_ <= 0.0 ||
      object_track_smoothing_alpha_ > 1.0 || object_track_hold_frames_ < 1)
    {
      throw std::runtime_error("Invalid object tracking distance, smoothing alpha, or hold frames");
    }
    if (!std::isfinite(debug_image_period_sec_) || debug_image_period_sec_ <= 0.0) {
      throw std::runtime_error("debug_image_period_sec must be positive");
    }
    if (debug_jpeg_quality_ < 1 || debug_jpeg_quality_ > 100) {
      throw std::runtime_error("debug_jpeg_quality must be in [1, 100]");
    }
    if (!std::isfinite(rgb_image_period_sec_) || rgb_image_period_sec_ <= 0.0) {
      throw std::runtime_error("rgb_image_period_sec must be positive");
    }
    if (rgb_jpeg_quality_ < 1 || rgb_jpeg_quality_ > 100) {
      throw std::runtime_error("rgb_jpeg_quality must be in [1, 100]");
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
    auto color_stream = profile_.get_stream(RS2_STREAM_COLOR).as<rs2::video_stream_profile>();
    depth_intrinsics_ = depth_stream.get_intrinsics();
    color_intrinsics_ = color_stream.get_intrinsics();
    configureCameraCalibration(depth_stream, color_stream);
    RCLCPP_INFO(
      get_logger(),
      "%s/%s depth+color started: %dx%d @ %d fps, fx=%.2f fy=%.2f ppx=%.2f ppy=%.2f "
      "(DD-GNG depth frame; YOLO uses raw RGB with depth aligned to color)",
      camera_profile_.version.c_str(), camera_profile_.model.c_str(), width_, height_, fps_,
      depth_intrinsics_.fx, depth_intrinsics_.fy,
      depth_intrinsics_.ppx, depth_intrinsics_.ppy);
    if (camera_profile_.version == "v2" && !calibration_verified_) {
      RCLCPP_WARN(get_logger(),
        "V2 hand-eye calibration is not verified (%s). Read-only mapping/preview only; "
        "camera and body TF must both come from V2.", calibration_reason_.c_str());
      if (std::abs(nominal_world_z_offset_m_) > 1.0e-9) {
        RCLCPP_WARN(get_logger(),
          "Applying provisional world-Z correction %.4fm to nominal V2 depth and RGB poses. "
          "This does not mark hand-eye calibration verified or unlock execution.",
          nominal_world_z_offset_m_);
      }
    } else if (calibration_verified_) {
      RCLCPP_INFO(get_logger(),
        "Using verified eye-in-hand calibration for camera serial %s: %d poses, "
        "translation RMSE %.4fm, rotation RMSE %.4frad, sha256=%s",
        camera_serial_.c_str(), camera_calibration_samples_,
        camera_calibration_translation_rmse_m_, camera_calibration_rotation_rmse_rad_,
        camera_calibration_sha256_.c_str());
    }
  }

  void configureCameraCalibration(
    const rs2::video_stream_profile & depth_stream,
    const rs2::video_stream_profile & color_stream)
  {
    calibration_verified_ = false;
    calibration_reason_ = "artifact_not_configured";
    transform_source_ = "urdf_nominal";
    if (camera_profile_.version == "v2" && std::abs(nominal_world_z_offset_m_) > 1.0e-9) {
      transform_source_ = "urdf_nominal_plus_provisional_world_z_offset";
    }
    calibration_parent_to_color_.setIdentity();
    color_from_depth_.setIdentity();

    // RealSense returns the transform that maps native depth-optical points
    // into colour-optical coordinates. librealsense stores the 3x3 matrix in
    // column-major order (the same indexing used by rs2_transform_point_to_point).
    const rs2_extrinsics depth_to_color = depth_stream.get_extrinsics_to(color_stream);
    color_from_depth_.setBasis(tf2::Matrix3x3(
      depth_to_color.rotation[0], depth_to_color.rotation[3], depth_to_color.rotation[6],
      depth_to_color.rotation[1], depth_to_color.rotation[4], depth_to_color.rotation[7],
      depth_to_color.rotation[2], depth_to_color.rotation[5], depth_to_color.rotation[8]));
    color_from_depth_.setOrigin(tf2::Vector3(
      depth_to_color.translation[0], depth_to_color.translation[1],
      depth_to_color.translation[2]));

    if (camera_calibration_file_.empty()) {
      return;
    }
    const auto finite = [](const std::vector<double> & values) {
        return std::all_of(values.begin(), values.end(), [](double value) {
          return std::isfinite(value);
        });
      };
    if (camera_profile_.version != "v2") {
      calibration_reason_ = "artifact_requires_v2";
      return;
    }
    if (camera_calibration_schema_ != "om6dof.hand_eye.v1") {
      calibration_reason_ = "artifact_schema_invalid";
      return;
    }
    if (camera_calibration_model_ != "D435I" || camera_profile_.model != "D435i") {
      calibration_reason_ = "camera_model_mismatch";
      return;
    }
    if (camera_calibration_serial_.empty() || camera_calibration_serial_ != camera_serial_) {
      calibration_reason_ = "camera_serial_mismatch";
      return;
    }
    if (camera_calibration_parent_frame_ != "end_effector_link") {
      calibration_reason_ = "parent_frame_mismatch";
      return;
    }
    if (camera_calibration_color_xyz_.size() != 3 ||
      camera_calibration_color_quaternion_xyzw_.size() != 4 ||
      !finite(camera_calibration_color_xyz_) ||
      !finite(camera_calibration_color_quaternion_xyzw_))
    {
      calibration_reason_ = "transform_invalid";
      return;
    }
    if (camera_calibration_method_ != "opencv_calibrateHandEye_eye_in_hand" ||
      camera_calibration_samples_ < 8 ||
      !std::isfinite(camera_calibration_translation_rmse_m_) ||
      camera_calibration_translation_rmse_m_ < 0.0 ||
      camera_calibration_translation_rmse_m_ > 0.015 ||
      !std::isfinite(camera_calibration_rotation_rmse_rad_) ||
      camera_calibration_rotation_rmse_rad_ < 0.0 ||
      camera_calibration_rotation_rmse_rad_ > 0.08)
    {
      calibration_reason_ = "validation_quality_failed";
      return;
    }
    if (camera_calibration_sha256_.size() != 64U) {
      calibration_reason_ = "artifact_hash_missing";
      return;
    }
    tf2::Quaternion rotation(
      camera_calibration_color_quaternion_xyzw_[0],
      camera_calibration_color_quaternion_xyzw_[1],
      camera_calibration_color_quaternion_xyzw_[2],
      camera_calibration_color_quaternion_xyzw_[3]);
    if (!std::isfinite(rotation.length2()) || std::abs(rotation.length2() - 1.0) > 1.0e-3) {
      calibration_reason_ = "quaternion_not_normalized";
      return;
    }
    rotation.normalize();
    calibration_parent_to_color_.setOrigin(tf2::Vector3(
      camera_calibration_color_xyz_[0], camera_calibration_color_xyz_[1],
      camera_calibration_color_xyz_[2]));
    calibration_parent_to_color_.setRotation(rotation);
    calibration_verified_ = true;
    calibration_reason_ = "verified";
    transform_source_ = "measured_hand_eye_plus_realsense_factory_extrinsics";
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
         << ",\"yolo_display_classes\":\"all\""
         << ",\"semantic_target_classes\":"
         << std::quoted(targetClassesForStatus())
         << ",\"environment_algorithm\":\"semantic_dd_gng_v1\""
         << ",\"calibration_verified\":" << (calibration_verified_ ? "true" : "false")
         << ",\"calibration_reason\":" << std::quoted(calibration_reason_)
         << ",\"transform_source\":" << std::quoted(transform_source_)
         << ",\"calibration_sha256\":" << std::quoted(camera_calibration_sha256_)
         << ",\"calibration_samples\":" << camera_calibration_samples_
         << ",\"calibration_translation_rmse_m\":"
         << camera_calibration_translation_rmse_m_
         << ",\"calibration_rotation_rmse_rad\":"
         << camera_calibration_rotation_rmse_rad_
         << ",\"nominal_world_z_offset_m\":" << nominal_world_z_offset_m_
         << ",\"nominal_world_z_offset_applied\":"
         << ((!calibration_verified_ && camera_profile_.version == "v2" &&
      std::abs(nominal_world_z_offset_m_) > 1.0e-9) ? "true" : "false")
         << ",\"timestamp_source\":\"ros_host_frame_receipt\",\"hardware_time_synchronized\":false}";
    std_msgs::msg::String msg;
    msg.data = json.str();
    status_pub_->publish(msg);
  }

  std::string targetClassesForStatus() const
  {
    std::lock_guard<std::mutex> lock(target_classes_mutex_);
    if (target_classes_.empty()) {
      return "all";
    }
    std::ostringstream selected;
    for (size_t i = 0; i < target_classes_.size(); ++i) {
      if (i > 0) {
        selected << ',';
      }
      selected << target_classes_[i];
    }
    return selected.str();
  }

  std::vector<std::string> targetClassesSnapshot() const
  {
    std::lock_guard<std::mutex> lock(target_classes_mutex_);
    return target_classes_;
  }

  static bool classIsSelected(
    int16_t class_id, const std::vector<std::string> & selected_classes)
  {
    if (selected_classes.empty()) {
      return class_id >= 0;
    }
    if (class_id < 0 ||
      class_id >= static_cast<int16_t>(om6dof_dd_gng::kCocoClasses.size()))
    {
      return false;
    }
    const std::string class_name =
      om6dof_dd_gng::kCocoClasses[static_cast<size_t>(class_id)];
    return std::find(selected_classes.begin(), selected_classes.end(), class_name) !=
           selected_classes.end();
  }

  void onTargetClassSelection(const std_msgs::msg::String::SharedPtr message)
  {
    const std::string requested = normalizeTargetClass(message->data);
    std::vector<std::string> selected;
    if (!requested.empty() && requested != "all") {
      const auto known = std::find_if(
        om6dof_dd_gng::kCocoClasses.begin(), om6dof_dd_gng::kCocoClasses.end(),
        [&requested](const char * name) {return requested == name;});
      if (known == om6dof_dd_gng::kCocoClasses.end()) {
        RCLCPP_WARN(get_logger(),
          "Ignoring unsupported semantic target %s; expected one COCO class or all.",
          requested.empty() ? "(empty)" : requested.c_str());
        return;
      }
      selected.push_back(requested);
    }
    {
      std::lock_guard<std::mutex> lock(target_classes_mutex_);
      target_classes_ = std::move(selected);
    }
    RCLCPP_INFO(get_logger(),
      "DD-GNG attention/planning target changed to %s; YOLO display remains all classes.",
      targetClassesForStatus().c_str());
  }

  void markDropped(FrameMetrics & metrics, const std::string & reason)
  {
    if (!metrics.drop_reason.empty()) {
      return;
    }
    metrics.drop_reason = reason;
    std::lock_guard<std::mutex> lock(drop_counters_mutex_);
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

    DropCounters counters;
    {
      std::lock_guard<std::mutex> lock(drop_counters_mutex_);
      counters = drop_counters_;
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
         << ",\"fusion_detections\":" << metrics.fusion_detections
         << ",\"labeled_nodes\":" << metrics.labeled_nodes
         << ",\"inherited_cluster_nodes\":" << metrics.inherited_cluster_nodes
         << ",\"planning_target_nodes\":" << metrics.planning_target_nodes << "}"
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
         << ",\"failed_count\":" << metrics.yolo_failed_count
         << ",\"inference_ms\":" << metrics.yolo_inference_ms
         << ",\"result_age_ms\":" << metrics.yolo_result_age_ms
         << ",\"source_age_ms\":" << metrics.yolo_source_age_ms
         << ",\"camera_translation_m\":" << metrics.yolo_camera_translation_m
         << ",\"camera_rotation_rad\":" << metrics.yolo_camera_rotation_rad
         << ",\"geometry_usable\":" << (metrics.yolo_geometry_usable ? "true" : "false")
         << ",\"geometry_rejection_reason\":"
         << std::quoted(metrics.yolo_geometry_rejection_reason)
         << ",\"last_error\":" << std::quoted(metrics.yolo_last_error)
         << ",\"busy\":" << (metrics.yolo_busy ? "true" : "false") << "}"
         << ",\"density\":{\"algorithm\":\"semantic_dd_gng_v1\""
         << ",\"active_attention_regions\":" << metrics.density_attention_regions
         << ",\"focused_nodes\":" << metrics.density_focused_nodes
         << ",\"focused_samples_cumulative\":" << metrics.density_focused_samples
         << ",\"uniform_samples_cumulative\":" << metrics.density_uniform_samples
         << ",\"attention_regions_next_frame\":" << metrics.density_regions_next_frame << "}"
         << ",\"drops_cumulative\":{"
         << "\"capture_attempts\":" << counters.capture_attempts
         << ",\"received_frames\":" << counters.received_frames
         << ",\"published_frames\":" << counters.published_frames
         << ",\"dropped_received_frames\":" << counters.dropped_received_frames
         << ",\"capture_failures\":" << counters.capture_failures
         << ",\"camera_timeouts\":" << counters.camera_timeouts
         << ",\"camera_errors\":" << counters.camera_errors
         << ",\"invalid_frames\":" << counters.invalid_frames
         << ",\"stale_before_tf\":" << counters.stale_before_tf
         << ",\"camera_tf_unavailable\":" << counters.camera_tf_unavailable
         << ",\"body_tf_unavailable\":" << counters.body_tf_unavailable
         << ",\"stale_before_deprojection\":" << counters.stale_before_deprojection
         << ",\"stale_before_publish\":" << counters.stale_before_publish
         << ",\"processing_errors\":" << counters.processing_errors
         << ",\"processing_backlog_drops\":" << counters.processing_backlog_drops
         << ",\"device_frame_gap_total\":" << counters.device_frame_gap_total
         << ",\"device_frame_number_resets\":"
         << counters.device_frame_number_resets << "}"
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
    rs2::align align_to_color(RS2_STREAM_COLOR);
    while (running_ && rclcpp::ok()) {
      FrameMetrics metrics;
      metrics.sequence = ++metrics_sequence_;
      metrics.cycle_started = SteadyClock::now();
      {
        std::lock_guard<std::mutex> lock(drop_counters_mutex_);
        ++drop_counters_.capture_attempts;
      }

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
      {
        std::lock_guard<std::mutex> lock(drop_counters_mutex_);
        ++drop_counters_.received_frames;
      }

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
              std::lock_guard<std::mutex> lock(drop_counters_mutex_);
              ++drop_counters_.device_frame_number_resets;
            } else if (metrics.depth_frame_number > previous_depth_frame_number_ + 1) {
              metrics.device_frame_gap =
                metrics.depth_frame_number - previous_depth_frame_number_ - 1;
              std::lock_guard<std::mutex> lock(drop_counters_mutex_);
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
          publishRawRgb(raw_color, frame_stamp);
        }

        const auto alignment_started = SteadyClock::now();
        const rs2::frameset aligned_frames = align_to_depth.process(frames);
        const rs2::frameset color_aligned_frames = align_to_color.process(frames);
        metrics.alignment_ms = elapsedMs(alignment_started);
        const rs2::depth_frame depth = aligned_frames.get_depth_frame();
        const rs2::video_frame color = aligned_frames.get_color_frame();
        const rs2::depth_frame yolo_depth = color_aligned_frames.get_depth_frame();
        if (!depth || !color || !raw_color || !yolo_depth) {
          markDropped(metrics, "invalid_frames");
          publishStatus("invalid_frames", false);
          completeAndPublishMetrics(metrics);
          continue;
        }

        DepthImage depth_copy(depth, width_, height_);
        cv::Mat raw_color_view(
          cv::Size(width_, height_), CV_8UC3,
          const_cast<void *>(raw_color.get_data()), cv::Mat::AUTO_STEP);
        cv::Mat raw_color_copy = raw_color_view.clone();
        DepthImage yolo_depth_copy(yolo_depth, width_, height_);

        // DD-GNG may take longer than one camera period. Keep only the newest
        // synchronized deep copy so RGB publication stays at camera rate,
        // librealsense buffers are released immediately, and mapping never
        // works through an increasingly stale backlog.
        {
          std::lock_guard<std::mutex> queue_lock(frame_queue_mutex_);
          if (!pending_frames_.empty()) {
            pending_frames_.clear();
            std::lock_guard<std::mutex> counters_lock(drop_counters_mutex_);
            ++drop_counters_.dropped_received_frames;
            ++drop_counters_.processing_backlog_drops;
          }
          pending_frames_.push_back(PendingFrame{
            std::move(depth_copy), std::move(yolo_depth_copy), std::move(raw_color_copy),
            frame_stamp, std::move(metrics)});
        }
        frame_queue_cv_.notify_one();
      } catch (const std::exception & ex) {
        markDropped(metrics, "frame_processing_error");
        RCLCPP_ERROR_THROTTLE(
          get_logger(), *get_clock(), 5000, "RealSense frame capture failed: %s", ex.what());
        publishStatus("frame_processing_error", false);
        completeAndPublishMetrics(metrics);
      }
    }
  }

  void processingLoop()
  {
    while (rclcpp::ok()) {
      std::unique_ptr<PendingFrame> pending;
      {
        std::unique_lock<std::mutex> lock(frame_queue_mutex_);
        frame_queue_cv_.wait(lock, [this]() {
          return !running_ || !pending_frames_.empty();
        });
        if (!running_) {
          return;
        }
        pending = std::make_unique<PendingFrame>(std::move(pending_frames_.back()));
        pending_frames_.clear();
      }

      auto & metrics = pending->metrics;
      try {
        processFrame(
          pending->depth, pending->yolo_depth, pending->yolo_color_bgr,
          pending->frame_stamp, metrics);
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
    const DepthImage & depth, const DepthImage & yolo_depth, const cv::Mat & yolo_color,
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

    // YOLOX is strictly an RGB 2D stage. Submit before any TF lookup so the
    // camera viewer and detector remain useful when the robot pose tree is
    // unavailable. Depth and pose are retained only for later 3D fusion.
    const auto yolo_submit_started = SteadyClock::now();
    const AsyncYolo::SubmitOutcome yolo_submit_outcome =
      async_yolo_->submit(yolo_color, metrics.sequence);
    if (yolo_submit_outcome == AsyncYolo::SubmitOutcome::Started) {
      yolo_source_frames_.emplace_back(
        metrics.sequence, metrics.host_receipt_steady, frame_stamp, yolo_depth, yolo_color.clone());
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
    metrics.yolo_failed_count = yolo_snapshot.failed_count;
    metrics.yolo_inference_ms = yolo_snapshot.inference_ms;
    metrics.yolo_result_age_ms = yolo_snapshot.result_age_ms;
    metrics.yolo_last_error = std::move(yolo_snapshot.last_error);
    metrics.yolo_busy = yolo_snapshot.busy;
    std::vector<YoloDetection> detections = std::move(yolo_snapshot.detections);
    metrics.detections = static_cast<uint64_t>(detections.size());
    auto source_frame = std::find_if(yolo_source_frames_.begin(), yolo_source_frames_.end(),
      [&yolo_snapshot](const YoloSourceFrame & source) {
        return source.sequence == yolo_snapshot.input_frame_sequence;
      });

    geometry_msgs::msg::TransformStamped tf_msg;
    const auto camera_tf_started = SteadyClock::now();
    if (!lookupCameraToWorld(frame_stamp, tf_msg, metrics.tf_mode)) {
      metrics.camera_tf_ms = elapsedMs(camera_tf_started);
      metrics.yolo_geometry_rejection_reason = "camera_tf_unavailable";
      if (source_frame != yolo_source_frames_.end()) {
        metrics.yolo_source_age_ms = 1000.0 *
          (steadySeconds() - steadySeconds(source_frame->receipt_time));
        std::vector<YoloDetection> viewer_detections = detections;
        if (!std::isfinite(metrics.yolo_source_age_ms) || metrics.yolo_source_age_ms < 0.0 ||
          metrics.yolo_source_age_ms > 1000.0 * semantic_max_source_age_sec_)
        {
          viewer_detections.clear();
        }
        const std::vector<GngPoint3f> no_nodes;
        const std::vector<int16_t> no_labels;
        tf2::Transform identity;
        identity.setIdentity();
        metrics.debug_image_published = publishDebugImage(
          source_frame->color_bgr, viewer_detections, identity, color_intrinsics_, no_nodes, no_labels,
          source_frame->stamp, metrics);
      }
      return drop("camera_tf_unavailable", "camera_tf_unavailable");
    }
    metrics.camera_tf_ms = elapsedMs(camera_tf_started);
    tf2::Transform camera_to_world;
    tf2::fromMsg(tf_msg.transform, camera_to_world);
    const tf2::Transform world_to_camera = camera_to_world.inverse();
    geometry_msgs::msg::TransformStamped color_tf_msg;
    tf2::Transform color_camera_to_world;
    color_camera_to_world.setIdentity();
    const bool color_pose_available = lookupColorCameraToWorld(frame_stamp, color_tf_msg);
    if (color_pose_available) {
      tf2::fromMsg(color_tf_msg.transform, color_camera_to_world);
    }
    auto current_source = std::find_if(yolo_source_frames_.begin(), yolo_source_frames_.end(),
      [&metrics](const YoloSourceFrame & source) {return source.sequence == metrics.sequence;});
    if (current_source != yolo_source_frames_.end() && color_pose_available) {
      current_source->camera_to_world = color_camera_to_world;
      current_source->pose_valid = true;
    }

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

    double source_time_sec = -1.0;
    if (yolo_snapshot.result_sequence == 0) {
      metrics.yolo_geometry_rejection_reason = metrics.yolo_last_error.empty() ?
        "no_completed_result" : "worker_error";
    } else if (source_frame == yolo_source_frames_.end()) {
      metrics.yolo_geometry_rejection_reason = "source_frame_not_buffered";
    } else if (!source_frame->pose_valid) {
      metrics.yolo_geometry_rejection_reason = "source_camera_tf_unavailable";
    } else {
      source_time_sec = steadySeconds(source_frame->receipt_time);
      metrics.yolo_source_age_ms = (steadySeconds() - source_time_sec) * 1000.0;
      metrics.yolo_camera_translation_m = (source_frame->camera_to_world.getOrigin() -
        color_camera_to_world.getOrigin()).length();
      const auto previous_rotation = source_frame->camera_to_world.getRotation();
      const auto current_rotation = color_camera_to_world.getRotation();
      const double quaternion_dot = previous_rotation.x() * current_rotation.x() +
        previous_rotation.y() * current_rotation.y() + previous_rotation.z() * current_rotation.z() +
        previous_rotation.w() * current_rotation.w();
      metrics.yolo_camera_rotation_rad =
        2.0 * std::acos(std::clamp(std::abs(quaternion_dot), 0.0, 1.0));
      if (!metrics.yolo_last_error.empty()) {
        metrics.yolo_geometry_rejection_reason = "worker_error";
      } else if (!std::isfinite(metrics.yolo_source_age_ms) || metrics.yolo_source_age_ms < 0.0 ||
        metrics.yolo_source_age_ms > 1000.0 * semantic_max_source_age_sec_)
      {
        metrics.yolo_geometry_rejection_reason = "source_age_exceeded";
      } else if (!std::isfinite(metrics.yolo_camera_translation_m) ||
        !std::isfinite(metrics.yolo_camera_rotation_rad))
      {
        metrics.yolo_geometry_rejection_reason = "invalid_camera_pose_delta";
      } else if (metrics.yolo_camera_translation_m > semantic_max_camera_translation_m_) {
        metrics.yolo_geometry_rejection_reason = "camera_translation_exceeded";
      } else if (metrics.yolo_camera_rotation_rad > semantic_max_camera_rotation_rad_) {
        metrics.yolo_geometry_rejection_reason = "camera_rotation_exceeded";
      } else {
        metrics.yolo_geometry_usable = true;
        metrics.yolo_geometry_rejection_reason = "none";
      }
    }
    if (!metrics.yolo_geometry_usable) {
      detections.clear();
    }
    metrics.fusion_detections = static_cast<uint64_t>(detections.size());

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
    std::vector<int> node_detection_index;
    const auto semantic_fusion_started = SteadyClock::now();
    const DepthImage & fusion_depth = source_frame != yolo_source_frames_.end() ?
      source_frame->depth : depth;
    const tf2::Transform fusion_world_to_camera =
      source_frame != yolo_source_frames_.end() && source_frame->pose_valid ?
      source_frame->camera_to_world.inverse() : world_to_camera;
    labelGraph(fusion_depth, detections, fusion_world_to_camera, color_intrinsics_, nodes, node_ids,
      node_class_id, node_confidence, &observed_scores, &node_detection_index);
    metrics.inherited_cluster_nodes = om6dof_dd_gng::propagateUnknownClusterLabels(
      nodes, edges, node_class_id, node_confidence, semantic_cluster_propagation_);

    // Preserve the full multi-class semantic coloring above. Selection only
    // gates which direct evidence can steer DD-GNG density and which labeled
    // nodes are exposed as targets to the independent reachability planner.
    const std::vector<std::string> selected_target_classes = targetClassesSnapshot();
    std::vector<float> attention_scores = observed_scores;
    if (!selected_target_classes.empty()) {
      for (size_t index = 0; index < attention_scores.size(); ++index) {
        if (!classIsSelected(node_class_id[index], selected_target_classes)) {
          attention_scores[index] = 0.0F;
        }
      }
    }
    density_attention_->observe(yolo_snapshot.result_sequence, source_time_sec, steadySeconds(),
      metrics.yolo_geometry_usable, nodes, attention_scores);
    metrics.density_regions_next_frame = density_attention_->regions(steadySeconds()).size();
    metrics.semantic_fusion_ms = elapsedMs(semantic_fusion_started);
    metrics.labeled_nodes = static_cast<uint64_t>(std::count_if(
      node_class_id.begin(), node_class_id.end(), [](int16_t value) {return value >= 0;}));
    metrics.planning_target_nodes = static_cast<uint64_t>(std::count_if(
      node_class_id.begin(), node_class_id.end(),
      [&selected_target_classes](int16_t value) {
        return classIsSelected(value, selected_target_classes);
      }));

    if (!om6dof_dd_gng::freshCapture(frame_stamp.nanoseconds(), now().nanoseconds(), max_frame_age_sec_)) {
      return drop("stale_before_publish", "stale_capture");
    }
    const auto publication_started = SteadyClock::now();
    publishEnvironmentGraph(nodes, edges, node_class_id, node_detection_index, detections);
    publishEnvironmentGraphData(
      nodes, node_ids, edges, node_class_id, node_confidence, selected_target_classes);
    publishLabels(nodes, node_ids, node_class_id, node_confidence);
    publishObjectClusters(detections, nodes, node_ids, node_detection_index);
    publishStatus("mapping", true);
    metrics.publication_ms = elapsedMs(publication_started);

    metrics.accepted = true;
    {
      std::lock_guard<std::mutex> lock(drop_counters_mutex_);
      ++drop_counters_.published_frames;
    }
    const auto debug_image_started = SteadyClock::now();
    const cv::Mat & debug_color = source_frame != yolo_source_frames_.end() ?
      source_frame->color_bgr : yolo_color;
    const rclcpp::Time debug_stamp = source_frame != yolo_source_frames_.end() ?
      source_frame->stamp : frame_stamp;
    metrics.debug_image_published = publishDebugImage(
      debug_color, detections, fusion_world_to_camera, color_intrinsics_, nodes, node_class_id,
      debug_stamp, metrics);
    metrics.debug_image_ms = elapsedMs(debug_image_started);
    metrics.processing_total_ms = elapsedMs(processing_started);
    return true;
  }

  bool publishDebugImage(
    const cv::Mat & color,
    const std::vector<YoloDetection> & detections,
    const tf2::Transform & world_to_camera,
    const rs2_intrinsics & projection_intrinsics,
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
        rs2_project_point_to_pixel(pixel, &projection_intrinsics, point);
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

  void publishRawRgb(const rs2::video_frame & color, const rclcpp::Time & frame_stamp)
  {
    if (!rgb_image_pub_) {
      return;
    }
    const auto now_steady = SteadyClock::now();
    if (last_rgb_image_publish_.time_since_epoch().count() != 0 &&
      std::chrono::duration<double>(now_steady - last_rgb_image_publish_).count() <
      rgb_image_period_sec_)
    {
      return;
    }
    try {
      cv::Mat bgr(cv::Size(width_, height_), CV_8UC3,
        const_cast<void *>(color.get_data()), cv::Mat::AUTO_STEP);
      std::vector<unsigned char> encoded;
      const std::vector<int> jpeg_parameters{
        cv::IMWRITE_JPEG_QUALITY, rgb_jpeg_quality_};
      if (!cv::imencode(".jpg", bgr, encoded, jpeg_parameters)) {
        return;
      }
      sensor_msgs::msg::CompressedImage message;
      message.header.stamp = frame_stamp;
      message.header.frame_id = color_frame_;
      message.format = "jpeg";
      message.data = std::move(encoded);
      rgb_image_pub_->publish(message);
      last_rgb_image_publish_ = now_steady;
    } catch (const std::exception & ex) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000, "Raw RGB publication failed: %s", ex.what());
    }
  }

  // Depth over the box interior in the retained YOLO source frame, in metres. Mirrors
  // TopoVLA's enrichDepth: inset the box 20% on each side (the label match
  // below needs the depth *inside* the object, not at its silhouette edge,
  // where background/foreground mixing is worst), subsample so a huge box
  // doesn't scan every pixel, then take the median and the median absolute
  // deviation of the surviving in-range samples.
  BoxDepth boxDepth(const DepthImage & depth, const YoloDetection & det) const
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

  // Median depth over the 3x3 neighbourhood around (u, v) in the retained
  // YOLO source frame. Used to check whether a world node is consistent with
  // what the camera saw when the 2D box was produced -- an occluded node fails this and never gets or
  // keeps a label, matching TopoVLA's observedDepth().
  float observedDepth(const DepthImage & depth, int u, int v) const
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
    const DepthImage & depth,
    const std::vector<YoloDetection> & detections,
    const tf2::Transform & world_to_camera,
    const rs2_intrinsics & projection_intrinsics,
    const std::vector<GngPoint3f> & nodes,
    const std::vector<uint32_t> & node_ids,
    std::vector<int16_t> & node_class_id,
    std::vector<float> & node_confidence, std::vector<float> * observed_scores = nullptr,
    std::vector<int> * node_detection_index = nullptr)
  {
    const size_t node_count = nodes.size();
    node_class_id.assign(node_count, int16_t{-1});
    node_confidence.assign(node_count, 0.0F);
    if (observed_scores) {
      observed_scores->assign(node_count, 0.0F);
    }
    if (node_detection_index) {
      node_detection_index->assign(node_count, -1);
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
      rs2_project_point_to_pixel(pixel, &projection_intrinsics, point);
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
        if (node_detection_index) {
          // Only a direct, current accepted YOLO/depth match belongs to an
          // object instance. Temporally held labels deliberately never create
          // or prolong an object cluster.
          (*node_detection_index)[node_index] = detection_index;
        }
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
    if (calibration_verified_) {
      return lookupCalibratedCameraToWorld(frame_stamp, false, out, tf_mode);
    }
    const auto timeout = tf2::durationFromSec(tf_timeout_sec_);
    try {
      out = tf_buffer_->lookupTransform(world_frame_, camera_frame_, frame_stamp, timeout);
      applyNominalWorldOffset(out);
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
        applyNominalWorldOffset(out);
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

  bool lookupColorCameraToWorld(
    const rclcpp::Time & frame_stamp, geometry_msgs::msg::TransformStamped & out)
  {
    if (calibration_verified_) {
      std::string ignored_mode;
      return lookupCalibratedCameraToWorld(frame_stamp, true, out, ignored_mode);
    }
    try {
      out = tf_buffer_->lookupTransform(
        world_frame_, color_frame_, frame_stamp, tf2::durationFromSec(tf_timeout_sec_));
      applyNominalWorldOffset(out);
      return true;
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
        "RGB source pose %s -> %s unavailable (%s); withholding RGB-to-3D labels.",
        world_frame_.c_str(), color_frame_.c_str(), ex.what());
      return false;
    }
  }

  bool lookupCalibratedCameraToWorld(
    const rclcpp::Time & frame_stamp, bool color,
    geometry_msgs::msg::TransformStamped & out, std::string & tf_mode)
  {
    try {
      const auto parent_msg = tf_buffer_->lookupTransform(
        world_frame_, camera_calibration_parent_frame_, frame_stamp,
        tf2::durationFromSec(tf_timeout_sec_));
      tf2::Transform world_from_parent;
      tf2::fromMsg(parent_msg.transform, world_from_parent);
      const tf2::Transform parent_from_sensor = color ?
        calibration_parent_to_color_ :
        calibration_parent_to_color_ * color_from_depth_;
      out.header.stamp = frame_stamp;
      out.header.frame_id = world_frame_;
      out.child_frame_id = color ? color_frame_ : camera_frame_;
      out.transform = tf2::toMsg(world_from_parent * parent_from_sensor);
      tf_mode = "exact_frame_stamp_calibrated_hand_eye";
      return true;
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Calibrated capture-time TF %s -> %s unavailable (%s); dropping frame.",
        world_frame_.c_str(), camera_calibration_parent_frame_.c_str(), ex.what());
      tf_mode = "calibrated_parent_unavailable";
      return false;
    }
  }

  void applyNominalWorldOffset(geometry_msgs::msg::TransformStamped & transform) const
  {
    // Keep the provisional correction separate from both the URDF and the
    // measured hand-eye artifact. Depth and raw-RGB poses receive the same
    // world-axis translation so graph construction, semantic reprojection,
    // debug overlays and camera-motion gates share coherent geometry.
    if (!calibration_verified_ && camera_profile_.version == "v2") {
      transform.transform.translation.z += nominal_world_z_offset_m_;
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
    const std::vector<int16_t> & node_class_id,
    const std::vector<int> & node_detection_index,
    const std::vector<YoloDetection> & detections)
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
      const int detection_index = i < node_detection_index.size() ? node_detection_index[i] : -1;
      if (detection_index >= 0 && detection_index < static_cast<int>(detections.size())) {
        node_marker.colors.push_back(objectClusterColor(
          detections[static_cast<size_t>(detection_index)].class_id, detection_index));
      } else {
        node_marker.colors.push_back(semanticNodeColor(node_class_id[i]));
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
    edge_marker.colors.reserve(edges.size() * 2);
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
      const int a_detection = a < node_detection_index.size() ? node_detection_index[a] : -1;
      const int b_detection = b < node_detection_index.size() ? node_detection_index[b] : -1;
      edge_marker.colors.push_back(
        a_detection >= 0 && a_detection < static_cast<int>(detections.size()) ?
        objectClusterColor(detections[static_cast<size_t>(a_detection)].class_id, a_detection, 0.8F) :
        semanticNodeColor(node_class_id[a], 0.8F));
      edge_marker.colors.push_back(
        b_detection >= 0 && b_detection < static_cast<int>(detections.size()) ?
        objectClusterColor(detections[static_cast<size_t>(b_detection)].class_id, b_detection, 0.8F) :
        semanticNodeColor(node_class_id[b], 0.8F));
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

  void publishObjectClusters(
    const std::vector<YoloDetection> & detections, const std::vector<GngPoint3f> & nodes,
    const std::vector<uint32_t> & node_ids, const std::vector<int> & node_detection_index)
  {
    std::vector<ObjectCluster> clusters(detections.size());
    std::vector<size_t> counts(detections.size(), 0);
    for (size_t index = 0; index < detections.size(); ++index) {
      const auto & detection = detections[index];
      auto & cluster = clusters[index];
      cluster.detection_index = static_cast<int>(index);
      cluster.class_id = detection.class_id;
      cluster.class_name = detection.className();
      cluster.confidence = detection.score;
      cluster.box_x = detection.x;
      cluster.box_y = detection.y;
      cluster.box_w = detection.w;
      cluster.box_h = detection.h;
    }
    for (size_t index = 0; index < nodes.size() && index < node_detection_index.size(); ++index) {
      const int detection_index = node_detection_index[index];
      if (detection_index < 0 || detection_index >= static_cast<int>(clusters.size())) {continue;}
      auto & cluster = clusters[static_cast<size_t>(detection_index)];
      cluster.x += nodes[index].x;
      cluster.y += nodes[index].y;
      cluster.z += nodes[index].z;
      cluster.node_ids.push_back(node_ids[index]);
      cluster.member_points.push_back(nodes[index]);
      ++counts[static_cast<size_t>(detection_index)];
    }
    std::vector<ObjectCluster> accepted;
    accepted.reserve(clusters.size());
    for (size_t index = 0; index < clusters.size(); ++index) {
      if (counts[index] < static_cast<size_t>(min_label_nodes_)) {continue;}
      auto & cluster = clusters[index];
      cluster.x /= static_cast<float>(counts[index]);
      cluster.y /= static_cast<float>(counts[index]);
      cluster.z /= static_cast<float>(counts[index]);
      accepted.push_back(std::move(cluster));
    }
    assignObjectTracks(accepted);
    std_msgs::msg::String message;
    message.data = clustersToJson(accepted);
    object_clusters_pub_->publish(std::move(message));
    publishObjectClusterMarkers(accepted);
  }

  void assignObjectTracks(std::vector<ObjectCluster> & clusters)
  {
    ++object_track_frame_;
    const uint64_t oldest = object_track_frame_ > static_cast<uint64_t>(object_track_hold_frames_) ?
      object_track_frame_ - static_cast<uint64_t>(object_track_hold_frames_) : 0U;
    object_tracks_.erase(
      std::remove_if(
        object_tracks_.begin(), object_tracks_.end(),
        [oldest](const ObjectTrack & track) {return track.last_seen_frame < oldest;}),
      object_tracks_.end());

    using Candidate = std::tuple<float, size_t, size_t>;
    std::vector<Candidate> candidates;
    for (size_t cluster_index = 0; cluster_index < clusters.size(); ++cluster_index) {
      const auto & cluster = clusters[cluster_index];
      for (size_t track_index = 0; track_index < object_tracks_.size(); ++track_index) {
        const auto & track = object_tracks_[track_index];
        if (track.class_id != cluster.class_id) {continue;}
        const float dx = cluster.x - track.x;
        const float dy = cluster.y - track.y;
        const float dz = cluster.z - track.z;
        const float distance = std::sqrt(dx * dx + dy * dy + dz * dz);
        if (distance <= static_cast<float>(object_track_max_distance_m_)) {
          candidates.emplace_back(distance, cluster_index, track_index);
        }
      }
    }
    std::sort(candidates.begin(), candidates.end());
    std::vector<bool> cluster_used(clusters.size(), false);
    std::vector<bool> track_used(object_tracks_.size(), false);
    const float alpha = static_cast<float>(object_track_smoothing_alpha_);
    for (const auto & candidate : candidates) {
      const size_t cluster_index = std::get<1>(candidate);
      const size_t track_index = std::get<2>(candidate);
      if (cluster_used[cluster_index] || track_used[track_index]) {continue;}
      auto & cluster = clusters[cluster_index];
      auto & track = object_tracks_[track_index];
      track.x += alpha * (cluster.x - track.x);
      track.y += alpha * (cluster.y - track.y);
      track.z += alpha * (cluster.z - track.z);
      track.last_seen_frame = object_track_frame_;
      cluster.track_id = track.id;
      cluster.tracked_x = track.x;
      cluster.tracked_y = track.y;
      cluster.tracked_z = track.z;
      cluster_used[cluster_index] = true;
      track_used[track_index] = true;
    }
    for (size_t cluster_index = 0; cluster_index < clusters.size(); ++cluster_index) {
      if (cluster_used[cluster_index]) {continue;}
      auto & cluster = clusters[cluster_index];
      ObjectTrack track;
      track.id = next_object_track_id_++;
      track.class_id = cluster.class_id;
      track.x = cluster.x;
      track.y = cluster.y;
      track.z = cluster.z;
      track.last_seen_frame = object_track_frame_;
      cluster.track_id = track.id;
      cluster.tracked_x = track.x;
      cluster.tracked_y = track.y;
      cluster.tracked_z = track.z;
      object_tracks_.push_back(track);
    }
  }

  void publishObjectClusterMarkers(const std::vector<ObjectCluster> & clusters)
  {
    visualization_msgs::msg::MarkerArray array;
    visualization_msgs::msg::Marker clear;
    clear.action = visualization_msgs::msg::Marker::DELETEALL;
    array.markers.push_back(clear);
    const auto stamp = last_frame_stamp_;
    for (size_t index = 0; index < clusters.size(); ++index) {
      const auto & cluster = clusters[index];
      const auto rgb = objectClusterColor(
        cluster.class_id, static_cast<int>(cluster.track_id % 1000000U));
      const int marker_id = static_cast<int>(index) * 3;
      visualization_msgs::msg::Marker centroid;
      centroid.header.frame_id = world_frame_;
      centroid.header.stamp = stamp;
      centroid.ns = "yolo_object_centroids";
      centroid.id = marker_id;
      centroid.type = visualization_msgs::msg::Marker::SPHERE;
      centroid.action = visualization_msgs::msg::Marker::ADD;
      centroid.pose.position.x = cluster.x;
      centroid.pose.position.y = cluster.y;
      centroid.pose.position.z = cluster.z;
      centroid.pose.orientation.w = 1.0;
      centroid.scale.x = 0.06;
      centroid.scale.y = 0.06;
      centroid.scale.z = 0.06;
      centroid.color = rgb;
      centroid.color.a = 0.95F;
      array.markers.push_back(centroid);

      visualization_msgs::msg::Marker links;
      links.header = centroid.header;
      links.ns = "yolo_object_members";
      links.id = marker_id + 1;
      links.type = visualization_msgs::msg::Marker::LINE_LIST;
      links.action = visualization_msgs::msg::Marker::ADD;
      links.pose.orientation.w = 1.0;
      links.scale.x = 0.004;
      links.color = rgb;
      links.color.a = 0.65F;
      geometry_msgs::msg::Point center;
      center.x = cluster.x;
      center.y = cluster.y;
      center.z = cluster.z;
      for (const auto & member : cluster.member_points) {
        geometry_msgs::msg::Point point;
        point.x = member.x;
        point.y = member.y;
        point.z = member.z;
        links.points.push_back(center);
        links.points.push_back(point);
      }
      array.markers.push_back(links);

      visualization_msgs::msg::Marker text;
      text.header = centroid.header;
      text.ns = "yolo_object_labels";
      text.id = marker_id + 2;
      text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
      text.action = visualization_msgs::msg::Marker::ADD;
      text.pose.position = centroid.pose.position;
      text.pose.position.z += 0.07;
      text.pose.orientation.w = 1.0;
      text.scale.z = 0.045;
      text.color = rgb;
      text.color.a = 1.0F;
      std::ostringstream label;
      label << cluster.class_name << " track #" << cluster.track_id
            << " (" << cluster.node_ids.size() << " nodes)";
      text.text = label.str();
      array.markers.push_back(text);
    }
    object_clusters_marker_pub_->publish(std::move(array));
  }

  // Typed planning interface. MarkerArray remains the RViz representation,
  // while this message preserves stable DD-GNG node IDs, semantic state and
  // edge identity for the independent reachability planner.
  void publishEnvironmentGraphData(
    const std::vector<GngPoint3f> & nodes,
    const std::vector<uint32_t> & node_ids,
    const std::vector<std::pair<uint16_t, uint16_t>> & edges,
    const std::vector<int16_t> & node_class_id,
    const std::vector<float> & node_confidence,
    const std::vector<std::string> & selected_target_classes)
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
      // Reachability interprets every non-negative class id as a goal. Keep
      // its existing algorithm unchanged by masking only the planning view;
      // RViz, debug imagery and ~/labels already received the full classes.
      const bool planning_target = classIsSelected(
        node_class_id[index], selected_target_classes);
      output.class_id = planning_target ? node_class_id[index] : -1;
      output.confidence = planning_target ? node_confidence[index] : 0.0F;
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
  std::string color_frame_;
  om6dof_dd_gng::CameraProfile camera_profile_;
  std::string camera_serial_;
  std::string camera_calibration_file_;
  std::string camera_calibration_sha256_;
  std::string camera_calibration_schema_;
  std::string camera_calibration_model_;
  std::string camera_calibration_serial_;
  std::string camera_calibration_parent_frame_;
  std::vector<double> camera_calibration_color_xyz_;
  std::vector<double> camera_calibration_color_quaternion_xyzw_;
  std::string camera_calibration_method_;
  int camera_calibration_samples_ = 0;
  double camera_calibration_translation_rmse_m_ = -1.0;
  double camera_calibration_rotation_rmse_rad_ = -1.0;
  double nominal_world_z_offset_m_ = 0.0;
  bool calibration_verified_ = false;
  std::string calibration_reason_ = "artifact_not_configured";
  std::string transform_source_ = "urdf_nominal";
  tf2::Transform calibration_parent_to_color_;
  tf2::Transform color_from_depth_;
  double max_frame_age_sec_ = 0.5;
  rclcpp::Time last_frame_stamp_{0, 0, RCL_ROS_TIME};
  std::string environment_graph_topic_;
  std::string environment_graph_data_topic_;
  double tf_timeout_sec_ = 0.1;
  double node_marker_scale_ = 0.02;
  double edge_marker_width_ = 0.004;

  std::string yolo_model_path_;
  std::string yolo_backend_ = "opencv";
  std::string yolo_engine_path_;
  double yolo_confidence_ = 0.35;
  double yolo_nms_threshold_ = 0.5;
  double yolo_period_sec_ = 0.5;
  std::vector<std::string> target_classes_;
  std::string target_classes_topic_;
  double label_confidence_ = 0.35;
  int min_label_nodes_ = 3;
  om6dof_dd_gng::SemanticClusterPropagationParameters semantic_cluster_propagation_;
  double semantic_max_source_age_sec_ = 1.0;
  double semantic_max_camera_translation_m_ = 0.02;
  double semantic_max_camera_rotation_rad_ = 0.10;
  std::string labels_topic_;
  std::string object_clusters_topic_;
  std::string object_clusters_marker_topic_;
  double object_track_max_distance_m_ = 0.08;
  double object_track_smoothing_alpha_ = 0.35;
  int object_track_hold_frames_ = 30;
  uint64_t object_track_frame_ = 0U;
  uint64_t next_object_track_id_ = 1U;
  std::vector<ObjectTrack> object_tracks_;
  std::string perception_metrics_topic_;
  std::string debug_image_topic_;
  double debug_image_period_sec_ = 0.5;
  int debug_jpeg_quality_ = 80;
  std::string rgb_image_topic_;
  double rgb_image_period_sec_ = 1.0 / 30.0;
  int rgb_jpeg_quality_ = 80;

  std::string robot_graph_topic_;
  double body_segment_spacing_ = 0.05;
  double body_mask_margin_ = 0.01;
  std::unordered_map<std::string, double> body_radius_;
  std::vector<BodySegment> body_segments_;

  // RealSense
  rs2::pipeline pipe_;
  rs2::pipeline_profile profile_;
  rs2_intrinsics depth_intrinsics_{};
  rs2_intrinsics color_intrinsics_{};

  // GNG core
  std::unique_ptr<om6dof_dd_gng::DynamicDensityGrowingNeuralGas> gng_;
  std::unique_ptr<om6dof_dd_gng::SemanticDensityAttention> density_attention_;

  // YOLO
  std::unique_ptr<AsyncYolo> async_yolo_;
  mutable std::mutex target_classes_mutex_;
  std::deque<YoloSourceFrame> yolo_source_frames_;
  std::unordered_map<uint32_t, TemporalNodeLabel> temporal_labels_;

  // tf2
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;

  // ROS I/O
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr env_graph_pub_;
  rclcpp::Publisher<om6dof_dd_gng::msg::EnvironmentGraph>::SharedPtr env_graph_data_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr labels_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr object_clusters_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr object_clusters_marker_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr robot_graph_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr perception_metrics_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr debug_image_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr rgb_image_pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr target_classes_sub_;

  // Capture and mapping run independently so cumulative counters and the
  // newest-frame handoff need small, explicit critical sections.
  uint64_t metrics_sequence_ = 0;
  uint64_t previous_depth_frame_number_ = 0;
  std::mutex drop_counters_mutex_;
  DropCounters drop_counters_;
  SteadyClock::time_point last_debug_image_publish_{};
  SteadyClock::time_point last_rgb_image_publish_{};

  std::mutex frame_queue_mutex_;
  std::condition_variable frame_queue_cv_;
  std::deque<PendingFrame> pending_frames_;

  // Capture publishes raw RGB at camera rate. Mapping consumes only the
  // newest synchronized frameset, independently, so DD-GNG cannot stall the
  // Web Monitor camera.
  std::thread capture_thread_;
  std::thread processing_thread_;
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
