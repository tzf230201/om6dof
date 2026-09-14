// yolo_3d_box_node: YOLO + 3D bounding-box + DD-GNG, camera to output, no
// robot FK, no world-frame projection. Deliberately the smallest slice of the
// om6dof_dd_gng pipeline that answers one question: "what does the camera
// see, roughly where is it in 3D, and what does the local point-cloud
// structure around it look like?" -- meant to run at the camera's own
// 30 fps, not the ~0.5-2 Hz an async detector settles for once other work
// (TF, reachability, world-frame re-projection) shares the loop.
//
// Pipeline, once per captured frame:
//   RealSense D435(i) color + depth, depth aligned into the COLOR pixel grid
//     (rs2::align) so a YOLO box's (x, y, w, h) indexes the same pixels in
//     both images with no separate registration step.
//   -> TensorRT YOLOX-S (om6dof_dd_gng::TensorRtYoloXDetector, the same
//      verified yolox_s_fp16.engine topo_gng_node/yolox_viewer_node use)
//      run SYNCHRONOUSLY, once per frame -- not AsyncYolo's decoupled
//      once-per-period pattern. This node's whole job is the detection, so
//      there is nothing else in the loop for a slow detector to block; if the
//      engine cannot keep up with the camera's frame rate that shows up
//      directly in the measured FPS this node logs, rather than being hidden
//      behind a "latest available result" snapshot.
//   -> per-detection 3D box: robust front-surface depth segmentation inside
//      the (inset) 2D box, deprojected with the color stream's own live
//      intrinsics (rs2_deproject_pixel_to_point) -- the same quantile-based
//      foreground/outlier rejection realsense_ddgng/dd_gng_yolo.py's
//      segment_3d_box uses, ported here as pixel-indexed depth rather than a
//      precomputed SDK point-cloud grid.
//   -> om6dof_dd_gng::DynamicDensityGrowingNeuralGas fed with a pixel-strided
//      sample of the WHOLE aligned depth frame (not just inside boxes), with
//      one attention region set per detected 3D box so the graph naturally
//      grows denser there -- that density is the "cluster" a detection
//      labels. Because detection and depth are from the same instant (no
//      async gap, no camera motion to compensate for), labelling a node is
//      just "is this node's current position inside a padded box right now",
//      with no cross-frame re-projection needed (unlike topo_gng_node, which
//      solves that harder problem for a moving wrist camera + async YOLO).
//      A node keeps its label for label_ttl_sec after last matching a box,
//      via a stable-node-id map (DD-GNG never reuses an id within an epoch).
//   -> sensor_msgs/CompressedImage: annotated 2D + projected 3D wireframe
//      overlay.
//   -> std_msgs/String: one JSON object per detection (label, confidence,
//      2D bbox, 3D min/max/center/size in the colour-optical frame, metres).
//   -> visualization_msgs/MarkerArray + om6dof_dd_gng/EnvironmentGraph: the
//      DD-GNG node/edge graph, coloured and labelled by class_id.
//
// No TF, no MoveIt, no world-frame persistence -- purely
// camera -> YOLO -> 3D box -> DD-GNG -> ROS output, all in one camera-frame
// instant per loop iteration.

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <iterator>
#include <limits>
#include <memory>
#include <sstream>
#include <string>
#include <thread>
#include <unordered_map>
#include <utility>
#include <vector>

#include <librealsense2/rs.hpp>
#include <librealsense2/rsutil.h>

#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <geometry_msgs/msg/point.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <std_msgs/msg/string.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "om6dof_dd_gng/camera_profile.hpp"
#include "om6dof_dd_gng/dynamic_density_gng.hpp"
#include "om6dof_dd_gng/msg/environment_graph.hpp"
#include "om6dof_dd_gng/tensorrt_yolox_detector.hpp"
#include "om6dof_dd_gng/yolox_detector.hpp"

namespace
{

using SteadyClock = std::chrono::steady_clock;

std::string expandHome(const std::string & path)
{
  if (path.rfind("~/", 0) != 0) {
    return path;
  }
  const char * home = std::getenv("HOME");
  return home ? std::string(home) + path.substr(1) : path;
}

double elapsedMs(SteadyClock::time_point started)
{
  return std::chrono::duration<double, std::milli>(SteadyClock::now() - started).count();
}

// Stable bright BGR colour per YOLO class id (matches
// realsense_ddgng/dd_gng_yolo.py's class_color: same hue step, same HSV->BGR
// conversion, so a class reads the same colour in both tools).
cv::Scalar classColor(int class_id)
{
  const int hue = ((class_id * 47) % 180 + 180) % 180;
  cv::Mat pixel(1, 1, CV_8UC3, cv::Scalar(hue, 220, 255));
  cv::Mat bgr;
  cv::cvtColor(pixel, bgr, cv::COLOR_HSV2BGR);
  const cv::Vec3b value = bgr.at<cv::Vec3b>(0, 0);
  return cv::Scalar(value[0], value[1], value[2]);
}

// Ported from topo_gng_node.cpp's classColor/semanticNodeColor/
// objectClusterColor (RGBA float variants, for RViz markers -- distinct from
// classColor() above, which is BGR uint8 for the OpenCV overlay). Kept as a
// direct copy rather than a shared header so the two nodes' RViz palettes
// stay consistent without coupling their build targets.
std_msgs::msg::ColorRGBA classColorRgba(int class_id)
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
    auto color = classColorRgba(class_id);
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

// A class colour identifies semantics; an instance colour identifies one
// YOLO box within that class -- distinguishes two same-class objects' node
// clusters from each other in RViz.
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

// One robustly-segmented 3D box in the colour-optical frame, metres.
struct Box3D
{
  bool valid = false;
  float min[3] = {0, 0, 0};
  float max[3] = {0, 0, 0};
  float center[3] = {0, 0, 0};
  float size[3] = {0, 0, 0};
  int point_count = 0;
};

// Percentile of a mutable buffer via nth_element -- avoids a full sort when
// only one or two quantiles are needed, which matters once a detection
// covers a large fraction of the frame.
float percentile(std::vector<float> & values, double q)
{
  if (values.empty()) {
    return std::numeric_limits<float>::quiet_NaN();
  }
  const size_t index = std::min(
    values.size() - 1,
    static_cast<size_t>(std::llround(q * static_cast<double>(values.size() - 1))));
  std::nth_element(values.begin(), values.begin() + static_cast<long>(index), values.end());
  return values[index];
}

// Port of realsense_ddgng/dd_gng_yolo.py's segment_3d_box: YOLO supplies the
// class/2D region, depth supplies the actual foreground geometry. Quantile
// bounds reject isolated depth outliers while keeping a stable axis-aligned
// box. Operates directly on the aligned depth frame's raw uint16 buffer
// instead of a precomputed point-cloud grid.
Box3D segment3DBox(
  const uint16_t * depth_data, int depth_width, int depth_height, float depth_scale,
  const rs2_intrinsics & color_intrinsics, float x, float y, float w, float h,
  float z_min, float z_max, float depth_band_m, int min_points)
{
  Box3D box;
  const int inset_x = static_cast<int>(w * 0.06F);
  const int inset_y = static_cast<int>(h * 0.06F);
  const int x0 = std::max(0, static_cast<int>(x) + inset_x);
  const int x1 = std::min(depth_width, static_cast<int>(x + w) - inset_x);
  const int y0 = std::max(0, static_cast<int>(y) + inset_y);
  const int y1 = std::min(depth_height, static_cast<int>(y + h) - inset_y);
  if (x1 <= x0 || y1 <= y0) {
    return box;
  }

  std::vector<float> depths_m;
  depths_m.reserve(static_cast<size_t>((x1 - x0) * (y1 - y0)));
  for (int v = y0; v < y1; ++v) {
    const uint16_t * row = depth_data + static_cast<size_t>(v) * depth_width;
    for (int u = x0; u < x1; ++u) {
      const float z = static_cast<float>(row[u]) * depth_scale;
      if (std::isfinite(z) && z > z_min && z < z_max) {
        depths_m.push_back(z);
      }
    }
  }
  if (static_cast<int>(depths_m.size()) < min_points) {
    return box;
  }
  const float anchor = percentile(depths_m, 0.15);
  const float foreground_ceiling = anchor + depth_band_m;

  std::vector<float> xs, ys, zs;
  xs.reserve(depths_m.size());
  ys.reserve(depths_m.size());
  zs.reserve(depths_m.size());
  for (int v = y0; v < y1; ++v) {
    const uint16_t * row = depth_data + static_cast<size_t>(v) * depth_width;
    for (int u = x0; u < x1; ++u) {
      const float z = static_cast<float>(row[u]) * depth_scale;
      if (!std::isfinite(z) || z <= z_min || z >= z_max || z > foreground_ceiling) {
        continue;
      }
      const float pixel[2] = {static_cast<float>(u), static_cast<float>(v)};
      float point[3];
      rs2_deproject_pixel_to_point(point, &color_intrinsics, pixel, z);
      xs.push_back(point[0]);
      ys.push_back(point[1]);
      zs.push_back(point[2]);
    }
  }
  if (static_cast<int>(xs.size()) < min_points) {
    return box;
  }

  const float lower[3] = {percentile(xs, 0.02), percentile(ys, 0.02), percentile(zs, 0.02)};
  const float upper[3] = {percentile(xs, 0.98), percentile(ys, 0.98), percentile(zs, 0.98)};
  const float min_size[3] = {0.005F, 0.005F, 0.010F};
  for (int axis = 0; axis < 3; ++axis) {
    if (!std::isfinite(lower[axis]) || !std::isfinite(upper[axis])) {
      return box;
    }
    const float center = (lower[axis] + upper[axis]) * 0.5F;
    const float size = std::max(upper[axis] - lower[axis], min_size[axis]);
    box.center[axis] = center;
    box.size[axis] = size;
    box.min[axis] = center - size * 0.5F;
    box.max[axis] = center + size * 0.5F;
  }
  box.point_count = static_cast<int>(xs.size());
  box.valid = true;
  return box;
}

// Pixel-strided sample of the WHOLE aligned depth frame, for DD-GNG -- a
// general scene skeleton, separate from segment3DBox()'s box-only sample.
// Same stride/z-range convention as realsense_ddgng/ddgng_realsense.py's
// PIXEL_STEP/Z_MIN/Z_MAX so behaviour matches the Python tool at the same
// settings.
std::vector<GngPoint3f> sampleScenePoints(
  const uint16_t * depth_data, int depth_width, int depth_height, float depth_scale,
  const rs2_intrinsics & color_intrinsics, int pixel_step, float z_min, float z_max)
{
  std::vector<GngPoint3f> points;
  points.reserve(
    static_cast<size_t>(depth_width / pixel_step + 1) *
    static_cast<size_t>(depth_height / pixel_step + 1));
  for (int v = 0; v < depth_height; v += pixel_step) {
    const uint16_t * row = depth_data + static_cast<size_t>(v) * depth_width;
    for (int u = 0; u < depth_width; u += pixel_step) {
      const float z = static_cast<float>(row[u]) * depth_scale;
      if (!std::isfinite(z) || z <= z_min || z >= z_max) {
        continue;
      }
      const float pixel[2] = {static_cast<float>(u), static_cast<float>(v)};
      float point[3];
      rs2_deproject_pixel_to_point(point, &color_intrinsics, pixel, z);
      points.push_back({point[0], point[1], point[2]});
    }
  }
  return points;
}

// Port of realsense_ddgng/dd_gng_yolo.py's assign_node_labels_3d: a node is
// labelled by the highest-confidence detection whose padded 3D box contains
// it. Ties/overlaps resolve to whichever detection was ranked first.
bool pointInsidePaddedBox(const GngPoint3f & point, const Box3D & box, float padding_m)
{
  return point.x >= box.min[0] - padding_m && point.x <= box.max[0] + padding_m &&
    point.y >= box.min[1] - padding_m && point.y <= box.max[1] + padding_m &&
    point.z >= box.min[2] - padding_m && point.z <= box.max[2] + padding_m;
}

// Project the eight corners of a Box3D and draw its wireframe on `image`,
// using the SDK's own projection (handles the stream's real distortion
// model instead of assuming a bare pinhole).
void drawProjectedBox(
  cv::Mat & image, const Box3D & box, const rs2_intrinsics & color_intrinsics,
  const cv::Scalar & color)
{
  const float corners[8][3] = {
    {box.min[0], box.min[1], box.min[2]}, {box.max[0], box.min[1], box.min[2]},
    {box.max[0], box.max[1], box.min[2]}, {box.min[0], box.max[1], box.min[2]},
    {box.min[0], box.min[1], box.max[2]}, {box.max[0], box.min[1], box.max[2]},
    {box.max[0], box.max[1], box.max[2]}, {box.min[0], box.max[1], box.max[2]},
  };
  cv::Point uv[8];
  for (int i = 0; i < 8; ++i) {
    float pixel[2];
    rs2_project_point_to_pixel(pixel, &color_intrinsics, corners[i]);
    uv[i] = cv::Point(
      std::clamp(static_cast<int>(std::lround(pixel[0])), -10000, 10000),
      std::clamp(static_cast<int>(std::lround(pixel[1])), -10000, 10000));
  }
  static constexpr int kEdges[12][2] = {
    {0, 1}, {1, 2}, {2, 3}, {3, 0}, {4, 5}, {5, 6},
    {6, 7}, {7, 4}, {0, 4}, {1, 5}, {2, 6}, {3, 7},
  };
  for (const auto & edge : kEdges) {
    cv::line(image, uv[edge[0]], uv[edge[1]], color, 2, cv::LINE_AA);
  }
}

std::string jsonEscape(const std::string & value)
{
  std::string out;
  out.reserve(value.size());
  for (const char c : value) {
    if (c == '"' || c == '\\') {
      out.push_back('\\');
    }
    out.push_back(c);
  }
  return out;
}

}  // namespace

class Yolo3DBoxNode final : public rclcpp::Node
{
public:
  Yolo3DBoxNode()
  : Node("yolo_3d_box")
  {
    model_version_ = declare_parameter<std::string>("model_version", "v2");
    camera_model_ = declare_parameter<std::string>("camera_model", "auto");
    camera_serial_ = declare_parameter<std::string>("camera_serial", "");
    width_ = declare_parameter<int>("width", 640);
    height_ = declare_parameter<int>("height", 480);
    fps_ = declare_parameter<int>("fps", 30);
    engine_path_ = expandHome(declare_parameter<std::string>(
      "yolo_engine_path", "~/.cache/om6dof_perception/yolox_s_fp16.engine"));
    confidence_ = static_cast<float>(declare_parameter<double>("confidence", 0.35));
    nms_threshold_ = static_cast<float>(declare_parameter<double>("nms_threshold", 0.5));
    z_min_ = static_cast<float>(declare_parameter<double>("z_min", 0.2));
    z_max_ = static_cast<float>(declare_parameter<double>("z_max", 4.0));
    depth_band_m_ = static_cast<float>(declare_parameter<double>("depth_band_m", 0.12));
    min_points_ = declare_parameter<int>("min_points", 40);
    image_topic_ = declare_parameter<std::string>(
      "image_topic", "/om6dof_yolo3d/debug_image/compressed");
    objects_topic_ = declare_parameter<std::string>(
      "objects_topic", "/om6dof_yolo3d/objects");
    jpeg_quality_ = declare_parameter<int>("jpeg_quality", 70);
    publish_overlay_ = declare_parameter<bool>("publish_overlay", true);
    display_window_ = declare_parameter<bool>("display_window", false);

    enable_dd_gng_ = declare_parameter<bool>("enable_dd_gng", true);
    gng_max_nodes_ = declare_parameter<int>("gng_max_nodes", 500);
    gng_insertion_interval_ = declare_parameter<int>("gng_insertion_interval", 100);
    gng_max_edge_age_ = declare_parameter<int>("gng_max_edge_age", 50);
    gng_updates_per_frame_ = declare_parameter<int>("gng_updates_per_frame", 150);
    gng_pixel_step_ = declare_parameter<int>("gng_pixel_step", 6);
    attention_strength_ = static_cast<float>(declare_parameter<double>("attention_strength", 6.0));
    attention_radius_margin_ =
      static_cast<float>(declare_parameter<double>("attention_radius_margin", 1.3));
    label_padding_m_ = static_cast<float>(declare_parameter<double>("label_padding_m", 0.015));
    label_ttl_sec_ = declare_parameter<double>("label_ttl_sec", 1.5);
    publish_graph_markers_ = declare_parameter<bool>("publish_graph_markers", true);
    graph_markers_topic_ = declare_parameter<std::string>(
      "graph_markers_topic", "/om6dof_yolo3d/graph_markers");
    graph_data_topic_ = declare_parameter<std::string>(
      "graph_data_topic", "/om6dof_yolo3d/graph_data");

    if (width_ <= 0 || height_ <= 0 || fps_ <= 0) {
      throw std::runtime_error("width/height/fps must be positive");
    }
    if (jpeg_quality_ < 1 || jpeg_quality_ > 100) {
      throw std::runtime_error("jpeg_quality must be in [1, 100]");
    }
    if (gng_pixel_step_ <= 0) {
      throw std::runtime_error("gng_pixel_step must be positive");
    }
    if (gng_updates_per_frame_ < 0) {
      throw std::runtime_error("gng_updates_per_frame must be non-negative");
    }
    if (!std::isfinite(label_ttl_sec_) || label_ttl_sec_ < 0.0) {
      throw std::runtime_error("label_ttl_sec must be non-negative");
    }

    image_publisher_ = create_publisher<sensor_msgs::msg::CompressedImage>(image_topic_, 2);
    objects_publisher_ = create_publisher<std_msgs::msg::String>(objects_topic_, 2);
    if (enable_dd_gng_) {
      om6dof_dd_gng::DynamicDensityGngParameters gng_parameters;
      gng_parameters.max_nodes = gng_max_nodes_;
      gng_parameters.insertion_interval = gng_insertion_interval_;
      gng_parameters.max_edge_age = gng_max_edge_age_;
      gng_ = std::make_unique<om6dof_dd_gng::DynamicDensityGrowingNeuralGas>(gng_parameters);
      if (publish_graph_markers_) {
        graph_markers_publisher_ =
          create_publisher<visualization_msgs::msg::MarkerArray>(graph_markers_topic_, 2);
      }
      graph_data_publisher_ =
        create_publisher<om6dof_dd_gng::msg::EnvironmentGraph>(graph_data_topic_, 2);
    }

    detector_ = std::make_shared<om6dof_dd_gng::TensorRtYoloXDetector>(
      engine_path_, confidence_, nms_threshold_);
    RCLCPP_INFO(
      get_logger(), "YOLOX-S TensorRT engine loaded: %s", engine_path_.c_str());

    startCamera();

    running_ = true;
    worker_ = std::thread(&Yolo3DBoxNode::captureLoop, this);
  }

  ~Yolo3DBoxNode() override
  {
    running_ = false;
    if (worker_.joinable()) {
      worker_.join();
    }
    pipeline_.stop();
    if (display_window_ready_) {
      cv::destroyWindow(window_name_);
    }
  }

private:
  void startCamera()
  {
    const auto profile = om6dof_dd_gng::cameraProfile(model_version_, camera_model_);
    rs2::context context;
    std::vector<om6dof_dd_gng::CameraIdentity> devices;
    for (const auto & device : context.query_devices()) {
      if (device.supports(RS2_CAMERA_INFO_NAME) &&
        device.supports(RS2_CAMERA_INFO_SERIAL_NUMBER))
      {
        devices.push_back(
          {device.get_info(RS2_CAMERA_INFO_NAME), device.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER)});
      }
    }
    camera_serial_ = om6dof_dd_gng::selectCameraSerial(devices, profile, camera_serial_);

    rs2::config config;
    config.enable_device(camera_serial_);
    config.enable_stream(RS2_STREAM_DEPTH, width_, height_, RS2_FORMAT_Z16, fps_);
    config.enable_stream(RS2_STREAM_COLOR, width_, height_, RS2_FORMAT_BGR8, fps_);
    rs2::pipeline_profile pipeline_profile;
    try {
      pipeline_profile = pipeline_.start(config);
    } catch (const rs2::error & e) {
      RCLCPP_FATAL(
        get_logger(),
        "Failed to start the RealSense %s (%s). If another process owns it, stop it first: "
        "systemctl --user stop om6dof-dd-gng.service om6dof-perception.service",
        profile.model.c_str(), e.what());
      throw;
    }
    depth_scale_ = pipeline_profile.get_device().first<rs2::depth_sensor>().get_depth_scale();
    const auto color_stream =
      pipeline_profile.get_stream(RS2_STREAM_COLOR).as<rs2::video_stream_profile>();
    color_intrinsics_ = color_stream.get_intrinsics();
    RCLCPP_INFO(
      get_logger(),
      "%s (%s) started: %dx%d @ %d fps, fx=%.2f fy=%.2f ppx=%.2f ppy=%.2f, depth_scale=%.6f",
      profile.model.c_str(), camera_serial_.c_str(), width_, height_, fps_,
      color_intrinsics_.fx, color_intrinsics_.fy, color_intrinsics_.ppx, color_intrinsics_.ppy,
      depth_scale_);
  }

  void captureLoop()
  {
    rs2::align align_to_color(RS2_STREAM_COLOR);
    uint64_t frame_count = 0;
    SteadyClock::time_point fps_window_started = SteadyClock::now();
    double inference_ms_sum = 0.0;
    double loop_ms_sum = 0.0;

    while (running_ && rclcpp::ok()) {
      const auto loop_started = SteadyClock::now();
      rs2::frameset frames;
      if (!pipeline_.try_wait_for_frames(&frames, 1000)) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "RealSense frame wait timed out");
        continue;
      }
      const rs2::frameset aligned = align_to_color.process(frames);
      const rs2::video_frame color = aligned.get_color_frame();
      const rs2::depth_frame depth = aligned.get_depth_frame();
      if (!color || !depth) {
        continue;
      }
      const rclcpp::Time stamp = now();

      cv::Mat bgr(
        cv::Size(color.get_width(), color.get_height()), CV_8UC3,
        const_cast<void *>(color.get_data()), cv::Mat::AUTO_STEP);
      const auto * depth_data = static_cast<const uint16_t *>(depth.get_data());
      const int depth_width = depth.get_width();
      const int depth_height = depth.get_height();

      const auto inference_started = SteadyClock::now();
      std::vector<om6dof_dd_gng::YoloDetection> detections;
      try {
        detections = detector_->detect(bgr);
      } catch (const std::exception & e) {
        RCLCPP_ERROR_THROTTLE(
          get_logger(), *get_clock(), 2000, "YOLOX inference failed: %s", e.what());
        continue;
      }
      const double inference_ms = elapsedMs(inference_started);

      cv::Mat overlay;
      if (publish_overlay_ || display_window_) {
        overlay = bgr.clone();
      }

      std::ostringstream json;
      json << "{\"stamp\":" << stamp.seconds()
           << ",\"camera_model\":\"" << jsonEscape(camera_model_resolved())
           << "\",\"camera_serial\":\"" << jsonEscape(camera_serial_)
           << "\",\"frame\":\"colour_optical\",\"coordinate_space\":\"camera_optical\""
           << ",\"units\":\"metres\",\"inference_ms\":" << inference_ms
           << ",\"objects\":[";
      std::vector<Box3D> boxes;
      boxes.reserve(detections.size());
      bool first_object = true;
      for (const auto & detection : detections) {
        const Box3D box = segment3DBox(
          depth_data, depth_width, depth_height, static_cast<float>(depth_scale_),
          color_intrinsics_, detection.x, detection.y, detection.w, detection.h,
          z_min_, z_max_, depth_band_m_, min_points_);
        boxes.push_back(box);
        const cv::Scalar color_bgr = classColor(detection.class_id);
        if (publish_overlay_ || display_window_) {
          cv::rectangle(
            overlay, cv::Rect(
              static_cast<int>(detection.x), static_cast<int>(detection.y),
              static_cast<int>(detection.w), static_cast<int>(detection.h)),
            color_bgr, 2, cv::LINE_AA);
          std::ostringstream label;
          label << detection.className() << " " << std::round(detection.score * 100.0F) << "%";
          if (box.valid) {
            label << " " << std::round(box.center[2] * 1000.0F) << "mm";
          }
          const int text_y = std::max(14, static_cast<int>(detection.y) - 6);
          cv::putText(
            overlay, label.str(), cv::Point(static_cast<int>(detection.x), text_y),
            cv::FONT_HERSHEY_SIMPLEX, 0.5, color_bgr, 2, cv::LINE_AA);
          if (box.valid) {
            drawProjectedBox(overlay, box, color_intrinsics_, color_bgr);
          }
        }
        if (!first_object) {
          json << ",";
        }
        first_object = false;
        json << "{\"label\":\"" << jsonEscape(detection.className())
             << "\",\"class_id\":" << detection.class_id
             << ",\"confidence\":" << detection.score
             << ",\"bbox\":[" << detection.x << "," << detection.y << ","
             << detection.w << "," << detection.h << "]";
        if (box.valid) {
          json << ",\"min\":[" << box.min[0] << "," << box.min[1] << "," << box.min[2] << "]"
               << ",\"max\":[" << box.max[0] << "," << box.max[1] << "," << box.max[2] << "]"
               << ",\"center\":[" << box.center[0] << "," << box.center[1] << ","
               << box.center[2] << "]"
               << ",\"size\":[" << box.size[0] << "," << box.size[1] << "," << box.size[2] << "]"
               << ",\"point_count\":" << box.point_count;
        } else {
          json << ",\"box3d\":null";
        }
        json << "}";
      }
      json << "]}";

      std_msgs::msg::String objects_msg;
      objects_msg.data = json.str();
      objects_publisher_->publish(objects_msg);

      if (enable_dd_gng_) {
        updateAndPublishGraph(
          depth_data, depth_width, depth_height, detections, boxes, stamp, overlay);
      }

      if (publish_overlay_) {
        std::vector<uchar> jpeg;
        cv::imencode(".jpg", overlay, jpeg, {cv::IMWRITE_JPEG_QUALITY, jpeg_quality_});
        sensor_msgs::msg::CompressedImage image_msg;
        image_msg.header.stamp = stamp;
        image_msg.header.frame_id = "colour_optical";
        image_msg.format = "jpeg";
        image_msg.data.assign(jpeg.begin(), jpeg.end());
        image_publisher_->publish(image_msg);
      }
      showWindow(overlay);

      const double loop_ms = elapsedMs(loop_started);
      inference_ms_sum += inference_ms;
      loop_ms_sum += loop_ms;
      ++frame_count;
      const double window_s = std::chrono::duration<double>(
        SteadyClock::now() - fps_window_started).count();
      if (window_s >= 2.0) {
        RCLCPP_INFO(
          get_logger(),
          "%.1f fps over %llu frames | avg inference %.1f ms | avg loop %.1f ms | %zu detections",
          static_cast<double>(frame_count) / window_s,
          static_cast<unsigned long long>(frame_count),
          inference_ms_sum / static_cast<double>(frame_count),
          loop_ms_sum / static_cast<double>(frame_count), detections.size());
        frame_count = 0;
        inference_ms_sum = 0.0;
        loop_ms_sum = 0.0;
        fps_window_started = SteadyClock::now();
      }
    }
  }

  // Feeds this frame's scene into DD-GNG, pulls attention toward each
  // detected 3D box, labels the resulting nodes, and publishes the graph.
  // Everything here is in the SAME camera-frame instant as `detections`/
  // `boxes` -- no cross-frame re-projection, no TF, no motion compensation.
  void updateAndPublishGraph(
    const uint16_t * depth_data, int depth_width, int depth_height,
    const std::vector<om6dof_dd_gng::YoloDetection> & detections,
    const std::vector<Box3D> & boxes, const rclcpp::Time & stamp, cv::Mat & overlay)
  {
    std::vector<om6dof_dd_gng::DensityAttentionRegion> regions;
    regions.reserve(boxes.size());
    for (const auto & box : boxes) {
      if (!box.valid) {
        continue;
      }
      const float half_extent = std::max({box.size[0], box.size[1], box.size[2]}) * 0.5F;
      om6dof_dd_gng::DensityAttentionRegion region;
      region.center = {box.center[0], box.center[1], box.center[2]};
      region.radius = std::clamp(half_extent * attention_radius_margin_, 0.02F, 0.75F);
      region.strength = std::clamp(attention_strength_, 1.0F, 20.0F);
      regions.push_back(region);
    }
    gng_->setAttentionRegions(regions);

    const auto points = sampleScenePoints(
      depth_data, depth_width, depth_height, static_cast<float>(depth_scale_),
      color_intrinsics_, gng_pixel_step_, z_min_, z_max_);
    gng_->partialFit(points, gng_updates_per_frame_);

    std::vector<GngPoint3f> nodes;
    std::vector<uint32_t> node_ids;
    std::vector<std::pair<uint16_t, uint16_t>> edges;
    gng_->copyGraph(nodes, node_ids, edges);

    // Rank detections by confidence so, when two boxes overlap, the more
    // confident one claims a shared node first -- mirrors
    // dd_gng_yolo.py's assign_node_labels_3d.
    std::vector<size_t> ranked(boxes.size());
    for (size_t i = 0; i < ranked.size(); ++i) {
      ranked[i] = i;
    }
    std::sort(ranked.begin(), ranked.end(), [&detections](size_t a, size_t b) {
      return detections[a].score > detections[b].score;
    });

    const auto now_steady = SteadyClock::now();
    std::vector<bool> matched(nodes.size(), false);
    std::vector<int> node_detection_index(nodes.size(), -1);
    for (const size_t detection_index : ranked) {
      const Box3D & box = boxes[detection_index];
      if (!box.valid) {
        continue;
      }
      for (size_t node_index = 0; node_index < nodes.size(); ++node_index) {
        if (matched[node_index]) {
          continue;
        }
        if (!pointInsidePaddedBox(nodes[node_index], box, label_padding_m_)) {
          continue;
        }
        matched[node_index] = true;
        node_detection_index[node_index] = static_cast<int>(detection_index);
        NodeLabel & label = node_labels_[node_ids[node_index]];
        label.class_id = detections[detection_index].class_id;
        label.confidence = detections[detection_index].score;
        label.last_seen = now_steady;
      }
    }

    // Nodes DD-GNG has since removed/recycled can never come back (ids are
    // never reused within an epoch), so their labels are permanently stale.
    if (!node_labels_.empty()) {
      std::unordered_map<uint32_t, char> alive;
      alive.reserve(node_ids.size());
      for (const uint32_t id : node_ids) {
        alive.emplace(id, 0);
      }
      for (auto it = node_labels_.begin(); it != node_labels_.end();) {
        it = alive.count(it->first) == 0 ? node_labels_.erase(it) : std::next(it);
      }
    }

    std::vector<int16_t> node_class_id(nodes.size(), -1);
    std::vector<float> node_confidence(nodes.size(), 0.0F);
    for (size_t i = 0; i < nodes.size(); ++i) {
      const auto found = node_labels_.find(node_ids[i]);
      if (found == node_labels_.end()) {
        continue;
      }
      const double age_sec = std::chrono::duration<double>(now_steady - found->second.last_seen)
        .count();
      if (age_sec > label_ttl_sec_) {
        node_labels_.erase(found);
        continue;
      }
      node_class_id[i] = static_cast<int16_t>(found->second.class_id);
      node_confidence[i] = found->second.confidence;
    }

    if (!overlay.empty()) {
      drawGraphOverlay(overlay, nodes, edges, node_class_id, node_detection_index, detections);
    }

    if (publish_graph_markers_ && graph_markers_publisher_) {
      publishGraphMarkers(
        nodes, edges, node_class_id, node_detection_index, detections, stamp);
    }
    publishGraphData(nodes, node_ids, edges, node_class_id, node_confidence, stamp);
  }

  // Draws the DD-GNG node/edge graph onto the same overlay the 2D/3D YOLO
  // boxes are drawn on, so `display_window`/the published debug image show
  // the cluster forming around a detection, not just its box. Node colour
  // matches classColor() -- the same BGR palette already used for that
  // detection's rectangle -- so a coloured dot cluster and its box read as
  // the same object at a glance.
  void drawGraphOverlay(
    cv::Mat & overlay, const std::vector<GngPoint3f> & nodes,
    const std::vector<std::pair<uint16_t, uint16_t>> & edges,
    const std::vector<int16_t> & node_class_id, const std::vector<int> & node_detection_index,
    const std::vector<om6dof_dd_gng::YoloDetection> & detections)
  {
    std::vector<cv::Point> uv(nodes.size());
    std::vector<bool> on_screen(nodes.size(), false);
    for (size_t i = 0; i < nodes.size(); ++i) {
      if (nodes[i].z <= 0.0F) {
        continue;
      }
      const float point[3] = {nodes[i].x, nodes[i].y, nodes[i].z};
      float pixel[2];
      rs2_project_point_to_pixel(pixel, &color_intrinsics_, point);
      const int x = static_cast<int>(std::lround(pixel[0]));
      const int y = static_cast<int>(std::lround(pixel[1]));
      if (x < 0 || y < 0 || x >= overlay.cols || y >= overlay.rows) {
        continue;
      }
      uv[i] = cv::Point(x, y);
      on_screen[i] = true;
    }
    static const cv::Scalar kUnlabelled(160, 160, 160);
    for (const auto & [a, b] : edges) {
      if (!on_screen[a] || !on_screen[b]) {
        continue;
      }
      const cv::Scalar color = node_class_id[a] >= 0 ? classColor(node_class_id[a]) : kUnlabelled;
      cv::line(overlay, uv[a], uv[b], color, 1, cv::LINE_AA);
    }
    for (size_t i = 0; i < nodes.size(); ++i) {
      if (!on_screen[i]) {
        continue;
      }
      const int detection_index = node_detection_index[i];
      const cv::Scalar color =
        detection_index >= 0 ? classColor(detections[static_cast<size_t>(detection_index)].class_id) :
        node_class_id[i] >= 0 ? classColor(node_class_id[i]) : kUnlabelled;
      cv::circle(overlay, uv[i], 3, color, cv::FILLED, cv::LINE_AA);
    }
  }

  void publishGraphMarkers(
    const std::vector<GngPoint3f> & nodes,
    const std::vector<std::pair<uint16_t, uint16_t>> & edges,
    const std::vector<int16_t> & node_class_id,
    const std::vector<int> & node_detection_index,
    const std::vector<om6dof_dd_gng::YoloDetection> & detections, const rclcpp::Time & stamp)
  {
    visualization_msgs::msg::Marker node_marker;
    node_marker.header.frame_id = "colour_optical";
    node_marker.header.stamp = stamp;
    node_marker.ns = "yolo_3d_box_nodes";
    node_marker.id = 0;
    node_marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
    node_marker.action = visualization_msgs::msg::Marker::ADD;
    node_marker.pose.orientation.w = 1.0;
    node_marker.scale.x = node_marker.scale.y = node_marker.scale.z = 0.012;
    node_marker.points.reserve(nodes.size());
    node_marker.colors.reserve(nodes.size());
    for (size_t i = 0; i < nodes.size(); ++i) {
      geometry_msgs::msg::Point point;
      point.x = nodes[i].x;
      point.y = nodes[i].y;
      point.z = nodes[i].z;
      node_marker.points.push_back(point);
      const int detection_index = node_detection_index[i];
      node_marker.colors.push_back(
        detection_index >= 0 ?
        objectClusterColor(detections[static_cast<size_t>(detection_index)].class_id, detection_index) :
        semanticNodeColor(node_class_id[i]));
    }

    visualization_msgs::msg::Marker edge_marker;
    edge_marker.header.frame_id = "colour_optical";
    edge_marker.header.stamp = stamp;
    edge_marker.ns = "yolo_3d_box_edges";
    edge_marker.id = 1;
    edge_marker.type = visualization_msgs::msg::Marker::LINE_LIST;
    edge_marker.action = visualization_msgs::msg::Marker::ADD;
    edge_marker.pose.orientation.w = 1.0;
    edge_marker.scale.x = 0.003;
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
      edge_marker.colors.push_back(semanticNodeColor(node_class_id[a], 0.8F));
      edge_marker.colors.push_back(semanticNodeColor(node_class_id[b], 0.8F));
    }

    visualization_msgs::msg::MarkerArray array;
    array.markers.push_back(node_marker);
    array.markers.push_back(edge_marker);
    graph_markers_publisher_->publish(array);
  }

  void publishGraphData(
    const std::vector<GngPoint3f> & nodes, const std::vector<uint32_t> & node_ids,
    const std::vector<std::pair<uint16_t, uint16_t>> & edges,
    const std::vector<int16_t> & node_class_id, const std::vector<float> & node_confidence,
    const rclcpp::Time & stamp)
  {
    om6dof_dd_gng::msg::EnvironmentGraph message;
    message.header.frame_id = "colour_optical";
    message.header.stamp = stamp;
    message.nodes.reserve(nodes.size());
    for (size_t i = 0; i < nodes.size(); ++i) {
      om6dof_dd_gng::msg::EnvironmentNode output;
      output.id = node_ids[i];
      output.position.x = nodes[i].x;
      output.position.y = nodes[i].y;
      output.position.z = nodes[i].z;
      output.class_id = node_class_id[i];
      output.confidence = node_confidence[i];
      message.nodes.push_back(output);
    }
    message.edges.reserve(edges.size());
    for (const auto & [source_index, target_index] : edges) {
      om6dof_dd_gng::msg::TopologyEdge output;
      output.source_id = node_ids[source_index];
      output.target_id = node_ids[target_index];
      const double dx = static_cast<double>(nodes[source_index].x - nodes[target_index].x);
      const double dy = static_cast<double>(nodes[source_index].y - nodes[target_index].y);
      const double dz = static_cast<double>(nodes[source_index].z - nodes[target_index].z);
      output.cost = std::sqrt(dx * dx + dy * dy + dz * dz);
      message.edges.push_back(output);
    }
    graph_data_publisher_->publish(message);
  }

  void showWindow(const cv::Mat & overlay)
  {
    if (!display_window_ || overlay.empty()) {
      return;
    }
    if (!display_window_ready_) {
      if (std::getenv("DISPLAY") == nullptr) {
        RCLCPP_ERROR(get_logger(), "display_window:=true requires a local AGX desktop DISPLAY");
        display_window_ = false;
        return;
      }
      try {
        cv::namedWindow(window_name_, cv::WINDOW_NORMAL);
        cv::resizeWindow(window_name_, 960, 720);
        display_window_ready_ = true;
      } catch (const cv::Exception & error) {
        RCLCPP_ERROR(get_logger(), "Cannot create the display window: %s", error.what());
        display_window_ = false;
        return;
      }
    }
    try {
      cv::imshow(window_name_, overlay);
      cv::waitKey(1);
    } catch (const cv::Exception & error) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 3000, "cv::imshow failed: %s", error.what());
    }
  }

  std::string camera_model_resolved() const
  {
    return om6dof_dd_gng::cameraProfile(model_version_, camera_model_).model;
  }

  std::string model_version_;
  std::string camera_model_;
  std::string camera_serial_;
  int width_ = 640;
  int height_ = 480;
  int fps_ = 30;
  std::string engine_path_;
  float confidence_ = 0.35F;
  float nms_threshold_ = 0.5F;
  float z_min_ = 0.2F;
  float z_max_ = 4.0F;
  float depth_band_m_ = 0.12F;
  int min_points_ = 40;
  std::string image_topic_;
  std::string objects_topic_;
  int jpeg_quality_ = 70;
  bool publish_overlay_ = true;
  bool display_window_ = false;
  bool display_window_ready_ = false;
  const std::string window_name_ = "YOLO 3D box";

  bool enable_dd_gng_ = true;
  int gng_max_nodes_ = 500;
  int gng_insertion_interval_ = 100;
  int gng_max_edge_age_ = 50;
  int gng_updates_per_frame_ = 150;
  int gng_pixel_step_ = 6;
  float attention_strength_ = 6.0F;
  float attention_radius_margin_ = 1.3F;
  float label_padding_m_ = 0.015F;
  double label_ttl_sec_ = 1.5;
  bool publish_graph_markers_ = true;
  std::string graph_markers_topic_;
  std::string graph_data_topic_;

  // One entry per currently- or recently-labelled node, keyed by DD-GNG's
  // stable node id (never reused within an epoch, so a missing key means the
  // node itself is gone, not merely unlabelled this frame).
  struct NodeLabel
  {
    int class_id = -1;
    float confidence = 0.0F;
    SteadyClock::time_point last_seen{};
  };
  std::unordered_map<uint32_t, NodeLabel> node_labels_;

  rs2::pipeline pipeline_;
  double depth_scale_ = 0.0;
  rs2_intrinsics color_intrinsics_{};

  std::shared_ptr<om6dof_dd_gng::YoloDetector> detector_;
  std::unique_ptr<om6dof_dd_gng::DynamicDensityGrowingNeuralGas> gng_;

  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr image_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr objects_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr graph_markers_publisher_;
  rclcpp::Publisher<om6dof_dd_gng::msg::EnvironmentGraph>::SharedPtr graph_data_publisher_;

  std::atomic<bool> running_{false};
  std::thread worker_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  int result = 0;
  try {
    auto node = std::make_shared<Yolo3DBoxNode>();
    rclcpp::spin(node);
  } catch (const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger("yolo_3d_box"), "Fatal startup error: %s", e.what());
    result = 1;
  }
  rclcpp::shutdown();
  return result;
}
