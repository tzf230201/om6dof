// ROS transport and RViz presentation for world-coordinate native GNG cores.
// A transform is always evaluated at depth acquisition time. This node neither
// estimates camera motion nor commands the robot.
#include "comparison_mapper.hpp"

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/buffer.hpp>
#include <tf2_ros/transform_listener.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <bit>
#include <cstring>
#include <deque>
#include <iomanip>
#include <optional>
#include <sstream>

namespace om6dof_f_gng {
using Marker = visualization_msgs::msg::Marker;
using MarkerArray = visualization_msgs::msg::MarkerArray;
using Image = sensor_msgs::msg::Image;
using CameraInfo = sensor_msgs::msg::CameraInfo;
using Point = world_fgng::Point;
using SteadyClock = std::chrono::steady_clock;

geometry_msgs::msg::Point point_message(const Point &p) {
  geometry_msgs::msg::Point result;
  result.x = p.x; result.y = p.y; result.z = p.z;
  return result;
}

std_msgs::msg::ColorRGBA color(float r, float g, float b, float a = 1.0F) {
  std_msgs::msg::ColorRGBA result;
  result.r = r; result.g = g; result.b = b; result.a = a;
  return result;
}

world_fgng::Pose pose_from_transform(const geometry_msgs::msg::Transform &t) {
  const auto &q = t.rotation;
  const double norm = std::hypot(std::hypot(q.x, q.y), std::hypot(q.z, q.w));
  if (!std::isfinite(norm) || std::abs(norm - 1.0) > 1e-3)
    throw std::invalid_argument("TF quaternion is not finite and normalized");
  tf2::Quaternion rotation(q.x / norm, q.y / norm, q.z / norm, q.w / norm);
  const tf2::Matrix3x3 matrix(rotation);
  world_fgng::Pose pose;
  for (int row = 0; row < 3; ++row)
    for (int column = 0; column < 3; ++column)
      pose.m[row * 4 + column] = matrix[row][column];
  pose.m[3] = t.translation.x;
  pose.m[7] = t.translation.y;
  pose.m[11] = t.translation.z;
  pose.validate();  // Includes determinant +1: a reflection is never accepted.
  return pose;
}

class FGngNode final : public rclcpp::Node {
public:
  explicit FGngNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
      : Node("f_gng", options), tf_buffer_(get_clock()),
        tf_listener_(tf_buffer_, this, true) {
    world_frame_ = fixed_parameter<std::string>("world_frame", "world");
    camera_frame_ = fixed_parameter<std::string>("camera_frame", "d435_depth_optical_frame");
    if (world_frame_.empty() || camera_frame_.empty() || world_frame_[0] == '/' ||
        camera_frame_[0] == '/' || world_frame_ == camera_frame_)
      throw std::invalid_argument("world_frame and camera_frame must be distinct nonempty TF names without leading /");
    tf_timeout_ = fixed_parameter<double>("tf_timeout", 0.5);
    max_pose_age_ = fixed_parameter<double>("max_pose_age", 0.5);
    require_joint_states_ = fixed_parameter<bool>("require_joint_states", true);
    for (const auto duration : {tf_timeout_, max_pose_age_})
      if (!std::isfinite(duration) || duration <= 0 || duration > 10)
        throw std::invalid_argument("tf_timeout and max_pose_age must be in (0, 10] seconds");
    algorithm_ = fixed_parameter<std::string>("algorithm", "fgng");
    memory_mode_ = fixed_parameter<std::string>("memory_mode", "world");
    dd_updates_ = bounded_integer("dd_updates", 500, 1, 100000);
    dbl_edge_cut_period_frames_ = bounded_integer("dbl_edge_cut_period_frames", 10, 1, 10000);
    config_.focus.minDepth = fixed_parameter<double>("min_depth", 0.15);
    config_.focus.maxDepth = fixed_parameter<double>("max_depth", 3.0);
    config_.graph.maxNodes = bounded_integer("max_nodes", 768, 2, 4096);
    config_.memoryCapacity = bounded_integer("memory_capacity", 16000, 3, 1000000);
    config_.replayBudget = bounded_integer("replay_budget", 4000, 3, 1000000);
    config_.inputBudget = bounded_integer("input_budget", 12000, 3, 1000000);
    config_.epochsPerFrame = bounded_integer("epochs_per_frame", 1, 1, 100);
    config_.voxelSize = fixed_parameter<double>("voxel_size", 0.01);
    config_.edgeSupportRadius = fixed_parameter<double>("edge_support_radius", algorithm_ == "fgng" ? 0.05 : 0.0);
    mapper_ = make_mapper();  // Reject unknown algorithms/memory modes before subscribing.
    if (mapper_->attentionEnabled()) {
      declare_parameter<std::string>("focus_mode", "auto");
      declare_parameter<double>("gaze_u", 0.5);
      declare_parameter<double>("gaze_v", 0.5);
      declare_parameter<double>("focus_distance", 1.0);
      declare_parameter<double>("world_target_x", 0.0);
      declare_parameter<double>("world_target_y", 0.0);
      declare_parameter<double>("world_target_z", 0.0);
      control_ = control_from_parameters({});
      parameter_callback_ = add_on_set_parameters_callback(
        [this](const std::vector<rclcpp::Parameter> &parameters) {
          rcl_interfaces::msg::SetParametersResult result;
          try {
            const auto next = control_from_parameters(parameters);
            control_ = next;
            result.successful = true;
          } catch (const std::exception &error) {
            result.successful = false;
            result.reason = error.what();
          }
          return result;
        });
    }

    const auto sensor_qos = rclcpp::SensorDataQoS().keep_last(4);
    info_subscription_ = create_subscription<CameraInfo>("depth/camera_info", sensor_qos,
        [this](CameraInfo::ConstSharedPtr info) {
          camera_infos_.push_back(std::move(info));
          while (camera_infos_.size() > 16) camera_infos_.pop_front();
        });
    depth_subscription_ = create_subscription<Image>("depth/image_raw", sensor_qos,
        [this](Image::ConstSharedPtr image) { receive_depth(std::move(image)); });
    joint_subscription_ = create_subscription<sensor_msgs::msg::JointState>(
        "/joint_states", sensor_qos,
        [this](sensor_msgs::msg::JointState::ConstSharedPtr joints) {
          if (joints->name.size() != joints->position.size()) return;
          for (int index = 1; index <= 6; ++index) {
            const auto found = std::find(joints->name.begin(), joints->name.end(),
                                         "joint" + std::to_string(index));
            if (found == joints->name.end() ||
                !std::isfinite(joints->position[std::distance(joints->name.begin(), found)])) return;
          }
          const auto stamp = rclcpp::Time(joints->header.stamp, get_clock()->get_clock_type());
          if (stamp.nanoseconds() > 0 && (now() - stamp).seconds() >= -0.05 &&
              (!joint_stamp_ || stamp > *joint_stamp_))
            joint_stamp_ = stamp;
        });
    if (mapper_->attentionEnabled()) {
      clicked_subscription_ = create_subscription<geometry_msgs::msg::PointStamped>(
          "/clicked_point", rclcpp::QoS(10),
          [this](geometry_msgs::msg::PointStamped::ConstSharedPtr clicked) { select_target(*clicked); });
    }

    points_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>("points", sensor_qos);
    // Reliable graph snapshots keep late RViz joins useful without accumulating history.
    const auto snapshot_qos = rclcpp::QoS(1).reliable().transient_local();
    graph_publisher_ = create_publisher<MarkerArray>("graph", snapshot_qos);
    focus_publisher_ = create_publisher<MarkerArray>("focus", snapshot_qos);
    status_publisher_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>("status", rclcpp::QoS(1));
    reset_service_ = create_service<std_srvs::srv::Trigger>("reset",
        [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
               std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
          mapper_ = make_mapper();
          pending_.clear(); last_mapped_ns_ = 0; last_received_ns_ = 0;
          joint_stamp_.reset();
          mapped_frames_ = 0; dropped_frames_ = 0;
          latest_stats_ = {}; last_success_.reset();
          clear_visualization();
          response->success = true;
          response->message = algorithm_ + " graph and observation memory cleared";
          publish_status(1, "Reset; waiting for a fresh depth frame");
        });
    process_timer_ = create_wall_timer(std::chrono::milliseconds(10), [this] { process_pending(); });
    heartbeat_timer_ = create_wall_timer(std::chrono::seconds(1), [this] {
      if (!last_success_ || (now() - *last_success_).seconds() > max_pose_age_)
        publish_status(1, "No recent mapped frame: " + last_wait_reason_);
    });
    RCLCPP_INFO(get_logger(), "%s ready: depth -> 3D point cloud in %s using %s TF at acquisition time; memory_mode=%s",
                algorithm_.c_str(), world_frame_.c_str(), camera_frame_.c_str(), memory_mode_.c_str());
    if (mapper_->attentionEnabled())
      RCLCPP_INFO(get_logger(), "Use RViz Publish Point to lock F-GNG focus");
  }

private:
  struct Pending {
    Image::ConstSharedPtr image;
    SteadyClock::time_point arrived;
  };

  template<typename T> T fixed_parameter(const std::string &name, const T &value) {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.read_only = true;
    descriptor.description = "Startup parameter; restart the node to change it.";
    return declare_parameter<T>(name, value, descriptor);
  }

  int bounded_integer(const std::string &name, int initial, int minimum, int maximum) {
    const int64_t value = fixed_parameter<int64_t>(name, initial);
    if (value < minimum || value > maximum)
      throw std::invalid_argument(name + " is outside supported allocation limits");
    return static_cast<int>(value);
  }

  std::unique_ptr<comparison::Mapper> make_mapper() const {
    comparison::Config options;
    options.mapping = config_;
    options.algorithm = algorithm_;
    options.memoryMode = memory_mode_;
    options.ddUpdates = dd_updates_;
    options.dblEdgeCutPeriodFrames = dbl_edge_cut_period_frames_;
    return std::make_unique<comparison::Mapper>(options);
  }

  bio_fgng::FocusControl control_from_parameters(const std::vector<rclcpp::Parameter> &updates) {
    const auto parameter = [this, &updates](const std::string &name) {
      const auto it = std::find_if(updates.begin(), updates.end(),
                                  [&name](const auto &p) { return p.get_name() == name; });
      return it == updates.end() ? get_parameter(name) : *it;
    };
    bio_fgng::FocusControl next;
    const auto mode = parameter("focus_mode").as_string();
    if (mode == "auto") next.mode = bio_fgng::FocusMode::Auto;
    else if (mode == "manual") next.mode = bio_fgng::FocusMode::Manual;
    else if (mode == "world_lock") next.mode = bio_fgng::FocusMode::WorldLock;
    else throw std::invalid_argument("focus_mode must be auto, manual or world_lock");
    const auto finite_value = [&parameter](const std::string &name) {
      const double value = parameter(name).as_double();
      if (!std::isfinite(value) || std::abs(value) > 1e6)
        throw std::invalid_argument(name + " must be finite and within +/- 1e6");
      return static_cast<float>(value);
    };
    next.gazeU = finite_value("gaze_u"); next.gazeV = finite_value("gaze_v");
    next.focusDistance = finite_value("focus_distance");
    next.worldTarget = {finite_value("world_target_x"), finite_value("world_target_y"),
                        finite_value("world_target_z")};
    if (next.gazeU < 0 || next.gazeU > 1 || next.gazeV < 0 || next.gazeV > 1 ||
        next.focusDistance <= 0 || !std::isfinite(1.0F / next.focusDistance))
      throw std::invalid_argument("gaze_u/v must be in [0,1]; focus_distance must be positive");
    return next;
  }

  void receive_depth(Image::ConstSharedPtr image) {
    const int64_t stamp = rclcpp::Time(image->header.stamp, get_clock()->get_clock_type()).nanoseconds();
    if (stamp <= 0 || stamp <= last_received_ns_) {
      ++dropped_frames_;
      last_wait_reason_ = "Depth timestamps are zero, repeated or backwards; reset after a clock restart";
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "%s", last_wait_reason_.c_str());
      return;
    }
    // Header checks precede acceptance so a malformed future stamp cannot poison the stream.
    const double age = (now().nanoseconds() - stamp) * 1e-9;
    if (age > max_pose_age_ || age < -0.05 || image->header.frame_id != camera_frame_) {
      ++dropped_frames_;
      last_wait_reason_ = "Depth frame is stale/future-dated or has the wrong optical frame_id";
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "%s", last_wait_reason_.c_str());
      return;
    }
    last_received_ns_ = stamp;
    pending_.push_back({std::move(image), SteadyClock::now()});
    while (pending_.size() > 4) { pending_.pop_front(); ++dropped_frames_; }
  }

  CameraInfo::ConstSharedPtr matching_info(const Image &image) const {
    const auto matches = [&image](const CameraInfo::ConstSharedPtr &info) {
      return info->header.frame_id == image.header.frame_id && info->width == image.width &&
             info->height == image.height &&
             (info->header.stamp == image.header.stamp ||
              (info->header.stamp.sec == 0 && info->header.stamp.nanosec == 0));
    };
    const auto found = std::find_if(camera_infos_.rbegin(), camera_infos_.rend(), matches);
    return found == camera_infos_.rend() ? nullptr : *found;
  }

  bio_fgng::DepthFrame decode_frame(const Image &image, const CameraInfo &info,
                                    const world_fgng::Pose &pose) const {
    const bool float_depth = image.encoding == "32FC1";
    if (!float_depth && image.encoding != "16UC1")
      throw std::invalid_argument("Depth encoding must be 16UC1 millimetres or 32FC1 metres");
    const size_t bytes = float_depth ? 4 : 2;
    const uint64_t pixels = uint64_t(image.width) * image.height;
    if (image.width == 0 || image.height == 0 || pixels > 4000000 ||
        uint64_t(image.step) < uint64_t(image.width) * bytes ||
        uint64_t(image.step) * image.height != image.data.size() || image.data.size() > 64000000)
      throw std::invalid_argument("Depth dimensions, step or buffer length are invalid/too large");
    if (info.binning_x > 1 || info.binning_y > 1 || info.roi.x_offset != 0 || info.roi.y_offset != 0 ||
        (info.roi.width != 0 && info.roi.width != info.width) ||
        (info.roi.height != 0 && info.roi.height != info.height))
      throw std::invalid_argument("CameraInfo must describe the delivered image directly, without extra binning/ROI");
    if (!info.distortion_model.empty() && info.distortion_model != "plumb_bob" &&
        info.distortion_model != "rational_polynomial" && info.distortion_model != "none")
      throw std::invalid_argument("Unsupported camera distortion model; publish pinhole-equivalent depth");
    for (const double coefficient : info.d)
      if (!std::isfinite(coefficient) || std::abs(coefficient) > 1e-9)
        throw std::invalid_argument("Nonzero depth distortion requires rectification before GNG mapping");
    for (const double value : info.k)
      if (!std::isfinite(value)) throw std::invalid_argument("CameraInfo K contains nonfinite values");
    if (std::abs(info.k[1]) > 1e-9 || std::abs(info.k[3]) > 1e-9 || std::abs(info.k[6]) > 1e-9 ||
        std::abs(info.k[7]) > 1e-9 || std::abs(info.k[8] - 1) > 1e-9)
      throw std::invalid_argument("CameraInfo K is not a zero-skew pinhole matrix");
    bio_fgng::DepthFrame frame;
    frame.intrinsics = {static_cast<int>(image.width), static_cast<int>(image.height),
                        static_cast<float>(info.k[0]), static_cast<float>(info.k[4]),
                        static_cast<float>(info.k[2]), static_cast<float>(info.k[5])};
    frame.intrinsics.validate();
    frame.timestamp = rclcpp::Time(image.header.stamp, get_clock()->get_clock_type()).seconds();
    frame.pose = pose;
    frame.depth.resize(static_cast<size_t>(pixels));
    const bool swap = bool(image.is_bigendian) != (std::endian::native == std::endian::big);
    for (size_t y = 0; y < image.height; ++y) for (size_t x = 0; x < image.width; ++x) {
      const uint8_t *source = image.data.data() + y * image.step + x * bytes;
      float z;
      if (float_depth) {
        std::array<uint8_t, 4> value{source[0], source[1], source[2], source[3]};
        if (swap) std::reverse(value.begin(), value.end());
        std::memcpy(&z, value.data(), sizeof(z));
      } else {
        uint16_t value;
        std::memcpy(&value, source, sizeof(value));
        if (swap) value = static_cast<uint16_t>((value >> 8) | (value << 8));
        z = value == 65535 ? 0.0F : float(value) * 0.001F;
      }
      frame.depth[y * image.width + x] =
          std::isfinite(z) && z >= config_.focus.minDepth && z <= config_.focus.maxDepth ? z : 0.0F;
    }
    return frame;
  }

  void process_pending() {
    if (pending_.empty()) return;
    const auto &pending = pending_.front();
    const auto &image = *pending.image;
    const rclcpp::Time stamp(image.header.stamp, get_clock()->get_clock_type());
    const double age = (now() - stamp).seconds();
    const double waited = std::chrono::duration<double>(SteadyClock::now() - pending.arrived).count();
    if (age > max_pose_age_ || age < -0.05 || waited > tf_timeout_) {
      ++dropped_frames_; pending_.pop_front();
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "Dropping depth frame waiting for %s", last_wait_reason_.c_str());
      return;
    }
    if (require_joint_states_) {
      if (!joint_stamp_ || (now() - *joint_stamp_).seconds() > max_pose_age_ ||
          (now() - *joint_stamp_).seconds() < -0.05) {
        last_wait_reason_ = "fresh complete joint1..joint6 on /joint_states";
        return;
      }
    }
    const auto info = matching_info(image);
    if (!info) { last_wait_reason_ = "matching CameraInfo timestamp, optical frame and dimensions"; return; }
    std::string reason;
    if (!tf_buffer_.canTransform(world_frame_, camera_frame_, stamp,
                                  rclcpp::Duration::from_seconds(0), &reason)) {
      last_wait_reason_ = "TF at depth acquisition time: " + reason;
      return;
    }
    try {
      const auto transform = tf_buffer_.lookupTransform(world_frame_, camera_frame_, stamp);
      const auto frame = decode_frame(image, *info, pose_from_transform(transform.transform));
      if (stamp.nanoseconds() <= last_mapped_ns_)
        throw std::invalid_argument("Mapping timestamps must increase; reset after a clock restart");
      latest_stats_ = mapper_->update(frame, control_);
      last_mapped_ns_ = stamp.nanoseconds();
      last_success_ = stamp; ++mapped_frames_;
      std_msgs::msg::Header header = image.header; header.frame_id = world_frame_;
      publish_points(frame, header);
      publish_graph(header);
      publish_focus(frame, header);
      last_wait_reason_ = "next depth frame";
      publish_status(0, "Mapping with acquisition-time TF");
    } catch (const std::exception &error) {
      ++dropped_frames_;
      last_wait_reason_ = error.what();
      publish_status(2, last_wait_reason_);
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000, "Depth rejected: %s", error.what());
    }
    pending_.pop_front();
  }

  void select_target(const geometry_msgs::msg::PointStamped &clicked) {
    try {
      Point target{static_cast<float>(clicked.point.x), static_cast<float>(clicked.point.y),
                   static_cast<float>(clicked.point.z)};
      if (!world_fgng::finite(target) || clicked.header.frame_id.empty())
        throw std::invalid_argument("Clicked point must be finite and have a frame_id");
      if (clicked.header.frame_id != world_frame_) {
        const rclcpp::Time stamp(clicked.header.stamp, get_clock()->get_clock_type());
        if (stamp.nanoseconds() <= 0) throw std::invalid_argument("Clicked point transform needs a nonzero timestamp");
        const auto transform = tf_buffer_.lookupTransform(world_frame_, clicked.header.frame_id, stamp);
        target = pose_from_transform(transform.transform).toWorld(target);
      }
      const auto result = set_parameters_atomically({
          rclcpp::Parameter("focus_mode", "world_lock"),
          rclcpp::Parameter("world_target_x", double(target.x)),
          rclcpp::Parameter("world_target_y", double(target.y)),
          rclcpp::Parameter("world_target_z", double(target.z))});
      if (!result.successful) throw std::invalid_argument(result.reason);
      RCLCPP_INFO(get_logger(), "World focus target: [%.3f, %.3f, %.3f] m in %s",
                   target.x, target.y, target.z, world_frame_.c_str());
    } catch (const std::exception &error) {
      RCLCPP_WARN(get_logger(), "Cannot select focus target: %s", error.what());
    }
  }

  Marker marker(const std_msgs::msg::Header &header, const std::string &space, int id, int type) const {
    Marker result;
    result.header = header; result.ns = space; result.id = id; result.type = type;
    result.action = Marker::ADD; result.pose.orientation.w = 1;
    result.color = color(1, 1, 1);
    return result;
  }

  void publish_points(const bio_fgng::DepthFrame &frame, const std_msgs::msg::Header &header) {
    sensor_msgs::msg::PointCloud2 cloud;
    cloud.header = header;
    sensor_msgs::PointCloud2Modifier modifier(cloud);
    modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");
    modifier.resize(latest_stats_.validPoints);
    cloud.is_dense = true;
    // Humble's PointCloud2Iterator constructor dereferences vector::front();
    // an empty observation must be published without constructing iterators.
    if (latest_stats_.validPoints == 0) {
      points_publisher_->publish(std::move(cloud));
      return;
    }
    sensor_msgs::PointCloud2Iterator<float> x(cloud, "x"), y(cloud, "y"), z(cloud, "z");
    sensor_msgs::PointCloud2Iterator<uint8_t> r(cloud, "r"), g(cloud, "g"), b(cloud, "b");
    const auto &k = frame.intrinsics;
    for (size_t index = 0; index < frame.depth.size(); ++index) {
      const float depth = frame.depth[index];
      if (!(depth >= config_.focus.minDepth && depth <= config_.focus.maxDepth)) continue;
      const auto p = frame.pose.toWorld({(float(index % k.width) - k.cx) * depth / k.fx,
                                        (float(index / k.width) - k.cy) * depth / k.fy, depth});
      *x = p.x; *y = p.y; *z = p.z; *r = 69; *g = 133; *b = 164;
      ++x; ++y; ++z; ++r; ++g; ++b;
    }
    points_publisher_->publish(std::move(cloud));
  }

  void publish_graph(const std_msgs::msg::Header &header) {
    const auto graph = mapper_->graph();
    auto nodes = marker(header, "nodes", 0, Marker::SPHERE_LIST);
    nodes.scale.x = nodes.scale.y = nodes.scale.z = 0.009;
    nodes.points.reserve(graph.nodes.size()); nodes.colors.reserve(graph.nodes.size());
    const bool attention_enabled = mapper_->attentionEnabled();
    const auto baseline_color = algorithm_ == "ddgng" ? color(1.0F, 0.42F, 0.08F) :
                                                       color(0.70F, 0.36F, 1.0F);
    for (const auto &point : graph.nodes) {
      nodes.points.push_back(point_message(point));
      if (attention_enabled) {
        const float attention = std::clamp((mapper_->focus().evaluate(point) - config_.focus.peripheralFloor) /
                                           std::max(config_.focus.gain, 1e-6F), 0.0F, 1.0F);
        nodes.colors.push_back(color(0.12F + 0.88F * attention,
                                      0.83F - 0.10F * attention, 1.0F - 0.88F * attention));
      } else {
        nodes.colors.push_back(baseline_color);
      }
    }
    auto edges = marker(header, "edges", 0, Marker::LINE_LIST);
    edges.scale.x = 0.002;
    edges.color = attention_enabled ? color(0.45F, 0.68F, 0.76F, 0.72F) : baseline_color;
    if (!attention_enabled) edges.color.a = 0.72F;
    edges.points.reserve(graph.edges.size() * 2);
    for (const auto &edge : graph.edges) {
      edges.points.push_back(point_message(graph.nodes.at(edge.first)));
      edges.points.push_back(point_message(graph.nodes.at(edge.second)));
    }
    MarkerArray result;
    result.markers.push_back(std::move(edges)); result.markers.push_back(std::move(nodes));
    graph_publisher_->publish(std::move(result));
  }

  void publish_focus(const bio_fgng::DepthFrame &frame, const std_msgs::msg::Header &header) {
    const auto origin = frame.pose.toWorld({0, 0, 0});
    const auto &k = frame.intrinsics;
    const float length = 0.18F;
    const std::array<Point, 4> corners{{
      frame.pose.toWorld({-k.cx * length / k.fx, -k.cy * length / k.fy, length}),
      frame.pose.toWorld({(k.width - 1 - k.cx) * length / k.fx, -k.cy * length / k.fy, length}),
      frame.pose.toWorld({(k.width - 1 - k.cx) * length / k.fx, (k.height - 1 - k.cy) * length / k.fy, length}),
      frame.pose.toWorld({-k.cx * length / k.fx, (k.height - 1 - k.cy) * length / k.fy, length})}};
    auto frustum = marker(header, "camera", 0, Marker::LINE_LIST);
    frustum.scale.x = 0.002; frustum.color = color(0.44F, 0.56F, 0.64F, 0.85F);
    for (size_t index = 0; index < corners.size(); ++index) {
      frustum.points.push_back(point_message(origin));
      frustum.points.push_back(point_message(corners[index]));
      frustum.points.push_back(point_message(corners[index]));
      frustum.points.push_back(point_message(corners[(index + 1) % corners.size()]));
    }
    if (!mapper_->attentionEnabled()) {
      // Switching an existing RViz session from F-GNG to a baseline must clear
      // cached attention markers. Camera geometry remains valid for all cores.
      MarkerArray result;
      result.markers.push_back(std::move(frustum));
      for (const std::string space : {"gaze", "accommodation", "world_target"}) {
        auto deletion = marker(header, space, 0, Marker::SPHERE);
        deletion.action = Marker::DELETE;
        result.markers.push_back(std::move(deletion));
      }
      focus_publisher_->publish(std::move(result));
      return;
    }
    const auto &status = mapper_->focus().status();
    const auto demand = frame.pose.toWorld({status.gaze.x * status.activeDistance,
                                          status.gaze.y * status.activeDistance,
                                          status.gaze.z * status.activeDistance});
    auto gaze = marker(header, "gaze", 0, Marker::LINE_LIST);
    gaze.scale.x = 0.003;
    gaze.color = status.targetValid ? color(1.0F, 0.72F, 0.12F) : color(0.58F, 0.58F, 0.58F);
    gaze.points = {point_message(origin), point_message(demand)};
    auto focus = marker(header, "accommodation", 0, Marker::SPHERE);
    focus.pose.position = point_message(demand);
    focus.scale.x = focus.scale.y = focus.scale.z = 0.028;
    focus.color = gaze.color; focus.color.a = 0.8F;
    auto target = marker(header, "world_target", 0, Marker::SPHERE);
    target.scale.x = target.scale.y = target.scale.z = 0.036;
    target.color = color(1.0F, 0.35F, 0.3F, 0.75F);
    if (control_.mode == bio_fgng::FocusMode::WorldLock)
      target.pose.position = point_message(control_.worldTarget);
    else target.action = Marker::DELETE;
    MarkerArray result;
    result.markers = {std::move(frustum), std::move(gaze), std::move(focus), std::move(target)};
    focus_publisher_->publish(std::move(result));
  }

  void clear_visualization() {
    std_msgs::msg::Header header;
    header.stamp = now(); header.frame_id = world_frame_;
    auto deletion = marker(header, "", 0, Marker::SPHERE);
    deletion.action = Marker::DELETEALL;
    MarkerArray empty; empty.markers.push_back(deletion);
    graph_publisher_->publish(empty); focus_publisher_->publish(empty);
    sensor_msgs::msg::PointCloud2 cloud;
    cloud.header = header;
    sensor_msgs::PointCloud2Modifier modifier(cloud);
    modifier.setPointCloud2FieldsByString(2, "xyz", "rgb"); modifier.resize(0);
    points_publisher_->publish(cloud);
  }

  void publish_status(uint8_t level, const std::string &message) {
    diagnostic_msgs::msg::DiagnosticArray result;
    result.header.stamp = now(); result.header.frame_id = world_frame_;
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = get_fully_qualified_name(); status.hardware_id = camera_frame_;
    status.level = level; status.message = message;
    const auto value = [&status](const std::string &key, const auto &entry) {
      diagnostic_msgs::msg::KeyValue item;
      item.key = key;
      std::ostringstream stream; stream << std::setprecision(12) << entry;
      item.value = stream.str(); status.values.push_back(std::move(item));
    };
    value("algorithm", algorithm_); value("memory_mode", memory_mode_);
    value("native_input", "3D world-coordinate XYZ point cloud");
    value("observation_preprocessing", memory_mode_ == "world" ?
          "depth backprojection, acquisition-time TF, world voxels and uniform replay" :
          "depth backprojection, acquisition-time TF, current frame sampling; graph state persists");
    value("attention_enabled", mapper_->attentionEnabled());
    value("dd_updates", dd_updates_);
    value("dd_updates_applied", algorithm_ == "ddgng");
    value("dbl_edge_cut_period_frames", dbl_edge_cut_period_frames_);
    value("epochs_per_frame", config_.epochsPerFrame);
    value("node_id_semantics", algorithm_ == "dblgng" ? "snapshot indices" : "persistent native IDs");
    value("world_frame", world_frame_); value("camera_frame", camera_frame_);
    value("mapped_frames", mapped_frames_); value("frames_processed", mapped_frames_);
    value("dropped_frames", dropped_frames_); value("last_reason", last_wait_reason_);
    value("pending_frames", pending_.size()); value("nodes", latest_stats_.nodes);
    value("edges", latest_stats_.edges); value("memory_points", latest_stats_.memoryPoints);
    value("input_pixels", latest_stats_.inputPixels);
    value("valid_points", latest_stats_.validPoints);
    value("inserted_points", latest_stats_.insertedPoints);
    value("replay_points", latest_stats_.replayPoints);
    value("input_budget", config_.inputBudget); value("replay_budget", config_.replayBudget);
    value("update_ms", latest_stats_.updateMs);
    value("raw_edges", latest_stats_.rawEdges);
    value("rejected_edges", latest_stats_.rejectedEdges);
    value("rejected_unsupported_edges", latest_stats_.rejectedUnsupportedEdges);
    if (mapper_->attentionEnabled()) {
      value("focus_mode", get_parameter("focus_mode").as_string());
      value("focus_state", mapper_->focus().status().state);
      value("focus_target_valid", mapper_->focus().status().targetValid);
      value("active_distance_m", mapper_->focus().status().activeDistance);
    }
    value("depth_age_s", last_success_ ? (now() - *last_success_).seconds() : -1.0);
    value("joint_age_s", joint_stamp_ ? (now() - *joint_stamp_).seconds() : -1.0);
    result.status.push_back(std::move(status));
    status_publisher_->publish(std::move(result));
  }

  std::string world_frame_, camera_frame_, last_wait_reason_ = "depth input";
  double tf_timeout_ = 0.5, max_pose_age_ = 0.5;
  bool require_joint_states_ = true;
  std::string algorithm_, memory_mode_;
  int dd_updates_ = 500, dbl_edge_cut_period_frames_ = 10;
  bio_fgng::MapperConfig config_;
  bio_fgng::FocusControl control_;
  std::unique_ptr<comparison::Mapper> mapper_;
  bio_fgng::MapperStats latest_stats_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  std::deque<Pending> pending_;
  std::deque<CameraInfo::ConstSharedPtr> camera_infos_;
  std::optional<rclcpp::Time> joint_stamp_, last_success_;
  int64_t last_received_ns_ = 0, last_mapped_ns_ = 0;
  uint64_t mapped_frames_ = 0, dropped_frames_ = 0;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_callback_;
  rclcpp::Subscription<Image>::SharedPtr depth_subscription_;
  rclcpp::Subscription<CameraInfo>::SharedPtr info_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr clicked_subscription_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr points_publisher_;
  rclcpp::Publisher<MarkerArray>::SharedPtr graph_publisher_, focus_publisher_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr status_publisher_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_service_;
  rclcpp::TimerBase::SharedPtr process_timer_, heartbeat_timer_;
};
}  // namespace om6dof_f_gng

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  int exit_code = 0;
  try {
    rclcpp::spin(std::make_shared<om6dof_f_gng::FGngNode>());
  } catch (const std::exception &error) {
    RCLCPP_FATAL(rclcpp::get_logger("f_gng"), "GNG mapping stopped: %s", error.what());
    exit_code = 1;
  }
  if (rclcpp::ok()) rclcpp::shutdown();
  return exit_code;
}
