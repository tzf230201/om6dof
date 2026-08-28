// Reachability roadmap for the OM6DOF end effector.
//
// This node is deliberately independent of topo_gng_node: it never opens the
// RealSense and never publishes to a controller. It builds either a
// deterministic Halton/PRM baseline or a GNG-quantized joint-space roadmap,
// exposes the full pose+q graph as a typed message, intersects it with the
// typed DD-GNG environment graph, and publishes only a preview path.

#include <algorithm>
#include <array>
#include <chrono>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <functional>
#include <limits>
#include <memory>
#include <optional>
#include <queue>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <Eigen/Geometry>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/color_rgba.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "nav_msgs/msg/path.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include "moveit/collision_detection/collision_common.h"
#include "moveit/collision_detection/collision_matrix.h"
#include "moveit/planning_scene/planning_scene.h"
#include "moveit/robot_model/link_model.h"
#include "moveit/robot_model/robot_model.h"
#include "moveit/robot_model_loader/robot_model_loader.h"
#include "moveit/robot_state/robot_state.h"
#include "moveit_msgs/msg/collision_object.hpp"
#include "shape_msgs/msg/solid_primitive.hpp"

#include "om6dof_dd_gng/msg/environment_graph.hpp"
#include "om6dof_dd_gng/msg/reachability_graph.hpp"
#include "om6dof_dd_gng/msg/reachability_plan.hpp"
#include "om6dof_dd_gng/msg/reachability_query.hpp"
#include "om6dof_dd_gng/reachability_graph.hpp"
#include "om6dof_dd_gng/srv/validate_reachability_scene.hpp"

using namespace std::chrono_literals;
namespace reach = om6dof_dd_gng::reachability;

namespace
{

constexpr std::array<unsigned int, 12> kHaltonBases = {
  2U, 3U, 5U, 7U, 11U, 13U, 17U, 19U, 23U, 29U, 31U, 37U};

constexpr const char * kEnvironmentCollisionObjectId = "dd_gng_environment";

struct BodyLinkSpec
{
  const char * frame_id;
  const char * radius_parameter;
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

constexpr std::array<std::pair<std::size_t, std::size_t>, 10> kBodyEdges = {{
  {0U, 1U}, {1U, 2U}, {2U, 3U}, {3U, 4U}, {4U, 5U}, {5U, 6U},
  {6U, 7U}, {6U, 8U}, {6U, 9U}, {6U, 10U},
}};

struct BodySweep
{
  std::vector<reach::Capsule> capsules;
  reach::Point3 minimum;
  reach::Point3 maximum;
  bool has_bounds = false;
};

geometry_msgs::msg::Point toPointMessage(const reach::Point3 & point)
{
  geometry_msgs::msg::Point msg;
  msg.x = point.x;
  msg.y = point.y;
  msg.z = point.z;
  return msg;
}

geometry_msgs::msg::Pose toPoseMessage(const reach::Node & node)
{
  geometry_msgs::msg::Pose pose;
  pose.position = toPointMessage(node.position);
  pose.orientation.x = node.orientation.x;
  pose.orientation.y = node.orientation.y;
  pose.orientation.z = node.orientation.z;
  pose.orientation.w = node.orientation.w;
  return pose;
}

std_msgs::msg::ColorRGBA color(float red, float green, float blue, float alpha)
{
  std_msgs::msg::ColorRGBA result;
  result.r = red;
  result.g = green;
  result.b = blue;
  result.a = alpha;
  return result;
}

std::uint64_t edgeKey(std::size_t a, std::size_t b)
{
  if (a > b) {
    std::swap(a, b);
  }
  return (static_cast<std::uint64_t>(a) << 32U) | static_cast<std::uint64_t>(b);
}

bool validSha256Hex(const std::string & value)
{
  return value.size() == 64U && std::all_of(
    value.begin(), value.end(), [](const unsigned char character) {
      return std::isdigit(character) != 0 ||
             (character >= static_cast<unsigned char>('a') &&
             character <= static_cast<unsigned char>('f'));
    });
}

}  // namespace

class ReachabilityGraphNode : public rclcpp::Node
{
public:
  ReachabilityGraphNode()
  : rclcpp::Node("reachability_graph_node")
  {
    declareParameters();
    loadParameters();

    rclcpp::QoS latched_qos(1);
    latched_qos.reliable().transient_local();
    graph_pub_ = create_publisher<om6dof_dd_gng::msg::ReachabilityGraph>(
      graph_data_topic_, latched_qos);
    marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      marker_topic_, latched_qos);
    plan_pub_ = create_publisher<om6dof_dd_gng::msg::ReachabilityPlan>(
      plan_topic_, latched_qos);
    path_pub_ = create_publisher<nav_msgs::msg::Path>(path_topic_, latched_qos);

    environment_sub_ = create_subscription<om6dof_dd_gng::msg::EnvironmentGraph>(
      environment_graph_topic_, rclcpp::QoS(2).reliable(),
      std::bind(&ReachabilityGraphNode::environmentCallback, this, std::placeholders::_1));
    query_sub_ = create_subscription<om6dof_dd_gng::msg::ReachabilityQuery>(
      query_topic_, rclcpp::QoS(1).reliable().transient_local(),
      std::bind(&ReachabilityGraphNode::queryCallback, this, std::placeholders::_1));
    joint_state_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      joint_state_topic_, rclcpp::SensorDataQoS(),
      std::bind(&ReachabilityGraphNode::jointStateCallback, this, std::placeholders::_1));

    rebuild_service_ = create_service<std_srvs::srv::Trigger>(
      rebuild_service_name_,
      std::bind(
        &ReachabilityGraphNode::rebuildCallback, this,
        std::placeholders::_1, std::placeholders::_2));
    plan_service_ = create_service<std_srvs::srv::Trigger>(
      plan_service_name_,
      std::bind(
        &ReachabilityGraphNode::planCallback, this,
        std::placeholders::_1, std::placeholders::_2));
    scene_validation_service_ = create_service<
      om6dof_dd_gng::srv::ValidateReachabilityScene>(
      scene_validation_service_name_,
      std::bind(
        &ReachabilityGraphNode::validateSceneCallback, this,
        std::placeholders::_1, std::placeholders::_2));

    const auto period = std::chrono::milliseconds(
      std::max(100, static_cast<int>(planning_period_sec_ * 1000.0)));
    maintenance_timer_ = create_wall_timer(period, [this]() {
      if (!initialized_ && !initialization_failed_) {
        initializeModelAndGraph();
      } else if (dirty_) {
        updatePlanAndMarkers();
      }
    });

    RCLCPP_INFO(
      get_logger(),
      "Reachability node started in preview-only mode; it has no controller publisher or action client.");
  }

private:
  void declareParameters()
  {
    declare_parameter<std::string>("robot_description", "");
    declare_parameter<std::string>("robot_description_semantic", "");
    declare_parameter<std::string>("expanded_urdf_sha256", "");
    declare_parameter<std::string>("srdf_sha256", "");
    declare_parameter<std::string>("reachability_parameters_sha256", "");
    declare_parameter<std::string>("group_name", "arm");
    declare_parameter<std::string>("end_effector_link", "end_effector_link");
    declare_parameter<std::string>("world_frame", "world");

    declare_parameter<std::string>("graph_method", "gng");
    declare_parameter<int>("sample_count", 800);
    declare_parameter<int>("max_sampling_attempts", 20000);
    declare_parameter<int>("halton_start_index", 17);
    declare_parameter<std::int64_t>("sample_stream_seed", 0);
    declare_parameter<int>("gng_training_samples", 4000);
    declare_parameter<int>("gng_max_epochs", 4);
    declare_parameter<int>("gng_insert_interval", 20);
    declare_parameter<int>("gng_max_edge_age", 50);
    declare_parameter<double>("gng_winner_learning_rate", 0.05);
    declare_parameter<double>("gng_neighbor_learning_rate", 0.0006);
    declare_parameter<double>("gng_error_reduction", 0.5);
    declare_parameter<double>("gng_error_decay", 0.995);
    declare_parameter<double>("gng_guard_fraction", 0.25);
    declare_parameter<int>("neighbors", 10);
    declare_parameter<double>("max_normalized_joint_distance", 0.75);
    declare_parameter<double>("max_cartesian_edge_length", 0.14);
    declare_parameter<double>("edge_validation_step", 0.15);
    declare_parameter<bool>("strict_self_collision", true);

    declare_parameter<double>("target_intersection_radius", 0.05);
    declare_parameter<double>("obstacle_clearance", 0.035);
    declare_parameter<double>("target_exclusion_radius", 0.055);
    declare_parameter<int>("start_connect_candidates", 20);
    declare_parameter<double>("start_max_normalized_joint_distance", 0.85);
    declare_parameter<double>("preview_joint_velocity", 0.35);
    declare_parameter<double>("planning_period_sec", 0.5);
    declare_parameter<double>("body_collision_step", 0.08);
    declare_parameter<int>("body_collision_first_edge", 1);
    declare_parameter<bool>("exact_collision_enabled", true);
    declare_parameter<double>("exact_collision_step", 0.05);
    declare_parameter<double>("exact_environment_point_radius", 0.012);
    declare_parameter<double>("exact_environment_edge_radius", 0.006);
    declare_parameter<int>("exact_max_replans", 20);
    declare_parameter<std::vector<std::string>>(
      "exact_environment_ignored_links", std::vector<std::string>{"link1"});

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

    declare_parameter<double>("node_marker_scale", 0.012);
    declare_parameter<double>("edge_marker_width", 0.002);
    declare_parameter<double>("path_marker_width", 0.008);
    declare_parameter<bool>("query_mode", false);

    declare_parameter<std::string>(
      "environment_graph_topic", "/om6dof_topo_gng/environment_graph_data");
    declare_parameter<std::string>(
      "query_topic", "/om6dof_topo_gng/reachability_query");
    declare_parameter<std::string>("joint_state_topic", "/joint_states");
    declare_parameter<std::string>(
      "graph_data_topic", "/om6dof_topo_gng/reachability_graph_data");
    declare_parameter<std::string>(
      "marker_topic", "/om6dof_topo_gng/reachability_graph");
    declare_parameter<std::string>(
      "plan_topic", "/om6dof_topo_gng/reachability_plan");
    declare_parameter<std::string>(
      "path_topic", "/om6dof_topo_gng/reachability_path");
    declare_parameter<std::string>(
      "rebuild_service", "/om6dof_topo_gng/rebuild_reachability");
    declare_parameter<std::string>(
      "plan_service", "/om6dof_topo_gng/plan_reachability");
    declare_parameter<std::string>(
      "scene_validation_service", "/om6dof_topo_gng/validate_reachability_scene");
  }

  void loadParameters()
  {
    group_name_ = get_parameter("group_name").as_string();
    end_effector_link_ = get_parameter("end_effector_link").as_string();
    world_frame_ = get_parameter("world_frame").as_string();
    expanded_urdf_sha256_ = get_parameter("expanded_urdf_sha256").as_string();
    srdf_sha256_ = get_parameter("srdf_sha256").as_string();
    reachability_parameters_sha256_ =
      get_parameter("reachability_parameters_sha256").as_string();
    graph_method_ = get_parameter("graph_method").as_string();
    sample_count_ = static_cast<int>(get_parameter("sample_count").as_int());
    max_sampling_attempts_ = static_cast<int>(get_parameter("max_sampling_attempts").as_int());
    halton_start_index_ = static_cast<int>(get_parameter("halton_start_index").as_int());
    sample_stream_seed_ = get_parameter("sample_stream_seed").as_int();
    gng_training_samples_ = static_cast<int>(get_parameter("gng_training_samples").as_int());
    gng_max_epochs_ = static_cast<int>(get_parameter("gng_max_epochs").as_int());
    gng_insert_interval_ = static_cast<int>(get_parameter("gng_insert_interval").as_int());
    gng_max_edge_age_ = static_cast<int>(get_parameter("gng_max_edge_age").as_int());
    gng_winner_learning_rate_ = get_parameter("gng_winner_learning_rate").as_double();
    gng_neighbor_learning_rate_ = get_parameter("gng_neighbor_learning_rate").as_double();
    gng_error_reduction_ = get_parameter("gng_error_reduction").as_double();
    gng_error_decay_ = get_parameter("gng_error_decay").as_double();
    gng_guard_fraction_ = get_parameter("gng_guard_fraction").as_double();
    neighbors_ = static_cast<int>(get_parameter("neighbors").as_int());
    max_normalized_joint_distance_ = get_parameter("max_normalized_joint_distance").as_double();
    max_cartesian_edge_length_ = get_parameter("max_cartesian_edge_length").as_double();
    edge_validation_step_ = get_parameter("edge_validation_step").as_double();
    strict_self_collision_ = get_parameter("strict_self_collision").as_bool();
    target_intersection_radius_ = get_parameter("target_intersection_radius").as_double();
    obstacle_clearance_ = get_parameter("obstacle_clearance").as_double();
    target_exclusion_radius_ = get_parameter("target_exclusion_radius").as_double();
    start_connect_candidates_ = static_cast<int>(get_parameter("start_connect_candidates").as_int());
    start_max_normalized_joint_distance_ =
      get_parameter("start_max_normalized_joint_distance").as_double();
    preview_joint_velocity_ = get_parameter("preview_joint_velocity").as_double();
    planning_period_sec_ = get_parameter("planning_period_sec").as_double();
    body_collision_step_ = get_parameter("body_collision_step").as_double();
    body_collision_first_edge_ = static_cast<int>(
      get_parameter("body_collision_first_edge").as_int());
    exact_collision_enabled_ = get_parameter("exact_collision_enabled").as_bool();
    exact_collision_step_ = get_parameter("exact_collision_step").as_double();
    exact_environment_point_radius_ =
      get_parameter("exact_environment_point_radius").as_double();
    exact_environment_edge_radius_ =
      get_parameter("exact_environment_edge_radius").as_double();
    exact_max_replans_ = static_cast<int>(get_parameter("exact_max_replans").as_int());
    exact_environment_ignored_links_ =
      get_parameter("exact_environment_ignored_links").as_string_array();
    body_radii_.clear();
    for (const auto & body_link : kBodyLinks) {
      body_radii_[body_link.frame_id] = get_parameter(body_link.radius_parameter).as_double();
    }
    node_marker_scale_ = get_parameter("node_marker_scale").as_double();
    edge_marker_width_ = get_parameter("edge_marker_width").as_double();
    path_marker_width_ = get_parameter("path_marker_width").as_double();
    query_mode_ = get_parameter("query_mode").as_bool();
    environment_graph_topic_ = get_parameter("environment_graph_topic").as_string();
    query_topic_ = get_parameter("query_topic").as_string();
    joint_state_topic_ = get_parameter("joint_state_topic").as_string();
    graph_data_topic_ = get_parameter("graph_data_topic").as_string();
    marker_topic_ = get_parameter("marker_topic").as_string();
    plan_topic_ = get_parameter("plan_topic").as_string();
    path_topic_ = get_parameter("path_topic").as_string();
    rebuild_service_name_ = get_parameter("rebuild_service").as_string();
    plan_service_name_ = get_parameter("plan_service").as_string();
    scene_validation_service_name_ = get_parameter("scene_validation_service").as_string();

    if (graph_method_ != "halton_prm" && graph_method_ != "gng" &&
      graph_method_ != "guarded_gng")
    {
      throw std::runtime_error(
              "graph_method must be 'halton_prm', 'gng', or 'guarded_gng'");
    }
    if (!validSha256Hex(expanded_urdf_sha256_) || !validSha256Hex(srdf_sha256_) ||
      !validSha256Hex(reachability_parameters_sha256_))
    {
      throw std::runtime_error(
              "expanded URDF, SRDF, and resolved parameter provenance must be "
              "lowercase SHA-256 digests");
    }
    if (sample_count_ < 2 || max_sampling_attempts_ < sample_count_ || halton_start_index_ < 0 ||
      sample_stream_seed_ < 0 ||
      neighbors_ < 1 ||
      gng_training_samples_ < 2 || gng_max_epochs_ < 1 || gng_insert_interval_ < 1 ||
      gng_max_edge_age_ < 1)
    {
      throw std::runtime_error(
              "reachability sampling/GNG count parameters are invalid");
    }
    if (!std::isfinite(gng_guard_fraction_) || gng_guard_fraction_ < 0.0 ||
      gng_guard_fraction_ > 0.90)
    {
      throw std::runtime_error("gng_guard_fraction must be within [0.0, 0.90]");
    }
    if (edge_validation_step_ <= 0.0 || target_intersection_radius_ <= 0.0 ||
      obstacle_clearance_ < 0.0 || preview_joint_velocity_ <= 0.0 ||
      body_collision_step_ <= 0.0 || body_collision_first_edge_ < 0 ||
      body_collision_first_edge_ >= static_cast<int>(kBodyEdges.size()) ||
      exact_collision_step_ <= 0.0 || exact_environment_point_radius_ <= 0.0 ||
      exact_environment_edge_radius_ <= 0.0 || exact_max_replans_ < 0)
    {
      throw std::runtime_error("reachability distance/step/velocity parameters are invalid");
    }
    for (const auto & [name, radius] : body_radii_) {
      if (!std::isfinite(radius) || radius <= 0.0) {
        throw std::runtime_error("invalid body capsule radius for " + name);
      }
    }
  }

  bool initializeModelAndGraph()
  {
    const std::string urdf = get_parameter("robot_description").as_string();
    const std::string srdf = get_parameter("robot_description_semantic").as_string();
    if (urdf.empty() || srdf.empty()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Waiting for robot_description and robot_description_semantic parameters. "
        "Use reachability_graph.launch.py or topo_gng_node.launch.py.");
      return false;
    }

    try {
      robot_model_loader::RobotModelLoader::Options options(urdf, srdf);
      options.load_kinematics_solvers_ = false;
      model_loader_ = std::make_shared<robot_model_loader::RobotModelLoader>(
        shared_from_this(), options);
      robot_model_ = model_loader_->getModel();
      if (!robot_model_) {
        throw std::runtime_error("RobotModelLoader returned a null model");
      }
      joint_model_group_ = robot_model_->getJointModelGroup(group_name_);
      if (joint_model_group_ == nullptr) {
        throw std::runtime_error("MoveIt group '" + group_name_ + "' does not exist");
      }
      if (!robot_model_->hasLinkModel(end_effector_link_)) {
        throw std::runtime_error("end-effector link '" + end_effector_link_ + "' does not exist");
      }
      if (robot_model_->getModelFrame() != world_frame_) {
        throw std::runtime_error(
                "PlanningScene model frame '" + robot_model_->getModelFrame() +
                "' differs from configured world_frame '" + world_frame_ + "'");
      }
      for (const auto & body_link : kBodyLinks) {
        if (!robot_model_->hasLinkModel(body_link.frame_id)) {
          throw std::runtime_error(
                  "body capsule link '" + std::string(body_link.frame_id) + "' does not exist");
        }
      }

      joint_names_ = joint_model_group_->getVariableNames();
      if (joint_names_.empty() || joint_names_.size() > kHaltonBases.size()) {
        throw std::runtime_error("unsupported number of arm variables for Halton sampling");
      }
      halton_digit_permutations_.clear();
      halton_digit_permutations_.reserve(joint_names_.size());
      for (std::size_t variable = 0U; variable < joint_names_.size(); ++variable) {
        halton_digit_permutations_.push_back(reach::haltonDigitPermutation(
            kHaltonBases[variable], static_cast<std::uint64_t>(sample_stream_seed_), variable));
      }
      lower_bounds_.clear();
      upper_bounds_.clear();
      joint_ranges_.clear();
      for (const std::string & name : joint_names_) {
        const auto & bounds = robot_model_->getVariableBounds(name);
        if (!bounds.position_bounded_ || bounds.max_position_ <= bounds.min_position_) {
          throw std::runtime_error("joint variable '" + name + "' has no finite position bounds");
        }
        lower_bounds_.push_back(bounds.min_position_);
        upper_bounds_.push_back(bounds.max_position_);
        joint_ranges_.push_back(bounds.max_position_ - bounds.min_position_);
      }

      planning_scene_ = std::make_shared<planning_scene::PlanningScene>(robot_model_);
      buildStrictCollisionMatrix();
      if (!buildGraph()) {
        throw std::runtime_error("reachability graph generation produced too few nodes or no edges");
      }
      rebuildExactEnvironmentScene();
      initialized_ = true;
      dirty_ = true;
      publishGraphData();
      RCLCPP_INFO(
        get_logger(),
        "Reachability graph ready: method=%s, %zu nodes, %zu edges, group=%s, tip=%s, "
        "strict_collision=%s, exact_collision=%s",
        graph_method_.c_str(), nodes_.size(), edges_.size(), group_name_.c_str(),
        end_effector_link_.c_str(),
        strict_self_collision_ ? "true" : "false",
        exact_collision_enabled_ ? "true" : "false");
      return true;
    } catch (const std::exception & error) {
      RCLCPP_ERROR(get_logger(), "Reachability initialization failed: %s", error.what());
      initialization_failed_ = true;
      model_loader_.reset();
      robot_model_.reset();
      planning_scene_.reset();
      joint_model_group_ = nullptr;
      return false;
    }
  }

  void buildStrictCollisionMatrix()
  {
    strict_collision_matrix_ = std::make_unique<collision_detection::AllowedCollisionMatrix>(
      robot_model_->getLinkModelNames(), false);
    for (const moveit::core::LinkModel * link : robot_model_->getLinkModels()) {
      const moveit::core::LinkModel * parent = link->getParentLinkModel();
      if (parent != nullptr) {
        strict_collision_matrix_->setEntry(link->getName(), parent->getName(), true);
      }
    }
  }

  bool stateIsValid(const std::vector<double> & joints) const
  {
    if (!robot_model_ || joints.size() != joint_names_.size()) {
      return false;
    }
    moveit::core::RobotState state(robot_model_);
    state.setToDefaultValues();
    state.setJointGroupPositions(joint_model_group_, joints);
    state.update();
    if (!state.satisfiesBounds(joint_model_group_)) {
      return false;
    }
    if (strict_self_collision_) {
      state.updateCollisionBodyTransforms();
      collision_detection::CollisionRequest request;
      request.group_name = group_name_;
      collision_detection::CollisionResult result;
      planning_scene_->checkSelfCollision(request, result, state, *strict_collision_matrix_);
      return !result.collision;
    }
    return !planning_scene_->isStateColliding(state, group_name_, false);
  }

  reach::Node nodeFromJoints(std::uint32_t id, const std::vector<double> & joints) const
  {
    moveit::core::RobotState state(robot_model_);
    state.setToDefaultValues();
    state.setJointGroupPositions(joint_model_group_, joints);
    state.update();
    const Eigen::Isometry3d & transform = state.getGlobalLinkTransform(end_effector_link_);
    Eigen::Quaterniond orientation(transform.linear());
    orientation.normalize();

    reach::Node node;
    node.id = id;
    node.position = {
      transform.translation().x(), transform.translation().y(), transform.translation().z()};
    node.orientation = {
      orientation.x(), orientation.y(), orientation.z(), orientation.w()};
    node.joints = joints;
    return node;
  }

  bool appendNodeIfValid(const std::vector<double> & joints)
  {
    for (const reach::Node & existing : nodes_) {
      if (reach::normalizedJointDistance(existing.joints, joints, joint_ranges_) < 1.0e-9) {
        return false;
      }
    }
    if (!stateIsValid(joints)) {
      return false;
    }
    nodes_.push_back(nodeFromJoints(static_cast<std::uint32_t>(nodes_.size()), joints));
    return true;
  }

  std::optional<std::vector<double>> currentJoints() const
  {
    std::vector<double> result;
    result.reserve(joint_names_.size());
    for (const std::string & name : joint_names_) {
      const auto found = latest_joint_positions_.find(name);
      if (found == latest_joint_positions_.end() || !std::isfinite(found->second)) {
        return std::nullopt;
      }
      result.push_back(found->second);
    }
    return result;
  }

  bool buildGraph()
  {
    const auto build_started = std::chrono::steady_clock::now();
    nodes_.clear();
    edges_.clear();
    edge_index_by_key_.clear();
    node_body_sweeps_.clear();
    edge_body_sweeps_.clear();
    exact_blocked_edges_.clear();
    last_anchor_node_count_ = 0U;
    last_prototype_budget_ = 0U;
    last_prototype_node_count_ = 0U;
    last_gng_training_sample_count_ = 0U;
    last_guard_node_count_ = 0U;
    last_fill_sample_node_count_ = 0U;
    last_candidate_attempts_ = 0U;
    last_requested_guard_node_count_ = 0U;

    moveit::core::RobotState seed_state(robot_model_);
    seed_state.setToDefaultValues();
    std::vector<double> seed;
    seed_state.copyJointGroupPositions(joint_model_group_, seed);
    appendNodeIfValid(seed);
    for (const std::string & state_name : {std::string("init_pose"), std::string("home_pose")}) {
      seed_state.setToDefaultValues();
      if (seed_state.setToDefaultValues(group_name_, state_name)) {
        seed_state.copyJointGroupPositions(joint_model_group_, seed);
        appendNodeIfValid(seed);
      }
    }
    if (const auto measured = currentJoints()) {
      appendNodeIfValid(*measured);
    }
    last_anchor_node_count_ = nodes_.size();

    int attempts = 0;
    if (graph_method_ == "gng") {
      attempts = sampleGngNodes(0.0, false);
    } else if (graph_method_ == "guarded_gng") {
      attempts = sampleGngNodes(gng_guard_fraction_, true);
    } else {
      const std::size_t before_samples = nodes_.size();
      attempts = sampleHaltonNodes(0);
      last_fill_sample_node_count_ = nodes_.size() - before_samples;
    }
    last_candidate_attempts_ = static_cast<std::size_t>(std::max(0, attempts));
    if (nodes_.size() < 2U) {
      RCLCPP_ERROR(
        get_logger(), "Only %zu valid reachability samples after %d attempts", nodes_.size(), attempts);
      return false;
    }

    connectRoadmapEdges();
    if (edges_.empty()) {
      return false;
    }
    cacheBodySweeps();
    exact_blocked_edges_.assign(edges_.size(), false);
    RCLCPP_INFO(
      get_logger(),
      "Built method=%s with %zu/%d nodes (%zu anchors, %zu prototypes, %zu/%zu guards, "
      "%zu fill samples) after %d candidate attempts; retained %zu validated edges in %zu "
      "components and cached %zu full-body edge sweeps",
      graph_method_.c_str(), nodes_.size(), sample_count_, last_anchor_node_count_,
      last_prototype_node_count_, last_guard_node_count_, last_requested_guard_node_count_,
      last_fill_sample_node_count_, attempts, edges_.size(),
      connectedComponentCount(), edge_body_sweeps_.size());
    const bool valid_cache = node_body_sweeps_.size() == nodes_.size() &&
      edge_body_sweeps_.size() == edges_.size();
    last_build_time_ms_ = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - build_started).count();
    if (valid_cache) {
      ++graph_revision_;
    }
    return valid_cache;
  }

  std::vector<double> haltonJointSample(int attempt) const
  {
    const std::uint64_t index = static_cast<std::uint64_t>(halton_start_index_ + attempt + 1);
    std::vector<double> joints(joint_names_.size(), 0.0);
    for (std::size_t variable = 0; variable < joints.size(); ++variable) {
      const double unit = reach::haltonRadicalInverse(
        index, kHaltonBases[variable], halton_digit_permutations_[variable]);
      joints[variable] = lower_bounds_[variable] + unit * joint_ranges_[variable];
    }
    return joints;
  }

  int sampleHaltonNodes(int first_attempt)
  {
    int attempts = first_attempt;
    while (static_cast<int>(nodes_.size()) < sample_count_ && attempts < max_sampling_attempts_) {
      appendNodeIfValid(haltonJointSample(attempts));
      ++attempts;
    }
    return attempts;
  }

  int sampleGngNodes(double requested_guard_fraction, bool reserve_guard_budget)
  {
    std::vector<std::vector<double>> normalized_training_samples;
    normalized_training_samples.reserve(static_cast<std::size_t>(gng_training_samples_));
    int attempts = 0;
    while (static_cast<int>(normalized_training_samples.size()) < gng_training_samples_ &&
      attempts < max_sampling_attempts_)
    {
      const std::vector<double> joints = haltonJointSample(attempts);
      ++attempts;
      if (!stateIsValid(joints)) {
        continue;
      }
      std::vector<double> normalized(joints.size(), 0.0);
      for (std::size_t variable = 0; variable < joints.size(); ++variable) {
        normalized[variable] =
          (joints[variable] - lower_bounds_[variable]) / joint_ranges_[variable];
      }
      normalized_training_samples.push_back(std::move(normalized));
    }
    last_gng_training_sample_count_ = normalized_training_samples.size();
    if (normalized_training_samples.size() < 2U) {
      return attempts;
    }

    const std::size_t initial_node_count = nodes_.size();
    const reach::GngNodeBudget budget = reach::allocateGngNodeBudget(
      static_cast<std::size_t>(sample_count_), initial_node_count,
      requested_guard_fraction, reserve_guard_budget);
    const std::size_t guard_count = budget.guard_count;
    const std::size_t prototype_budget = budget.prototype_count;
    last_prototype_budget_ = prototype_budget;
    last_requested_guard_node_count_ = guard_count;

    std::vector<std::vector<double>> prototypes;
    if (prototype_budget >= 2U) {
      reach::GngParameters parameters;
      parameters.max_units = prototype_budget;
      parameters.insertion_interval = static_cast<std::size_t>(gng_insert_interval_);
      parameters.max_edge_age = gng_max_edge_age_;
      parameters.winner_learning_rate = gng_winner_learning_rate_;
      parameters.neighbor_learning_rate = gng_neighbor_learning_rate_;
      parameters.error_reduction = gng_error_reduction_;
      parameters.error_decay = gng_error_decay_;
      parameters.max_epochs = static_cast<std::size_t>(gng_max_epochs_);
      prototypes = reach::growingNeuralGas(normalized_training_samples, parameters);
    }

    auto denormalize = [this](const std::vector<double> & normalized) {
        std::vector<double> joints(normalized.size(), 0.0);
        for (std::size_t variable = 0; variable < normalized.size(); ++variable) {
          joints[variable] = lower_bounds_[variable] +
            std::clamp(normalized[variable], 0.0, 1.0) * joint_ranges_[variable];
        }
        return joints;
      };
    for (const auto & prototype : prototypes) {
      if (static_cast<int>(nodes_.size()) >= sample_count_) {
        break;
      }
      appendNodeIfValid(denormalize(prototype));
    }
    const std::size_t prototype_node_count = nodes_.size() - initial_node_count;
    last_prototype_node_count_ = prototype_node_count;
    for (const std::size_t sample_index : reach::stratifiedGuardIndices(
        normalized_training_samples.size(), guard_count))
    {
      if (static_cast<int>(nodes_.size()) >= sample_count_) {
        break;
      }
      appendNodeIfValid(denormalize(normalized_training_samples[sample_index]));
    }
    const std::size_t guard_node_count =
      nodes_.size() - initial_node_count - prototype_node_count;
    last_guard_node_count_ = guard_node_count;
    RCLCPP_INFO(
      get_logger(),
      "GNG sampling retained %zu prototypes and %zu/%zu deterministic guard samples",
      prototype_node_count, guard_node_count, guard_count);
    // Averages can fall into invalid C-space even when all training samples
    // are valid. Fill filtered units from the same deterministic valid stream
    // so both graph methods retain the requested node budget.
    const std::size_t before_fill = nodes_.size();
    for (const auto & sample : normalized_training_samples) {
      if (static_cast<int>(nodes_.size()) >= sample_count_) {
        break;
      }
      appendNodeIfValid(denormalize(sample));
    }
    const int final_attempts = sampleHaltonNodes(attempts);
    last_fill_sample_node_count_ = nodes_.size() - before_fill;
    return final_attempts;
  }

  void connectRoadmapEdges()
  {
    std::set<std::uint64_t> seen_edges;
    for (std::size_t i = 0; i < nodes_.size(); ++i) {
      std::vector<std::pair<double, std::size_t>> candidates;
      candidates.reserve(nodes_.size() - 1U);
      for (std::size_t j = 0; j < nodes_.size(); ++j) {
        if (i == j) {
          continue;
        }
        const double joint_distance = reach::normalizedJointDistance(
          nodes_[i].joints, nodes_[j].joints, joint_ranges_);
        if (joint_distance > max_normalized_joint_distance_ ||
          reach::distance(nodes_[i].position, nodes_[j].position) > max_cartesian_edge_length_)
        {
          continue;
        }
        candidates.emplace_back(joint_distance, j);
      }
      std::sort(candidates.begin(), candidates.end());
      const std::size_t limit = std::min<std::size_t>(
        candidates.size(), static_cast<std::size_t>(neighbors_));
      for (std::size_t candidate_index = 0; candidate_index < limit; ++candidate_index) {
        const std::size_t j = candidates[candidate_index].second;
        const std::uint64_t key = edgeKey(i, j);
        if (seen_edges.count(key) != 0U || !edgeStateIsValid(nodes_[i].joints, nodes_[j].joints)) {
          continue;
        }
        const double cost = reach::jointPathCost(nodes_[i].joints, nodes_[j].joints);
        if (std::isfinite(cost) && cost > 1.0e-12) {
          edges_.push_back({i, j, cost});
          edge_index_by_key_[key] = edges_.size() - 1U;
          seen_edges.insert(key);
        }
      }
    }
  }

  bool edgeStateIsValid(
    const std::vector<double> & from,
    const std::vector<double> & to) const
  {
    if (from.size() != to.size() || from.empty()) {
      return false;
    }
    double max_delta = 0.0;
    for (std::size_t i = 0; i < from.size(); ++i) {
      max_delta = std::max(max_delta, std::abs(to[i] - from[i]));
    }
    const int steps = std::max(1, static_cast<int>(std::ceil(max_delta / edge_validation_step_)));
    std::vector<double> interpolated(from.size(), 0.0);
    for (int step = 0; step <= steps; ++step) {
      const double t = static_cast<double>(step) / static_cast<double>(steps);
      for (std::size_t i = 0; i < from.size(); ++i) {
        interpolated[i] = from[i] + (to[i] - from[i]) * t;
      }
      if (!stateIsValid(interpolated)) {
        return false;
      }
    }
    return true;
  }

  std::vector<reach::Capsule> bodyCapsulesForJoints(
    const std::vector<double> & joints) const
  {
    moveit::core::RobotState state(robot_model_);
    state.setToDefaultValues();
    state.setJointGroupPositions(joint_model_group_, joints);
    state.update();

    std::array<reach::Point3, kBodyLinks.size()> positions;
    for (std::size_t i = 0U; i < kBodyLinks.size(); ++i) {
      const Eigen::Isometry3d & transform = state.getGlobalLinkTransform(kBodyLinks[i].frame_id);
      positions[i] = {
        transform.translation().x(), transform.translation().y(), transform.translation().z()};
    }

    std::vector<reach::Capsule> capsules;
    capsules.reserve(kBodyEdges.size() - static_cast<std::size_t>(body_collision_first_edge_));
    for (std::size_t edge_index = static_cast<std::size_t>(body_collision_first_edge_);
      edge_index < kBodyEdges.size(); ++edge_index)
    {
      const auto [a, b] = kBodyEdges[edge_index];
      capsules.push_back({
        {positions[a], positions[b]},
        std::max(
          body_radii_.at(kBodyLinks[a].frame_id),
          body_radii_.at(kBodyLinks[b].frame_id))});
    }
    return capsules;
  }

  static void appendCapsule(BodySweep & sweep, const reach::Capsule & capsule)
  {
    sweep.capsules.push_back(capsule);
    const double radius = std::max(0.0, capsule.radius);
    const reach::Point3 minimum{
      std::min(capsule.axis.a.x, capsule.axis.b.x) - radius,
      std::min(capsule.axis.a.y, capsule.axis.b.y) - radius,
      std::min(capsule.axis.a.z, capsule.axis.b.z) - radius};
    const reach::Point3 maximum{
      std::max(capsule.axis.a.x, capsule.axis.b.x) + radius,
      std::max(capsule.axis.a.y, capsule.axis.b.y) + radius,
      std::max(capsule.axis.a.z, capsule.axis.b.z) + radius};
    if (!sweep.has_bounds) {
      sweep.minimum = minimum;
      sweep.maximum = maximum;
      sweep.has_bounds = true;
      return;
    }
    sweep.minimum.x = std::min(sweep.minimum.x, minimum.x);
    sweep.minimum.y = std::min(sweep.minimum.y, minimum.y);
    sweep.minimum.z = std::min(sweep.minimum.z, minimum.z);
    sweep.maximum.x = std::max(sweep.maximum.x, maximum.x);
    sweep.maximum.y = std::max(sweep.maximum.y, maximum.y);
    sweep.maximum.z = std::max(sweep.maximum.z, maximum.z);
  }

  BodySweep bodySweepForTransition(
    const std::vector<double> & from,
    const std::vector<double> & to) const
  {
    BodySweep sweep;
    double max_delta = 0.0;
    for (std::size_t i = 0U; i < from.size(); ++i) {
      max_delta = std::max(max_delta, std::abs(to[i] - from[i]));
    }
    const int steps = std::max(1, static_cast<int>(std::ceil(max_delta / body_collision_step_)));
    std::vector<double> interpolated(from.size(), 0.0);
    std::vector<reach::Capsule> previous;
    for (int step = 0; step <= steps; ++step) {
      const double t = static_cast<double>(step) / static_cast<double>(steps);
      for (std::size_t i = 0U; i < from.size(); ++i) {
        interpolated[i] = from[i] + (to[i] - from[i]) * t;
      }
      const auto current = bodyCapsulesForJoints(interpolated);
      for (const reach::Capsule & capsule : current) {
        appendCapsule(sweep, capsule);
      }
      if (previous.size() == current.size()) {
        for (std::size_t i = 0U; i < current.size(); ++i) {
          const double radius = std::max(previous[i].radius, current[i].radius);
          appendCapsule(sweep, {{previous[i].axis.a, current[i].axis.a}, radius});
          appendCapsule(sweep, {{previous[i].axis.b, current[i].axis.b}, radius});
        }
      }
      previous = current;
    }
    return sweep;
  }

  BodySweep bodySweepForState(const std::vector<double> & joints) const
  {
    BodySweep sweep;
    for (const reach::Capsule & capsule : bodyCapsulesForJoints(joints)) {
      appendCapsule(sweep, capsule);
    }
    return sweep;
  }

  void cacheBodySweeps()
  {
    node_body_sweeps_.clear();
    node_body_sweeps_.reserve(nodes_.size());
    for (const reach::Node & node : nodes_) {
      node_body_sweeps_.push_back(bodySweepForState(node.joints));
    }
    edge_body_sweeps_.clear();
    edge_body_sweeps_.reserve(edges_.size());
    for (const reach::Edge & edge : edges_) {
      edge_body_sweeps_.push_back(
        bodySweepForTransition(nodes_[edge.a].joints, nodes_[edge.b].joints));
    }
  }

  bool bodySweepBlocked(const BodySweep & sweep) const
  {
    if (!sweep.has_bounds) {
      return false;
    }
    const auto point_in_bounds = [this, &sweep](const reach::Point3 & point) {
        return point.x >= sweep.minimum.x - obstacle_clearance_ &&
               point.x <= sweep.maximum.x + obstacle_clearance_ &&
               point.y >= sweep.minimum.y - obstacle_clearance_ &&
               point.y <= sweep.maximum.y + obstacle_clearance_ &&
               point.z >= sweep.minimum.z - obstacle_clearance_ &&
               point.z <= sweep.maximum.z + obstacle_clearance_;
      };
    for (const reach::Point3 & obstacle : obstacle_points_) {
      if (!point_in_bounds(obstacle)) {
        continue;
      }
      for (const reach::Capsule & capsule : sweep.capsules) {
        if (reach::pointSegmentDistance(obstacle, capsule.axis) <
          capsule.radius + obstacle_clearance_)
        {
          return true;
        }
      }
    }
    for (const reach::Segment & obstacle : obstacle_segments_) {
      const reach::Point3 obstacle_minimum{
        std::min(obstacle.a.x, obstacle.b.x),
        std::min(obstacle.a.y, obstacle.b.y),
        std::min(obstacle.a.z, obstacle.b.z)};
      const reach::Point3 obstacle_maximum{
        std::max(obstacle.a.x, obstacle.b.x),
        std::max(obstacle.a.y, obstacle.b.y),
        std::max(obstacle.a.z, obstacle.b.z)};
      if (obstacle_maximum.x < sweep.minimum.x - obstacle_clearance_ ||
        obstacle_minimum.x > sweep.maximum.x + obstacle_clearance_ ||
        obstacle_maximum.y < sweep.minimum.y - obstacle_clearance_ ||
        obstacle_minimum.y > sweep.maximum.y + obstacle_clearance_ ||
        obstacle_maximum.z < sweep.minimum.z - obstacle_clearance_ ||
        obstacle_minimum.z > sweep.maximum.z + obstacle_clearance_)
      {
        continue;
      }
      for (const reach::Capsule & capsule : sweep.capsules) {
        if (reach::segmentSegmentDistance(obstacle, capsule.axis) <
          capsule.radius + obstacle_clearance_)
        {
          return true;
        }
      }
    }
    return false;
  }

  std::vector<bool> fullBodyBlockedNodes() const
  {
    std::vector<bool> blocked(nodes_.size(), false);
    for (std::size_t i = 0U; i < node_body_sweeps_.size(); ++i) {
      blocked[i] = bodySweepBlocked(node_body_sweeps_[i]);
    }
    return blocked;
  }

  std::vector<bool> fullBodyBlockedEdges(const std::vector<bool> & blocked_nodes) const
  {
    std::vector<bool> blocked(edges_.size(), false);
    for (std::size_t i = 0U; i < edge_body_sweeps_.size(); ++i) {
      const reach::Edge & edge = edges_[i];
      blocked[i] =
        (edge.a < blocked_nodes.size() && blocked_nodes[edge.a]) ||
        (edge.b < blocked_nodes.size() && blocked_nodes[edge.b]) ||
        bodySweepBlocked(edge_body_sweeps_[i]);
    }
    return blocked;
  }

  std::size_t connectedComponentCount() const
  {
    std::vector<std::vector<std::size_t>> adjacency(nodes_.size());
    for (const reach::Edge & edge : edges_) {
      adjacency[edge.a].push_back(edge.b);
      adjacency[edge.b].push_back(edge.a);
    }
    std::vector<bool> visited(nodes_.size(), false);
    std::size_t components = 0;
    for (std::size_t root = 0; root < nodes_.size(); ++root) {
      if (visited[root]) {
        continue;
      }
      ++components;
      std::queue<std::size_t> queue;
      queue.push(root);
      visited[root] = true;
      while (!queue.empty()) {
        const std::size_t current = queue.front();
        queue.pop();
        for (const std::size_t next : adjacency[current]) {
          if (!visited[next]) {
            visited[next] = true;
            queue.push(next);
          }
        }
      }
    }
    return components;
  }

  void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr message)
  {
    if (query_mode_) {
      return;
    }
    const std::size_t count = std::min(message->name.size(), message->position.size());
    for (std::size_t i = 0; i < count; ++i) {
      latest_joint_positions_[message->name[i]] = message->position[i];
    }
    dirty_ = true;
  }

  bool applyEnvironment(
    const om6dof_dd_gng::msg::EnvironmentGraph & message,
    bool strict,
    std::string & error)
  {
    if (!message.header.frame_id.empty() && message.header.frame_id != world_frame_) {
      error = "environment_frame_mismatch";
      return false;
    }

    std::vector<reach::Target> targets;
    std::vector<reach::Point3> raw_obstacle_points;
    std::unordered_map<std::uint32_t, std::pair<reach::Point3, bool>> node_by_id;
    node_by_id.reserve(message.nodes.size());
    for (const auto & node : message.nodes) {
      const reach::Point3 point{node.position.x, node.position.y, node.position.z};
      if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) {
        error = "environment_contains_nonfinite_point";
        return false;
      }
      const bool is_target = node.class_id >= 0;
      const auto inserted = node_by_id.emplace(node.id, std::make_pair(point, is_target));
      if (!inserted.second) {
        error = "environment_contains_duplicate_node_id";
        return false;
      }
      if (is_target) {
        targets.push_back({node.id, point});
      } else {
        raw_obstacle_points.push_back(point);
      }
    }

    std::vector<reach::Segment> raw_obstacle_segments;
    raw_obstacle_segments.reserve(message.edges.size());
    for (const auto & edge : message.edges) {
      const auto source = node_by_id.find(edge.source_id);
      const auto target = node_by_id.find(edge.target_id);
      if (source == node_by_id.end() || target == node_by_id.end()) {
        if (strict) {
          error = "environment_edge_references_missing_node";
          return false;
        }
        continue;
      }
      if (source->second.second || target->second.second) {
        continue;
      }
      raw_obstacle_segments.push_back({source->second.first, target->second.first});
    }

    std::vector<reach::Point3> obstacle_points;
    for (const reach::Point3 & obstacle : raw_obstacle_points) {
      bool belongs_to_target_cluster = false;
      for (const reach::Target & target : targets) {
        if (reach::distance(obstacle, target.position) < target_exclusion_radius_) {
          belongs_to_target_cluster = true;
          break;
        }
      }
      if (!belongs_to_target_cluster) {
        obstacle_points.push_back(obstacle);
      }
    }

    std::vector<reach::Segment> obstacle_segments;
    for (const reach::Segment & obstacle : raw_obstacle_segments) {
      bool belongs_to_target_cluster = false;
      for (const reach::Target & target : targets) {
        if (reach::pointSegmentDistance(target.position, obstacle) < target_exclusion_radius_) {
          belongs_to_target_cluster = true;
          break;
        }
      }
      if (!belongs_to_target_cluster) {
        obstacle_segments.push_back(obstacle);
      }
    }

    targets_ = std::move(targets);
    obstacle_points_ = std::move(obstacle_points);
    obstacle_segments_ = std::move(obstacle_segments);
    if (initialized_) {
      rebuildExactEnvironmentScene();
      exact_blocked_edges_.assign(edges_.size(), false);
    }
    error.clear();
    return true;
  }

  bool orderedJoints(
    const std::vector<std::string> & names,
    const std::vector<double> & values,
    std::vector<double> & ordered,
    std::string & error)
    const
  {
    if (names.size() != values.size()) {
      error = "joint_name_position_size_mismatch";
      return false;
    }
    std::unordered_map<std::string, double> positions;
    positions.reserve(names.size());
    for (std::size_t i = 0U; i < names.size(); ++i) {
      const std::string & name = names[i];
      const double position = values[i];
      if (!std::isfinite(position) ||
        std::find(joint_names_.begin(), joint_names_.end(), name) == joint_names_.end())
      {
        error = "joint_state_contains_invalid_joint";
        return false;
      }
      if (!positions.emplace(name, position).second) {
        error = "joint_state_contains_duplicate_joint";
        return false;
      }
    }
    if (positions.size() != joint_names_.size() ||
      !std::all_of(
        joint_names_.begin(), joint_names_.end(),
        [&positions](const std::string & name) {return positions.count(name) == 1U;}))
    {
      error = "joint_state_is_incomplete";
      return false;
    }
    ordered.clear();
    ordered.reserve(joint_names_.size());
    for (const std::string & name : joint_names_) {
      ordered.push_back(positions.at(name));
    }
    error.clear();
    return true;
  }

  bool applyBenchmarkStart(
    const sensor_msgs::msg::JointState & start_state,
    std::string & error)
  {
    if (!initialized_) {
      error = "graph_not_initialized";
      return false;
    }
    std::vector<double> ordered;
    if (!orderedJoints(start_state.name, start_state.position, ordered, error)) {
      return false;
    }
    latest_joint_positions_.clear();
    for (std::size_t i = 0U; i < joint_names_.size(); ++i) {
      latest_joint_positions_[joint_names_[i]] = ordered[i];
    }
    return true;
  }

  void publishQueryError(const std::string & error)
  {
    last_plan_ = reach::PlanResult{};
    plan_reason_ = "invalid_reachability_query:" + error;
    last_planning_time_ms_ = 0.0;
    last_exact_collision_valid_ = false;
    last_exact_state_checks_ = 0U;
    last_exact_replans_ = 0U;
    last_exact_validation_time_ms_ = 0.0;
    last_start_connection_cost_ = -1.0;
    blocked_nodes_.clear();
    blocked_edges_.clear();
    intersection_nodes_.clear();
    dirty_ = false;
    publishPlan(currentJoints());
  }

  void queryCallback(const om6dof_dd_gng::msg::ReachabilityQuery::SharedPtr message)
  {
    if (!query_mode_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Ignoring ReachabilityQuery because query_mode is disabled");
      return;
    }
    active_query_id_ = message->query_id;
    active_scene_id_ = message->scene_id;
    active_requested_target_id_ = message->target_environment_node_id;
    active_requested_target_position_ = {
      message->target_position.x,
      message->target_position.y,
      message->target_position.z};
    std::string error;
    if (message->query_id == 0U) {
      publishQueryError("query_id_must_be_nonzero");
      return;
    }
    if (message->scene_id.empty()) {
      publishQueryError("scene_id_must_not_be_empty");
      return;
    }
    if (!message->header.frame_id.empty() && message->header.frame_id != world_frame_) {
      publishQueryError("query_frame_mismatch");
      return;
    }
    if (!applyBenchmarkStart(message->start_state, error) ||
      !applyEnvironment(message->environment, true, error))
    {
      publishQueryError(error);
      return;
    }
    if (!std::isfinite(active_requested_target_position_.x) ||
      !std::isfinite(active_requested_target_position_.y) ||
      !std::isfinite(active_requested_target_position_.z))
    {
      publishQueryError("requested_target_position_is_nonfinite");
      return;
    }
    if (targets_.size() != 1U ||
      targets_.front().environment_node_id != active_requested_target_id_ ||
      reach::distance(targets_.front().position, active_requested_target_position_) > 1.0e-9)
    {
      publishQueryError("requested_target_does_not_match_environment");
      return;
    }
    dirty_ = true;
    updatePlanAndMarkers();
  }

  void environmentCallback(const om6dof_dd_gng::msg::EnvironmentGraph::SharedPtr message)
  {
    if (query_mode_) {
      return;
    }
    active_query_id_ = 0U;
    active_scene_id_.clear();
    active_requested_target_id_ = std::numeric_limits<std::uint32_t>::max();
    active_requested_target_position_ = {};
    std::string error;
    if (!applyEnvironment(*message, false, error)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Ignoring invalid environment graph: %s", error.c_str());
      return;
    }
    dirty_ = true;
  }

  void rebuildExactEnvironmentScene()
  {
    if (!planning_scene_) {
      return;
    }
    if (planning_scene_->getWorld()->hasObject(kEnvironmentCollisionObjectId)) {
      moveit_msgs::msg::CollisionObject remove;
      remove.header.frame_id = world_frame_;
      remove.id = kEnvironmentCollisionObjectId;
      remove.operation = moveit_msgs::msg::CollisionObject::REMOVE;
      planning_scene_->processCollisionObjectMsg(remove);
    }

    moveit_msgs::msg::CollisionObject environment;
    environment.header.frame_id = world_frame_;
    environment.id = kEnvironmentCollisionObjectId;
    environment.operation = moveit_msgs::msg::CollisionObject::ADD;
    environment.primitives.reserve(obstacle_points_.size() + obstacle_segments_.size());
    environment.primitive_poses.reserve(environment.primitives.capacity());

    for (const reach::Point3 & point : obstacle_points_) {
      shape_msgs::msg::SolidPrimitive sphere;
      sphere.type = shape_msgs::msg::SolidPrimitive::SPHERE;
      sphere.dimensions.resize(1U);
      sphere.dimensions[shape_msgs::msg::SolidPrimitive::SPHERE_RADIUS] =
        exact_environment_point_radius_;
      geometry_msgs::msg::Pose pose;
      pose.position = toPointMessage(point);
      pose.orientation.w = 1.0;
      environment.primitives.push_back(std::move(sphere));
      environment.primitive_poses.push_back(pose);
    }
    for (const reach::Segment & segment : obstacle_segments_) {
      const Eigen::Vector3d a(segment.a.x, segment.a.y, segment.a.z);
      const Eigen::Vector3d b(segment.b.x, segment.b.y, segment.b.z);
      const Eigen::Vector3d direction = b - a;
      const double length = direction.norm();
      if (length <= 1.0e-6) {
        continue;
      }
      shape_msgs::msg::SolidPrimitive cylinder;
      cylinder.type = shape_msgs::msg::SolidPrimitive::CYLINDER;
      cylinder.dimensions.resize(2U);
      cylinder.dimensions[shape_msgs::msg::SolidPrimitive::CYLINDER_HEIGHT] = length;
      cylinder.dimensions[shape_msgs::msg::SolidPrimitive::CYLINDER_RADIUS] =
        exact_environment_edge_radius_;
      const Eigen::Quaterniond orientation = Eigen::Quaterniond::FromTwoVectors(
        Eigen::Vector3d::UnitZ(), direction / length);
      geometry_msgs::msg::Pose pose;
      pose.position.x = 0.5 * (segment.a.x + segment.b.x);
      pose.position.y = 0.5 * (segment.a.y + segment.b.y);
      pose.position.z = 0.5 * (segment.a.z + segment.b.z);
      pose.orientation.x = orientation.x();
      pose.orientation.y = orientation.y();
      pose.orientation.z = orientation.z();
      pose.orientation.w = orientation.w();
      environment.primitives.push_back(std::move(cylinder));
      environment.primitive_poses.push_back(pose);
    }
    if (!environment.primitives.empty() &&
      !planning_scene_->processCollisionObjectMsg(environment))
    {
      RCLCPP_ERROR(get_logger(), "Failed to update exact DD-GNG PlanningScene object");
    }

    exact_collision_matrix_ = std::make_unique<collision_detection::AllowedCollisionMatrix>(
      *strict_collision_matrix_);
    for (const std::string & ignored_link : exact_environment_ignored_links_) {
      if (!robot_model_->hasLinkModel(ignored_link)) {
        RCLCPP_WARN(
          get_logger(), "Ignoring unknown exact_environment_ignored_link '%s'",
          ignored_link.c_str());
        continue;
      }
      exact_collision_matrix_->setEntry(
        ignored_link, kEnvironmentCollisionObjectId, true);
    }
  }

  bool exactStateIsValid(const std::vector<double> & joints)
  {
    if (!exact_collision_enabled_) {
      return true;
    }
    const auto check_started = std::chrono::steady_clock::now();
    ++last_exact_state_checks_;
    moveit::core::RobotState state(robot_model_);
    state.setToDefaultValues();
    state.setJointGroupPositions(joint_model_group_, joints);
    state.update();
    collision_detection::CollisionRequest request;
    request.group_name.clear();
    collision_detection::CollisionResult result;
    planning_scene_->checkCollision(
      request, result, state,
      exact_collision_matrix_ ? *exact_collision_matrix_ : *strict_collision_matrix_);
    last_exact_validation_time_ms_ += std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - check_started).count();
    return !result.collision;
  }

  bool exactTransitionIsValid(
    const std::vector<double> & from,
    const std::vector<double> & to)
  {
    if (!exact_collision_enabled_) {
      return true;
    }
    double max_delta = 0.0;
    for (std::size_t i = 0U; i < from.size(); ++i) {
      max_delta = std::max(max_delta, std::abs(to[i] - from[i]));
    }
    const int steps = std::max(
      1, static_cast<int>(std::ceil(max_delta / exact_collision_step_)));
    std::vector<double> interpolated(from.size(), 0.0);
    for (int step = 0; step <= steps; ++step) {
      const double t = static_cast<double>(step) / static_cast<double>(steps);
      for (std::size_t i = 0U; i < from.size(); ++i) {
        interpolated[i] = from[i] + (to[i] - from[i]) * t;
      }
      if (!exactStateIsValid(interpolated)) {
        return false;
      }
    }
    return true;
  }

  std::optional<std::size_t> validatedStartNode(
    const std::vector<double> & current,
    const std::vector<bool> & blocked_nodes)
  {
    if (!stateIsValid(current)) {
      return std::nullopt;
    }
    if (bodySweepBlocked(bodySweepForState(current))) {
      return std::nullopt;
    }
    const reach::Node current_node = nodeFromJoints(0U, current);
    const auto ranked = reach::rankNodesByJointDistance(nodes_, current, joint_ranges_, blocked_nodes);
    const std::size_t candidate_count = std::min<std::size_t>(
      ranked.size(), static_cast<std::size_t>(start_connect_candidates_));
    for (std::size_t rank = 0; rank < candidate_count; ++rank) {
      const std::size_t candidate = ranked[rank];
      if (reach::normalizedJointDistance(nodes_[candidate].joints, current, joint_ranges_) >
        start_max_normalized_joint_distance_)
      {
        continue;
      }
      if (reach::distance(nodes_[candidate].position, current_node.position) >
        max_cartesian_edge_length_ * 1.5)
      {
        continue;
      }
      if (edgeStateIsValid(current, nodes_[candidate].joints) &&
        !bodySweepBlocked(bodySweepForTransition(current, nodes_[candidate].joints)) &&
        exactTransitionIsValid(current, nodes_[candidate].joints))
      {
        return candidate;
      }
    }
    return std::nullopt;
  }

  void planWithExactValidation(std::size_t start)
  {
    bool rejected_by_exact_collision = false;
    for (int attempt = 0; attempt <= exact_max_replans_; ++attempt) {
      last_plan_ = reach::planToNearestTarget(
        nodes_, edges_, blocked_nodes_, blocked_edges_, start,
        targets_, target_intersection_radius_);
      if (!last_plan_.success) {
        if (!last_plan_.has_intersection) {
          plan_reason_ = "target_outside_reachability_intersection";
        } else if (rejected_by_exact_collision) {
          plan_reason_ = "target_intersection_exact_blocked_or_disconnected";
        } else {
          plan_reason_ = "target_intersection_blocked_or_disconnected";
        }
        return;
      }
      if (!exact_collision_enabled_) {
        last_exact_collision_valid_ = false;
        plan_reason_ = "path_ready_capsule_validated_preview_only";
        return;
      }

      std::size_t failed_edge = reach::kInvalidIndex;
      for (std::size_t i = 1U; i < last_plan_.path.size(); ++i) {
        const std::size_t a = last_plan_.path[i - 1U];
        const std::size_t b = last_plan_.path[i];
        const auto found = edge_index_by_key_.find(edgeKey(a, b));
        if (found == edge_index_by_key_.end()) {
          failed_edge = reach::kInvalidIndex;
          plan_reason_ = "internal_path_edge_missing";
          last_plan_.success = false;
          return;
        }
        if (!exactTransitionIsValid(nodes_[a].joints, nodes_[b].joints)) {
          failed_edge = found->second;
          break;
        }
      }
      if (failed_edge == reach::kInvalidIndex) {
        last_exact_collision_valid_ = true;
        plan_reason_ = "path_ready_exact_validated_preview_only";
        return;
      }

      rejected_by_exact_collision = true;
      blocked_edges_[failed_edge] = true;
      if (failed_edge < exact_blocked_edges_.size()) {
        exact_blocked_edges_[failed_edge] = true;
      }
      ++last_exact_replans_;
    }
    last_plan_.success = false;
    plan_reason_ = "exact_collision_replan_exhausted";
  }

  void updatePlanAndMarkers()
  {
    const auto planning_started = std::chrono::steady_clock::now();
    last_exact_collision_valid_ = false;
    last_exact_state_checks_ = 0U;
    last_exact_replans_ = 0U;
    last_exact_validation_time_ms_ = 0.0;
    last_start_connection_cost_ = -1.0;
    dirty_ = false;
    blocked_nodes_ = fullBodyBlockedNodes();
    blocked_edges_ = fullBodyBlockedEdges(blocked_nodes_);
    for (std::size_t i = 0U; i < blocked_edges_.size() && i < exact_blocked_edges_.size(); ++i) {
      blocked_edges_[i] = blocked_edges_[i] || exact_blocked_edges_[i];
    }
    intersection_nodes_ = reach::targetIntersectionMask(
      nodes_, targets_, target_intersection_radius_);

    last_plan_ = reach::PlanResult{};
    plan_reason_.clear();
    const auto measured = currentJoints();
    if (query_mode_ && active_query_id_ == 0U) {
      plan_reason_ = "waiting_for_reachability_query";
    } else if (!measured) {
      plan_reason_ = "waiting_for_complete_joint_state";
    } else if (!stateIsValid(*measured)) {
      plan_reason_ = "current_joint_state_invalid_or_self_colliding";
    } else if (bodySweepBlocked(bodySweepForState(*measured))) {
      plan_reason_ = "current_full_body_intersects_environment";
    } else {
      const auto start = validatedStartNode(*measured, blocked_nodes_);
      if (!start) {
        plan_reason_ = "current_state_cannot_connect_to_roadmap";
      } else if (targets_.empty()) {
        last_start_connection_cost_ = reach::jointPathCost(
          *measured, nodes_[*start].joints);
        last_plan_.start = *start;
        last_plan_.has_start = true;
        plan_reason_ = "waiting_for_labeled_environment_target";
      } else {
        last_start_connection_cost_ = reach::jointPathCost(
          *measured, nodes_[*start].joints);
        planWithExactValidation(*start);
      }
    }

    last_planning_time_ms_ = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - planning_started).count();

    publishPlan(measured);
    publishMarkers();
  }

  void publishGraphData()
  {
    om6dof_dd_gng::msg::ReachabilityGraph message;
    message.header.stamp = now();
    message.header.frame_id = world_frame_;
    message.graph_method = graph_method_;
    message.graph_revision = graph_revision_;
    message.expanded_urdf_sha256 = expanded_urdf_sha256_;
    message.srdf_sha256 = srdf_sha256_;
    message.reachability_parameters_sha256 = reachability_parameters_sha256_;
    message.group_name = group_name_;
    message.end_effector_link = end_effector_link_;
    message.joint_names = joint_names_;
    message.joint_lower_bounds = lower_bounds_;
    message.joint_upper_bounds = upper_bounds_;
    message.requested_node_count = static_cast<std::uint32_t>(sample_count_);
    message.anchor_node_count = static_cast<std::uint32_t>(last_anchor_node_count_);
    message.prototype_budget = static_cast<std::uint32_t>(last_prototype_budget_);
    message.prototype_node_count = static_cast<std::uint32_t>(last_prototype_node_count_);
    message.requested_guard_node_count = static_cast<std::uint32_t>(
      last_requested_guard_node_count_);
    message.guard_node_count = static_cast<std::uint32_t>(last_guard_node_count_);
    message.fill_sample_node_count = static_cast<std::uint32_t>(last_fill_sample_node_count_);
    message.candidate_attempts = static_cast<std::uint32_t>(last_candidate_attempts_);
    message.halton_start_index = static_cast<std::uint32_t>(halton_start_index_);
    message.sample_stream_seed = static_cast<std::uint64_t>(sample_stream_seed_);
    message.sample_stream_type = sample_stream_seed_ == 0 ?
      "halton" : "digit_permuted_halton";
    message.gng_training_sample_count = static_cast<std::uint32_t>(
      last_gng_training_sample_count_);
    message.effective_guard_fraction =
      graph_method_ == "guarded_gng" ? gng_guard_fraction_ : 0.0;
    message.connected_components = static_cast<std::uint32_t>(connectedComponentCount());
    message.build_time_ms = last_build_time_ms_;
    message.nodes.reserve(nodes_.size());
    for (const reach::Node & node : nodes_) {
      om6dof_dd_gng::msg::ReachabilityNode output;
      output.id = node.id;
      output.pose = toPoseMessage(node);
      output.joint_positions = node.joints;
      message.nodes.push_back(std::move(output));
    }
    message.edges.reserve(edges_.size());
    for (const reach::Edge & edge : edges_) {
      om6dof_dd_gng::msg::TopologyEdge output;
      output.source_id = nodes_[edge.a].id;
      output.target_id = nodes_[edge.b].id;
      output.cost = edge.cost;
      message.edges.push_back(output);
    }
    graph_pub_->publish(message);
  }

  void publishPlan(const std::optional<std::vector<double>> & measured)
  {
    constexpr std::uint32_t invalid_id = std::numeric_limits<std::uint32_t>::max();
    om6dof_dd_gng::msg::ReachabilityPlan message;
    message.header.stamp = now();
    message.header.frame_id = world_frame_;
    message.graph_method = graph_method_;
    message.graph_revision = graph_revision_;
    message.query_id = active_query_id_;
    message.scene_id = active_scene_id_;
    message.requested_target_environment_node_id = active_requested_target_id_;
    message.requested_target_position = toPointMessage(active_requested_target_position_);
    message.valid = last_plan_.success;
    message.reason = plan_reason_;
    message.blocked_node_count = static_cast<std::uint32_t>(
      std::count(blocked_nodes_.begin(), blocked_nodes_.end(), true));
    message.blocked_edge_count = static_cast<std::uint32_t>(
      std::count(blocked_edges_.begin(), blocked_edges_.end(), true));
    message.planning_time_ms = last_planning_time_ms_;
    message.exact_collision_valid = last_exact_collision_valid_;
    message.exact_state_checks = last_exact_state_checks_;
    message.exact_replans = last_exact_replans_;
    message.exact_validation_time_ms = last_exact_validation_time_ms_;
    message.start_node_id = last_plan_.has_start ? nodes_[last_plan_.start].id : invalid_id;
    message.goal_node_id = last_plan_.success ? nodes_[last_plan_.goal].id : invalid_id;
    message.target_environment_node_id =
      last_plan_.success ? targets_[last_plan_.target_index].environment_node_id : invalid_id;
    message.target_distance = last_plan_.success ? last_plan_.target_distance : -1.0;
    message.graph_cost = last_plan_.success ? last_plan_.graph_cost : -1.0;
    message.start_connection_cost = last_start_connection_cost_;
    message.total_joint_path_cost = last_plan_.success && last_start_connection_cost_ >= 0.0 ?
      last_start_connection_cost_ + last_plan_.graph_cost : -1.0;

    message.joint_path_preview.header = message.header;
    message.joint_path_preview.joint_names = joint_names_;
    message.end_effector_path.header = message.header;

    double elapsed = 0.0;
    std::vector<double> previous;
    if (measured) {
      previous = *measured;
      trajectory_msgs::msg::JointTrajectoryPoint point;
      point.positions = previous;
      point.time_from_start = rclcpp::Duration::from_seconds(0.0);
      message.joint_path_preview.points.push_back(point);

      const reach::Node current_pose = nodeFromJoints(0U, previous);
      geometry_msgs::msg::PoseStamped pose;
      pose.header = message.header;
      pose.pose = toPoseMessage(current_pose);
      message.end_effector_path.poses.push_back(pose);
    }

    if (last_plan_.success) {
      for (const std::size_t node_index : last_plan_.path) {
        message.reachability_node_ids.push_back(nodes_[node_index].id);
        if (!previous.empty()) {
          double max_delta = 0.0;
          for (std::size_t joint = 0; joint < previous.size(); ++joint) {
            max_delta = std::max(
              max_delta, std::abs(nodes_[node_index].joints[joint] - previous[joint]));
          }
          elapsed += std::max(0.05, max_delta / preview_joint_velocity_);
        }
        trajectory_msgs::msg::JointTrajectoryPoint point;
        point.positions = nodes_[node_index].joints;
        point.time_from_start = rclcpp::Duration::from_seconds(elapsed);
        message.joint_path_preview.points.push_back(point);
        previous = nodes_[node_index].joints;

        geometry_msgs::msg::PoseStamped pose;
        pose.header = message.header;
        pose.pose = toPoseMessage(nodes_[node_index]);
        message.end_effector_path.poses.push_back(pose);
      }
    }
    plan_pub_->publish(message);
    path_pub_->publish(message.end_effector_path);
  }

  void publishMarkers()
  {
    const auto stamp = now();
    visualization_msgs::msg::MarkerArray array;
    visualization_msgs::msg::Marker clear;
    clear.header.frame_id = world_frame_;
    clear.header.stamp = stamp;
    clear.action = visualization_msgs::msg::Marker::DELETEALL;
    array.markers.push_back(clear);

    visualization_msgs::msg::Marker node_marker;
    node_marker.header.frame_id = world_frame_;
    node_marker.header.stamp = stamp;
    node_marker.ns = "reachability_nodes";
    node_marker.id = 0;
    node_marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
    node_marker.action = visualization_msgs::msg::Marker::ADD;
    node_marker.pose.orientation.w = 1.0;
    node_marker.scale.x = node_marker_scale_;
    node_marker.scale.y = node_marker_scale_;
    node_marker.scale.z = node_marker_scale_;
    node_marker.points.reserve(nodes_.size());
    node_marker.colors.reserve(nodes_.size());
    for (std::size_t i = 0; i < nodes_.size(); ++i) {
      node_marker.points.push_back(toPointMessage(nodes_[i].position));
      if (i < blocked_nodes_.size() && blocked_nodes_[i]) {
        node_marker.colors.push_back(color(0.95F, 0.15F, 0.12F, 0.18F));
      } else if (i < intersection_nodes_.size() && intersection_nodes_[i]) {
        node_marker.colors.push_back(color(0.15F, 1.0F, 0.25F, 1.0F));
      } else {
        node_marker.colors.push_back(color(0.10F, 0.80F, 0.95F, 0.38F));
      }
    }
    array.markers.push_back(node_marker);

    visualization_msgs::msg::Marker edge_marker;
    edge_marker.header = node_marker.header;
    edge_marker.ns = "reachability_edges";
    edge_marker.id = 1;
    edge_marker.type = visualization_msgs::msg::Marker::LINE_LIST;
    edge_marker.action = visualization_msgs::msg::Marker::ADD;
    edge_marker.pose.orientation.w = 1.0;
    edge_marker.scale.x = edge_marker_width_;
    edge_marker.color = color(0.05F, 0.65F, 0.75F, 0.28F);
    for (std::size_t edge_index = 0; edge_index < edges_.size(); ++edge_index) {
      if (edge_index < blocked_edges_.size() && blocked_edges_[edge_index]) {
        continue;
      }
      edge_marker.points.push_back(toPointMessage(nodes_[edges_[edge_index].a].position));
      edge_marker.points.push_back(toPointMessage(nodes_[edges_[edge_index].b].position));
    }
    array.markers.push_back(edge_marker);

    if (last_plan_.has_start) {
      visualization_msgs::msg::Marker start;
      start.header = node_marker.header;
      start.ns = "reachability_start";
      start.id = 2;
      start.type = visualization_msgs::msg::Marker::SPHERE;
      start.action = visualization_msgs::msg::Marker::ADD;
      start.pose.position = toPointMessage(nodes_[last_plan_.start].position);
      start.pose.orientation.w = 1.0;
      start.scale.x = node_marker_scale_ * 2.2;
      start.scale.y = start.scale.x;
      start.scale.z = start.scale.x;
      start.color = color(0.10F, 1.0F, 0.85F, 1.0F);
      array.markers.push_back(start);
    }

    if (last_plan_.success) {
      visualization_msgs::msg::Marker goal;
      goal.header = node_marker.header;
      goal.ns = "reachability_goal";
      goal.id = 3;
      goal.type = visualization_msgs::msg::Marker::SPHERE;
      goal.action = visualization_msgs::msg::Marker::ADD;
      goal.pose.position = toPointMessage(nodes_[last_plan_.goal].position);
      goal.pose.orientation.w = 1.0;
      goal.scale.x = node_marker_scale_ * 2.5;
      goal.scale.y = goal.scale.x;
      goal.scale.z = goal.scale.x;
      goal.color = color(1.0F, 0.15F, 0.85F, 1.0F);
      array.markers.push_back(goal);

      visualization_msgs::msg::Marker path;
      path.header = node_marker.header;
      path.ns = "reachability_path";
      path.id = 4;
      path.type = visualization_msgs::msg::Marker::LINE_STRIP;
      path.action = visualization_msgs::msg::Marker::ADD;
      path.pose.orientation.w = 1.0;
      path.scale.x = path_marker_width_;
      path.color = color(1.0F, 0.85F, 0.05F, 1.0F);
      for (const std::size_t node_index : last_plan_.path) {
        path.points.push_back(toPointMessage(nodes_[node_index].position));
      }
      array.markers.push_back(path);
    }
    marker_pub_->publish(array);
  }

  void validateSceneCallback(
    const std::shared_ptr<om6dof_dd_gng::srv::ValidateReachabilityScene::Request> request,
    std::shared_ptr<om6dof_dd_gng::srv::ValidateReachabilityScene::Response> response)
  {
    response->evaluated = false;
    if (!initialized_) {
      response->reason = "graph_not_initialized";
      return;
    }
    if (!query_mode_) {
      response->reason = "scene_validation_requires_query_mode";
      return;
    }
    if (!std::isfinite(request->hit_fraction) ||
      request->hit_fraction < 0.0 || request->hit_fraction > 1.0)
    {
      response->reason = "hit_fraction_must_be_within_zero_and_one";
      return;
    }
    std::string error;
    std::vector<double> start;
    std::vector<double> target;
    std::vector<double> detour;
    if (!orderedJoints(request->joint_names, request->start_joint_positions, start, error) ||
      !orderedJoints(request->joint_names, request->target_joint_positions, target, error) ||
      !orderedJoints(request->joint_names, request->detour_joint_positions, detour, error))
    {
      response->reason = error;
      return;
    }

    std::vector<double> hit(start.size(), 0.0);
    for (std::size_t i = 0U; i < start.size(); ++i) {
      hit[i] = start[i] + (target[i] - start[i]) * request->hit_fraction;
    }
    response->start_pose = toPoseMessage(nodeFromJoints(0U, start));
    response->target_pose = toPoseMessage(nodeFromJoints(0U, target));
    response->hit_pose = toPoseMessage(nodeFromJoints(0U, hit));
    response->start_self_valid = stateIsValid(start);
    response->target_self_valid = stateIsValid(target);
    response->detour_self_valid = stateIsValid(detour);

    om6dof_dd_gng::msg::EnvironmentGraph empty_environment;
    empty_environment.header.frame_id = world_frame_;
    if (!applyEnvironment(empty_environment, true, error)) {
      response->reason = "failed_to_clear_environment:" + error;
      return;
    }
    response->clear_capsule_direct_valid =
      response->start_self_valid && response->target_self_valid &&
      edgeStateIsValid(start, target) &&
      !bodySweepBlocked(bodySweepForTransition(start, target));
    response->clear_exact_direct_valid =
      response->start_self_valid && response->target_self_valid &&
      exactTransitionIsValid(start, target);

    if (std::any_of(
        request->environment.nodes.begin(), request->environment.nodes.end(),
        [](const auto & node) {return node.class_id >= 0;}))
    {
      response->reason = "scene_validation_environment_must_contain_obstacles_only";
      return;
    }
    if (!applyEnvironment(request->environment, true, error)) {
      response->reason = "invalid_dynamic_environment:" + error;
      return;
    }
    auto dynamicStateValid = [this](const std::vector<double> & joints) {
        return stateIsValid(joints) &&
               !bodySweepBlocked(bodySweepForState(joints)) &&
               exactStateIsValid(joints);
      };
    response->dynamic_start_valid = dynamicStateValid(start);
    response->dynamic_target_valid = dynamicStateValid(target);
    response->dynamic_detour_state_valid = dynamicStateValid(detour);
    response->dynamic_capsule_direct_valid =
      edgeStateIsValid(start, target) &&
      !bodySweepBlocked(bodySweepForTransition(start, target));
    response->dynamic_exact_direct_valid = exactTransitionIsValid(start, target);
    response->dynamic_capsule_detour_valid =
      edgeStateIsValid(start, detour) && edgeStateIsValid(detour, target) &&
      !bodySweepBlocked(bodySweepForTransition(start, detour)) &&
      !bodySweepBlocked(bodySweepForTransition(detour, target));
    response->dynamic_exact_detour_valid =
      exactTransitionIsValid(start, detour) && exactTransitionIsValid(detour, target);
    response->evaluated = true;
    response->reason = "evaluated";
  }

  void rebuildCallback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    if (!initialized_) {
      initialization_failed_ = false;
      response->success = initializeModelAndGraph();
    } else {
      response->success = buildGraph();
      if (response->success) {
        publishGraphData();
        dirty_ = true;
      }
    }
    std::ostringstream message;
    message << (response->success ? "rebuilt" : "failed") << ": "
            << nodes_.size() << " nodes, " << edges_.size() << " edges";
    response->message = message.str();
  }

  void planCallback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    if (!initialized_) {
      response->success = false;
      response->message = "reachability graph is not initialized";
      return;
    }
    dirty_ = true;
    updatePlanAndMarkers();
    response->success = last_plan_.success;
    response->message = plan_reason_;
  }

  // Model and static roadmap
  robot_model_loader::RobotModelLoaderPtr model_loader_;
  moveit::core::RobotModelPtr robot_model_;
  const moveit::core::JointModelGroup * joint_model_group_ = nullptr;
  planning_scene::PlanningScenePtr planning_scene_;
  std::unique_ptr<collision_detection::AllowedCollisionMatrix> strict_collision_matrix_;
  std::unique_ptr<collision_detection::AllowedCollisionMatrix> exact_collision_matrix_;
  std::vector<std::string> joint_names_;
  std::vector<double> lower_bounds_;
  std::vector<double> upper_bounds_;
  std::vector<double> joint_ranges_;
  std::vector<std::vector<unsigned int>> halton_digit_permutations_;
  std::vector<reach::Node> nodes_;
  std::vector<reach::Edge> edges_;
  std::unordered_map<std::uint64_t, std::size_t> edge_index_by_key_;
  std::vector<BodySweep> node_body_sweeps_;
  std::vector<BodySweep> edge_body_sweeps_;

  // Dynamic environment and plan state
  std::unordered_map<std::string, double> latest_joint_positions_;
  std::vector<reach::Target> targets_;
  std::vector<reach::Point3> obstacle_points_;
  std::vector<reach::Segment> obstacle_segments_;
  std::vector<bool> blocked_nodes_;
  std::vector<bool> blocked_edges_;
  std::vector<bool> exact_blocked_edges_;
  std::vector<bool> intersection_nodes_;
  reach::PlanResult last_plan_;
  std::string plan_reason_ = "initializing";
  std::uint64_t graph_revision_ = 0U;
  std::uint64_t active_query_id_ = 0U;
  std::string active_scene_id_;
  std::uint32_t active_requested_target_id_ = std::numeric_limits<std::uint32_t>::max();
  reach::Point3 active_requested_target_position_;
  double last_planning_time_ms_ = 0.0;
  double last_start_connection_cost_ = -1.0;
  double last_build_time_ms_ = 0.0;
  std::size_t last_anchor_node_count_ = 0U;
  std::size_t last_prototype_budget_ = 0U;
  std::size_t last_prototype_node_count_ = 0U;
  std::size_t last_gng_training_sample_count_ = 0U;
  std::size_t last_guard_node_count_ = 0U;
  std::size_t last_fill_sample_node_count_ = 0U;
  std::size_t last_candidate_attempts_ = 0U;
  std::size_t last_requested_guard_node_count_ = 0U;
  bool last_exact_collision_valid_ = false;
  std::uint32_t last_exact_state_checks_ = 0U;
  std::uint32_t last_exact_replans_ = 0U;
  double last_exact_validation_time_ms_ = 0.0;
  bool initialized_ = false;
  bool initialization_failed_ = false;
  bool dirty_ = true;

  // Parameters
  std::string group_name_;
  std::string end_effector_link_;
  std::string world_frame_;
  std::string expanded_urdf_sha256_;
  std::string srdf_sha256_;
  std::string reachability_parameters_sha256_;
  std::string graph_method_ = "gng";
  int sample_count_ = 800;
  int max_sampling_attempts_ = 20000;
  int halton_start_index_ = 17;
  std::int64_t sample_stream_seed_ = 0;
  int gng_training_samples_ = 4000;
  int gng_max_epochs_ = 4;
  int gng_insert_interval_ = 20;
  int gng_max_edge_age_ = 50;
  double gng_winner_learning_rate_ = 0.05;
  double gng_neighbor_learning_rate_ = 0.0006;
  double gng_error_reduction_ = 0.5;
  double gng_error_decay_ = 0.995;
  double gng_guard_fraction_ = 0.25;
  int neighbors_ = 10;
  double max_normalized_joint_distance_ = 0.75;
  double max_cartesian_edge_length_ = 0.14;
  double edge_validation_step_ = 0.15;
  bool strict_self_collision_ = true;
  double target_intersection_radius_ = 0.05;
  double obstacle_clearance_ = 0.035;
  double target_exclusion_radius_ = 0.055;
  int start_connect_candidates_ = 20;
  double start_max_normalized_joint_distance_ = 0.85;
  double preview_joint_velocity_ = 0.35;
  double planning_period_sec_ = 0.5;
  double body_collision_step_ = 0.08;
  int body_collision_first_edge_ = 1;
  std::unordered_map<std::string, double> body_radii_;
  bool exact_collision_enabled_ = true;
  double exact_collision_step_ = 0.05;
  double exact_environment_point_radius_ = 0.012;
  double exact_environment_edge_radius_ = 0.006;
  int exact_max_replans_ = 20;
  std::vector<std::string> exact_environment_ignored_links_{"link1"};
  double node_marker_scale_ = 0.012;
  double edge_marker_width_ = 0.002;
  double path_marker_width_ = 0.008;
  bool query_mode_ = false;
  std::string environment_graph_topic_;
  std::string query_topic_;
  std::string joint_state_topic_;
  std::string graph_data_topic_;
  std::string marker_topic_;
  std::string plan_topic_;
  std::string path_topic_;
  std::string rebuild_service_name_;
  std::string plan_service_name_;
  std::string scene_validation_service_name_;

  // ROS interfaces: no controller publisher/action client by design.
  rclcpp::Publisher<om6dof_dd_gng::msg::ReachabilityGraph>::SharedPtr graph_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::Publisher<om6dof_dd_gng::msg::ReachabilityPlan>::SharedPtr plan_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Subscription<om6dof_dd_gng::msg::EnvironmentGraph>::SharedPtr environment_sub_;
  rclcpp::Subscription<om6dof_dd_gng::msg::ReachabilityQuery>::SharedPtr query_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr rebuild_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr plan_service_;
  rclcpp::Service<om6dof_dd_gng::srv::ValidateReachabilityScene>::SharedPtr
  scene_validation_service_;
  rclcpp::TimerBase::SharedPtr maintenance_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  int result = 0;
  try {
    rclcpp::spin(std::make_shared<ReachabilityGraphNode>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(
      rclcpp::get_logger("reachability_graph_node"), "Fatal error: %s", error.what());
    result = 1;
  }
  rclcpp::shutdown();
  return result;
}
