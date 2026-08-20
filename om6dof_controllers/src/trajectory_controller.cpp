// Copyright 2026 OM6DOF maintainers.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "om6dof_controllers/trajectory_controller.hpp"

#include <algorithm>
#include <cmath>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "control_msgs/msg/joint_tolerance.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "om6dof_controllers/controller_utils.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace om6dof_controllers
{
namespace
{
constexpr double kDefaultGoalTimeTolerance = 0.0;
}  // namespace

controller_interface::CallbackReturn TrajectoryController::on_init()
{
  try {
    auto_declare<std::vector<std::string>>("joints", {});
    auto_declare<std::string>("command_interface", "position");

    auto_declare<std::vector<double>>("gains.p", {});
    auto_declare<std::vector<double>>("gains.d", {});
    auto_declare<std::vector<double>>("effort_scale", {});
    auto_declare<std::vector<double>>("max_effort", {});

    auto_declare<double>("constraints.goal_time", kDefaultGoalTimeTolerance);
    auto_declare<double>("constraints.stopped_velocity_tolerance", 0.01);

    auto_declare<bool>("gravity_feedforward", false);
    auto_declare<std::vector<double>>("gravity_gain", {});
    auto_declare<std::string>("base_link", "world");
    auto_declare<std::string>("tip_link", "end_effector_link");
    auto_declare<std::vector<double>>("gravity", {0.0, 0.0, -9.80665});

    auto_declare<double>("state_publish_rate", 50.0);
    auto_declare<double>("action_monitor_rate", 20.0);

    auto_declare<std::string>("robot_description", "");
    auto_declare<std::string>("robot_description_topic", "/robot_description");
    auto_declare<double>("robot_description_timeout", 10.0);
  } catch (const std::exception & exception) {
    RCLCPP_ERROR(
      get_node()->get_logger(), "exception while declaring parameters: %s", exception.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
TrajectoryController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration configuration;
  configuration.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  const std::string interface = command_mode_ == CommandMode::Effort ?
    hardware_interface::HW_IF_EFFORT :
    hardware_interface::HW_IF_POSITION;
  for (const auto & joint : joint_names_) {
    configuration.names.push_back(joint + "/" + interface);
  }
  return configuration;
}

controller_interface::InterfaceConfiguration
TrajectoryController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration configuration;
  configuration.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto & joint : joint_names_) {
    configuration.names.push_back(joint + "/" + hardware_interface::HW_IF_POSITION);
    configuration.names.push_back(joint + "/" + hardware_interface::HW_IF_VELOCITY);
  }
  return configuration;
}

controller_interface::CallbackReturn TrajectoryController::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  const auto logger = get_node()->get_logger();

  joint_names_ = get_node()->get_parameter("joints").as_string_array();
  if (joint_names_.empty()) {
    RCLCPP_ERROR(logger, "parameter 'joints' is empty");
    return controller_interface::CallbackReturn::ERROR;
  }
  const size_t n = joint_names_.size();

  const std::string interface = get_node()->get_parameter("command_interface").as_string();
  if (interface == "position") {
    command_mode_ = CommandMode::Position;
  } else if (interface == "effort") {
    command_mode_ = CommandMode::Effort;
  } else {
    RCLCPP_ERROR(
      logger, "parameter 'command_interface' is '%s'; it must be 'position' or 'effort'",
      interface.c_str());
    return controller_interface::CallbackReturn::ERROR;
  }

  std::string error;
  const auto read = [&](const char * name, double fallback, std::vector<double> & out) {
      return perJointParameter(
        name, get_node()->get_parameter(name).as_double_array(), n, fallback, out, error);
    };

  if (!read("gains.p", 0.0, kp_) || !read("gains.d", 0.0, kd_) ||
    !read("effort_scale", 1.0, effort_scale_) || !read("max_effort", 0.0, max_effort_) ||
    !read("gravity_gain", 1.0, gravity_gain_))
  {
    RCLCPP_ERROR(logger, "%s", error.c_str());
    return controller_interface::CallbackReturn::ERROR;
  }

  if (command_mode_ == CommandMode::Effort &&
    std::all_of(kp_.begin(), kp_.end(), [](double gain) {return gain <= 0.0;}))
  {
    RCLCPP_ERROR(
      logger, "effort mode with every 'gains.p' at zero would never track the trajectory");
    return controller_interface::CallbackReturn::ERROR;
  }

  // Per-joint tolerances are named after the joints, so they can only be
  // declared once the joint list is known.
  default_tolerances_.resize(n);
  for (size_t i = 0; i < n; ++i) {
    const std::string prefix = "constraints." + joint_names_[i];
    default_tolerances_[i].path = auto_declare<double>(prefix + ".trajectory", 0.0);
    default_tolerances_[i].goal = auto_declare<double>(prefix + ".goal", 0.0);
  }
  default_goal_time_tolerance_ = get_node()->get_parameter("constraints.goal_time").as_double();
  stopped_velocity_tolerance_ =
    get_node()->get_parameter("constraints.stopped_velocity_tolerance").as_double();

  gravity_feedforward_ = get_node()->get_parameter("gravity_feedforward").as_bool();
  if (gravity_feedforward_ && command_mode_ != CommandMode::Effort) {
    RCLCPP_WARN(
      logger, "'gravity_feedforward' does nothing in position mode; ignoring it");
    gravity_feedforward_ = false;
  }

  if (gravity_feedforward_) {
    const auto gravity_parameter = get_node()->get_parameter("gravity").as_double_array();
    if (gravity_parameter.size() != 3) {
      RCLCPP_ERROR(logger, "parameter 'gravity' needs exactly three values");
      return controller_interface::CallbackReturn::ERROR;
    }
    const std::array<double, 3> gravity{
      gravity_parameter[0], gravity_parameter[1], gravity_parameter[2]};

    const std::string urdf = fetchRobotDescription(
      get_node()->get_parameter("robot_description").as_string(), get_node()->get_namespace(),
      get_node()->get_parameter("robot_description_topic").as_string(),
      get_node()->get_parameter("robot_description_timeout").as_double(), error);
    if (urdf.empty()) {
      RCLCPP_ERROR(logger, "%s", error.c_str());
      return controller_interface::CallbackReturn::ERROR;
    }

    error = model_.configure(
      urdf, get_node()->get_parameter("base_link").as_string(),
      get_node()->get_parameter("tip_link").as_string(), joint_names_, gravity);
    if (!error.empty()) {
      RCLCPP_ERROR(logger, "gravity model: %s", error.c_str());
      return controller_interface::CallbackReturn::ERROR;
    }
  }

  positions_.assign(n, 0.0);
  velocities_.assign(n, 0.0);
  desired_positions_.assign(n, 0.0);
  desired_velocities_.assign(n, 0.0);
  gravity_torque_.assign(n, 0.0);
  hold_positions_.assign(n, 0.0);
  active_tolerances_ = default_tolerances_;
  active_goal_time_tolerance_ = default_goal_time_tolerance_;

  const double state_publish_rate = get_node()->get_parameter("state_publish_rate").as_double();
  state_publish_period_ = state_publish_rate > 0.0 ? 1.0 / state_publish_rate : 0.0;

  const double action_monitor_rate = get_node()->get_parameter("action_monitor_rate").as_double();
  action_monitor_period_ = rclcpp::Duration::from_seconds(
    action_monitor_rate > 0.0 ? 1.0 / action_monitor_rate : 0.05);

  state_publisher_ = get_node()->create_publisher<control_msgs::msg::JointTrajectoryControllerState>(
    "~/controller_state", rclcpp::SystemDefaultsQoS());
  realtime_state_publisher_ = std::make_unique<StatePublisher>(state_publisher_);
  {
    auto & message = realtime_state_publisher_->msg_;
    message.joint_names = joint_names_;
    message.reference.positions.assign(n, 0.0);
    message.reference.velocities.assign(n, 0.0);
    message.feedback.positions.assign(n, 0.0);
    message.feedback.velocities.assign(n, 0.0);
    message.error.positions.assign(n, 0.0);
    message.error.velocities.assign(n, 0.0);
    message.output.positions.assign(n, 0.0);
    message.desired = message.reference;
    message.actual = message.feedback;
  }

  trajectory_subscription_ = get_node()->create_subscription<trajectory_msgs::msg::JointTrajectory>(
    "~/joint_trajectory", rclcpp::SystemDefaultsQoS(),
    [this](const trajectory_msgs::msg::JointTrajectory::SharedPtr message) {
      auto command = std::make_shared<TrajectoryCommand>();
      std::string convert_error;
      if (!convertTrajectory(*message, command->waypoints, convert_error)) {
        RCLCPP_WARN(
          get_node()->get_logger(), "rejected trajectory on ~/joint_trajectory: %s",
          convert_error.c_str());
        return;
      }
      command->tolerances = default_tolerances_;
      command->goal_time_tolerance = default_goal_time_tolerance_;
      command->start_time = message->header.stamp;
      incoming_command_.writeFromNonRT(command);
    });

  goal_handle_timer_ = get_node()->create_wall_timer(
    action_monitor_period_.to_chrono<std::chrono::nanoseconds>(),
    [this]() {runGoalHandles();});

  action_server_ = rclcpp_action::create_server<FollowJointTrajectory>(
    get_node()->get_node_base_interface(), get_node()->get_node_clock_interface(),
    get_node()->get_node_logging_interface(), get_node()->get_node_waitables_interface(),
    std::string(get_node()->get_name()) + "/follow_joint_trajectory",
    std::bind(&TrajectoryController::handleGoal, this, std::placeholders::_1,
    std::placeholders::_2),
    std::bind(&TrajectoryController::handleCancel, this, std::placeholders::_1),
    std::bind(&TrajectoryController::handleAccepted, this, std::placeholders::_1));

  RCLCPP_INFO(
    logger, "trajectory controller ready over %zu joints in %s mode%s", n,
    command_mode_ == CommandMode::Effort ? "effort" : "position",
    gravity_feedforward_ ? " with gravity feed-forward" : "");

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn TrajectoryController::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  if (command_interfaces_.size() != joint_names_.size() ||
    state_interfaces_.size() != 2 * joint_names_.size())
  {
    RCLCPP_ERROR(
      get_node()->get_logger(), "claimed %zu command and %zu state interfaces for %zu joints",
      command_interfaces_.size(), state_interfaces_.size(), joint_names_.size());
    return controller_interface::CallbackReturn::ERROR;
  }

  if (!readState()) {
    RCLCPP_ERROR(get_node()->get_logger(), "joint state is non-finite at activation");
    return controller_interface::CallbackReturn::ERROR;
  }

  // Nothing to follow yet, so the trajectory the controller is executing is
  // "stay exactly here".
  hold_positions_ = positions_;
  desired_positions_ = positions_;
  std::fill(desired_velocities_.begin(), desired_velocities_.end(), 0.0);
  sampler_.clear();
  trajectory_active_ = false;
  active_goal_.reset();
  installed_command_.reset();
  incoming_command_.writeFromNonRT(std::shared_ptr<TrajectoryCommand>{});
  cancel_requested_.store(false);
  last_state_publish_time_ = get_node()->now();

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn TrajectoryController::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  abortActiveGoal(
    FollowJointTrajectory::Result::INVALID_GOAL, "controller was deactivated mid-trajectory");

  if (command_mode_ == CommandMode::Effort) {
    for (auto & interface : command_interfaces_) {
      interface.set_value(0.0);
    }
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

bool TrajectoryController::readState()
{
  bool finite = true;
  for (size_t i = 0; i < joint_names_.size(); ++i) {
    positions_[i] = state_interfaces_[2 * i].get_value();
    velocities_[i] = state_interfaces_[2 * i + 1].get_value();
    finite = finite && std::isfinite(positions_[i]) && std::isfinite(velocities_[i]);
  }
  return finite;
}

void TrajectoryController::abortActiveGoal(int32_t error_code, const std::string & message)
{
  if (active_goal_) {
    auto result = std::make_shared<FollowJointTrajectory::Result>();
    result->error_code = error_code;
    result->error_string = message;
    active_goal_->setAborted(result);
    active_goal_.reset();
  }
  sampler_.clear();
  trajectory_active_ = false;
  hold_positions_ = positions_;
}

void TrajectoryController::writeCommand()
{
  const size_t n = joint_names_.size();

  if (command_mode_ == CommandMode::Position) {
    for (size_t i = 0; i < n; ++i) {
      command_interfaces_[i].set_value(desired_positions_[i]);
    }
    return;
  }

  if (gravity_feedforward_) {
    model_.compute(positions_, gravity_torque_);
  }

  for (size_t i = 0; i < n; ++i) {
    double torque = kp_[i] * (desired_positions_[i] - positions_[i]) +
      kd_[i] * (desired_velocities_[i] - velocities_[i]);
    if (gravity_feedforward_) {
      torque += gravity_gain_[i] * gravity_torque_[i];
    }
    command_interfaces_[i].set_value(clampSymmetric(torque * effort_scale_[i], max_effort_[i]));
  }
}

void TrajectoryController::publishState(const rclcpp::Time & time)
{
  if (state_publish_period_ <= 0.0 || !realtime_state_publisher_) {
    return;
  }
  if ((time - last_state_publish_time_).seconds() < state_publish_period_) {
    return;
  }
  if (!realtime_state_publisher_->trylock()) {
    return;
  }

  auto & message = realtime_state_publisher_->msg_;
  message.header.stamp = time;
  message.reference.positions = desired_positions_;
  message.reference.velocities = desired_velocities_;
  message.feedback.positions = positions_;
  message.feedback.velocities = velocities_;
  for (size_t i = 0; i < joint_names_.size(); ++i) {
    message.error.positions[i] = desired_positions_[i] - positions_[i];
    message.error.velocities[i] = desired_velocities_[i] - velocities_[i];
    message.output.positions[i] = command_interfaces_[i].get_value();
  }
  message.desired = message.reference;
  message.actual = message.feedback;

  realtime_state_publisher_->unlockAndPublish();
  last_state_publish_time_ = time;
}

controller_interface::return_type TrajectoryController::update(
  const rclcpp::Time & time, const rclcpp::Duration & /*period*/)
{
  if (!readState()) {
    RCLCPP_WARN_THROTTLE(
      get_node()->get_logger(), *get_node()->get_clock(), 1000,
      "joint state read back non-finite; abandoning the trajectory");
    abortActiveGoal(
      FollowJointTrajectory::Result::INVALID_GOAL, "joint state read back non-finite");
    return controller_interface::return_type::OK;
  }

  const size_t n = joint_names_.size();

  // ----- take over a newly arrived trajectory -----
  const auto incoming = *incoming_command_.readFromRT();
  if (incoming && incoming != installed_command_) {
    if (active_goal_ && active_goal_ != incoming->goal) {
      auto result = std::make_shared<FollowJointTrajectory::Result>();
      result->error_code = FollowJointTrajectory::Result::SUCCESSFUL;
      result->error_string = "preempted by a newer trajectory";
      active_goal_->setAborted(result);
      active_goal_.reset();
    }

    std::string error;
    if (sampler_.set(incoming->waypoints, positions_, error)) {
      installed_command_ = incoming;
      active_goal_ = incoming->goal;
      active_tolerances_ = incoming->tolerances;
      active_goal_time_tolerance_ = incoming->goal_time_tolerance;
      trajectory_start_ = incoming->start_time.nanoseconds() == 0 ? time : incoming->start_time;
      trajectory_active_ = true;
      cancel_requested_.store(false);
    } else {
      RCLCPP_WARN(get_node()->get_logger(), "could not install trajectory: %s", error.c_str());
      installed_command_ = incoming;
      abortActiveGoal(FollowJointTrajectory::Result::INVALID_GOAL, error);
    }
  }

  // ----- cancellation -----
  if (cancel_requested_.exchange(false) && trajectory_active_) {
    if (active_goal_) {
      auto result = std::make_shared<FollowJointTrajectory::Result>();
      result->error_code = FollowJointTrajectory::Result::SUCCESSFUL;
      result->error_string = "cancelled";
      active_goal_->setCanceled(result);
      active_goal_.reset();
    }
    sampler_.clear();
    trajectory_active_ = false;
    hold_positions_ = positions_;
  }

  // ----- where should the arm be -----
  if (trajectory_active_) {
    const double elapsed = (time - trajectory_start_).seconds();
    sampler_.sample(elapsed, desired_positions_, desired_velocities_);

    if (elapsed < sampler_.duration()) {
      for (size_t i = 0; i < n; ++i) {
        const double deviation = std::abs(desired_positions_[i] - positions_[i]);
        if (active_tolerances_[i].path > 0.0 && deviation > active_tolerances_[i].path) {
          abortActiveGoal(
            FollowJointTrajectory::Result::PATH_TOLERANCE_VIOLATED,
            "joint '" + joint_names_[i] + "' is " + std::to_string(deviation) +
            " rad off the path, tolerance is " + std::to_string(active_tolerances_[i].path));
          break;
        }
      }
    } else {
      bool settled = true;
      for (size_t i = 0; i < n && settled; ++i) {
        const double deviation = std::abs(desired_positions_[i] - positions_[i]);
        settled = (active_tolerances_[i].goal <= 0.0 || deviation <= active_tolerances_[i].goal) &&
          (stopped_velocity_tolerance_ <= 0.0 ||
          std::abs(velocities_[i]) <= stopped_velocity_tolerance_);
      }

      if (settled) {
        if (active_goal_) {
          auto result = std::make_shared<FollowJointTrajectory::Result>();
          result->error_code = FollowJointTrajectory::Result::SUCCESSFUL;
          active_goal_->setSucceeded(result);
          active_goal_.reset();
        }
        hold_positions_ = desired_positions_;
        sampler_.clear();
        trajectory_active_ = false;
      } else if (active_goal_time_tolerance_ > 0.0 &&
        elapsed > sampler_.duration() + active_goal_time_tolerance_)
      {
        abortActiveGoal(
          FollowJointTrajectory::Result::GOAL_TOLERANCE_VIOLATED,
          "the arm did not settle inside the goal tolerance within " +
          std::to_string(active_goal_time_tolerance_) + " s of the trajectory end");
      }
    }
  }

  if (!trajectory_active_) {
    desired_positions_ = hold_positions_;
    std::fill(desired_velocities_.begin(), desired_velocities_.end(), 0.0);
  }

  writeCommand();

  if (active_goal_) {
    auto & feedback = *active_goal_->preallocated_feedback_;
    feedback.header.stamp = time;
    feedback.desired.positions = desired_positions_;
    feedback.desired.velocities = desired_velocities_;
    feedback.actual.positions = positions_;
    feedback.actual.velocities = velocities_;
    for (size_t i = 0; i < n; ++i) {
      feedback.error.positions[i] = desired_positions_[i] - positions_[i];
      feedback.error.velocities[i] = desired_velocities_[i] - velocities_[i];
    }
    active_goal_->setFeedback(active_goal_->preallocated_feedback_);
  }

  publishState(time);

  return controller_interface::return_type::OK;
}

bool TrajectoryController::convertTrajectory(
  const trajectory_msgs::msg::JointTrajectory & message, std::vector<Waypoint> & waypoints,
  std::string & error) const
{
  error.clear();
  waypoints.clear();

  const size_t n = joint_names_.size();

  if (message.joint_names.size() != n) {
    error = "trajectory names " + std::to_string(message.joint_names.size()) +
      " joints, this controller owns " + std::to_string(n);
    return false;
  }

  // The order a sender uses is its own business; map it onto ours once here so
  // the realtime side never has to look a joint name up.
  std::vector<size_t> source_index(n);
  for (size_t i = 0; i < n; ++i) {
    const auto it =
      std::find(message.joint_names.begin(), message.joint_names.end(), joint_names_[i]);
    if (it == message.joint_names.end()) {
      error = "trajectory does not mention joint '" + joint_names_[i] + "'";
      return false;
    }
    source_index[i] = static_cast<size_t>(std::distance(message.joint_names.begin(), it));
  }

  if (message.points.empty()) {
    error = "trajectory has no points";
    return false;
  }

  waypoints.reserve(message.points.size());
  for (const auto & point : message.points) {
    if (point.positions.size() != n) {
      error = "a trajectory point carries " + std::to_string(point.positions.size()) +
        " positions instead of " + std::to_string(n);
      return false;
    }
    if (!point.velocities.empty() && point.velocities.size() != n) {
      error = "a trajectory point carries velocities for only some joints";
      return false;
    }

    Waypoint waypoint;
    waypoint.time_from_start = rclcpp::Duration(point.time_from_start).seconds();
    waypoint.positions.resize(n);
    if (!point.velocities.empty()) {
      waypoint.velocities.resize(n);
    }
    for (size_t i = 0; i < n; ++i) {
      waypoint.positions[i] = point.positions[source_index[i]];
      if (!point.velocities.empty()) {
        waypoint.velocities[i] = point.velocities[source_index[i]];
      }
    }
    waypoints.push_back(std::move(waypoint));
  }

  // Reuse the sampler's own validation so a trajectory can never be accepted
  // here and then rejected in update(), where nobody is listening any more.
  TrajectorySampler probe;
  const std::vector<double> placeholder(n, 0.0);
  return probe.set(waypoints, placeholder, error);
}

rclcpp_action::GoalResponse TrajectoryController::handleGoal(
  const rclcpp_action::GoalUUID & /*uuid*/, std::shared_ptr<const FollowJointTrajectory::Goal> goal)
{
  if (get_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
    RCLCPP_WARN(get_node()->get_logger(), "rejecting a goal: the controller is not active");
    return rclcpp_action::GoalResponse::REJECT;
  }

  std::vector<Waypoint> waypoints;
  std::string error;
  if (!convertTrajectory(goal->trajectory, waypoints, error)) {
    RCLCPP_WARN(get_node()->get_logger(), "rejecting a goal: %s", error.c_str());
    return rclcpp_action::GoalResponse::REJECT;
  }

  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse TrajectoryController::handleCancel(
  std::shared_ptr<GoalHandle> /*goal_handle*/)
{
  // Only one goal is ever active — a new one preempts the old — so a cancel can
  // only ever mean the running trajectory.
  cancel_requested_.store(true);
  return rclcpp_action::CancelResponse::ACCEPT;
}

void TrajectoryController::handleAccepted(std::shared_ptr<GoalHandle> goal_handle)
{
  auto command = std::make_shared<TrajectoryCommand>();
  std::string error;
  if (!convertTrajectory(goal_handle->get_goal()->trajectory, command->waypoints, error)) {
    auto result = std::make_shared<FollowJointTrajectory::Result>();
    result->error_code = FollowJointTrajectory::Result::INVALID_GOAL;
    result->error_string = error;
    goal_handle->abort(result);
    return;
  }

  // Tolerances named in the goal replace the configured ones joint by joint;
  // a zero there means "whatever the parameter says", per the action contract.
  command->tolerances = default_tolerances_;
  const auto apply = [this, &command](
    const std::vector<control_msgs::msg::JointTolerance> & tolerances, bool is_goal) {
      for (const auto & tolerance : tolerances) {
        const auto it =
          std::find(joint_names_.begin(), joint_names_.end(), tolerance.name);
        if (it == joint_names_.end() || tolerance.position <= 0.0) {
          continue;
        }
        const size_t index = static_cast<size_t>(std::distance(joint_names_.begin(), it));
        (is_goal ? command->tolerances[index].goal : command->tolerances[index].path) =
          tolerance.position;
      }
    };
  apply(goal_handle->get_goal()->path_tolerance, false);
  apply(goal_handle->get_goal()->goal_tolerance, true);

  const double goal_time = rclcpp::Duration(goal_handle->get_goal()->goal_time_tolerance).seconds();
  command->goal_time_tolerance = goal_time > 0.0 ? goal_time : default_goal_time_tolerance_;
  command->start_time = goal_handle->get_goal()->trajectory.header.stamp;

  auto realtime_goal = std::make_shared<RealtimeGoalHandle>(goal_handle);
  {
    // Sized here so update() only ever assigns into vectors that already exist.
    auto & feedback = *realtime_goal->preallocated_feedback_;
    const size_t n = joint_names_.size();
    feedback.joint_names = joint_names_;
    feedback.desired.positions.assign(n, 0.0);
    feedback.desired.velocities.assign(n, 0.0);
    feedback.actual.positions.assign(n, 0.0);
    feedback.actual.velocities.assign(n, 0.0);
    feedback.error.positions.assign(n, 0.0);
    feedback.error.velocities.assign(n, 0.0);
  }
  command->goal = realtime_goal;

  // RealtimeGoalHandle drops setSucceeded/setAborted/setCanceled on the floor
  // until it has been told the goal is executing, so this call is what makes
  // every later verdict from update() land.
  realtime_goal->execute();

  {
    std::lock_guard<std::mutex> lock(live_goals_mutex_);
    live_goals_.push_back(realtime_goal);
  }

  incoming_command_.writeFromNonRT(command);
}

void TrajectoryController::runGoalHandles()
{
  std::lock_guard<std::mutex> lock(live_goals_mutex_);
  for (auto it = live_goals_.begin(); it != live_goals_.end(); ) {
    (*it)->runNonRealtime();
    // Once the action server has taken the goal to a terminal state there is
    // nothing left to deliver, so stop holding it alive.
    it = (*it)->gh_->is_active() ? std::next(it) : live_goals_.erase(it);
  }
}

}  // namespace om6dof_controllers

PLUGINLIB_EXPORT_CLASS(
  om6dof_controllers::TrajectoryController, controller_interface::ControllerInterface)
