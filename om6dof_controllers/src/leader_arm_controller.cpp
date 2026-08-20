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

#include "om6dof_controllers/leader_arm_controller.hpp"

#include <algorithm>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "om6dof_controllers/controller_utils.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace om6dof_controllers
{

controller_interface::CallbackReturn LeaderArmController::on_init()
{
  try {
    auto_declare<std::vector<std::string>>("joints", {});

    auto_declare<std::string>("base_link", "world");
    auto_declare<std::string>("tip_link", "end_effector_link");
    auto_declare<std::vector<double>>("gravity", {0.0, 0.0, -9.80665});

    auto_declare<std::vector<double>>("gain", {});
    auto_declare<std::vector<double>>("effort_scale", {});
    auto_declare<std::vector<double>>("min_effort", {});
    auto_declare<std::vector<double>>("max_effort", {});
    auto_declare<std::vector<double>>("headroom", {});
    auto_declare<std::vector<double>>("friction.coulomb", {});
    auto_declare<std::vector<double>>("friction.viscous", {});
    auto_declare<double>("friction.deadzone", 0.05);

    auto_declare<std::vector<double>>("setpoint_deadband", {});
    auto_declare<double>("ramp_seconds", 1.0);
    auto_declare<double>("publish_rate", 50.0);

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
LeaderArmController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration configuration;
  configuration.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  // Position first, then effort, both for every joint: the setpoint and the
  // ceiling are two halves of one behaviour and neither works alone.
  for (const auto & joint : joint_names_) {
    configuration.names.push_back(joint + "/" + hardware_interface::HW_IF_POSITION);
  }
  for (const auto & joint : joint_names_) {
    configuration.names.push_back(joint + "/" + hardware_interface::HW_IF_EFFORT);
  }
  return configuration;
}

controller_interface::InterfaceConfiguration
LeaderArmController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration configuration;
  configuration.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto & joint : joint_names_) {
    configuration.names.push_back(joint + "/" + hardware_interface::HW_IF_POSITION);
    configuration.names.push_back(joint + "/" + hardware_interface::HW_IF_VELOCITY);
  }
  return configuration;
}

controller_interface::CallbackReturn LeaderArmController::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  const auto logger = get_node()->get_logger();

  joint_names_ = get_node()->get_parameter("joints").as_string_array();
  if (joint_names_.empty()) {
    RCLCPP_ERROR(logger, "parameter 'joints' is empty");
    return controller_interface::CallbackReturn::ERROR;
  }
  const size_t n = joint_names_.size();

  std::string error;
  std::vector<double> coulomb;
  std::vector<double> viscous;
  const auto read = [&](const char * name, double fallback, std::vector<double> & out) {
      return perJointParameter(
        name, get_node()->get_parameter(name).as_double_array(), n, fallback, out, error);
    };

  if (!read("gain", 1.0, gain_) || !read("effort_scale", 1.0, effort_scale_) ||
    !read("min_effort", 0.0, min_effort_) || !read("max_effort", 0.0, max_effort_) ||
    !read("headroom", 0.0, headroom_) || !read("setpoint_deadband", 0.0, deadband_) ||
    !read("friction.coulomb", 0.0, coulomb) ||
    !read("friction.viscous", 0.0, viscous))
  {
    RCLCPP_ERROR(logger, "%s", error.c_str());
    return controller_interface::CallbackReturn::ERROR;
  }

  for (size_t i = 0; i < n; ++i) {
    // Without a ceiling above what holding costs, gravity moves the arm; the
    // motion gate then reads that as the operator, tracks the setpoint into the
    // sag, and the arm walks itself down. Refuse rather than discover it.
    if (max_effort_[i] <= 0.0 || min_effort_[i] <= 0.0) {
      RCLCPP_ERROR(
        logger, "joint '%s' needs a positive min_effort and max_effort", joint_names_[i].c_str());
      return controller_interface::CallbackReturn::ERROR;
    }
    if (min_effort_[i] > max_effort_[i]) {
      RCLCPP_ERROR(logger, "min_effort exceeds max_effort on joint '%s'", joint_names_[i].c_str());
      return controller_interface::CallbackReturn::ERROR;
    }
    if (deadband_[i] <= 0.0) {
      // A zero band pins the setpoint to the measurement, so the loop never
      // sees error, never generates holding current, and the arm sinks.
      RCLCPP_ERROR(
        logger, "joint '%s' needs a positive setpoint_deadband", joint_names_[i].c_str());
      return controller_interface::CallbackReturn::ERROR;
    }
  }

  const double deadzone = get_node()->get_parameter("friction.deadzone").as_double();
  friction_.resize(n);
  for (size_t i = 0; i < n; ++i) {
    friction_[i] = FrictionParameters{coulomb[i], viscous[i], deadzone};
  }

  ramp_seconds_ = std::max(get_node()->get_parameter("ramp_seconds").as_double(), 0.0);

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

  // Say out loud what was hanging off the chain. KDL drops branches silently,
  // and on this arm that was the gripper and the wrist payload: mass the model
  // would otherwise never have known about.
  if (model_.folded_mass() > 0.0) {
    std::string names;
    for (const auto & link : model_.folded_links()) {
      names += (names.empty() ? "" : ", ") + link;
    }
    RCLCPP_INFO(
      logger, "folded %.4f kg of off-chain mass into the gravity model: %s",
      model_.folded_mass(), names.c_str());
  }

  positions_.assign(n, 0.0);
  velocities_.assign(n, 0.0);
  gravity_torque_.assign(n, 0.0);
  setpoints_.assign(n, 0.0);
  commanded_effort_.assign(n, 0.0);

  const double publish_rate = get_node()->get_parameter("publish_rate").as_double();
  publish_period_ = publish_rate > 0.0 ? 1.0 / publish_rate : 0.0;

  effort_publisher_ = get_node()->create_publisher<std_msgs::msg::Float64MultiArray>(
    "~/commanded_effort", rclcpp::SystemDefaultsQoS());
  realtime_effort_publisher_ = std::make_unique<ArrayPublisher>(effort_publisher_);
  realtime_effort_publisher_->msg_.data.assign(n, 0.0);

  // What a follower arm should copy: where the leader actually is.
  lead_publisher_ = get_node()->create_publisher<sensor_msgs::msg::JointState>(
    "~/lead", rclcpp::SystemDefaultsQoS());
  realtime_lead_publisher_ =
    std::make_unique<realtime_tools::RealtimePublisher<sensor_msgs::msg::JointState>>(
    lead_publisher_);
  realtime_lead_publisher_->msg_.name = joint_names_;
  realtime_lead_publisher_->msg_.position.assign(n, 0.0);
  realtime_lead_publisher_->msg_.velocity.assign(n, 0.0);

  RCLCPP_INFO(logger, "leader arm controller ready over %zu joints", n);
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn LeaderArmController::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  const size_t n = joint_names_.size();
  if (command_interfaces_.size() != 2 * n || state_interfaces_.size() != 2 * n) {
    RCLCPP_ERROR(
      get_node()->get_logger(), "claimed %zu command and %zu state interfaces for %zu joints",
      command_interfaces_.size(), state_interfaces_.size(), n);
    return controller_interface::CallbackReturn::ERROR;
  }

  if (!readState()) {
    RCLCPP_ERROR(get_node()->get_logger(), "joint state is non-finite at activation");
    return controller_interface::CallbackReturn::ERROR;
  }

  // Hold exactly where the arm stands, so taking over from arm_controller does
  // not move it.
  setpoints_ = positions_;
  ramp_ = 0.0;
  last_publish_time_ = get_node()->now();

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn LeaderArmController::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  const size_t n = joint_names_.size();
  if (command_interfaces_.size() == 2 * n && readState()) {
    for (size_t i = 0; i < n; ++i) {
      // Hand the arm over where it stands, at full authority: whatever takes
      // over next must be able to hold it.
      command_interfaces_[i].set_value(positions_[i]);
      command_interfaces_[n + i].set_value(max_effort_[i]);
    }
  }
  ramp_ = 0.0;
  return controller_interface::CallbackReturn::SUCCESS;
}

bool LeaderArmController::readState()
{
  bool finite = true;
  for (size_t i = 0; i < joint_names_.size(); ++i) {
    positions_[i] = state_interfaces_[2 * i].get_value();
    velocities_[i] = state_interfaces_[2 * i + 1].get_value();
    finite = finite && std::isfinite(positions_[i]) && std::isfinite(velocities_[i]);
  }
  return finite;
}

controller_interface::return_type LeaderArmController::update(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
  const size_t n = joint_names_.size();

  if (!readState()) {
    RCLCPP_WARN_THROTTLE(
      get_node()->get_logger(), *get_node()->get_clock(), 1000,
      "joint state read back non-finite; freezing the setpoint at full authority");
    for (size_t i = 0; i < n; ++i) {
      command_interfaces_[i].set_value(setpoints_[i]);
      command_interfaces_[n + i].set_value(max_effort_[i]);
    }
    return controller_interface::return_type::OK;
  }

  ramp_ = ramp_seconds_ > 0.0 ? std::min(1.0, ramp_ + period.seconds() / ramp_seconds_) : 1.0;

  model_.compute(positions_, gravity_torque_);

  for (size_t i = 0; i < n; ++i) {
    const double torque = gain_[i] * gravity_torque_[i] +
      GravityModel::friction(velocities_[i], friction_[i]);

    commanded_effort_[i] = currentLimitCommand(
      torque, effort_scale_[i], headroom_[i], min_effort_[i], max_effort_[i], ramp_);

    const double needed = std::abs(torque) * effort_scale_[i];
    if (needed > max_effort_[i]) {
      RCLCPP_WARN_THROTTLE(
        get_node()->get_logger(), *get_node()->get_clock(), 2000,
        "joint '%s' needs %.0f to hold this pose but max_effort caps it at %.0f; the arm will sag "
        "and the setpoint will follow it down", joint_names_[i].c_str(), needed, max_effort_[i]);
    }

    // The band follows the arm, so the loop never sees more than a deadband of
    // error however far the arm travels, and there is nothing to spring back to
    // when the operator lets go.
    setpoints_[i] = followSetpoint(setpoints_[i], positions_[i], deadband_[i]);

    command_interfaces_[i].set_value(setpoints_[i]);
    command_interfaces_[n + i].set_value(commanded_effort_[i]);
  }

  if (publish_period_ > 0.0 && (time - last_publish_time_).seconds() >= publish_period_) {
    bool published = false;
    if (realtime_effort_publisher_ && realtime_effort_publisher_->trylock()) {
      realtime_effort_publisher_->msg_.data = commanded_effort_;
      realtime_effort_publisher_->unlockAndPublish();
      published = true;
    }
    if (realtime_lead_publisher_ && realtime_lead_publisher_->trylock()) {
      realtime_lead_publisher_->msg_.header.stamp = time;
      realtime_lead_publisher_->msg_.position = positions_;
      realtime_lead_publisher_->msg_.velocity = velocities_;
      realtime_lead_publisher_->unlockAndPublish();
      published = true;
    }
    if (published) {
      last_publish_time_ = time;
    }
  }

  return controller_interface::return_type::OK;
}

}  // namespace om6dof_controllers

PLUGINLIB_EXPORT_CLASS(
  om6dof_controllers::LeaderArmController, controller_interface::ControllerInterface)
