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

#include "om6dof_controllers/spring_actuator_controller.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "om6dof_controllers/controller_utils.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace om6dof_controllers
{

controller_interface::CallbackReturn SpringActuatorController::on_init()
{
  try {
    auto_declare<std::vector<std::string>>("joints", {});

    auto_declare<std::vector<double>>("stiffness", {});
    auto_declare<std::vector<double>>("damping", {});
    auto_declare<std::vector<double>>("rest_position", {});
    auto_declare<bool>("capture_rest_on_activate", true);
    auto_declare<double>("rest_slew_rate", 0.5);

    auto_declare<std::vector<double>>("effort_scale", {});
    auto_declare<std::vector<double>>("max_effort", {});
    auto_declare<double>("ramp_seconds", 1.0);
    auto_declare<double>("publish_rate", 20.0);

    auto_declare<bool>("gravity_compensation", true);
    auto_declare<std::vector<double>>("gravity_gain", {});
    auto_declare<std::string>("base_link", "world");
    auto_declare<std::string>("tip_link", "end_effector_link");
    auto_declare<std::vector<double>>("gravity", {0.0, 0.0, -9.80665});

    auto_declare<std::string>("robot_description", "");
    auto_declare<std::string>("robot_description_topic", "/robot_description");
    auto_declare<double>("robot_description_timeout", 10.0);
  } catch (const std::exception & exception) {
    RCLCPP_ERROR(get_node()->get_logger(), "exception while declaring parameters: %s",
      exception.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
SpringActuatorController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration configuration;
  configuration.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto & joint : joint_names_) {
    configuration.names.push_back(joint + "/" + hardware_interface::HW_IF_EFFORT);
  }
  return configuration;
}

controller_interface::InterfaceConfiguration
SpringActuatorController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration configuration;
  configuration.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto & joint : joint_names_) {
    configuration.names.push_back(joint + "/" + hardware_interface::HW_IF_POSITION);
    configuration.names.push_back(joint + "/" + hardware_interface::HW_IF_VELOCITY);
  }
  return configuration;
}

controller_interface::CallbackReturn SpringActuatorController::on_configure(
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
  const auto read = [&](const char * name, double fallback, std::vector<double> & out) {
      return perJointParameter(
        name, get_node()->get_parameter(name).as_double_array(), n, fallback, out, error);
    };

  if (!read("stiffness", 0.0, stiffness_) || !read("damping", 0.0, damping_) ||
    !read("effort_scale", 1.0, effort_scale_) || !read("max_effort", 0.0, max_effort_) ||
    !read("gravity_gain", 1.0, gravity_gain_))
  {
    RCLCPP_ERROR(logger, "%s", error.c_str());
    return controller_interface::CallbackReturn::ERROR;
  }

  for (size_t i = 0; i < n; ++i) {
    if (stiffness_[i] < 0.0 || damping_[i] < 0.0) {
      RCLCPP_ERROR(
        logger, "negative stiffness or damping on joint '%s' would push the arm away, not pull it",
        joint_names_[i].c_str());
      return controller_interface::CallbackReturn::ERROR;
    }
  }

  capture_rest_on_activate_ = get_node()->get_parameter("capture_rest_on_activate").as_bool();
  const auto rest_parameter = get_node()->get_parameter("rest_position").as_double_array();
  if (rest_parameter.empty()) {
    if (!capture_rest_on_activate_) {
      RCLCPP_ERROR(
        logger,
        "no 'rest_position' and 'capture_rest_on_activate' is false, so there is no pose to pull "
        "towards");
      return controller_interface::CallbackReturn::ERROR;
    }
    configured_rest_.clear();
  } else if (rest_parameter.size() != n) {
    RCLCPP_ERROR(logger, "parameter 'rest_position' needs one value per joint");
    return controller_interface::CallbackReturn::ERROR;
  } else {
    configured_rest_ = rest_parameter;
  }

  rest_slew_rate_ = std::max(get_node()->get_parameter("rest_slew_rate").as_double(), 0.0);
  ramp_seconds_ = std::max(get_node()->get_parameter("ramp_seconds").as_double(), 0.0);
  gravity_compensation_ = get_node()->get_parameter("gravity_compensation").as_bool();

  if (gravity_compensation_) {
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
  gravity_torque_.assign(n, 0.0);
  rest_position_.assign(n, 0.0);
  commanded_effort_.assign(n, 0.0);
  rest_command_.writeFromNonRT(std::vector<double>{});

  rest_subscription_ = get_node()->create_subscription<std_msgs::msg::Float64MultiArray>(
    "~/rest_position", rclcpp::SystemDefaultsQoS(),
    [this, n](const std_msgs::msg::Float64MultiArray::SharedPtr message) {
      if (message->data.size() != n) {
        RCLCPP_WARN_THROTTLE(
          get_node()->get_logger(), *get_node()->get_clock(), 2000,
          "rest_position needs %zu values, got %zu; ignored", n, message->data.size());
        return;
      }
      if (std::any_of(
          message->data.begin(), message->data.end(),
          [](double value) {return !std::isfinite(value);}))
      {
        RCLCPP_WARN_THROTTLE(
          get_node()->get_logger(), *get_node()->get_clock(), 2000,
          "rest_position contains a non-finite value; ignored");
        return;
      }
      rest_command_.writeFromNonRT(message->data);
    });

  const double publish_rate = get_node()->get_parameter("publish_rate").as_double();
  publish_period_ = publish_rate > 0.0 ? 1.0 / publish_rate : 0.0;
  effort_publisher_ = get_node()->create_publisher<std_msgs::msg::Float64MultiArray>(
    "~/commanded_effort", rclcpp::SystemDefaultsQoS());
  realtime_effort_publisher_ = std::make_unique<EffortPublisher>(effort_publisher_);
  realtime_effort_publisher_->msg_.data.assign(n, 0.0);

  RCLCPP_INFO(
    logger, "spring actuator ready over %zu joints, gravity compensation %s", n,
    gravity_compensation_ ? "on" : "off");

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn SpringActuatorController::on_activate(
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

  // Where the setpoint starts, and where it is heading, are two separate
  // questions. Capturing the measured pose makes activation bump-free; the
  // configured rest pose is then reached by slewing rather than by a step.
  rest_position_ = capture_rest_on_activate_ ? positions_ : configured_rest_;
  rest_command_.writeFromNonRT(configured_rest_);

  ramp_ = 0.0;
  last_publish_time_ = get_node()->now();
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn SpringActuatorController::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  for (auto & interface : command_interfaces_) {
    interface.set_value(0.0);
  }
  ramp_ = 0.0;
  return controller_interface::CallbackReturn::SUCCESS;
}

bool SpringActuatorController::readState()
{
  bool finite = true;
  for (size_t i = 0; i < joint_names_.size(); ++i) {
    positions_[i] = state_interfaces_[2 * i].get_value();
    velocities_[i] = state_interfaces_[2 * i + 1].get_value();
    finite = finite && std::isfinite(positions_[i]) && std::isfinite(velocities_[i]);
  }
  return finite;
}

controller_interface::return_type SpringActuatorController::update(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
  if (!readState()) {
    RCLCPP_WARN_THROTTLE(
      get_node()->get_logger(), *get_node()->get_clock(), 1000,
      "joint state read back non-finite; commanding zero effort");
    for (auto & interface : command_interfaces_) {
      interface.set_value(0.0);
    }
    return controller_interface::return_type::OK;
  }

  const size_t n = joint_names_.size();

  const std::vector<double> * target = rest_command_.readFromRT();
  if (target != nullptr && target->size() == n) {
    const double step = rest_slew_rate_ > 0.0 ?
      rest_slew_rate_ * period.seconds() :
      std::numeric_limits<double>::infinity();
    for (size_t i = 0; i < n; ++i) {
      const double delta = (*target)[i] - rest_position_[i];
      rest_position_[i] += std::clamp(delta, -step, step);
    }
  }

  ramp_ = ramp_seconds_ > 0.0 ?
    std::min(1.0, ramp_ + period.seconds() / ramp_seconds_) :
    1.0;

  if (gravity_compensation_) {
    model_.compute(positions_, gravity_torque_);
  }

  for (size_t i = 0; i < n; ++i) {
    double torque = stiffness_[i] * (rest_position_[i] - positions_[i]) -
      damping_[i] * velocities_[i];
    if (gravity_compensation_) {
      torque += gravity_gain_[i] * gravity_torque_[i];
    }
    commanded_effort_[i] = ramp_ * clampSymmetric(torque * effort_scale_[i], max_effort_[i]);
    command_interfaces_[i].set_value(commanded_effort_[i]);
  }

  if (publish_period_ > 0.0 && (time - last_publish_time_).seconds() >= publish_period_) {
    if (realtime_effort_publisher_ && realtime_effort_publisher_->trylock()) {
      realtime_effort_publisher_->msg_.data = commanded_effort_;
      realtime_effort_publisher_->unlockAndPublish();
      last_publish_time_ = time;
    }
  }

  return controller_interface::return_type::OK;
}

}  // namespace om6dof_controllers

PLUGINLIB_EXPORT_CLASS(
  om6dof_controllers::SpringActuatorController, controller_interface::ControllerInterface)
