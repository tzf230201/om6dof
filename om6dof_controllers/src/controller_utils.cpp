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

#include "om6dof_controllers/controller_utils.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

namespace om6dof_controllers
{

bool perJointParameter(
  const std::string & name, const std::vector<double> & raw, size_t joint_count, double fallback,
  std::vector<double> & out, std::string & error)
{
  error.clear();

  if (raw.empty()) {
    out.assign(joint_count, fallback);
  } else if (raw.size() == 1) {
    out.assign(joint_count, raw.front());
  } else if (raw.size() == joint_count) {
    out = raw;
  } else {
    error = "parameter '" + name + "' has " + std::to_string(raw.size()) + " values but there are " +
      std::to_string(joint_count) + " joints; give one value, one per joint, or none";
    return false;
  }

  for (const double value : out) {
    if (!std::isfinite(value)) {
      error = "parameter '" + name + "' contains a non-finite value";
      out.clear();
      return false;
    }
  }

  return true;
}

std::string fetchRobotDescription(
  const std::string & parameter_value, const std::string & node_namespace, const std::string & topic,
  double timeout_seconds, std::string & error)
{
  error.clear();

  if (!parameter_value.empty()) {
    return parameter_value;
  }

  if (topic.empty()) {
    error = "no 'robot_description' parameter and no 'robot_description_topic' to fall back to";
    return "";
  }

  std::string description;
  try {
    rclcpp::NodeOptions options;
    options.start_parameter_services(false);
    options.start_parameter_event_publisher(false);

    auto node = std::make_shared<rclcpp::Node>(
      "om6dof_controllers_robot_description_listener", node_namespace, options);

    auto subscription = node->create_subscription<std_msgs::msg::String>(
      topic, rclcpp::QoS(1).transient_local().reliable(),
      [&description](const std_msgs::msg::String::SharedPtr message) {
        description = message->data;
      });

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);

    const auto deadline = node->get_clock()->now() + rclcpp::Duration::from_seconds(
      std::max(timeout_seconds, 0.0));
    while (description.empty() && rclcpp::ok() && node->get_clock()->now() < deadline) {
      executor.spin_once(std::chrono::milliseconds(50));
    }

    executor.remove_node(node);
  } catch (const std::exception & exception) {
    error = std::string("failed while waiting for the robot description: ") + exception.what();
    return "";
  }

  if (description.empty()) {
    error = "no robot description arrived on '" + topic + "' within " +
      std::to_string(timeout_seconds) + " s";
  }

  return description;
}

double currentLimitCommand(
  double torque, double effort_scale, double headroom, double min_effort, double max_effort,
  double ramp)
{
  const double required = std::abs(torque) * effort_scale + headroom;
  const double limit = std::clamp(required, min_effort, max_effort);
  return max_effort + ramp * (limit - max_effort);
}

double followSetpoint(double setpoint, double measured, double deadband)
{
  const double width = std::max(deadband, 0.0);
  return std::clamp(setpoint, measured - width, measured + width);
}

double clampSymmetric(double value, double limit)
{
  if (!std::isfinite(limit) || limit <= 0.0) {
    return value;
  }
  return std::clamp(value, -limit, limit);
}

}  // namespace om6dof_controllers
