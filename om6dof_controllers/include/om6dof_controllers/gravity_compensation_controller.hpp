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

#ifndef OM6DOF_CONTROLLERS__GRAVITY_COMPENSATION_CONTROLLER_HPP_
#define OM6DOF_CONTROLLERS__GRAVITY_COMPENSATION_CONTROLLER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "realtime_tools/realtime_publisher.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

#include "om6dof_controllers/gravity_model.hpp"
#include "om6dof_controllers/visibility_control.h"

namespace om6dof_controllers
{

/// Uses the arm's own weight model to decide what to write to the effort
/// command interface. What that write *means* is set by `command_semantics`,
/// because the two hardware setups in this repo interpret it differently.
///
/// `torque` -- the interface is a commanded torque, so the controller writes
/// the signed effort that cancels gravity. There is no position setpoint: with
/// gravity carried the arm stays put and can be led by hand. Output ramps up
/// from zero over `ramp_seconds`. Correct for Dynamixel operating mode 0, or
/// any hardware whose effort command really is a torque.
///
/// `current_limit` -- the interface is a *ceiling* on the current the servo's
/// own position loop may draw, which is what operating mode 5 gives (see
/// om6dof_bringup's om6dof.ros2_control.current.xacro). Three things follow.
/// The output is a magnitude, so the sign of g(q) stops mattering and the arm
/// cannot go slack as gravity torque crosses zero. It is floored at
/// `min_effort`, because a zero limit means the servo may pull nothing and the
/// arm drops. And it ramps *down* from `max_effort` to the computed limit, so
/// activation goes from firmly held to compliant rather than through limp.
///
/// The compliance the operator feels is set by `headroom`: how much current the
/// position loop is allowed on top of what merely holding the pose costs.
class OM6DOF_CONTROLLERS_PUBLIC GravityCompensationController
  : public controller_interface::ControllerInterface
{
public:
  GravityCompensationController() = default;

  controller_interface::CallbackReturn on_init() override;

  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

protected:
  std::vector<std::string> joint_names_;

  std::vector<double> gain_;
  std::vector<double> effort_scale_;
  std::vector<double> max_effort_;
  std::vector<double> bias_;
  std::vector<double> deactivate_effort_;
  std::vector<double> min_effort_;
  std::vector<double> headroom_;

  enum class CommandSemantics
  {
    Torque,
    CurrentLimit
  };
  CommandSemantics semantics_{CommandSemantics::Torque};
  std::vector<FrictionParameters> friction_;
  double ramp_seconds_{1.0};

  GravityModel model_;

  std::vector<double> positions_;
  std::vector<double> velocities_;
  std::vector<double> gravity_torque_;
  std::vector<double> commanded_effort_;

  double ramp_{0.0};

  using TorquePublisher = realtime_tools::RealtimePublisher<std_msgs::msg::Float64MultiArray>;
  std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Float64MultiArray>> torque_publisher_;
  std::unique_ptr<TorquePublisher> realtime_torque_publisher_;
  std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Float64MultiArray>> command_publisher_;
  std::unique_ptr<TorquePublisher> realtime_command_publisher_;
  double publish_period_{0.05};
  rclcpp::Time last_publish_time_;

  /// Copies state interfaces into `positions_`/`velocities_`.
  /// \return false if any of them read back non-finite.
  bool readState();
};

}  // namespace om6dof_controllers

#endif  // OM6DOF_CONTROLLERS__GRAVITY_COMPENSATION_CONTROLLER_HPP_
