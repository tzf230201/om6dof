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

#ifndef OM6DOF_CONTROLLERS__LEADER_ARM_CONTROLLER_HPP_
#define OM6DOF_CONTROLLERS__LEADER_ARM_CONTROLLER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "realtime_tools/realtime_buffer.hpp"
#include "realtime_tools/realtime_publisher.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

#include "om6dof_controllers/gravity_model.hpp"
#include "om6dof_controllers/visibility_control.h"

namespace om6dof_controllers
{

/// Makes the arm a lead-by-hand leader: it holds where it is left, and gives
/// way when a person moves it.
///
/// This claims **both** the position and the effort command interfaces, so it
/// replaces `arm_controller` for the duration rather than running beside it.
/// That is the whole point. A JointTrajectoryController holds one fixed
/// setpoint, so pushing the arm means fighting it and letting go springs it
/// back -- the opposite of a leader arm, no matter what the current limit says.
///
/// Two things are written every cycle, and they do different jobs:
///
///   position  the setpoint, dragged along so it never sits further than
///             `setpoint_deadband` from where the arm actually is. Gravity
///             pulls the arm to the edge of that band and the servo holds it
///             there; a hand moving the arm pushes the band along, so there is
///             nothing to spring back to on release.
///   effort    the current ceiling, from the same gravity model
///             GravityCompensationController uses: how hard the servo is
///             allowed to pull. This is what makes it feel light.
///
/// Bounding the error is what makes this feel light. The servo's position loop
/// never sees more than `setpoint_deadband`, so the force it pushes back with
/// is bounded and constant rather than building until the arm breaks away --
/// which is what a freeze-and-track scheme does, and what made the shoulder
/// joint feel stuck.
///
/// The band follows a sagging arm just as readily as a pushed one, so it holds
/// only while the servo can carry the arm at `setpoint_deadband` of error. Too
/// small a band, or a ceiling below what holding costs, and the arm walks
/// itself down. `max_effort` is therefore load-bearing in the literal sense;
/// the controller refuses to start without a positive one and warns whenever
/// the cap binds.
class OM6DOF_CONTROLLERS_PUBLIC LeaderArmController
  : public controller_interface::ControllerInterface
{
public:
  LeaderArmController() = default;

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
  std::vector<double> min_effort_;
  std::vector<double> max_effort_;
  std::vector<FrictionParameters> friction_;

  double ramp_seconds_{1.0};

  /// Live-tunable, because the only way to know what the arm should feel like
  /// is to hold it while changing them. Written from the parameter callback,
  /// read in update(), so they go through a realtime buffer rather than being
  /// edited under the control loop's feet.
  realtime_tools::RealtimeBuffer<std::vector<double>> deadband_;
  realtime_tools::RealtimeBuffer<std::vector<double>> headroom_buffer_;

  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_callback_;
  rcl_interfaces::msg::SetParametersResult onParameters(
    const std::vector<rclcpp::Parameter> & parameters);

  GravityModel model_;

  std::vector<double> positions_;
  std::vector<double> velocities_;
  std::vector<double> gravity_torque_;
  std::vector<double> setpoints_;
  std::vector<double> commanded_effort_;

  double ramp_{0.0};

  using ArrayPublisher = realtime_tools::RealtimePublisher<std_msgs::msg::Float64MultiArray>;
  std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Float64MultiArray>> effort_publisher_;
  std::unique_ptr<ArrayPublisher> realtime_effort_publisher_;
  std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::JointState>> lead_publisher_;
  std::unique_ptr<realtime_tools::RealtimePublisher<sensor_msgs::msg::JointState>>
  realtime_lead_publisher_;
  double publish_period_{0.02};
  rclcpp::Time last_publish_time_;

  bool readState();
};

}  // namespace om6dof_controllers

#endif  // OM6DOF_CONTROLLERS__LEADER_ARM_CONTROLLER_HPP_
