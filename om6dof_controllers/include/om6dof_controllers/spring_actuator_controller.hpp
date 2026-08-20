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

#ifndef OM6DOF_CONTROLLERS__SPRING_ACTUATOR_CONTROLLER_HPP_
#define OM6DOF_CONTROLLERS__SPRING_ACTUATOR_CONTROLLER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "realtime_tools/realtime_buffer.hpp"
#include "realtime_tools/realtime_publisher.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

#include "om6dof_controllers/gravity_model.hpp"
#include "om6dof_controllers/visibility_control.h"

namespace om6dof_controllers
{

/// Makes each joint behave like a spring-damper pulling towards a rest pose.
///
///     tau = K (q_rest - q) - D qd  [+ g(q)]
///
/// With gravity compensation on, the spring is what the operator feels and
/// nothing else; with it off, the arm sags into an equilibrium where the spring
/// balances its own weight. The rest pose is either captured from wherever the
/// arm sits at activation or streamed in on `~/rest_position`, and it slews at
/// a bounded rate so a jump in the reference cannot become a jump in torque.
class OM6DOF_CONTROLLERS_PUBLIC SpringActuatorController
  : public controller_interface::ControllerInterface
{
public:
  SpringActuatorController() = default;

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

  std::vector<double> stiffness_;
  std::vector<double> damping_;
  std::vector<double> configured_rest_;
  std::vector<double> effort_scale_;
  std::vector<double> max_effort_;
  bool capture_rest_on_activate_{true};
  bool gravity_compensation_{true};
  std::vector<double> gravity_gain_;
  double rest_slew_rate_{0.0};
  double ramp_seconds_{1.0};

  GravityModel model_;

  std::vector<double> positions_;
  std::vector<double> velocities_;
  std::vector<double> gravity_torque_;
  std::vector<double> rest_position_;
  std::vector<double> commanded_effort_;

  double ramp_{0.0};

  realtime_tools::RealtimeBuffer<std::vector<double>> rest_command_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr rest_subscription_;

  using EffortPublisher = realtime_tools::RealtimePublisher<std_msgs::msg::Float64MultiArray>;
  std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Float64MultiArray>> effort_publisher_;
  std::unique_ptr<EffortPublisher> realtime_effort_publisher_;
  double publish_period_{0.05};
  rclcpp::Time last_publish_time_;

  bool readState();
};

}  // namespace om6dof_controllers

#endif  // OM6DOF_CONTROLLERS__SPRING_ACTUATOR_CONTROLLER_HPP_
