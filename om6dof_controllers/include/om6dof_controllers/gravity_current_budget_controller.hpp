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

#pragma once

#include <chrono>
#include <array>
#include <vector>
#include "control_msgs/msg/joint_trajectory_controller_state.hpp"
#include "controller_interface/controller_interface.hpp"
#include "om6dof_controllers/gravity_model.hpp"
#include "realtime_tools/realtime_publisher.hpp"
#include "realtime_tools/realtime_buffer.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

namespace om6dof_controllers
{
// Mode 5 only. Owns current ceilings, NOT position targets or PID registers.
class OM6DOF_CONTROLLERS_PUBLIC GravityCurrentBudgetController
  : public controller_interface::ControllerInterface
{
public:
  controller_interface::CallbackReturn on_init() override;
  controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State &) override;
  controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &) override;
  controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override;
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;
  controller_interface::return_type update(const rclcpp::Time &, const rclcpp::Duration &) override;

protected:
  void receiveReference(
    const control_msgs::msg::JointTrajectoryControllerState & message, bool admittance = false);
  // Observer packet schema: stamp, quality, q[6], dq[6], current_raw[6],
  // physical_model_Nm[6], commanded_current_ceiling_raw[6]. Quality 0 is usable.
  std::array<double, 32> observer_snapshot_{};

private:
  void publishObserver(const rclcpp::Time & time, double dt, bool healthy);
  struct MotionReference
  {
    std::array<double, 6> velocity{}, acceleration{};
    double received{0.0};
    double stamp{0.0};
    bool valid{false};
  };
  void zero();
  GravityModel model_;
  std::vector<double> q_, g_, scale_, lower_, upper_, effort_;
  std::vector<double> qd_filtered_, qdd_filtered_, model_torque_;
  std::vector<FrictionParameters> friction_;
  std::vector<size_t> velocities_, measured_currents_;
  double observer_filter_{0.10};
  std::vector<double> holding_, velocity_gain_, acceleration_gain_, max_current_, slew_, previous_current_;
  double gravity_scale_{1.0}, reference_timeout_{0.25};
  realtime_tools::RealtimeBuffer<MotionReference> reference_;
  realtime_tools::RealtimeBuffer<MotionReference> admittance_reference_;
  rclcpp::Subscription<control_msgs::msg::JointTrajectoryControllerState>::SharedPtr reference_sub_;
  rclcpp::Subscription<control_msgs::msg::JointTrajectoryControllerState>::SharedPtr admittance_reference_sub_;
  std::vector<size_t> positions_, limits_, commands_;
  size_t health_{0};
  bool fault_{true};
  double elapsed_{0.0};
  std::chrono::steady_clock::time_point previous_update_{};
  std::unique_ptr<realtime_tools::RealtimePublisher<std_msgs::msg::Float64MultiArray>> publisher_;
  unsigned publish_count_{0};
  unsigned observer_count_{0};
  std::unique_ptr<realtime_tools::RealtimePublisher<std_msgs::msg::Float64MultiArray>> observer_pub_;
};
}  // namespace om6dof_controllers
