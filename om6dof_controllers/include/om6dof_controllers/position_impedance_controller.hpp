// Copyright 2026 OM6DOF maintainers. Licensed under Apache-2.0.
#ifndef OM6DOF_CONTROLLERS__POSITION_IMPEDANCE_CONTROLLER_HPP_
#define OM6DOF_CONTROLLERS__POSITION_IMPEDANCE_CONTROLLER_HPP_

#include <atomic>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>
#include "controller_interface/controller_interface.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "realtime_tools/realtime_buffer.hpp"
#include "realtime_tools/realtime_publisher.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "om6dof_controllers/gravity_model.hpp"

namespace om6dof_controllers
{
// Nm; zero is a zero budget, NEVER the 'unlimited' convention of clampSymmetric.
OM6DOF_CONTROLLERS_PUBLIC double impedanceBudget(
  double gravity, double base, double ratio, double extra_max, double total_max);

class OM6DOF_CONTROLLERS_PUBLIC PositionImpedanceController
  : public controller_interface::ControllerInterface
{
public:
  controller_interface::CallbackReturn on_init() override;
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;
  controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State &) override;
  controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &) override;
  controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override;
  controller_interface::return_type update(const rclcpp::Time &, const rclcpp::Duration &) override;

protected:
  struct Goal {
    std::vector<double> q;
    int64_t received_ns{0};
    uint64_t epoch{0}, sequence{0};
  };
  struct Wrench {
    std::array<double, 6> value{{0, 0, 0, 0, 0, 0}};
    int64_t received_ns{0};
    uint64_t epoch{0}, sequence{0};
  };
  struct Health {
    int64_t received_ns{0}, clean_since_ns{0};
    uint64_t faults{0};
    bool healthy{false};
  };
  enum Status {WAITING = 0, TRACKING = 1, TIMEOUT = 2, FAULT = 3, RAMPING = 4};
  static int64_t steadyNow();
  void receiveGoal(const sensor_msgs::msg::JointState & message);
  void receiveWrench(const std_msgs::msg::Float64MultiArray & message);
  void receiveHealth(const diagnostic_msgs::msg::DiagnosticArray & message);
  bool readState();
  void zeroCommands();
  bool healthReady(int64_t now);
  void publish();

  std::vector<std::string> joints_;
  std::vector<double> stiffness_, damping_, scale_, max_raw_, base_, ratio_, extra_max_;
  std::vector<double> reference_rate_, lookahead_, torque_rate_, velocity_limit_;
  std::vector<double> lower_, upper_, total_max_, q_, v_, gravity_, reference_, target_;
  std::vector<double> residual_, budget_, raw_, idle_damping_;
  std::vector<double> jacobian_, joint_wrench_;
  std::vector<double> activation_pose_;
  bool require_activation_pose_{false};
  std::atomic<bool> activation_attempted_{false};
  double activation_pose_tolerance_{0.03};
  std::vector<size_t> position_indices_, velocity_indices_, effort_indices_;
  std::vector<FrictionParameters> friction_;
  GravityModel model_;
  bool use_friction_{false}, require_health_{true}, fault_{false}, hard_fault_{false};
  bool cartesian_wrench_enabled_{false};
  std::array<double, 6> max_cartesian_wrench_{{0, 0, 0, 0, 0, 0}};
  double timeout_{0.25}, health_timeout_{0.15}, clean_window_{1.0}, max_period_{0.05};
  double margin_{0.05}, ramp_seconds_{1.0}, ramp_{0.0};
  Status status_{WAITING};
  uint32_t fault_flags_{0};
  // First limit trip since activation: version, kind (1=position, 2=speed),
  // zero-based joint, measured value, lower bound, upper bound.
  std::array<double, 6> limit_event_{{1, 0, -1, 0, 0, 0}};
  bool current_limits_ok_{false};
  std::atomic<bool> active_{false};
  std::atomic<uint64_t> epoch_{0};
  std::atomic<uint64_t> command_sequence_{0};
  std::atomic<uint64_t> invalid_goals_{0};
  uint64_t activation_health_faults_{0}, activation_invalid_goals_{0}, activation_command_sequence_{0};
  int64_t activation_ns_{0}, last_goal_stamp_ns_{0}, last_health_stamp_ns_{0};
  uint64_t last_read_count_{0}, last_write_count_{0};
  std::string health_instance_;
  Health callback_health_;
  realtime_tools::RealtimeBuffer<Goal> goal_buffer_;
  realtime_tools::RealtimeBuffer<Wrench> wrench_buffer_;
  realtime_tools::RealtimeBuffer<Health> health_buffer_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr goal_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr wrench_sub_;
  rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr health_sub_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_guard_;
  using Publisher = realtime_tools::RealtimePublisher<std_msgs::msg::Float64MultiArray>;
  std::unique_ptr<Publisher> telemetry_;
  std::unique_ptr<Publisher> limit_telemetry_;
};
}  // namespace om6dof_controllers
#endif
