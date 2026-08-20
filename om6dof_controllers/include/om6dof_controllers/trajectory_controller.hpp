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

#ifndef OM6DOF_CONTROLLERS__TRAJECTORY_CONTROLLER_HPP_
#define OM6DOF_CONTROLLERS__TRAJECTORY_CONTROLLER_HPP_

#include <atomic>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "control_msgs/msg/joint_trajectory_controller_state.hpp"
#include "controller_interface/controller_interface.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "realtime_tools/realtime_buffer.hpp"
#include "realtime_tools/realtime_publisher.hpp"
#include "realtime_tools/realtime_server_goal_handle.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"

#include "om6dof_controllers/gravity_model.hpp"
#include "om6dof_controllers/trajectory_sampler.hpp"
#include "om6dof_controllers/visibility_control.h"

namespace om6dof_controllers
{

/// Follows joint trajectories on the OM6DOF arm.
///
/// Trajectories arrive either through the `~/follow_joint_trajectory` action —
/// the one MoveIt uses — or, for scripting and bring-up, as a plain message on
/// `~/joint_trajectory`, which is fire-and-forget and reports nothing back.
/// Points are reordered into this controller's joint order, so a trajectory may
/// list its joints in any order, and it must name all of them.
///
/// Two command modes:
///
///   position  the sampled position goes straight to the command interface and
///             the servo closes its own loop. This is the mode to use with the
///             standard OM6DOF description.
///   effort    the controller closes the loop itself, `Kp e + Kd edot`, on top
///             of a gravity feed-forward term. Needs an effort command
///             interface, so it needs om6dof.ros2_control.current.xacro.
///
/// Tolerance handling follows the FollowJointTrajectory contract: a path
/// tolerance violated mid-motion aborts immediately, and the goal tolerance is
/// checked from the end of the trajectory until `constraints.goal_time` has
/// elapsed. Tolerances given in the goal message win over the parameters.
class OM6DOF_CONTROLLERS_PUBLIC TrajectoryController
  : public controller_interface::ControllerInterface
{
public:
  using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
  using GoalHandle = rclcpp_action::ServerGoalHandle<FollowJointTrajectory>;
  using RealtimeGoalHandle = realtime_tools::RealtimeServerGoalHandle<FollowJointTrajectory>;
  using RealtimeGoalHandlePtr = std::shared_ptr<RealtimeGoalHandle>;

  TrajectoryController() = default;

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
  enum class CommandMode
  {
    Position,
    Effort
  };

  /// Per-joint tolerance set, either from parameters or from a goal message.
  struct Tolerance
  {
    double path{0.0};
    double goal{0.0};
  };

  /// A trajectory handed from a subscription or action callback to update().
  ///
  /// Built entirely outside the update loop: joints are already reordered and
  /// every value is already validated, so the realtime side only has to install
  /// it. `goal` is null for trajectories that came in over the topic.
  struct TrajectoryCommand
  {
    std::vector<Waypoint> waypoints;
    std::vector<Tolerance> tolerances;
    double goal_time_tolerance{0.0};

    /// Trajectory start. A zero stamp, which is what most senders use, means
    /// "start when it lands".
    rclcpp::Time start_time{0, 0, RCL_ROS_TIME};

    RealtimeGoalHandlePtr goal;
  };

  // ----- parameters -----
  std::vector<std::string> joint_names_;
  CommandMode command_mode_{CommandMode::Position};
  std::vector<double> kp_;
  std::vector<double> kd_;
  std::vector<double> effort_scale_;
  std::vector<double> max_effort_;
  std::vector<Tolerance> default_tolerances_;
  double default_goal_time_tolerance_{0.0};
  double stopped_velocity_tolerance_{0.01};
  bool gravity_feedforward_{false};
  std::vector<double> gravity_gain_;

  GravityModel model_;

  // ----- realtime state -----
  TrajectorySampler sampler_;
  std::vector<double> positions_;
  std::vector<double> velocities_;
  std::vector<double> desired_positions_;
  std::vector<double> desired_velocities_;
  std::vector<double> gravity_torque_;
  std::vector<double> hold_positions_;
  std::vector<Tolerance> active_tolerances_;
  double active_goal_time_tolerance_{0.0};

  rclcpp::Time trajectory_start_;
  bool trajectory_active_{false};
  RealtimeGoalHandlePtr active_goal_;
  std::shared_ptr<TrajectoryCommand> installed_command_;

  // ----- plumbing -----
  realtime_tools::RealtimeBuffer<std::shared_ptr<TrajectoryCommand>> incoming_command_;
  std::atomic<bool> cancel_requested_{false};

  rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr trajectory_subscription_;
  rclcpp_action::Server<FollowJointTrajectory>::SharedPtr action_server_;
  /// Goals whose verdicts still have to be delivered to the action server.
  ///
  /// update() only ever raises a flag on a RealtimeGoalHandle; somebody outside
  /// the realtime thread has to run it. One timer walks this list, so a goal
  /// that gets preempted is still carried to its terminal state after the
  /// controller has stopped tracking it as the active one.
  std::mutex live_goals_mutex_;
  std::vector<RealtimeGoalHandlePtr> live_goals_;
  rclcpp::TimerBase::SharedPtr goal_handle_timer_;
  rclcpp::Duration action_monitor_period_{std::chrono::milliseconds(50)};

  void runGoalHandles();

  using StatePublisher =
    realtime_tools::RealtimePublisher<control_msgs::msg::JointTrajectoryControllerState>;
  std::shared_ptr<rclcpp::Publisher<control_msgs::msg::JointTrajectoryControllerState>>
  state_publisher_;
  std::unique_ptr<StatePublisher> realtime_state_publisher_;
  double state_publish_period_{0.02};
  rclcpp::Time last_state_publish_time_;

  // ----- helpers -----
  bool readState();
  void writeCommand();
  void publishState(const rclcpp::Time & time);

  /// Reorder and validate a trajectory message into waypoints in this
  /// controller's joint order. Returns false with `error` filled in.
  bool convertTrajectory(
    const trajectory_msgs::msg::JointTrajectory & message, std::vector<Waypoint> & waypoints,
    std::string & error) const;

  /// Give up on the running trajectory and hold wherever the arm is.
  void abortActiveGoal(int32_t error_code, const std::string & message);

  rclcpp_action::GoalResponse handleGoal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const FollowJointTrajectory::Goal> goal);
  rclcpp_action::CancelResponse handleCancel(std::shared_ptr<GoalHandle> goal_handle);
  void handleAccepted(std::shared_ptr<GoalHandle> goal_handle);
};

}  // namespace om6dof_controllers

#endif  // OM6DOF_CONTROLLERS__TRAJECTORY_CONTROLLER_HPP_
