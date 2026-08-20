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

#ifndef OM6DOF_CONTROLLERS__CONTROLLER_UTILS_HPP_
#define OM6DOF_CONTROLLERS__CONTROLLER_UTILS_HPP_

#include <string>
#include <vector>

#include "om6dof_controllers/visibility_control.h"

namespace om6dof_controllers
{

/// Turn a per-joint YAML list into exactly `joint_count` values.
///
/// An empty list means "use `fallback` everywhere" and a single value is
/// broadcast to every joint, so a gain that happens to be uniform does not have
/// to be written out six times. Any other length is a configuration error
/// rather than something to pad or truncate silently.
OM6DOF_CONTROLLERS_PUBLIC bool perJointParameter(
  const std::string & name, const std::vector<double> & raw, size_t joint_count, double fallback,
  std::vector<double> & out, std::string & error);

/// Fetch the robot description, from the controller's own parameter when it has
/// one and from a latched topic otherwise.
///
/// Humble's controller_manager does not push the description down to
/// controllers, so the topic path is the one that normally runs: it listens on
/// `topic` with transient-local QoS, which robot_state_publisher already
/// publishes with. Spinning happens on a throwaway node of its own, never on
/// the controller's node, whose executor belongs to controller_manager.
///
/// \return the URDF string, or an empty string with `error` filled in.
OM6DOF_CONTROLLERS_PUBLIC std::string fetchRobotDescription(
  const std::string & parameter_value, const std::string & node_namespace,
  const std::string & topic, double timeout_seconds, std::string & error);

/// The current limit to command for a joint that needs `torque` newton-metres
/// to hold its pose, under `current_limit` semantics.
///
/// The result is a magnitude -- which way gravity pulls is the servo's problem,
/// not the limit's -- floored at `min_effort` so a joint is never told it may
/// pull nothing, and capped at `max_effort`. `ramp` runs 0 -> 1 and eases the
/// command *down* from `max_effort`, so activation goes from firmly held to
/// compliant and never passes through slack.
OM6DOF_CONTROLLERS_PUBLIC double currentLimitCommand(
  double torque, double effort_scale, double headroom, double min_effort, double max_effort,
  double ramp);

/// The position setpoint to command next, given where the arm actually is.
///
/// The setpoint is dragged along so it never sits further than `deadband` from
/// the measurement, and is otherwise left alone. That bound is the whole point:
/// the servo's position loop can only ever see `deadband` of error, so the
/// force it pushes back with is bounded and smooth instead of building until
/// something gives. Holding still and being pushed become the same case.
///
/// It also means the setpoint follows a sagging arm. That is safe only while
/// the current ceiling is enough to hold the arm at `deadband` of error -- if
/// it is not, the arm walks itself down. Size the deadband up until it holds.
OM6DOF_CONTROLLERS_PUBLIC double followSetpoint(
  double setpoint, double measured, double deadband);

/// `value` clamped to +/- `limit`. A non-positive or non-finite limit means no
/// clamping.
OM6DOF_CONTROLLERS_PUBLIC double clampSymmetric(double value, double limit);

}  // namespace om6dof_controllers

#endif  // OM6DOF_CONTROLLERS__CONTROLLER_UTILS_HPP_
