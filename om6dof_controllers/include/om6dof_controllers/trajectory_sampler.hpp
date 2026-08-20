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

#ifndef OM6DOF_CONTROLLERS__TRAJECTORY_SAMPLER_HPP_
#define OM6DOF_CONTROLLERS__TRAJECTORY_SAMPLER_HPP_

#include <string>
#include <vector>

#include "om6dof_controllers/visibility_control.h"

namespace om6dof_controllers
{

/// One trajectory waypoint, already reordered into the controller's joint
/// order. `velocities` is either empty or the same size as `positions`.
struct OM6DOF_CONTROLLERS_PUBLIC Waypoint
{
  double time_from_start{0.0};
  std::vector<double> positions;
  std::vector<double> velocities;
};

/// Time-indexed view of a joint trajectory.
///
/// Deliberately free of ROS types and of any clock, so the interpolation can be
/// unit-tested on its own. The controller converts messages and stamps into
/// waypoints and an elapsed time, and this class answers "where should the arm
/// be at t?".
///
/// Segments are cubic Hermite when both ends carry velocities, and linear
/// otherwise. Waypoint accelerations are ignored; a trajectory that needs them
/// honoured should be resampled more densely upstream.
class OM6DOF_CONTROLLERS_PUBLIC TrajectorySampler
{
public:
  /// Install a trajectory that starts from `start_positions` at t = 0.
  ///
  /// `waypoints` must be non-empty, uniformly sized, and strictly increasing in
  /// `time_from_start`, with every time strictly positive. An implicit
  /// zero-velocity waypoint at t = 0 is prepended unless the first waypoint is
  /// already at t = 0, so sampling never jumps away from where the arm is.
  ///
  /// \return false and fills `error` if the trajectory is rejected; the
  ///   previously installed trajectory is then left untouched.
  bool set(
    const std::vector<Waypoint> & waypoints, const std::vector<double> & start_positions,
    std::string & error);

  void clear() {points_.clear();}
  bool empty() const {return points_.empty();}
  size_t joint_count() const {return points_.empty() ? 0u : points_.front().positions.size();}

  /// Time of the last waypoint, in seconds. Zero when empty.
  double duration() const {return points_.empty() ? 0.0 : points_.back().time_from_start;}

  /// Desired state at `t` seconds after the trajectory start.
  ///
  /// Clamps to the first waypoint before t = 0 and holds the last waypoint at
  /// zero velocity after the end. Allocation-free when the outputs are already
  /// sized. Returns false only when no trajectory is installed.
  bool sample(double t, std::vector<double> & positions, std::vector<double> & velocities) const;

private:
  std::vector<Waypoint> points_;
};

}  // namespace om6dof_controllers

#endif  // OM6DOF_CONTROLLERS__TRAJECTORY_SAMPLER_HPP_
