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

#include "om6dof_controllers/trajectory_sampler.hpp"

#include <algorithm>
#include <cmath>

namespace om6dof_controllers
{
namespace
{

/// Cubic Hermite value and derivative on a unit-normalised segment.
void hermite(
  double p0, double v0, double p1, double v1, double dt, double s, double & position,
  double & velocity)
{
  const double s2 = s * s;
  const double s3 = s2 * s;
  const double h00 = 2.0 * s3 - 3.0 * s2 + 1.0;
  const double h10 = s3 - 2.0 * s2 + s;
  const double h01 = -2.0 * s3 + 3.0 * s2;
  const double h11 = s3 - s2;

  position = h00 * p0 + h10 * dt * v0 + h01 * p1 + h11 * dt * v1;

  const double dh00 = 6.0 * s2 - 6.0 * s;
  const double dh10 = 3.0 * s2 - 4.0 * s + 1.0;
  const double dh01 = -6.0 * s2 + 6.0 * s;
  const double dh11 = 3.0 * s2 - 2.0 * s;

  velocity = (dh00 * p0 + dh01 * p1) / dt + dh10 * v0 + dh11 * v1;
}

}  // namespace

bool TrajectorySampler::set(
  const std::vector<Waypoint> & waypoints, const std::vector<double> & start_positions,
  std::string & error)
{
  error.clear();

  if (waypoints.empty()) {
    error = "trajectory has no points";
    return false;
  }

  const size_t n = start_positions.size();
  double previous_time = -1.0;
  for (const auto & point : waypoints) {
    if (point.positions.size() != n) {
      error = "a trajectory point does not carry one position per controlled joint";
      return false;
    }
    if (!point.velocities.empty() && point.velocities.size() != n) {
      error = "a trajectory point carries velocities for only some joints";
      return false;
    }
    if (!std::isfinite(point.time_from_start) || point.time_from_start < 0.0) {
      error = "a trajectory point has a negative or non-finite time_from_start";
      return false;
    }
    if (point.time_from_start <= previous_time) {
      error = "trajectory times are not strictly increasing";
      return false;
    }
    for (size_t i = 0; i < n; ++i) {
      if (!std::isfinite(point.positions[i])) {
        error = "a trajectory point has a non-finite position";
        return false;
      }
      if (!point.velocities.empty() && !std::isfinite(point.velocities[i])) {
        error = "a trajectory point has a non-finite velocity";
        return false;
      }
    }
    previous_time = point.time_from_start;
  }

  std::vector<Waypoint> points;
  points.reserve(waypoints.size() + 1);

  // Start from where the arm is, so the first segment closes whatever gap the
  // trajectory's first point left rather than stepping onto it.
  if (waypoints.front().time_from_start > 0.0) {
    Waypoint start;
    start.time_from_start = 0.0;
    start.positions = start_positions;
    start.velocities.assign(n, 0.0);
    points.push_back(std::move(start));
  }
  points.insert(points.end(), waypoints.begin(), waypoints.end());

  points_ = std::move(points);
  return true;
}

bool TrajectorySampler::sample(
  double t, std::vector<double> & positions, std::vector<double> & velocities) const
{
  if (points_.empty()) {
    return false;
  }

  const size_t n = points_.front().positions.size();
  positions.resize(n);
  velocities.resize(n);

  if (!std::isfinite(t) || t <= points_.front().time_from_start) {
    positions = points_.front().positions;
    std::fill(velocities.begin(), velocities.end(), 0.0);
    return true;
  }

  if (t >= points_.back().time_from_start) {
    positions = points_.back().positions;
    std::fill(velocities.begin(), velocities.end(), 0.0);
    return true;
  }

  // Points are few and time advances monotonically in practice, but a linear
  // scan over a hundred waypoints is still nothing next to one update cycle.
  size_t index = 0;
  while (index + 2 < points_.size() && points_[index + 1].time_from_start <= t) {
    ++index;
  }

  const Waypoint & a = points_[index];
  const Waypoint & b = points_[index + 1];
  const double dt = b.time_from_start - a.time_from_start;
  const double s = (t - a.time_from_start) / dt;
  const bool cubic = !a.velocities.empty() && !b.velocities.empty();

  for (size_t i = 0; i < n; ++i) {
    if (cubic) {
      hermite(
        a.positions[i], a.velocities[i], b.positions[i], b.velocities[i], dt, s, positions[i],
        velocities[i]);
    } else {
      velocities[i] = (b.positions[i] - a.positions[i]) / dt;
      positions[i] = a.positions[i] + velocities[i] * (t - a.time_from_start);
    }
  }

  return true;
}

}  // namespace om6dof_controllers
