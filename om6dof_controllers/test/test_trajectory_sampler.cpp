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

#include <cmath>
#include <limits>
#include <string>
#include <vector>

#include "gtest/gtest.h"
#include "om6dof_controllers/trajectory_sampler.hpp"

using om6dof_controllers::TrajectorySampler;
using om6dof_controllers::Waypoint;

namespace
{

Waypoint makePoint(double time, std::vector<double> positions, std::vector<double> velocities = {})
{
  Waypoint point;
  point.time_from_start = time;
  point.positions = std::move(positions);
  point.velocities = std::move(velocities);
  return point;
}

}  // namespace

TEST(TrajectorySampler, RejectsEmptyTrajectory)
{
  TrajectorySampler sampler;
  std::string error;
  EXPECT_FALSE(sampler.set({}, {0.0}, error));
  EXPECT_FALSE(error.empty());
  EXPECT_TRUE(sampler.empty());
}

TEST(TrajectorySampler, RejectsMismatchedPointWidth)
{
  TrajectorySampler sampler;
  std::string error;
  EXPECT_FALSE(sampler.set({makePoint(1.0, {0.0, 1.0})}, {0.0}, error));
  EXPECT_FALSE(error.empty());
}

TEST(TrajectorySampler, RejectsNonMonotonicTimes)
{
  TrajectorySampler sampler;
  std::string error;
  EXPECT_FALSE(
    sampler.set({makePoint(1.0, {0.0}), makePoint(1.0, {1.0})}, {0.0}, error));
  EXPECT_FALSE(error.empty());
}

TEST(TrajectorySampler, RejectsNonFinitePosition)
{
  TrajectorySampler sampler;
  std::string error;
  EXPECT_FALSE(
    sampler.set({makePoint(1.0, {std::numeric_limits<double>::quiet_NaN()})}, {0.0}, error));
  EXPECT_FALSE(error.empty());
}

TEST(TrajectorySampler, KeepsAnEarlierTrajectoryWhenTheNewOneIsRejected)
{
  TrajectorySampler sampler;
  std::string error;
  ASSERT_TRUE(sampler.set({makePoint(2.0, {1.0})}, {0.0}, error));
  EXPECT_FALSE(sampler.set({}, {0.0}, error));
  EXPECT_FALSE(sampler.empty());
  EXPECT_DOUBLE_EQ(2.0, sampler.duration());
}

TEST(TrajectorySampler, StartsFromTheMeasuredPosition)
{
  TrajectorySampler sampler;
  std::string error;
  ASSERT_TRUE(sampler.set({makePoint(2.0, {1.0})}, {0.25}, error)) << error;

  std::vector<double> positions;
  std::vector<double> velocities;

  ASSERT_TRUE(sampler.sample(0.0, positions, velocities));
  EXPECT_DOUBLE_EQ(0.25, positions[0]);
  EXPECT_DOUBLE_EQ(0.0, velocities[0]);
}

TEST(TrajectorySampler, LinearSegmentWithoutVelocities)
{
  TrajectorySampler sampler;
  std::string error;
  ASSERT_TRUE(sampler.set({makePoint(0.0, {0.0}), makePoint(2.0, {4.0})}, {0.0}, error)) << error;
  EXPECT_DOUBLE_EQ(2.0, sampler.duration());

  std::vector<double> positions;
  std::vector<double> velocities;

  ASSERT_TRUE(sampler.sample(1.0, positions, velocities));
  EXPECT_DOUBLE_EQ(2.0, positions[0]);
  EXPECT_DOUBLE_EQ(2.0, velocities[0]);
}

TEST(TrajectorySampler, CubicSegmentHitsItsEndpointsAndSlopes)
{
  TrajectorySampler sampler;
  std::string error;
  ASSERT_TRUE(
    sampler.set(
      {makePoint(0.0, {0.0}, {0.0}), makePoint(2.0, {1.0}, {0.5})}, {0.0}, error)) << error;

  std::vector<double> positions;
  std::vector<double> velocities;

  ASSERT_TRUE(sampler.sample(1.0e-9, positions, velocities));
  EXPECT_NEAR(0.0, positions[0], 1.0e-6);
  EXPECT_NEAR(0.0, velocities[0], 1.0e-6);

  ASSERT_TRUE(sampler.sample(2.0 - 1.0e-9, positions, velocities));
  EXPECT_NEAR(1.0, positions[0], 1.0e-6);
  EXPECT_NEAR(0.5, velocities[0], 1.0e-6);

  // A Hermite segment between equal-signed endpoint slopes stays inside the
  // endpoints; overshoot here would mean the basis functions are wrong.
  ASSERT_TRUE(sampler.sample(1.0, positions, velocities));
  EXPECT_GT(positions[0], 0.0);
  EXPECT_LT(positions[0], 1.0);
}

TEST(TrajectorySampler, HoldsTheLastPointAfterTheEnd)
{
  TrajectorySampler sampler;
  std::string error;
  ASSERT_TRUE(
    sampler.set(
      {makePoint(1.0, {0.5, -0.5}, {1.0, -1.0})}, {0.0, 0.0}, error)) << error;

  std::vector<double> positions;
  std::vector<double> velocities;

  ASSERT_TRUE(sampler.sample(9.0, positions, velocities));
  EXPECT_DOUBLE_EQ(0.5, positions[0]);
  EXPECT_DOUBLE_EQ(-0.5, positions[1]);
  EXPECT_DOUBLE_EQ(0.0, velocities[0]);
  EXPECT_DOUBLE_EQ(0.0, velocities[1]);
}

TEST(TrajectorySampler, ClampsBeforeTheStartAndOnNonFiniteTime)
{
  TrajectorySampler sampler;
  std::string error;
  ASSERT_TRUE(sampler.set({makePoint(1.0, {2.0})}, {0.75}, error)) << error;

  std::vector<double> positions;
  std::vector<double> velocities;

  ASSERT_TRUE(sampler.sample(-5.0, positions, velocities));
  EXPECT_DOUBLE_EQ(0.75, positions[0]);

  ASSERT_TRUE(sampler.sample(std::numeric_limits<double>::quiet_NaN(), positions, velocities));
  EXPECT_DOUBLE_EQ(0.75, positions[0]);
}

TEST(TrajectorySampler, PicksTheRightSegmentOfAMultiPointTrajectory)
{
  TrajectorySampler sampler;
  std::string error;
  ASSERT_TRUE(
    sampler.set(
      {makePoint(0.0, {0.0}), makePoint(1.0, {1.0}), makePoint(2.0, {1.0}),
        makePoint(3.0, {0.0})},
      {0.0}, error)) << error;

  std::vector<double> positions;
  std::vector<double> velocities;

  ASSERT_TRUE(sampler.sample(0.5, positions, velocities));
  EXPECT_DOUBLE_EQ(0.5, positions[0]);

  ASSERT_TRUE(sampler.sample(1.5, positions, velocities));
  EXPECT_DOUBLE_EQ(1.0, positions[0]);
  EXPECT_DOUBLE_EQ(0.0, velocities[0]);

  ASSERT_TRUE(sampler.sample(2.5, positions, velocities));
  EXPECT_DOUBLE_EQ(0.5, positions[0]);
  EXPECT_DOUBLE_EQ(-1.0, velocities[0]);
}

TEST(TrajectorySampler, SampleFailsWhenNothingIsInstalled)
{
  TrajectorySampler sampler;
  std::vector<double> positions;
  std::vector<double> velocities;
  EXPECT_FALSE(sampler.sample(0.0, positions, velocities));
}
