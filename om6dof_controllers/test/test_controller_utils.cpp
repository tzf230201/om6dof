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

#include <limits>
#include <string>
#include <vector>

#include "gtest/gtest.h"
#include "om6dof_controllers/controller_utils.hpp"

using om6dof_controllers::clampSymmetric;
using om6dof_controllers::perJointParameter;

TEST(PerJointParameter, EmptyListFallsBack)
{
  std::vector<double> out;
  std::string error;
  ASSERT_TRUE(perJointParameter("gain", {}, 3, 1.5, out, error)) << error;
  EXPECT_EQ(std::vector<double>({1.5, 1.5, 1.5}), out);
}

TEST(PerJointParameter, SingleValueIsBroadcast)
{
  std::vector<double> out;
  std::string error;
  ASSERT_TRUE(perJointParameter("gain", {2.0}, 3, 0.0, out, error)) << error;
  EXPECT_EQ(std::vector<double>({2.0, 2.0, 2.0}), out);
}

TEST(PerJointParameter, OnePerJointIsKept)
{
  std::vector<double> out;
  std::string error;
  ASSERT_TRUE(perJointParameter("gain", {1.0, 2.0, 3.0}, 3, 0.0, out, error)) << error;
  EXPECT_EQ(std::vector<double>({1.0, 2.0, 3.0}), out);
}

TEST(PerJointParameter, AnyOtherLengthIsAnError)
{
  std::vector<double> out;
  std::string error;
  EXPECT_FALSE(perJointParameter("gain", {1.0, 2.0}, 3, 0.0, out, error));
  EXPECT_NE(std::string::npos, error.find("gain"));
}

TEST(PerJointParameter, NonFiniteValuesAreAnError)
{
  std::vector<double> out;
  std::string error;
  EXPECT_FALSE(
    perJointParameter(
      "gain", {1.0, std::numeric_limits<double>::infinity(), 3.0}, 3, 0.0, out, error));
  EXPECT_FALSE(error.empty());
}

TEST(ClampSymmetric, ClampsBothWays)
{
  EXPECT_DOUBLE_EQ(2.0, clampSymmetric(5.0, 2.0));
  EXPECT_DOUBLE_EQ(-2.0, clampSymmetric(-5.0, 2.0));
  EXPECT_DOUBLE_EQ(1.0, clampSymmetric(1.0, 2.0));
}

TEST(ClampSymmetric, NonPositiveLimitMeansNoLimit)
{
  EXPECT_DOUBLE_EQ(5.0, clampSymmetric(5.0, 0.0));
  EXPECT_DOUBLE_EQ(5.0, clampSymmetric(5.0, -1.0));
  EXPECT_DOUBLE_EQ(5.0, clampSymmetric(5.0, std::numeric_limits<double>::quiet_NaN()));
}

// ----- current_limit semantics -----
//
// These guard the safety contract of Dynamixel operating mode 5, where the
// effort command is a ceiling on the servo's own position loop rather than a
// torque: a command of zero means "you may pull nothing" and the arm drops.

using om6dof_controllers::currentLimitCommand;

TEST(CurrentLimitCommand, StartsAtTheCeilingSoActivationIsNeverSlack)
{
  // ramp = 0 is the instant of activation. Whatever the pose, whatever the
  // gains, the joint must still be allowed its full current.
  EXPECT_DOUBLE_EQ(500.0, currentLimitCommand(0.0, 285.4, 100.0, 120.0, 500.0, 0.0));
  EXPECT_DOUBLE_EQ(500.0, currentLimitCommand(9.9, 285.4, 100.0, 120.0, 500.0, 0.0));
  EXPECT_DOUBLE_EQ(500.0, currentLimitCommand(-9.9, 285.4, 100.0, 120.0, 500.0, 0.0));
}

TEST(CurrentLimitCommand, NeverFallsBelowTheFloor)
{
  // A pose where gravity does no work must not collapse the limit to nothing.
  EXPECT_DOUBLE_EQ(120.0, currentLimitCommand(0.0, 285.4, 0.0, 120.0, 500.0, 1.0));
  EXPECT_DOUBLE_EQ(120.0, currentLimitCommand(1e-9, 285.4, 0.0, 120.0, 500.0, 1.0));

  // And nowhere along the ramp either.
  for (int step = 0; step <= 10; ++step) {
    const double command =
      currentLimitCommand(0.0, 285.4, 0.0, 120.0, 500.0, 0.1 * step);
    EXPECT_GE(command, 120.0);
  }
}

TEST(CurrentLimitCommand, IsAMagnitudeSoTheSignOfGravityDoesNotMatter)
{
  const double up = currentLimitCommand(0.306, 384.7, 100.0, 120.0, 400.0, 1.0);
  const double down = currentLimitCommand(-0.306, 384.7, 100.0, 120.0, 400.0, 1.0);
  EXPECT_DOUBLE_EQ(up, down);
  EXPECT_NEAR(0.306 * 384.7 + 100.0, up, 1.0e-9);
}

TEST(CurrentLimitCommand, RespectsTheCeilingOnceRamped)
{
  EXPECT_DOUBLE_EQ(400.0, currentLimitCommand(5.0, 384.7, 100.0, 120.0, 400.0, 1.0));
}

TEST(CurrentLimitCommand, EasesDownwardsAndNeverOvershootsEitherEnd)
{
  const double ceiling = 500.0;
  const double settled = currentLimitCommand(0.306, 285.4, 100.0, 120.0, ceiling, 1.0);
  ASSERT_LT(settled, ceiling);

  double previous = currentLimitCommand(0.306, 285.4, 100.0, 120.0, ceiling, 0.0);
  EXPECT_DOUBLE_EQ(ceiling, previous);
  for (int step = 1; step <= 10; ++step) {
    const double command =
      currentLimitCommand(0.306, 285.4, 100.0, 120.0, ceiling, 0.1 * step);
    EXPECT_LE(command, previous);
    EXPECT_GE(command, settled);
    previous = command;
  }
  EXPECT_NEAR(settled, previous, 1.0e-9);
}

// ----- setpoint deadband -----
//
// The bound on error is the whole safety and feel story of the leader arm: the
// servo's position loop can only ever see `deadband`, so what it pushes back
// with is bounded no matter how far the arm travels.

using om6dof_controllers::followSetpoint;

TEST(FollowSetpoint, LeavesTheSetpointAloneInsideTheBand)
{
  EXPECT_DOUBLE_EQ(1.00, followSetpoint(1.00, 1.00, 0.05));
  EXPECT_DOUBLE_EQ(1.00, followSetpoint(1.00, 1.03, 0.05));
  EXPECT_DOUBLE_EQ(1.00, followSetpoint(1.00, 0.97, 0.05));
}

TEST(FollowSetpoint, NeverSitsFurtherThanTheBandFromTheArm)
{
  // However far the arm is dragged, in either direction, in one step or many.
  EXPECT_DOUBLE_EQ(1.95, followSetpoint(1.00, 2.00, 0.05));
  EXPECT_DOUBLE_EQ(-1.95, followSetpoint(-1.00, -2.00, 0.05));

  double setpoint = 0.0;
  for (int step = 1; step <= 50; ++step) {
    const double measured = 0.02 * step;
    setpoint = followSetpoint(setpoint, measured, 0.05);
    EXPECT_LE(std::abs(setpoint - measured), 0.05 + 1.0e-12);
  }
}

TEST(FollowSetpoint, HoldsItsGroundWhenTheArmComesBack)
{
  // Pushed out to the edge, then released: the setpoint stays where it was
  // dragged to rather than snapping back to the arm.
  const double dragged = followSetpoint(0.0, 0.30, 0.05);
  EXPECT_DOUBLE_EQ(0.25, dragged);
  EXPECT_DOUBLE_EQ(0.25, followSetpoint(dragged, 0.28, 0.05));
}

TEST(FollowSetpoint, ANonPositiveBandPinsTheSetpointToTheArm)
{
  // Configure rejects this, but the function must still be total.
  EXPECT_DOUBLE_EQ(2.0, followSetpoint(1.0, 2.0, 0.0));
  EXPECT_DOUBLE_EQ(2.0, followSetpoint(1.0, 2.0, -1.0));
}
