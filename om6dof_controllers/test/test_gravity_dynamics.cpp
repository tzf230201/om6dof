// Copyright 2026 OM6DOF maintainers.
// Licensed under the Apache License, Version 2.0.

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <string>
#include <vector>

#include "gtest/gtest.h"
#include "om6dof_controllers/gravity_model.hpp"

using om6dof_controllers::GravityModel;

namespace
{
constexpr double kG = 9.80665;
const std::array<double, 3> kGravity{0.0, 0.0, -kG};
const char * kPendulum = R"(<robot name="pendulum">
  <link name="base"/>
  <link name="arm"><inertial><origin xyz="0.5 0 0"/>
    <mass value="2"/>
    <inertia ixx="0.2" ixy="0" ixz="0" iyy="0.2" iyz="0" izz="0.2"/>
  </inertial></link>
  <joint name="joint1" type="revolute"><parent link="base"/><child link="arm"/>
    <axis xyz="0 1 0"/><limit lower="-3" upper="3" effort="50" velocity="10"/>
    <dynamics friction="0.8" damping="0.6"/>
  </joint>
</robot>)";

const char * kTwoLink = R"(<robot name="two_link">
  <link name="base"/>
  <link name="arm"><inertial><origin xyz="0.5 0 0"/><mass value="1"/>
    <inertia ixx="0.1" ixy="0" ixz="0" iyy="0.1" iyz="0" izz="0.1"/>
  </inertial></link>
  <link name="forearm"><inertial><origin xyz="0.5 0 0"/><mass value="1"/>
    <inertia ixx="0.1" ixy="0" ixz="0" iyy="0.1" iyz="0" izz="0.1"/>
  </inertial></link>
  <joint name="joint1" type="revolute"><parent link="base"/><child link="arm"/>
    <axis xyz="0 1 0"/><limit lower="-3" upper="3" effort="50" velocity="10"/>
  </joint>
  <joint name="joint2" type="revolute"><parent link="arm"/><child link="forearm"/>
    <origin xyz="1 0 0"/><axis xyz="0 1 0"/>
    <limit lower="-3" upper="3" effort="50" velocity="10"/>
  </joint>
</robot>)";
}  // namespace

TEST(GravityDynamics, StaticMatchesGravityAndExcludesUrdfFriction)
{
  GravityModel model;
  ASSERT_TRUE(model.configure(kPendulum, "base", "arm", {"joint1"}, kGravity).empty());
  std::vector<double> gravity(1), torque(1);
  for (double q : {0.0, 0.3, 1.1}) {
    ASSERT_TRUE(model.computeChecked({q}, gravity));
    ASSERT_TRUE(model.computeDynamicsChecked({q}, {0.0}, {0.0}, torque));
    EXPECT_NEAR(gravity[0], torque[0], 1e-12);
    // A one-link pendulum has no Coriolis torque, despite its nonzero
    // configured URDF damping and friction.
    ASSERT_TRUE(model.computeDynamicsChecked({q}, {0.8}, {0.0}, torque));
    EXPECT_NEAR(gravity[0], torque[0], 1e-12);
  }
  ASSERT_TRUE(model.computeDynamicsChecked({0.0}, {0.0}, {0.0}, torque));
  EXPECT_NEAR(-kG, torque[0], 1e-10);
}

TEST(GravityDynamics, PendulumAccelerationUsesParallelAxisInertia)
{
  GravityModel model;
  ASSERT_TRUE(model.configure(kPendulum, "base", "arm", {"joint1"}, kGravity).empty());
  std::vector<double> gravity(1), torque(1);
  ASSERT_TRUE(model.computeChecked({0.4}, gravity));
  for (double acceleration : {-2.0, 0.5, 3.0}) {
    ASSERT_TRUE(model.computeDynamicsChecked({0.4}, {0.7}, {acceleration}, torque));
    EXPECT_NEAR(gravity[0] + (0.2 + 2.0 * 0.5 * 0.5) * acceleration, torque[0], 1e-10);
  }
}

TEST(GravityDynamics, TwoLinkMatchesAnalyticMassAndCoriolisTerms)
{
  GravityModel model;
  ASSERT_TRUE(model.configure(kTwoLink, "base", "forearm", {"joint1", "joint2"}, kGravity).empty());
  const double q2 = 0.4, v1 = 0.8, v2 = 0.6, a1 = 0.5, a2 = -0.3;
  std::vector<double> gravity(2), torque(2);
  ASSERT_TRUE(model.computeChecked({0.3, q2}, gravity));
  ASSERT_TRUE(model.computeDynamicsChecked({0.3, q2}, {v1, v2}, {a1, a2}, torque));
  const double h11 = 1.7 + std::cos(q2);
  const double h12 = 0.35 + 0.5 * std::cos(q2);
  const double h22 = 0.35;
  const double c1 = -0.5 * std::sin(q2) * (2.0 * v1 * v2 + v2 * v2);
  const double c2 = 0.5 * std::sin(q2) * v1 * v1;
  EXPECT_NEAR(gravity[0] + h11 * a1 + h12 * a2 + c1, torque[0], 1e-10);
  EXPECT_NEAR(gravity[1] + h12 * a1 + h22 * a2 + c2, torque[1], 1e-10);
}

TEST(GravityDynamics, ReorderedControllerJointsMapEveryInputAndOutput)
{
  GravityModel natural, reordered;
  ASSERT_TRUE(natural.configure(kTwoLink, "base", "forearm", {"joint1", "joint2"}, kGravity).empty());
  ASSERT_TRUE(reordered.configure(kTwoLink, "base", "forearm", {"joint2", "joint1"}, kGravity).empty());
  std::vector<double> expected(2), actual(2);
  ASSERT_TRUE(natural.computeDynamicsChecked({0.3, -0.4}, {0.5, -0.2}, {0.1, 0.4}, expected));
  ASSERT_TRUE(reordered.computeDynamicsChecked({-0.4, 0.3}, {-0.2, 0.5}, {0.4, 0.1}, actual));
  EXPECT_NEAR(expected[0], actual[1], 1e-12);
  EXPECT_NEAR(expected[1], actual[0], 1e-12);
}

TEST(GravityDynamics, FixedOffChainToolContributesMassAndInertia)
{
  std::string description(kPendulum);
  description.insert(description.find("</robot>"), R"(
    <link name="tool"><inertial><mass value="1"/>
      <inertia ixx="0.05" ixy="0" ixz="0" iyy="0.05" iyz="0" izz="0.05"/>
    </inertial></link>
    <joint name="tool_fixed" type="fixed"><parent link="arm"/><child link="tool"/>
      <origin xyz="1 0 0"/>
    </joint>)");
  GravityModel model;
  ASSERT_TRUE(model.configure(description, "base", "arm", {"joint1"}, kGravity).empty());
  EXPECT_DOUBLE_EQ(1.0, model.folded_mass());
  std::vector<double> torque(1);
  ASSERT_TRUE(model.computeDynamicsChecked({0.0}, {0.7}, {2.0}, torque));
  EXPECT_NEAR(-2.0 * kG + (0.7 + 1.05) * 2.0, torque[0], 1e-10);
}

TEST(GravityDynamics, ExtraControllerJointHasZeroTorque)
{
  GravityModel model;
  ASSERT_TRUE(model.configure(kPendulum, "base", "arm", {"unrelated", "joint1"}, kGravity).empty());
  std::vector<double> torque{123.0, 123.0};
  ASSERT_TRUE(model.computeDynamicsChecked({0.8, 0.0}, {1.3, 0.4}, {2.0, 1.0}, torque));
  EXPECT_DOUBLE_EQ(0.0, torque[0]);
  EXPECT_NEAR(-kG + 0.7, torque[1], 1e-10);
}

TEST(GravityDynamics, BadSizesAndNonfiniteInputsZeroAllOutput)
{
  GravityModel model;
  ASSERT_TRUE(model.configure(kTwoLink, "base", "forearm", {"joint1", "joint2"}, kGravity).empty());
  std::vector<double> q(2), qd(2), qdd(2), torque(2, 123.0);
  for (size_t size : {0u, 1u, 3u}) {
    for (size_t input : {0u, 1u, 2u}) {
      auto positions = q, velocities = qd, accelerations = qdd;
      (input == 0 ? positions : input == 1 ? velocities : accelerations).resize(size);
      torque.assign(2, 123.0);
      EXPECT_FALSE(model.computeDynamicsChecked(positions, velocities, accelerations, torque));
      EXPECT_EQ((std::vector<double>{0.0, 0.0}), torque);
    }
    torque.assign(size, 123.0);
    EXPECT_FALSE(model.computeDynamicsChecked(q, qd, qdd, torque));
    EXPECT_EQ(std::vector<double>(size, 0.0), torque);
  }
  torque.resize(2);
  for (double invalid : {std::numeric_limits<double>::quiet_NaN(),
      std::numeric_limits<double>::infinity(), -std::numeric_limits<double>::infinity()})
  {
    for (size_t input : {0u, 1u, 2u}) {
      auto positions = q, velocities = qd, accelerations = qdd;
      (input == 0 ? positions : input == 1 ? velocities : accelerations)[1] = invalid;
      torque.assign(2, 123.0);
      EXPECT_FALSE(model.computeDynamicsChecked(positions, velocities, accelerations, torque));
      EXPECT_EQ((std::vector<double>{0.0, 0.0}), torque);
    }
  }
}

TEST(GravityDynamics, RejectsUnconfiguredAndAliasedOutput)
{
  GravityModel model;
  std::vector<double> torque{123.0};
  EXPECT_FALSE(model.computeDynamicsChecked({0.0}, {0.0}, {0.0}, torque));
  EXPECT_DOUBLE_EQ(0.0, torque[0]);
  ASSERT_TRUE(model.configure(kPendulum, "base", "arm", {"joint1"}, kGravity).empty());
  torque[0] = 0.4;
  EXPECT_FALSE(model.computeDynamicsChecked(torque, {0.0}, {0.0}, torque));
  EXPECT_DOUBLE_EQ(0.0, torque[0]);
}

TEST(GravityDynamics, NonfiniteComputedTorqueFailsWithoutPartialWrites)
{
  GravityModel model;
  ASSERT_TRUE(model.configure(kTwoLink, "base", "forearm", {"joint1", "joint2"}, kGravity).empty());
  std::vector<double> torque{123.0, 123.0};
  const double largest = std::numeric_limits<double>::max();
  EXPECT_FALSE(model.computeDynamicsChecked({0.0, 0.0}, {0.0, 0.0}, {largest, largest}, torque));
  EXPECT_EQ((std::vector<double>{0.0, 0.0}), torque);
}
