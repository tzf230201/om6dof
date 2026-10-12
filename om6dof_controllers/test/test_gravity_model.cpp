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

#include <array>
#include <cmath>
#include <limits>
#include <string>
#include <vector>

#include "gtest/gtest.h"
#include "om6dof_controllers/gravity_model.hpp"

using om6dof_controllers::FrictionParameters;
using om6dof_controllers::GravityModel;

namespace
{

constexpr double kG = 9.80665;
const std::array<double, 3> kGravity{0.0, 0.0, -kG};

/// Two unit-mass one-metre links, both rotating about +Y, centres of mass at
/// their midpoints. Straight out along +X at q = 0, so the gravity torques are
/// something one can work out by hand.
const char * kUrdf = R"(<?xml version="1.0"?>
<robot name="two_link">
  <link name="base_link"/>
  <link name="link1">
    <inertial>
      <origin xyz="0.5 0 0"/>
      <mass value="1.0"/>
      <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/>
    </inertial>
  </link>
  <link name="link2">
    <inertial>
      <origin xyz="0.5 0 0"/>
      <mass value="1.0"/>
      <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/>
    </inertial>
  </link>
  <joint name="joint1" type="revolute">
    <parent link="base_link"/>
    <child link="link1"/>
    <origin xyz="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-3.14" upper="3.14" effort="10" velocity="10"/>
  </joint>
  <joint name="joint2" type="revolute">
    <parent link="link1"/>
    <child link="link2"/>
    <origin xyz="1 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-3.14" upper="3.14" effort="10" velocity="10"/>
  </joint>
</robot>)";

/// The same arm with a 1 kg tool bolted to the end of link2, one metre further
/// out. It is a branch: the chain base_link -> link2 does not pass through it,
/// which is exactly how the real arm carries its gripper and wrist camera.
const char * kUrdfBranch = R"(<?xml version="1.0"?>
<robot name="two_link_tool">
  <link name="base_link"/>
  <link name="link1">
    <inertial>
      <origin xyz="0.5 0 0"/>
      <mass value="1.0"/>
      <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/>
    </inertial>
  </link>
  <link name="link2">
    <inertial>
      <origin xyz="0.5 0 0"/>
      <mass value="1.0"/>
      <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/>
    </inertial>
  </link>
  <link name="tool">
    <inertial>
      <origin xyz="0 0 0"/>
      <mass value="1.0"/>
      <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/>
    </inertial>
  </link>
  <joint name="joint1" type="revolute">
    <parent link="base_link"/>
    <child link="link1"/>
    <origin xyz="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-3.14" upper="3.14" effort="10" velocity="10"/>
  </joint>
  <joint name="joint2" type="revolute">
    <parent link="link1"/>
    <child link="link2"/>
    <origin xyz="1 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-3.14" upper="3.14" effort="10" velocity="10"/>
  </joint>
  <joint name="tool_joint" type="fixed">
    <parent link="link2"/>
    <child link="tool"/>
    <origin xyz="1 0 0"/>
  </joint>
</robot>)";

}  // namespace

TEST(GravityModel, RejectsAnEmptyDescription)
{
  GravityModel model;
  EXPECT_FALSE(model.configure("", "base_link", "link2", {"joint1", "joint2"}, kGravity).empty());
  EXPECT_FALSE(model.is_configured());
}

TEST(UrdfFriction, MissingDynamicsIsZeroAndOrderIsByJointName)
{
  std::string urdf(kUrdf);
  urdf.insert(urdf.find("</joint>"), "<dynamics friction=\"0.3\" damping=\"0.4\"/>");
  std::vector<FrictionParameters> f;
  ASSERT_TRUE(GravityModel::readUrdfFriction(urdf, {"joint2", "joint1"}, 0.05, f).empty());
  ASSERT_EQ(2u, f.size());
  EXPECT_DOUBLE_EQ(0.0, f[0].coulomb);
  EXPECT_DOUBLE_EQ(0.0, f[0].viscous);
  EXPECT_DOUBLE_EQ(0.3, f[1].coulomb);
  EXPECT_DOUBLE_EQ(0.4, f[1].viscous);
  EXPECT_DOUBLE_EQ(0.05, f[1].deadzone);
  EXPECT_NEAR(0.3 * std::tanh(4.0) + 0.4 * 0.2, GravityModel::friction(0.2, f[1]), 1e-12);
  EXPECT_NEAR(-GravityModel::friction(0.2, f[1]), GravityModel::friction(-0.2, f[1]), 1e-12);
  EXPECT_DOUBLE_EQ(0.0, GravityModel::friction(0.0, f[1]));
}

TEST(UrdfFriction, MissingAttributeIsZero)
{
  for (const std::string attribute : {"damping", "friction"}) {
    std::string urdf(kUrdf);
    urdf.insert(urdf.find("</joint>"), "<dynamics " + attribute + "=\"0.1\"/>");
    std::vector<FrictionParameters> f;
    ASSERT_TRUE(GravityModel::readUrdfFriction(urdf, {"joint1"}, 0.05, f).empty());
    EXPECT_DOUBLE_EQ(attribute == "damping" ? 0.1 : 0.0, f[0].viscous);
    EXPECT_DOUBLE_EQ(attribute == "friction" ? 0.1 : 0.0, f[0].coulomb);
  }
}

TEST(UrdfFriction, RejectsInvalidCoefficientsNamesAndSmoothing)
{
  std::vector<FrictionParameters> f(1);
  for (const std::string attribute : {"damping", "friction"}) {
    for (const std::string value : {"-0.1", "nan", "inf"}) {
      std::string urdf(kUrdf);
      urdf.insert(urdf.find("</joint>"), "<dynamics " + attribute + "=\"" + value + "\"/>");
      EXPECT_FALSE(GravityModel::readUrdfFriction(urdf, {"joint1"}, 0.05, f).empty());
      EXPECT_TRUE(f.empty());
    }
  }
  EXPECT_FALSE(GravityModel::readUrdfFriction(kUrdf, {"typo"}, 0.05, f).empty());
  EXPECT_FALSE(GravityModel::readUrdfFriction(kUrdf, {"joint1", "joint1"}, 0.05, f).empty());
  EXPECT_FALSE(GravityModel::readUrdfFriction(kUrdfBranch, {"tool_joint"}, 0.05, f).empty());
  for (double d : {0.0, -0.1, std::numeric_limits<double>::infinity()}) {
    EXPECT_FALSE(GravityModel::readUrdfFriction(kUrdf, {"joint1"}, d, f).empty());
  }
  EXPECT_FALSE(GravityModel::readUrdfFriction("not xml", {"joint1"}, 0.05, f).empty());
  EXPECT_TRUE(f.empty());
}

TEST(GravityModel, RejectsUnparseableXml)
{
  GravityModel model;
  EXPECT_FALSE(
    model.configure("not a robot", "base_link", "link2", {"joint1", "joint2"}, kGravity).empty());
}

TEST(GravityModel, RejectsAChainThatDoesNotExist)
{
  GravityModel model;
  const std::string error =
    model.configure(kUrdf, "base_link", "no_such_link", {"joint1", "joint2"}, kGravity);
  EXPECT_NE(std::string::npos, error.find("no KDL chain"));
}

TEST(GravityModel, RejectsAChainJointTheControllerDoesNotOwn)
{
  GravityModel model;
  const std::string error = model.configure(kUrdf, "base_link", "link2", {"joint1"}, kGravity);
  EXPECT_NE(std::string::npos, error.find("joint2"));
  EXPECT_FALSE(model.is_configured());
}

TEST(GravityModel, ReportsTheChainJoints)
{
  GravityModel model;
  ASSERT_TRUE(model.configure(kUrdf, "base_link", "link2", {"joint1", "joint2"}, kGravity).empty());
  ASSERT_EQ(2u, model.chain_joint_names().size());
  EXPECT_EQ("joint1", model.chain_joint_names()[0]);
  EXPECT_EQ("joint2", model.chain_joint_names()[1]);
}

TEST(GravityModel, HorizontalArmNeedsTheHandComputedTorque)
{
  GravityModel model;
  ASSERT_TRUE(model.configure(kUrdf, "base_link", "link2", {"joint1", "joint2"}, kGravity).empty());

  std::vector<double> torques(2, 0.0);
  model.compute({0.0, 0.0}, torques);

  // joint1 carries 1 kg at 0.5 m plus 1 kg at 1.5 m; joint2 carries 1 kg at
  // 0.5 m. The sign follows KDL's convention, so only the magnitude is asserted
  // here and the relative signs are checked below.
  EXPECT_NEAR(2.0 * kG, std::abs(torques[0]), 1.0e-6);
  EXPECT_NEAR(0.5 * kG, std::abs(torques[1]), 1.0e-6);
  EXPECT_GT(torques[0] * torques[1], 0.0);
}

TEST(GravityModel, VerticalArmNeedsNoTorque)
{
  GravityModel model;
  ASSERT_TRUE(model.configure(kUrdf, "base_link", "link2", {"joint1", "joint2"}, kGravity).empty());

  std::vector<double> torques(2, 0.0);
  model.compute({M_PI_2, 0.0}, torques);

  EXPECT_NEAR(0.0, torques[0], 1.0e-6);
  EXPECT_NEAR(0.0, torques[1], 1.0e-6);
}

TEST(GravityModel, TorqueFlipsWithTheArm)
{
  GravityModel model;
  ASSERT_TRUE(model.configure(kUrdf, "base_link", "link2", {"joint1", "joint2"}, kGravity).empty());

  std::vector<double> forward(2, 0.0);
  std::vector<double> backward(2, 0.0);
  model.compute({0.0, 0.0}, forward);
  model.compute({M_PI, 0.0}, backward);

  EXPECT_NEAR(forward[0], -backward[0], 1.0e-6);
  EXPECT_NEAR(forward[1], -backward[1], 1.0e-6);
}

TEST(GravityModel, RespectsTheControllersJointOrder)
{
  GravityModel natural;
  GravityModel reversed;
  ASSERT_TRUE(
    natural.configure(kUrdf, "base_link", "link2", {"joint1", "joint2"}, kGravity).empty());
  ASSERT_TRUE(
    reversed.configure(kUrdf, "base_link", "link2", {"joint2", "joint1"}, kGravity).empty());

  std::vector<double> in_order(2, 0.0);
  std::vector<double> out_of_order(2, 0.0);
  natural.compute({0.3, -0.4}, in_order);
  reversed.compute({-0.4, 0.3}, out_of_order);

  EXPECT_NEAR(in_order[0], out_of_order[1], 1.0e-9);
  EXPECT_NEAR(in_order[1], out_of_order[0], 1.0e-9);
}

TEST(GravityModel, JointsOffTheChainGetNoTorqueButTheirMassStillCounts)
{
  // Chain stops at link1, so joint2 is not a joint this model drives and its
  // output stays zero. link2's *mass*, however, is still bolted to link1 and
  // still pulls on joint1: 1 kg with its centre of mass 1.5 m out, on top of
  // link1's own 1 kg at 0.5 m.
  //
  // This used to assert 0.5 * kG, which was the model quietly discarding
  // everything past the end of the chain.
  GravityModel model;
  ASSERT_TRUE(
    model.configure(kUrdf, "base_link", "link1", {"joint1", "joint2"}, kGravity).empty());

  std::vector<double> torques{7.0, 7.0};
  model.compute({0.0, 0.0}, torques);

  EXPECT_NEAR((0.5 + 1.5) * kG, std::abs(torques[0]), 1.0e-6);
  EXPECT_DOUBLE_EQ(0.0, torques[1]);
  EXPECT_NEAR(1.0, model.folded_mass(), 1.0e-9);
}

TEST(GravityModel, AnUnconfiguredModelZeroesItsOutput)
{
  GravityModel model;
  std::vector<double> torques{3.0, -3.0};
  model.compute({0.0, 0.0}, torques);
  EXPECT_DOUBLE_EQ(0.0, torques[0]);
  EXPECT_DOUBLE_EQ(0.0, torques[1]);
}

TEST(GravityModel, FrictionIsSmoothThroughZeroAndOdd)
{
  const FrictionParameters parameters{0.5, 0.1, 0.05};

  EXPECT_DOUBLE_EQ(0.0, GravityModel::friction(0.0, parameters));
  EXPECT_NEAR(
    -GravityModel::friction(0.4, parameters), GravityModel::friction(-0.4, parameters), 1.0e-12);

  // Well past the deadzone the Coulomb term has saturated, so what is left is
  // Coulomb plus viscous.
  EXPECT_NEAR(0.5 + 0.1 * 2.0, GravityModel::friction(2.0, parameters), 1.0e-6);

  // Inside the deadzone it is still climbing towards the Coulomb value.
  EXPECT_LT(GravityModel::friction(0.01, parameters), 0.5);
  EXPECT_GT(GravityModel::friction(0.01, parameters), 0.0);
}


// ----- mass hanging off the chain -----
//
// KDL's getChain walks a single path and drops every branch. On the real arm
// that quietly lost the two gripper fingers and the wrist payload -- 0.114 kg,
// all of it at the far end of the longest lever -- and the model was wrong by
// tens of percent at the wrist with nothing to show for it.

TEST(GravityModel, ReportsMassItHadToFoldIn)
{
  GravityModel model;
  ASSERT_TRUE(
    model.configure(kUrdfBranch, "base_link", "link2", {"joint1", "joint2"}, kGravity).empty());

  EXPECT_NEAR(1.0, model.folded_mass(), 1.0e-9);
  ASSERT_EQ(1u, model.folded_links().size());
  EXPECT_EQ("tool", model.folded_links()[0]);
}

TEST(GravityModel, PlainChainFoldsNothing)
{
  GravityModel model;
  ASSERT_TRUE(model.configure(kUrdf, "base_link", "link2", {"joint1", "joint2"}, kGravity).empty());
  EXPECT_DOUBLE_EQ(0.0, model.folded_mass());
  EXPECT_TRUE(model.folded_links().empty());
}

TEST(GravityModel, OffChainMassPullsOnEveryJointAboveIt)
{
  GravityModel plain;
  GravityModel with_tool;
  ASSERT_TRUE(plain.configure(kUrdf, "base_link", "link2", {"joint1", "joint2"}, kGravity).empty());
  ASSERT_TRUE(
    with_tool.configure(
      kUrdfBranch, "base_link", "link2", {"joint1", "joint2"}, kGravity).empty());

  std::vector<double> without(2, 0.0);
  std::vector<double> withtool(2, 0.0);
  plain.compute({0.0, 0.0}, without);
  with_tool.compute({0.0, 0.0}, withtool);

  // Straight out along +X, the tool sits 2 m from joint1 and 1 m from joint2.
  EXPECT_NEAR(2.0 * 1.0 * kG, std::abs(withtool[0] - without[0]), 1.0e-6);
  EXPECT_NEAR(1.0 * 1.0 * kG, std::abs(withtool[1] - without[1]), 1.0e-6);
}

TEST(GravityModel, FoldedMassLoadsTheJointsByHandComputedAmounts)
{
  GravityModel with_tool;
  ASSERT_TRUE(
    with_tool.configure(
      kUrdfBranch, "base_link", "link2", {"joint1", "joint2"}, kGravity).empty());

  std::vector<double> torques(2, 0.0);
  with_tool.compute({0.0, 0.0}, torques);

  // link1 1 kg at 0.5 m, link2 1 kg at 1.5 m, tool 1 kg at 2.0 m.
  EXPECT_NEAR((0.5 + 1.5 + 2.0) * kG, std::abs(torques[0]), 1.0e-6);
  // From joint2: link2 1 kg at 0.5 m, tool 1 kg at 1.0 m.
  EXPECT_NEAR((0.5 + 1.0) * kG, std::abs(torques[1]), 1.0e-6);
}

TEST(GravityModel, ZeroComOffsetsMatchOriginalAcrossPoses)
{
  GravityModel original;
  GravityModel adjusted;
  ASSERT_TRUE(original.configure(
      kUrdfBranch, "base_link", "link2", {"joint1", "joint2"}, kGravity).empty());
  ASSERT_TRUE(adjusted.configure(
      kUrdfBranch, "base_link", "link2", {"joint1", "joint2"}, kGravity,
      {"link1", "link2", "tool"}, std::vector<double>(9, 0.0)).empty());
  for (double q : {-1.5, -0.3, 0.0, 0.7, 1.6}) {
    std::vector<double> a(2), b(2);
    original.compute({q, -q * 0.3}, a);
    adjusted.compute({q, -q * 0.3}, b);
    EXPECT_NEAR(a[0], b[0], 1e-12);
    EXPECT_NEAR(a[1], b[1], 1e-12);
  }
}

TEST(GravityModel, LinkOffsetHasAnalyticTorqueAndDoesNotAccumulate)
{
  GravityModel model;
  for (int reload = 0; reload < 2; ++reload) {
    ASSERT_TRUE(model.configure(
        kUrdf, "base_link", "link2", {"joint1", "joint2"}, kGravity,
        {"link2"}, {0.01, 0.0, 0.02}).empty());
    for (double q : {-1.1, 0.0, 0.8}) {
      std::vector<double> t(2);
      model.compute({q, 0.0}, t);
      // +Y rotation: world X = local X*cos(q) + local Z*sin(q).
      EXPECT_NEAR(-kG * (2.01 * std::cos(q) + 0.02 * std::sin(q)), t[0], 1e-9);
      EXPECT_NEAR(-kG * (0.51 * std::cos(q) + 0.02 * std::sin(q)), t[1], 1e-9);
    }
    ASSERT_EQ(1u, model.com_adjustments().size());
    EXPECT_DOUBLE_EQ(0.5, model.com_adjustments()[0].original[0]);
    EXPECT_DOUBLE_EQ(0.01, model.com_adjustments()[0].offset[0]);
  }
  ASSERT_TRUE(model.configure(
      kUrdf, "base_link", "link2", {"joint1", "joint2"}, kGravity).empty());
  EXPECT_TRUE(model.com_adjustments().empty());
  std::vector<double> t(2);
  model.compute({0.0, 0.0}, t);
  EXPECT_NEAR(-0.5 * kG, t[1], 1e-9);
}

TEST(GravityModel, BranchOffsetIsAppliedBeforeRotatedPayloadFolding)
{
  // Rotate the tool's frame +90 deg about Y. Tool local +Z becomes link2 +X.
  std::string urdf(kUrdfBranch);
  const auto pos = urdf.find("<origin xyz=\"1 0 0\"/>", urdf.find("name=\"tool_joint\""));
  ASSERT_NE(std::string::npos, pos);
  urdf.replace(pos, std::string("<origin xyz=\"1 0 0\"/>").size(),
    "<origin xyz=\"1 0 0\" rpy=\"0 1.5707963267948966 0\"/>");
  GravityModel model;
  ASSERT_TRUE(model.configure(
      urdf, "base_link", "link2", {"joint1", "joint2"}, kGravity,
      {"tool"}, {0.0, 0.0, 0.01}).empty());
  std::vector<double> t(2);
  model.compute({0.0, 0.0}, t);
  EXPECT_NEAR(-4.01 * kG, t[0], 1e-9);
  EXPECT_NEAR(-1.51 * kG, t[1], 1e-9);
  EXPECT_DOUBLE_EQ(1.0, model.folded_mass());
}

TEST(GravityModel, OffsetAxesAreLinkAxesNotRotatedInertialAxes)
{
  std::string urdf(kUrdf);
  const auto pos = urdf.find("<origin xyz=\"0.5 0 0\"/>");
  ASSERT_NE(std::string::npos, pos);
  urdf.replace(pos, std::string("<origin xyz=\"0.5 0 0\"/>").size(),
    "<origin xyz=\"0.5 0 0\" rpy=\"0 0 1.5707963267948966\"/>");
  GravityModel model;
  ASSERT_TRUE(model.configure(
      urdf, "base_link", "link2", {"joint1", "joint2"}, kGravity,
      {"link1"}, {0.01, 0.0, 0.0}).empty());
  std::vector<double> t(2);
  model.compute({0.0, 0.0}, t);
  EXPECT_NEAR(-2.01 * kG, t[0], 1e-9);
  EXPECT_NEAR(-0.5 * kG, t[1], 1e-9);
}

TEST(GravityModel, RejectsInvalidComOffsetsAndClearsOldSolver)
{
  GravityModel model;
  const auto check = [&](const std::vector<std::string> & links,
      const std::vector<double> & xyz) {
      ASSERT_TRUE(model.configure(
          kUrdf, "base_link", "link2", {"joint1", "joint2"}, kGravity).empty());
      EXPECT_FALSE(model.configure(
          kUrdf, "base_link", "link2", {"joint1", "joint2"}, kGravity, links, xyz).empty());
      EXPECT_FALSE(model.is_configured());
      EXPECT_TRUE(model.com_adjustments().empty());
    };
  check({"link2"}, {0.0, 0.0});
  check({}, {0.0, 0.0, 0.0});
  check({"link2", "link2"}, std::vector<double>(6, 0.0));
  check({""}, {0.0, 0.0, 0.0});
  check({"typo"}, {0.0, 0.0, 0.0});
  check({"base_link"}, {0.0, 0.0, 0.0});  // no inertial mass
  check({"link2"}, {std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0});
  check({"link2"}, {0.0, std::numeric_limits<double>::infinity(), 0.0});
  check({"link2"}, {0.051, 0.0, 0.0});
  check({"link2"}, {0.04, 0.04, 0.0});  // norm, not per-component limit
  // link1 has mass, but is the selected base, so cannot load this chain.
  EXPECT_FALSE(model.configure(kUrdf, "link1", "link2", {"joint2"}, kGravity,
      {"link1"}, {0.001, 0.0, 0.0}).empty());
  EXPECT_FALSE(model.is_configured());
}
