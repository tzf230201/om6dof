#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <vector>

#include "om6dof_dd_gng/final_grasp.hpp"

namespace
{
namespace grasp = om6dof_dd_gng::grasp;

// Analytic six-variable Cartesian/rotation fixture. Its control-frame +Z
// points horizontally forward and +X points down, matching the frame
// distinction in the archived scanner. No ROS/model/controller is involved.
grasp::KinematicsFn syntheticKinematics(double translation_scale = 1.0)
{
  return [translation_scale](const std::vector<double> & q,
           Eigen::Isometry3d & pose, Eigen::MatrixXd & jacobian) {
      if (q.size() != 6) {return false;}
      const Eigen::Matrix3d rz = Eigen::AngleAxisd(q[5], Eigen::Vector3d::UnitZ()).toRotationMatrix();
      const Eigen::Matrix3d ry = Eigen::AngleAxisd(q[4], Eigen::Vector3d::UnitY()).toRotationMatrix();
      const Eigen::Matrix3d rx = Eigen::AngleAxisd(q[3], Eigen::Vector3d::UnitX()).toRotationMatrix();
      const Eigen::Matrix3d frame = Eigen::AngleAxisd(
        std::acos(-1.0) * 0.5, Eigen::Vector3d::UnitY()).toRotationMatrix();
      pose = Eigen::Isometry3d::Identity();
      pose.linear() = rz * ry * rx * frame;
      pose.translation() = translation_scale * Eigen::Vector3d(q[0], q[1], q[2]);
      jacobian = Eigen::MatrixXd::Zero(6, 6);
      jacobian.topLeftCorner<3, 3>() = translation_scale * Eigen::Matrix3d::Identity();
      jacobian.block<3, 1>(3, 3) = rz * ry * Eigen::Vector3d::UnitX();
      jacobian.block<3, 1>(3, 4) = rz * Eigen::Vector3d::UnitY();
      jacobian.block<3, 1>(3, 5) = Eigen::Vector3d::UnitZ();
      return true;
    };
}

const std::vector<double> kLower(6, -2.0);
const std::vector<double> kUpper(6, 2.0);
const std::vector<double> kStart(6, 0.0);

grasp::FinalGraspParameters testParameters()
{
  grasp::FinalGraspParameters parameters;
  parameters.max_wall_time_sec = 2.0;
  return parameters;
}

TEST(FinalGrasp, ConvergesDeterministicallyAndPreservesOrientationAndPinchOffset)
{
  auto parameters = testParameters();
  parameters.tcp_to_pinch = {0.0, 0.0, 0.02};
  const std::vector<double> start{0.0, 0.0, 0.0, 0.2, 0.03, 0.3};
  const auto kinematics = syntheticKinematics();
  Eigen::Isometry3d initial;
  Eigen::MatrixXd jacobian;
  ASSERT_TRUE(kinematics(start, initial, jacobian));
  const Eigen::Vector3d direction = initial.linear() * Eigen::Vector3d::UnitZ();
  const Eigen::Vector3d target = initial * parameters.tcp_to_pinch + 0.04 * direction;
  const auto valid = [](const std::vector<double> &) {return true;};
  const auto first = grasp::planCartesianGrasp(
    start, kLower, kUpper, target, parameters, kinematics, valid);
  ASSERT_TRUE(first.valid) << first.reason;
  ASSERT_GT(first.joint_path.size(), 1U);
  EXPECT_EQ(first.joint_path.front(), start);
  EXPECT_LE(first.position_error_m, parameters.position_tolerance_m);
  EXPECT_LE(first.orientation_error_rad, 1.0e-9);
  EXPECT_NE(first.joint_path[1], start);
  const auto second = grasp::planCartesianGrasp(
    start, kLower, kUpper, target, parameters, kinematics, valid);
  ASSERT_TRUE(second.valid) << second.reason;
  EXPECT_EQ(first.joint_path, second.joint_path);
  for (const auto & q : first.joint_path) {
    Eigen::Isometry3d pose;
    ASSERT_TRUE(kinematics(q, pose, jacobian));
    const auto error = grasp::detail::poseError(pose.translation(), initial.linear(), pose);
    EXPECT_LE(error.tail<3>().norm(), parameters.orientation_tolerance_rad);
    const Eigen::Vector3d delta = pose.translation() - initial.translation();
    EXPECT_LE((delta - direction * delta.dot(direction)).norm(), 1.0e-8);
  }
}

TEST(FinalGrasp, DetectsCollisionBetweenOtherwiseValidCartesianEndpoints)
{
  auto parameters = testParameters();
  parameters.position_tolerance_m = 0.0001;
  parameters.collision_step_rad = 0.025;
  // 5 mm Cartesian translation requires about .05 in q0. Both endpoints
  // are free, while dense interpolated checking must encounter q0=.025.
  std::vector<double> checked;
  const auto valid = [&checked](const std::vector<double> & q) {
      checked.push_back(q[0]);
      return q[0] < 0.015 || q[0] > 0.035;
    };
  const auto result = grasp::planCartesianGrasp(
    kStart, kLower, kUpper, {0.005, 0.0, 0.0}, parameters,
    syntheticKinematics(0.1), valid);
  EXPECT_FALSE(result.valid);
  EXPECT_EQ(result.reason, "final_grasp_collision");
  EXPECT_TRUE(result.joint_path.empty());
  EXPECT_TRUE(std::any_of(checked.begin(), checked.end(),
    [](double value) {return value >= 0.015 && value <= 0.035;}));
}

TEST(FinalGrasp, LocalXPointingDownFailsEvenWhenAimedAtTarget)
{
  auto parameters = testParameters();
  parameters.tool_approach_axis = Eigen::Vector3d::UnitX();
  const auto result = grasp::planCartesianGrasp(
    kStart, kLower, kUpper, {0.0, 0.0, -0.04}, parameters,
    syntheticKinematics(), [](const auto &) {return true;});
  EXPECT_FALSE(result.valid);
  EXPECT_EQ(result.reason, "final_grasp_approach_not_horizontal");
  EXPECT_TRUE(result.joint_path.empty());
}

TEST(FinalGrasp, RejectsLateralInsertionButSubdividesRecoverableJointStep)
{
  auto parameters = testParameters();
  auto result = grasp::planCartesianGrasp(
    kStart, kLower, kUpper, {0.0, 0.04, 0.0}, parameters,
    syntheticKinematics(), [](const auto &) {return true;});
  EXPECT_EQ(result.reason, "final_grasp_approach_misaligned");
  parameters.position_tolerance_m = 0.0001;
  result = grasp::planCartesianGrasp(
    kStart, kLower, kUpper, {0.005, 0.0, 0.0}, parameters,
    syntheticKinematics(0.02), [](const auto &) {return true;});
  ASSERT_TRUE(result.valid) << result.reason;
  EXPECT_GT(result.cartesian_subdivisions, 0U);
  EXPECT_GT(result.peak_attempted_joint_step_rad, parameters.max_joint_step_rad);
  ASSERT_GE(result.joint_path.size(), 3U);
  for (std::size_t i = 1; i < result.joint_path.size(); ++i) {
    for (std::size_t j = 0; j < kStart.size(); ++j) {
      EXPECT_LE(std::abs(result.joint_path[i][j] - result.joint_path[i - 1][j]),
        parameters.max_joint_step_rad);
    }
  }
  EXPECT_LE(result.position_error_m, parameters.position_tolerance_m);
}

TEST(FinalGrasp, InvalidNumbersAndBudgetsFailClosed)
{
  auto parameters = testParameters();
  const auto valid = [](const auto &) {return true;};
  const auto kinematics = syntheticKinematics();
  auto invalid_start = kStart;
  invalid_start[0] = std::numeric_limits<double>::quiet_NaN();
  EXPECT_EQ(grasp::planCartesianGrasp(invalid_start, kLower, kUpper,
      {0.04, 0.0, 0.0}, parameters, kinematics, valid).reason, "final_grasp_invalid_parameters");
  parameters.max_iterations = 0;
  EXPECT_EQ(grasp::planCartesianGrasp(kStart, kLower, kUpper,
      {0.04, 0.0, 0.0}, parameters, kinematics, valid).reason, "final_grasp_invalid_parameters");
  parameters = testParameters();
  parameters.collision_step_rad = std::numeric_limits<double>::infinity();
  EXPECT_EQ(grasp::planCartesianGrasp(kStart, kLower, kUpper,
      {0.04, 0.0, 0.0}, parameters, kinematics, valid).reason, "final_grasp_invalid_parameters");
  parameters = testParameters();
  parameters.max_waypoints = 2;
  EXPECT_EQ(grasp::planCartesianGrasp(kStart, kLower, kUpper,
      {0.04, 0.0, 0.0}, parameters, kinematics, valid).reason, "final_grasp_waypoint_budget_exceeded");
  parameters = testParameters();
  parameters.max_wall_time_sec = 1.0e-12;
  EXPECT_EQ(grasp::planCartesianGrasp(kStart, kLower, kUpper,
      {0.04, 0.0, 0.0}, parameters, kinematics, valid).reason, "final_grasp_timeout");
}

TEST(FinalGrasp, RejectsInvalidKinematicsSingularityAndNoProgress)
{
  const auto parameters = testParameters();
  const auto valid = [](const auto &) {return true;};
  const auto singular = [](const auto & q, auto & pose, auto & jacobian) {
      syntheticKinematics()(q, pose, jacobian);
      jacobian.setZero();
      return true;
    };
  EXPECT_EQ(grasp::planCartesianGrasp(kStart, kLower, kUpper,
      {0.04, 0.0, 0.0}, parameters, singular, valid).reason, "final_grasp_ik_singular");
  const auto invalid = [](const auto & q, auto & pose, auto & jacobian) {
      syntheticKinematics()(q, pose, jacobian);
      pose.linear().setZero();
      return true;
    };
  EXPECT_EQ(grasp::planCartesianGrasp(kStart, kLower, kUpper,
      {0.04, 0.0, 0.0}, parameters, invalid, valid).reason, "final_grasp_invalid_kinematics");
  auto upper = kUpper;
  upper[0] = 0.001;
  EXPECT_EQ(grasp::planCartesianGrasp(kStart, kLower, upper,
      {0.04, 0.0, 0.0}, parameters, syntheticKinematics(), valid).reason,
    "final_grasp_ik_no_progress");
}

TEST(FinalGrasp, LateralPregraspBridgeUsesTcpDestinationThenStrictPinchInsertion)
{
  auto parameters = testParameters();
  parameters.tcp_to_pinch = {0.0, 0.0, 0.02};
  const auto kinematics = syntheticKinematics();
  const auto valid = [](const auto &) {return true;};
  const Eigen::Vector3d desired_tcp(0.0, 0.04, 0.0);
  const Eigen::Vector3d target_pinch(0.09, 0.04, 0.0);
  // Direct insertion is still rejected: the target is not on the seed's ray.
  const auto direct = grasp::planCartesianGrasp(
    kStart, kLower, kUpper, target_pinch, parameters, kinematics, valid);
  EXPECT_FALSE(direct.valid);
  EXPECT_EQ(direct.reason, "final_grasp_approach_misaligned");
  const auto bridge = grasp::planCartesianPregrasp(
    kStart, kLower, kUpper, desired_tcp, parameters, kinematics, valid);
  ASSERT_TRUE(bridge.valid) << bridge.reason;
  EXPECT_EQ(bridge.reason, "pregrasp_bridge_ready");
  ASSERT_GT(bridge.joint_path.size(), 1U);
  EXPECT_EQ(bridge.joint_path.front(), kStart);
  Eigen::Isometry3d initial, end;
  Eigen::MatrixXd jacobian;
  ASSERT_TRUE(kinematics(kStart, initial, jacobian));
  ASSERT_TRUE(kinematics(bridge.joint_path.back(), end, jacobian));
  EXPECT_LE((end.translation() - desired_tcp).norm(), parameters.position_tolerance_m);
  // The bridge destination is TCP, not pinch; offset is applied only in grasp.
  EXPECT_GT((end * parameters.tcp_to_pinch - desired_tcp).norm(), 0.019);
  EXPECT_LE(Eigen::AngleAxisd(end.linear() * initial.linear().transpose()).angle(), 1.0e-9);
  const auto insertion = grasp::planCartesianGrasp(
    bridge.joint_path.back(), kLower, kUpper, target_pinch, parameters, kinematics, valid);
  ASSERT_TRUE(insertion.valid) << insertion.reason;
  EXPECT_EQ(insertion.joint_path.front(), bridge.joint_path.back());
  EXPECT_LE(insertion.position_error_m, parameters.position_tolerance_m);
  const auto repeated = grasp::planCartesianPregrasp(
    kStart, kLower, kUpper, desired_tcp, parameters, kinematics, valid);
  EXPECT_EQ(bridge.joint_path, repeated.joint_path);
}

TEST(FinalGrasp, LateralBridgeStillChecksCollisionBetweenClearEndpoints)
{
  auto parameters = testParameters();
  parameters.position_tolerance_m = 0.0001;
  std::vector<double> checked;
  const auto valid = [&checked](const std::vector<double> & q) {
      checked.push_back(q[1]);
      return q[1] < 0.015 || q[1] > 0.035;
    };
  const auto bridge = grasp::planCartesianPregrasp(
    kStart, kLower, kUpper, {0.0, 0.005, 0.0}, parameters, syntheticKinematics(0.1), valid);
  EXPECT_FALSE(bridge.valid);
  EXPECT_EQ(bridge.reason, "pregrasp_bridge_collision");
  EXPECT_TRUE(bridge.joint_path.empty());
  EXPECT_TRUE(std::any_of(checked.begin(), checked.end(),
    [](double value) {return value >= 0.015 && value <= 0.035;}));
}

TEST(FinalGrasp, BridgeRetainsHorizontalAxisFiniteDistanceAndJointBudgetRequirements)
{
  auto parameters = testParameters();
  const auto valid = [](const auto &) {return true;};
  const auto kinematics = syntheticKinematics();
  EXPECT_EQ(grasp::planCartesianPregrasp(kStart, kLower, kUpper,
      {0.0, 0.060001, 0.0}, parameters, kinematics, valid).reason, "pregrasp_bridge_too_long");
  parameters.max_pregrasp_bridge_distance_m = 0.061;
  EXPECT_EQ(grasp::planCartesianPregrasp(kStart, kLower, kUpper,
      {0.0, 0.01, 0.0}, parameters, kinematics, valid).reason, "pregrasp_bridge_invalid_parameters");
  parameters = testParameters();
  EXPECT_EQ(grasp::planCartesianPregrasp(kStart, kLower, kUpper,
      {0.0, std::numeric_limits<double>::quiet_NaN(), 0.0}, parameters, kinematics, valid).reason,
    "pregrasp_bridge_invalid_parameters");
  parameters.tool_approach_axis = Eigen::Vector3d::UnitX();
  EXPECT_EQ(grasp::planCartesianPregrasp(kStart, kLower, kUpper,
      {0.0, 0.01, 0.0}, parameters, kinematics, valid).reason, "pregrasp_bridge_approach_not_horizontal");
  parameters = testParameters();
  parameters.position_tolerance_m = 0.0001;
  const auto bridge = grasp::planCartesianPregrasp(kStart, kLower, kUpper,
    {0.0, 0.005, 0.0}, parameters, syntheticKinematics(0.02), valid);
  EXPECT_TRUE(bridge.valid) << bridge.reason;
  EXPECT_GT(bridge.cartesian_subdivisions, 0U);
  parameters = testParameters();
  parameters.max_wall_time_sec = 1.0e-12;
  EXPECT_EQ(grasp::planCartesianPregrasp(kStart, kLower, kUpper,
    {0.0, 0.01, 0.0}, parameters, kinematics, valid).reason, "pregrasp_bridge_timeout");
}

TEST(FinalGrasp, SubdivisionMakesProgressDespiteThreeMillimeterTerminalTolerance)
{
  const auto parameters = testParameters();
  const auto valid = [](const auto &) {return true;};
  const auto kinematics = syntheticKinematics(0.02);
  // Each split is 2.5 mm, smaller than the default 3 mm terminal tolerance.
  // It must still produce an actual intermediate state, not swallow it and
  // immediately retry the original oversized jump.
  const auto result = grasp::planCartesianPregrasp(
    kStart, kLower, kUpper, {0.0, 0.005, 0.0}, parameters, kinematics, valid);
  ASSERT_TRUE(result.valid) << result.reason;
  EXPECT_GT(result.cartesian_subdivisions, 0U);
  ASSERT_GE(result.joint_path.size(), 3U);
  Eigen::Isometry3d middle;
  Eigen::MatrixXd jacobian;
  ASSERT_TRUE(kinematics(result.joint_path[1], middle, jacobian));
  EXPECT_GT(middle.translation().y(), 0.0015);
  EXPECT_LT(middle.translation().y(), 0.0035);
  for (std::size_t i = 1; i < result.joint_path.size(); ++i) {
    EXPECT_LE(std::abs(result.joint_path[i][1] - result.joint_path[i - 1][1]),
      parameters.max_joint_step_rad);
  }
}

TEST(FinalGrasp, AdaptiveSubdivisionRetainsCollisionChecksAndClearsPartialPath)
{
  auto parameters = testParameters();
  parameters.position_tolerance_m = 0.0001;
  std::vector<double> checked;
  const auto valid = [&checked](const std::vector<double> & q) {
      checked.push_back(q[1]);
      return q[1] < 0.04 || q[1] > 0.06;
    };
  const auto result = grasp::planCartesianPregrasp(
    kStart, kLower, kUpper, {0.0, 0.005, 0.0}, parameters, syntheticKinematics(0.02), valid);
  EXPECT_FALSE(result.valid);
  EXPECT_EQ(result.reason, "pregrasp_bridge_collision");
  EXPECT_TRUE(result.joint_path.empty());
  EXPECT_GT(result.cartesian_subdivisions, 0U);
  EXPECT_GT(result.peak_attempted_joint_step_rad, parameters.max_joint_step_rad);
  EXPECT_TRUE(std::any_of(checked.begin(), checked.end(),
    [](double value) {return value >= 0.04 && value <= 0.06;}));
}

TEST(FinalGrasp, SubdivisionCannotExceedWaypointOrMinimumStepBudgets)
{
  auto parameters = testParameters();
  parameters.position_tolerance_m = 0.0001;
  parameters.max_waypoints = 1;
  const auto valid = [](const auto &) {return true;};
  const auto budget = grasp::planCartesianPregrasp(kStart, kLower, kUpper,
    {0.0, 0.005, 0.0}, parameters, syntheticKinematics(0.02), valid);
  EXPECT_FALSE(budget.valid);
  EXPECT_EQ(budget.reason, "pregrasp_bridge_waypoint_budget_exceeded");
  EXPECT_TRUE(budget.joint_path.empty());
  EXPECT_EQ(budget.cartesian_subdivisions, 0U);
  EXPECT_GT(budget.peak_attempted_joint_step_rad, parameters.max_joint_step_rad);

  parameters = testParameters();
  parameters.position_tolerance_m = 1.0e-6;
  parameters.damping = 1.0e-6;
  // Even a 0.1 mm interval needs 0.5 rad. Refinement must stop at its
  // finite spatial floor rather than silently permitting that jump.
  const auto jump = grasp::planCartesianGrasp(kStart, kLower, kUpper,
    {0.0003, 0.0, 0.0}, parameters, syntheticKinematics(0.0002), valid);
  EXPECT_FALSE(jump.valid);
  EXPECT_EQ(jump.reason, "final_grasp_joint_jump");
  EXPECT_TRUE(jump.joint_path.empty());
  EXPECT_GT(jump.cartesian_subdivisions, 0U);
  EXPECT_GT(jump.peak_attempted_joint_step_rad, parameters.max_joint_step_rad);
}

}  // namespace
