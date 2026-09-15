// Offline numerical fixture: real archived V2 joint witnesses and kinematic
// model, exercised through MoveIt's FK/Jacobian implementation. Collision
// feasibility and hardware execution are deliberately not inferred here.
#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <stdexcept>

#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <srdfdom/model.h>
#include <urdf_parser/urdf_parser.h>

#include "om6dof_dd_gng/final_grasp.hpp"
#include "om6dof_dd_gng/workspace_samples.hpp"

namespace
{
namespace grasp = om6dof_dd_gng::grasp;
namespace reach = om6dof_dd_gng::reachability;

std::filesystem::path repositoryRoot()
{
#ifdef OM6DOF_TEST_REPO_ROOT
  return OM6DOF_TEST_REPO_ROOT;
#else
  return std::filesystem::absolute(__FILE__).parent_path().parent_path().parent_path();
#endif
}

std::string readText(const std::filesystem::path & path)
{
  std::ifstream file(path);
  if (!file) {throw std::runtime_error("missing test fixture: " + path.string());}
  std::ostringstream text;
  text << file.rdbuf();
  return text.str();
}

std::filesystem::path archiveDirectory()
{
  return repositoryRoot() /
         "experiments/cartesian_workspace/results/comparison_v1_v2_25mm_20260909/v2";
}

moveit::core::RobotModelPtr archivedModel(bool rotate_base = false)
{
  std::string xml = readText(archiveDirectory() / "model.urdf");
  const std::string package_uri = "package://om6dof_description/";
  const std::string source_uri = "file://" +
    (repositoryRoot() / "om6dof_description").string() + "/";
  for (std::size_t offset = 0; (offset = xml.find(package_uri, offset)) != std::string::npos;
    offset += source_uri.size())
  {
    xml.replace(offset, package_uri.size(), source_uri);
  }
  auto urdf = urdf::parseURDF(xml);
  if (!urdf) {throw std::runtime_error("archived URDF could not be parsed");}
  if (rotate_base) {
    // Perturb only the fixed base orientation in memory. A nonidentity base
    // catches a missing/double world conversion hidden by the usual fixture.
    auto fixed = urdf->joints_.find("world_fixed");
    if (fixed == urdf->joints_.end()) {throw std::runtime_error("world_fixed fixture missing");}
    fixed->second->parent_to_joint_origin_transform.rotation.setFromRPY(0.2, -0.1, 0.3);
  }
  auto srdf = std::make_shared<srdf::Model>();
  if (!srdf->initString(*urdf, readText(repositoryRoot() / "om6dof_moveit_config/config/om6dof.srdf"))) {
    throw std::runtime_error("SRDF fixture could not be parsed");
  }
  return std::make_shared<moveit::core::RobotModel>(urdf, srdf);
}

std::vector<double> greenWitness(double x_mm = 225.0, double y_mm = 0.0, double z_mm = 150.0)
{
  std::ifstream file(archiveDirectory() / "points.csv");
  std::string line;
  if (!std::getline(file, line)) {throw std::runtime_error("missing green witness fixture");}
  const auto fields = reach::workspaceCsvFields(line);
  std::map<std::string, std::size_t> columns;
  for (std::size_t i = 0; i < fields.size(); ++i) {columns[fields[i]] = i;}
  while (std::getline(file, line)) {
    const auto row = reach::workspaceCsvFields(line);
    if (row.size() != fields.size() || row[columns.at("status")] != "pose_found") {continue;}
    // An interior radial witness; choose it by archived target, not by
    // searching for whichever configuration happens to pass the new solver.
    if (std::abs(std::stod(row[columns.at("x_mm")]) - x_mm) > 1.0e-6 ||
      std::abs(std::stod(row[columns.at("y_mm")]) - y_mm) > 1.0e-6 ||
      std::abs(std::stod(row[columns.at("z_mm")]) - z_mm) > 1.0e-6) {continue;}
    std::vector<double> result;
    for (int i = 1; i <= 6; ++i) {
      result.push_back(std::stod(row[columns.at("q" + std::to_string(i) + "_rad")]));
    }
    return result;
  }
  throw std::runtime_error("fixed green witness missing from archive");
}

void setOpenState(moveit::core::RobotState & state,
  const moveit::core::JointModelGroup * group, const std::vector<double> & q)
{
  state.setToDefaultValues();
  state.setJointGroupPositions(group, q);
  // Humble's setVariablePosition propagates this value to mimic requests.
  state.setVariablePosition("gripper_left_joint", 0.019);
  state.update();
}

grasp::KinematicsFn worldKinematics(const moveit::core::RobotModelPtr & model)
{
  const auto * group = model->getJointModelGroup("arm");
  if (!group) {throw std::runtime_error("arm group missing");}
  auto state = std::make_shared<moveit::core::RobotState>(model);
  return [model, group, state](const std::vector<double> & q,
           Eigen::Isometry3d & pose, Eigen::MatrixXd & jacobian) {
      setOpenState(*state, group, q);
      pose = state->getGlobalLinkTransform("end_effector_link");
      if (!state->getJacobian(group, model->getLinkModel("end_effector_link"),
          Eigen::Vector3d::Zero(), jacobian)) {return false;}
      const auto * parent = group->getJointModels().front()->getParentLinkModel();
      if (parent) {
        const Eigen::Matrix3d rotation = state->getGlobalLinkTransform(parent).linear();
        jacobian.topRows(3) = (rotation * jacobian.topRows(3)).eval();
        jacobian.bottomRows(3) = (rotation * jacobian.bottomRows(3)).eval();
      }
      return true;
    };
}

TEST(FinalGraspRobot, WorldJacobianMatchesFiniteDifferencesWithRotatedBase)
{
  const auto q = greenWitness();
  constexpr double step = 1.0e-6;
  for (bool rotate_base : {false, true}) {
    SCOPED_TRACE(rotate_base ? "nonidentity fixed-base orientation" : "archived base orientation");
    const auto kinematics = worldKinematics(archivedModel(rotate_base));
    Eigen::Isometry3d base;
    Eigen::MatrixXd jacobian;
    ASSERT_TRUE(kinematics(q, base, jacobian));
    ASSERT_EQ(jacobian.rows(), 6);
    ASSERT_EQ(jacobian.cols(), 6);
    Eigen::Matrix<double, 6, 6> numerical;
    for (std::size_t joint = 0; joint < q.size(); ++joint) {
      auto plus_q = q, minus_q = q;
      plus_q[joint] += step;
      minus_q[joint] -= step;
      Eigen::Isometry3d plus, minus;
      Eigen::MatrixXd unused;
      ASSERT_TRUE(kinematics(plus_q, plus, unused));
      ASSERT_TRUE(kinematics(minus_q, minus, unused));
      numerical.block<3, 1>(0, joint) = (plus.translation() - minus.translation()) / (2.0 * step);
      const Eigen::AngleAxisd angle(plus.linear() * minus.linear().transpose());
      numerical.block<3, 1>(3, joint) = angle.axis() * angle.angle() / (2.0 * step);
    }
    EXPECT_LT((numerical - jacobian).cwiseAbs().maxCoeff(), 1.0e-6);
  }
}

TEST(FinalGraspRobot, GreenWitnessSupportsSevenCentimeterHorizontalInsertionNumerically)
{
  const auto model = archivedModel();
  const auto * group = model->getJointModelGroup("arm");
  ASSERT_NE(group, nullptr);
  ASSERT_EQ(group->getVariableCount(), 6U);
  const auto start = greenWitness();
  const auto kinematics = worldKinematics(model);
  std::vector<double> lower, upper;
  for (const auto & name : group->getVariableNames()) {
    const auto & bounds = model->getVariableBounds(name);
    ASSERT_TRUE(bounds.position_bounded_);
    lower.push_back(bounds.min_position_);
    upper.push_back(bounds.max_position_);
  }
  Eigen::Isometry3d seed;
  Eigen::MatrixXd jacobian;
  ASSERT_TRUE(kinematics(start, seed, jacobian));
  const Eigen::Vector3d axis = seed.linear() * Eigen::Vector3d::UnitZ();
  EXPECT_LT(std::abs(axis.z()), 0.01);
  EXPECT_LT((seed.linear() * Eigen::Vector3d::UnitX() + Eigen::Vector3d::UnitZ()).norm(), 0.01);
  const Eigen::Vector3d target = seed.translation() + 0.07 * axis;
  grasp::FinalGraspParameters parameters;
  // Numerical convergence test is independent of CPU-specific realtime budget.
  parameters.max_wall_time_sec = 3.0;
  auto validity_state = moveit::core::RobotState(model);
  const auto valid = [&](const std::vector<double> & q) {
      setOpenState(validity_state, group, q);
      EXPECT_DOUBLE_EQ(validity_state.getVariablePosition("gripper_right_joint"), 0.019);
      return validity_state.satisfiesBounds();
    };
  const auto result = grasp::planCartesianGrasp(start, lower, upper, target, parameters, kinematics, valid);
  ASSERT_TRUE(result.valid) << result.reason;
  ASSERT_GT(result.joint_path.size(), 2U);
  EXPECT_EQ(result.joint_path.front(), start);
  EXPECT_LE(result.position_error_m, parameters.position_tolerance_m);
  EXPECT_LE(result.orientation_error_rad, parameters.orientation_tolerance_rad);
  for (const auto & q : result.joint_path) {
    Eigen::Isometry3d pose;
    ASSERT_TRUE(kinematics(q, pose, jacobian));
    const Eigen::Vector3d delta = pose.translation() - seed.translation();
    EXPECT_LE((delta - axis * delta.dot(axis)).norm(), parameters.position_tolerance_m);
    EXPECT_LE(Eigen::AngleAxisd(pose.linear() * seed.linear().transpose()).angle(),
      parameters.orientation_tolerance_rad);
    EXPECT_TRUE(valid(q));
  }
  RecordProperty("selected_workspace_xyz_mm", "225,0,150");
  const auto precise = [](double value) {
      std::ostringstream text;
      text << std::setprecision(17) << value;
      return text.str();
    };
  // gtest has string/int property overloads; passing double truncates to int.
  RecordProperty("requested_insertion_m", precise(0.07));
  RecordProperty("final_position_error_m", precise(result.position_error_m));
  RecordProperty("final_orientation_error_rad", precise(result.orientation_error_rad));
}

TEST(FinalGraspRobot, GreenWitnessLateralBridgeAndInsertionWorkWithRotatedWorldBase)
{
  for (bool rotate_base : {false, true}) {
    SCOPED_TRACE(rotate_base ? "rotated world base" : "archived world base");
    const auto model = archivedModel(rotate_base);
    const auto * group = model->getJointModelGroup("arm");
    ASSERT_NE(group, nullptr);
    const auto start = greenWitness();
    const auto kinematics = worldKinematics(model);
    std::vector<double> lower, upper;
    for (const auto & name : group->getVariableNames()) {
      const auto & bounds = model->getVariableBounds(name);
      lower.push_back(bounds.min_position_);
      upper.push_back(bounds.max_position_);
    }
    Eigen::Isometry3d seed;
    Eigen::MatrixXd jacobian;
    ASSERT_TRUE(kinematics(start, seed, jacobian));
    Eigen::Vector3d horizontal = seed.linear() * Eigen::Vector3d::UnitZ();
    horizontal.z() = 0.0;
    horizontal.normalize();
    const Eigen::Vector3d lateral(-horizontal.y(), horizontal.x(), 0.0);
    const Eigen::Vector3d pregrasp_tcp = seed.translation() + 0.03 * lateral;
    const Eigen::Vector3d target = pregrasp_tcp + 0.07 * horizontal;
    grasp::FinalGraspParameters parameters;
    parameters.max_wall_time_sec = 3.0;
    moveit::core::RobotState state(model);
    const auto valid = [&](const std::vector<double> & q) {
        setOpenState(state, group, q);
        return state.satisfiesBounds();
      };
    const auto direct = grasp::planCartesianGrasp(
      start, lower, upper, target, parameters, kinematics, valid);
    EXPECT_FALSE(direct.valid);
    EXPECT_EQ(direct.reason, "final_grasp_approach_misaligned");

    const auto bridge = grasp::planCartesianPregrasp(
      start, lower, upper, pregrasp_tcp, parameters, kinematics, valid);
    ASSERT_TRUE(bridge.valid) << bridge.reason;
    ASSERT_GT(bridge.joint_path.size(), 1U);
    EXPECT_EQ(bridge.joint_path.front(), start);
    EXPECT_LE(bridge.position_error_m, parameters.position_tolerance_m);
    for (const auto & q : bridge.joint_path) {
      Eigen::Isometry3d pose;
      ASSERT_TRUE(kinematics(q, pose, jacobian));
      EXPECT_LE(Eigen::AngleAxisd(pose.linear() * seed.linear().transpose()).angle(),
        parameters.orientation_tolerance_rad);
      const Eigen::Vector3d delta = pose.translation() - seed.translation();
      EXPECT_LE((delta - lateral * delta.dot(lateral)).norm(), parameters.position_tolerance_m);
      EXPECT_TRUE(valid(q));
    }
    const auto insertion = grasp::planCartesianGrasp(
      bridge.joint_path.back(), lower, upper, target, parameters, kinematics, valid);
    ASSERT_TRUE(insertion.valid) << insertion.reason;
    EXPECT_LE(insertion.position_error_m, parameters.position_tolerance_m);
    EXPECT_LE(insertion.orientation_error_rad, parameters.orientation_tolerance_rad);

    // This geometric test callback deliberately places an obstacle band across
    // the bridge. Both endpoint poses are outside it. No hardware inference.
    std::size_t blocked_checks = 0U;
    const auto blocked = [&](const std::vector<double> & q) {
        if (!valid(q)) {return false;}
        const auto & pose = state.getGlobalLinkTransform("end_effector_link");
        const double traveled = (pose.translation() - seed.translation()).dot(lateral);
        if (traveled > 0.012 && traveled < 0.018) {++blocked_checks; return false;}
        return true;
      };
    ASSERT_TRUE(blocked(start));
    ASSERT_TRUE(blocked(bridge.joint_path.back()));
    const auto rejected = grasp::planCartesianPregrasp(
      start, lower, upper, pregrasp_tcp, parameters, kinematics, blocked);
    EXPECT_FALSE(rejected.valid);
    EXPECT_EQ(rejected.reason, "pregrasp_bridge_collision");
    EXPECT_TRUE(rejected.joint_path.empty());
    EXPECT_GT(blocked_checks, 0U);
  }
}

TEST(FinalGraspRobot, CapturedOverconstrainedBridgeIsRejectedAfterSubdivision)
{
  const auto model = archivedModel();
  const auto * group = model->getJointModelGroup("arm");
  ASSERT_NE(group, nullptr);
  // Captured failed bridge, frozen before this regression was run. The former
  // fixed-step solver rejected a >0.15 rad endpoint jump along this bridge.
  const auto start = greenWitness(125.0, -25.0, 150.0);
  const Eigen::Vector3d requested_tcp(
    0.1686217994051561, -0.006386257633918535, 0.11449165642261505);
  const auto kinematics = worldKinematics(model);
  std::vector<double> lower, upper;
  for (const auto & name : group->getVariableNames()) {
    const auto & bounds = model->getVariableBounds(name);
    lower.push_back(bounds.min_position_);
    upper.push_back(bounds.max_position_);
  }
  grasp::FinalGraspParameters parameters;
  parameters.max_wall_time_sec = 3.0;  // Numerical fixture, independent of build-machine load.
  moveit::core::RobotState state(model);
  const auto valid = [&](const std::vector<double> & q) {
      setOpenState(state, group, q);
      return state.satisfiesBounds();
    };
  Eigen::Isometry3d initial;
  Eigen::MatrixXd jacobian;
  ASSERT_TRUE(kinematics(start, initial, jacobian));
  const auto result = grasp::planCartesianPregrasp(
    start, lower, upper, requested_tcp, parameters, kinematics, valid);
  RecordProperty("captured_workspace_xyz_mm", "125,-25,150");
  RecordProperty("cartesian_subdivisions", static_cast<int>(result.cartesian_subdivisions));
  RecordProperty("peak_attempted_joint_step_rad", std::to_string(result.peak_attempted_joint_step_rad));
  // Subdivision resolves the coarse jump, but this particular fixed-pose
  // connection runs into the wrist bound. A planner must try another anchor;
  // a local solver must never claim an incomplete bridge as executable.
  EXPECT_FALSE(result.valid);
  EXPECT_EQ(result.reason, "pregrasp_bridge_ik_no_progress");
  EXPECT_TRUE(result.joint_path.empty());
  EXPECT_GT(result.cartesian_subdivisions, 0U);
  EXPECT_GT(result.peak_attempted_joint_step_rad, parameters.max_joint_step_rad);
}

}  // namespace
