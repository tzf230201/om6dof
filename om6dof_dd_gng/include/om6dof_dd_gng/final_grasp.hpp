#pragma once

// Local, bounded Cartesian bridge and insertion from validated graph states.
// Kinematics and scene policy are injected: no ROS/action or model mutation.
// All angular Jacobian rows and rotation errors are in the WORLD frame.

#include <Eigen/Cholesky>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/SVD>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <limits>
#include <string>
#include <vector>

namespace om6dof_dd_gng::grasp
{

struct FinalGraspParameters
{
  Eigen::Vector3d tool_approach_axis = Eigen::Vector3d::UnitZ();
  Eigen::Vector3d tcp_to_pinch = Eigen::Vector3d::Zero();
  double cartesian_step_m = 0.005;
  double collision_step_rad = 0.025;
  double max_joint_step_rad = 0.15;
  double position_tolerance_m = 0.003;
  double orientation_tolerance_rad = 0.03;
  double max_axis_vertical = 0.17364817766693033;  // sin(10 degrees)
  double minimum_approach_alignment = 0.95;
  double max_insertion_distance_m = 0.15;
  double max_pregrasp_bridge_distance_m = 0.06;
  double damping = 0.005;
  double orientation_weight_m = 0.10;
  double minimum_condition_ratio = 1.0e-6;
  double max_wall_time_sec = 0.15;
  int max_iterations = 100;
  int max_waypoints = 64;
  int max_backtracks = 10;
};

using KinematicsFn = std::function<bool(
    const std::vector<double> &, Eigen::Isometry3d &, Eigen::MatrixXd &)>;
using StateValidityFn = std::function<bool(const std::vector<double> &)>;

struct FinalGraspResult
{
  bool valid = false;
  std::string reason;
  // Includes the supplied seed exactly once; a failed result has no path.
  std::vector<std::vector<double>> joint_path;
  double position_error_m = std::numeric_limits<double>::infinity();
  double orientation_error_rad = std::numeric_limits<double>::infinity();
  std::size_t cartesian_subdivisions = 0U;
  double peak_attempted_joint_step_rad = 0.0;
};

namespace detail
{
inline bool finiteVector(const std::vector<double> & values)
{
  return std::all_of(values.begin(), values.end(), [](double x) {return std::isfinite(x);});
}

inline bool finitePose(const Eigen::Isometry3d & pose)
{
  return pose.matrix().allFinite() &&
    (pose.linear().transpose() * pose.linear() - Eigen::Matrix3d::Identity()).norm() < 1.0e-6 &&
    std::abs(pose.linear().determinant() - 1.0) < 1.0e-6 &&
    (pose.matrix().row(3) - Eigen::RowVector4d(0.0, 0.0, 0.0, 1.0)).norm() < 1.0e-9;
}

inline Eigen::Matrix<double, 6, 1> poseError(
  const Eigen::Vector3d & target_position, const Eigen::Matrix3d & target_rotation,
  const Eigen::Isometry3d & current)
{
  Eigen::Matrix<double, 6, 1> error;
  error.head<3>() = target_position - current.translation();
  const Eigen::AngleAxisd rotation_error(target_rotation * current.linear().transpose());
  error.tail<3>() = rotation_error.angle() * rotation_error.axis();
  return error;
}
// Private common solver. Insertion targets the pinch point and must advance
// along the tool ray. A pregrasp bridge targets the TCP and may move laterally,
// while preserving the same seed orientation and collision validation rules.
inline FinalGraspResult planCartesianTranslation(
  const std::vector<double> & start, const std::vector<double> & lower,
  const std::vector<double> & upper, const Eigen::Vector3d & target_world,
  const FinalGraspParameters & parameters, const KinematicsFn & kinematics,
  const StateValidityFn & valid_state, bool insertion)
{
  FinalGraspResult result;
  const auto fail = [&result, insertion](const char * reason) {
      result.valid = false;
      result.reason = insertion ? std::string(reason) :
        "pregrasp_bridge_" + std::string(reason).substr(std::string("final_grasp_").size());
      result.joint_path.clear();
      return result;
    };
  const auto positive = [](double value) {return std::isfinite(value) && value > 0.0;};
  if (!kinematics || !valid_state || start.size() != 6U || lower.size() != start.size() ||
    upper.size() != start.size() || !detail::finiteVector(start) ||
    !detail::finiteVector(lower) || !detail::finiteVector(upper) ||
    !target_world.allFinite() || !parameters.tool_approach_axis.allFinite() ||
    !std::isfinite(parameters.tool_approach_axis.norm()) ||
    parameters.tool_approach_axis.norm() < 1.0e-9 || !parameters.tcp_to_pinch.allFinite() ||
    !positive(parameters.cartesian_step_m) || !positive(parameters.collision_step_rad) ||
    !positive(parameters.max_joint_step_rad) || !positive(parameters.position_tolerance_m) ||
    !positive(parameters.orientation_tolerance_rad) || !positive(parameters.max_insertion_distance_m) ||
    parameters.cartesian_step_m > 0.005 || parameters.collision_step_rad > 0.025 ||
    parameters.max_joint_step_rad > 0.15 || parameters.position_tolerance_m > 0.003 ||
    parameters.orientation_tolerance_rad > 0.03 ||
    !positive(parameters.damping) || !positive(parameters.orientation_weight_m) ||
    !positive(parameters.max_wall_time_sec) || !positive(parameters.minimum_condition_ratio) ||
    parameters.minimum_condition_ratio > 1.0 || !std::isfinite(parameters.max_axis_vertical) ||
    parameters.max_axis_vertical < 0.0 || parameters.max_axis_vertical > 1.0 ||
    !std::isfinite(parameters.minimum_approach_alignment) ||
    parameters.minimum_approach_alignment <= 0.0 || parameters.minimum_approach_alignment > 1.0 ||
    (!insertion && (!positive(parameters.max_pregrasp_bridge_distance_m) ||
    parameters.max_pregrasp_bridge_distance_m > 0.06)) ||
    parameters.max_iterations <= 0 || parameters.max_waypoints <= 0 || parameters.max_backtracks <= 0)
  {
    return fail("final_grasp_invalid_parameters");
  }
  const auto withinBounds = [&lower, &upper](const std::vector<double> & joints) {
      if (!detail::finiteVector(joints)) {return false;}
      for (std::size_t i = 0; i < joints.size(); ++i) {
        if (lower[i] >= upper[i] || joints[i] < lower[i] || joints[i] > upper[i]) {return false;}
      }
      return true;
    };
  if (!withinBounds(start)) {return fail("final_grasp_start_out_of_bounds");}
  const auto began = std::chrono::steady_clock::now();
  const auto timedOut = [&began, &parameters]() {
      return std::chrono::duration<double>(std::chrono::steady_clock::now() - began).count() >
             parameters.max_wall_time_sec;
    };
  const auto evaluate = [&kinematics, &start](
    const std::vector<double> & joints, Eigen::Isometry3d & pose, Eigen::MatrixXd & jacobian) {
      return kinematics(joints, pose, jacobian) && detail::finitePose(pose) &&
             jacobian.rows() == 6 && jacobian.cols() == static_cast<int>(start.size()) &&
             jacobian.allFinite();
    };
  Eigen::Isometry3d initial;
  Eigen::MatrixXd jacobian;
  if (!evaluate(start, initial, jacobian)) {return fail("final_grasp_invalid_kinematics");}
  if (!valid_state(start)) {return fail("final_grasp_start_collision");}
  if (timedOut()) {return fail("final_grasp_timeout");}
  const Eigen::Vector3d axis = initial.linear() * parameters.tool_approach_axis.normalized();
  if (std::abs(axis.z()) > parameters.max_axis_vertical) {
    return fail("final_grasp_approach_not_horizontal");
  }
  const Eigen::Vector3d initial_reference = insertion ?
    Eigen::Vector3d(initial * parameters.tcp_to_pinch) : Eigen::Vector3d(initial.translation());
  const Eigen::Vector3d displacement = target_world - initial_reference;
  const double distance = displacement.norm();
  const double maximum_distance = insertion ? parameters.max_insertion_distance_m :
    parameters.max_pregrasp_bridge_distance_m;
  if (!std::isfinite(distance) || distance > maximum_distance) {
    return fail(insertion ? "final_grasp_insertion_too_long" : "final_grasp_too_long");
  }
  if (insertion && distance > parameters.position_tolerance_m &&
    axis.dot(displacement / distance) < parameters.minimum_approach_alignment)
  {
    return fail("final_grasp_approach_misaligned");
  }
  const double required = std::ceil(distance / parameters.cartesian_step_m);
  if (!std::isfinite(required) || required > parameters.max_waypoints) {
    return fail("final_grasp_waypoint_budget_exceeded");
  }
  const int segments = std::max(1, static_cast<int>(required));
  std::vector<double> waypoint_fractions;
  waypoint_fractions.reserve(static_cast<std::size_t>(segments));
  for (int segment = 1; segment <= segments; ++segment) {
    waypoint_fractions.push_back(static_cast<double>(segment) / segments);
  }
  result.joint_path.push_back(start);
  std::vector<double> previous = start;
  Eigen::Isometry3d final_pose = initial;
  double previous_fraction = 0.0;
  std::size_t segment = 0U;
  while (segment < waypoint_fractions.size()) {
    const double fraction = waypoint_fractions[segment];
    const double interval_distance = distance * (fraction - previous_fraction);
    const Eigen::Vector3d requested = initial.translation() +
      displacement * fraction;
    // Default terminal tolerance is 3 mm, which would swallow a small split
    // interval. Require progress toward every local waypoint, while never
    // loosening a caller's tighter tolerance. The numerical floor is 0.1 um.
    const double step_position_tolerance = std::min(parameters.position_tolerance_m,
      std::max(1.0e-7, interval_distance * 0.25));
    std::vector<double> joints = previous;
    bool solved = false;
    for (int iteration = 0; iteration < parameters.max_iterations; ++iteration) {
      if (timedOut()) {return fail("final_grasp_timeout");}
      Eigen::Isometry3d pose;
      if (!evaluate(joints, pose, jacobian)) {return fail("final_grasp_invalid_kinematics");}
      const auto error = detail::poseError(requested, initial.linear(), pose);
      if (!error.allFinite()) {return fail("final_grasp_invalid_kinematics");}
      if (error.head<3>().norm() <= step_position_tolerance &&
        error.tail<3>().norm() <= parameters.orientation_tolerance_rad)
      {
        final_pose = pose;
        solved = true;
        break;
      }
      Eigen::MatrixXd weighted_jacobian = jacobian;
      weighted_jacobian.bottomRows(3) *= parameters.orientation_weight_m;
      Eigen::Matrix<double, 6, 1> weighted_error = error;
      weighted_error.tail<3>() *= parameters.orientation_weight_m;
      Eigen::JacobiSVD<Eigen::MatrixXd> svd(weighted_jacobian);
      const auto singular = svd.singularValues();
      if (!singular.allFinite() || singular.size() != 6 || singular[0] <= 1.0e-12 ||
        singular[5] / singular[0] < parameters.minimum_condition_ratio)
      {
        return fail("final_grasp_ik_singular");
      }
      Eigen::MatrixXd hessian = weighted_jacobian.transpose() * weighted_jacobian;
      hessian.diagonal().array() += parameters.damping * parameters.damping;
      const Eigen::VectorXd step = hessian.ldlt().solve(
        weighted_jacobian.transpose() * weighted_error);
      if (!step.allFinite()) {return fail("final_grasp_ik_numerical_failure");}
      const double largest_step = step.cwiseAbs().maxCoeff();
      const double scale = largest_step > parameters.max_joint_step_rad ?
        parameters.max_joint_step_rad / largest_step : 1.0;
      const double previous_cost = weighted_error.squaredNorm();
      bool accepted = false;
      for (int backtrack = 0; backtrack < parameters.max_backtracks; ++backtrack) {
        if (timedOut()) {return fail("final_grasp_timeout");}
        const double fraction = std::ldexp(scale, -backtrack);
        std::vector<double> candidate(joints.size());
        for (std::size_t j = 0; j < candidate.size(); ++j) {
          candidate[j] = std::clamp(joints[j] + fraction * step[j], lower[j], upper[j]);
        }
        Eigen::Isometry3d candidate_pose;
        Eigen::MatrixXd candidate_jacobian;
        if (!evaluate(candidate, candidate_pose, candidate_jacobian)) {
          return fail("final_grasp_invalid_kinematics");
        }
        auto candidate_error = detail::poseError(requested, initial.linear(), candidate_pose);
        candidate_error.tail<3>() *= parameters.orientation_weight_m;
        if (candidate_error.squaredNorm() + 1.0e-16 < previous_cost) {
          joints = std::move(candidate);
          accepted = true;
          break;
        }
      }
      if (!accepted) {return fail("final_grasp_ik_no_progress");}
    }
    if (!solved) {return fail("final_grasp_ik_iteration_limit");}
    double max_delta = 0.0;
    for (std::size_t j = 0; j < joints.size(); ++j) {
      max_delta = std::max(max_delta, std::abs(joints[j] - previous[j]));
    }
    result.peak_attempted_joint_step_rad = std::max(result.peak_attempted_joint_step_rad, max_delta);
    if (max_delta > parameters.max_joint_step_rad) {
      // A continuous IK solution can legitimately require more joint motion
      // than one coarse Cartesian step permits. Retry two smaller intervals
      // from the last accepted state; never transmit the oversized jump.
      constexpr double minimum_subdivided_step_m = 0.0001;
      if (interval_distance * 0.5 < minimum_subdivided_step_m) {
        return fail("final_grasp_joint_jump");
      }
      if (waypoint_fractions.size() >= static_cast<std::size_t>(parameters.max_waypoints)) {
        return fail("final_grasp_waypoint_budget_exceeded");
      }
      const double midpoint = previous_fraction + (fraction - previous_fraction) * 0.5;
      if (!std::isfinite(midpoint) || midpoint <= previous_fraction || midpoint >= fraction) {
        return fail("final_grasp_joint_jump");
      }
      waypoint_fractions.insert(waypoint_fractions.begin() + segment, midpoint);
      ++result.cartesian_subdivisions;
      continue;
    }
    const double checks_required = std::ceil(max_delta / parameters.collision_step_rad);
    if (!std::isfinite(checks_required) || checks_required > 10000.0) {
      return fail("final_grasp_collision_budget_exceeded");
    }
    const int checks = std::max(1, static_cast<int>(checks_required));
    for (int check = 1; check <= checks; ++check) {
      if (timedOut()) {return fail("final_grasp_timeout");}
      const double fraction = static_cast<double>(check) / checks;
      std::vector<double> interpolated(joints.size());
      for (std::size_t j = 0; j < joints.size(); ++j) {
        interpolated[j] = previous[j] + fraction * (joints[j] - previous[j]);
      }
      if (!withinBounds(interpolated) || !valid_state(interpolated)) {
        return fail("final_grasp_collision");
      }
    }
    if (timedOut()) {return fail("final_grasp_timeout");}
    // Do not repeat an unchanged seed/waypoint when within solver tolerance.
    if (max_delta > 1.0e-12) {result.joint_path.push_back(joints);}
    previous = std::move(joints);
    previous_fraction = fraction;
    ++segment;
  }
  const Eigen::Vector3d final_reference = insertion ?
    Eigen::Vector3d(final_pose * parameters.tcp_to_pinch) : Eigen::Vector3d(final_pose.translation());
  result.position_error_m = (final_reference - target_world).norm();
  result.orientation_error_rad = Eigen::AngleAxisd(
    initial.linear() * final_pose.linear().transpose()).angle();
  if (!std::isfinite(result.position_error_m) || !std::isfinite(result.orientation_error_rad) ||
    result.position_error_m > parameters.position_tolerance_m ||
    result.orientation_error_rad > parameters.orientation_tolerance_rad)
  {
    return fail("final_grasp_terminal_error");
  }
  result.valid = true;
  result.reason = insertion ? "final_grasp_ready" : "pregrasp_bridge_ready";
  return result;
}

}  // namespace detail

inline FinalGraspResult planCartesianGrasp(
  const std::vector<double> & start, const std::vector<double> & lower,
  const std::vector<double> & upper, const Eigen::Vector3d & target_pinch_world,
  const FinalGraspParameters & parameters, const KinematicsFn & kinematics,
  const StateValidityFn & valid_state)
{
  return detail::planCartesianTranslation(
    start, lower, upper, target_pinch_world, parameters, kinematics, valid_state, true);
}

// Align a nearby graph anchor to a requested TCP pregrasp position without
// rotating it or inserting toward the object. The caller subsequently invokes
// planCartesianGrasp, whose tool-ray alignment requirement is unchanged.
inline FinalGraspResult planCartesianPregrasp(
  const std::vector<double> & start, const std::vector<double> & lower,
  const std::vector<double> & upper, const Eigen::Vector3d & desired_pregrasp_tcp_world,
  const FinalGraspParameters & parameters, const KinematicsFn & kinematics,
  const StateValidityFn & valid_state)
{
  return detail::planCartesianTranslation(
    start, lower, upper, desired_pregrasp_tcp_world, parameters, kinematics, valid_state, false);
}

}  // namespace om6dof_dd_gng::grasp
