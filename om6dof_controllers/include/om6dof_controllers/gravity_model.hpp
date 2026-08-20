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

#ifndef OM6DOF_CONTROLLERS__GRAVITY_MODEL_HPP_
#define OM6DOF_CONTROLLERS__GRAVITY_MODEL_HPP_

#include <array>
#include <memory>
#include <string>
#include <vector>

#include "kdl/chain.hpp"
#include "kdl/tree.hpp"
#include "kdl/chaindynparam.hpp"
#include "kdl/jntarray.hpp"

#include "om6dof_controllers/visibility_control.h"

namespace om6dof_controllers
{

/// Coulomb + viscous friction feed-forward for one joint.
///
/// `sign(qd)` is replaced by `tanh(qd / deadzone)` so the term fades smoothly
/// through zero instead of chattering on velocity quantisation noise.
struct OM6DOF_CONTROLLERS_PUBLIC FrictionParameters
{
  double coulomb{0.0};
  double viscous{0.0};
  double deadzone{0.05};
};

/// Gravity torques g(q) for a serial chain, taken straight from the robot
/// description.
///
/// The model is derived from the URDF link masses and inertias via KDL; it
/// carries no identified parameters of its own. Per-joint scaling and friction
/// belong to the controller that owns this object.
class OM6DOF_CONTROLLERS_PUBLIC GravityModel
{
public:
  GravityModel() = default;

  /// Build the chain and the dynamics solver.
  ///
  /// \return an empty string on success, or a human-readable reason.
  std::string configure(
    const std::string & urdf, const std::string & base_link, const std::string & tip_link,
    const std::vector<std::string> & joint_names, const std::array<double, 3> & gravity);

  bool is_configured() const {return dyn_param_ != nullptr;}

  /// Gravity torque per controller joint, in newton-metres.
  ///
  /// `positions` and `torques` are both indexed by the controller's joint
  /// order. Controller joints that are not part of the chain get zero.
  /// Allocation-free once configured, so it is safe from update().
  void compute(const std::vector<double> & positions, std::vector<double> & torques);

  /// Friction torque opposing `velocity`, in newton-metres.
  static double friction(double velocity, const FrictionParameters & parameters);

  /// Chain joint names, in chain order. Empty until configured.
  const std::vector<std::string> & chain_joint_names() const {return chain_joint_names_;}

  /// Mass that was hanging off the chain and had to be folded into it, in kg.
  double folded_mass() const {return folded_mass_;}

  /// Names of the off-chain links whose mass was folded in.
  const std::vector<std::string> & folded_links() const {return folded_links_;}

private:
  /// Add the inertia of every massive link hanging off the chain to the nearest
  /// chain link it is rigidly attached to. Without this, KDL's single-path
  /// getChain quietly drops them.
  void foldOffChainMass(const KDL::Tree & tree);

  KDL::Chain chain_;
  double folded_mass_{0.0};
  std::vector<std::string> folded_links_;
  std::unique_ptr<KDL::ChainDynParam> dyn_param_;
  std::vector<std::string> chain_joint_names_;

  /// For each chain joint, the index into the controller's joint vector.
  std::vector<size_t> chain_to_controller_;

  KDL::JntArray q_;
  KDL::JntArray gravity_torque_;
};

}  // namespace om6dof_controllers

#endif  // OM6DOF_CONTROLLERS__GRAVITY_MODEL_HPP_
