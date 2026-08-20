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

#include "om6dof_controllers/gravity_model.hpp"

#include <algorithm>
#include <cmath>
#include <map>
#include <set>
#include <sstream>
#include <vector>

#include "kdl/tree.hpp"
#include "kdl_parser/kdl_parser.hpp"

namespace om6dof_controllers
{

std::string GravityModel::configure(
  const std::string & urdf, const std::string & base_link, const std::string & tip_link,
  const std::vector<std::string> & joint_names, const std::array<double, 3> & gravity)
{
  dyn_param_.reset();
  chain_joint_names_.clear();
  chain_to_controller_.clear();

  if (urdf.empty()) {
    return "robot description is empty";
  }

  KDL::Tree tree;
  if (!kdl_parser::treeFromString(urdf, tree)) {
    return "could not parse the robot description into a KDL tree";
  }

  if (!tree.getChain(base_link, tip_link, chain_)) {
    return "no KDL chain from '" + base_link + "' to '" + tip_link + "'";
  }

  foldOffChainMass(tree);

  // Only joints the chain actually moves matter; KDL already dropped the fixed
  // ones. Every one of them has to be a joint this controller reads, otherwise
  // its position is unknown and g(q) would be evaluated at a guess.
  std::ostringstream missing;
  for (unsigned int i = 0; i < chain_.getNrOfSegments(); ++i) {
    const KDL::Joint & joint = chain_.getSegment(i).getJoint();
    if (joint.getType() == KDL::Joint::None) {
      continue;
    }
    const auto it = std::find(joint_names.begin(), joint_names.end(), joint.getName());
    if (it == joint_names.end()) {
      missing << (chain_joint_names_.empty() ? "" : ", ") << joint.getName();
      continue;
    }
    chain_joint_names_.push_back(joint.getName());
    chain_to_controller_.push_back(static_cast<size_t>(std::distance(joint_names.begin(), it)));
  }

  const std::string missing_names = missing.str();
  if (!missing_names.empty()) {
    chain_joint_names_.clear();
    chain_to_controller_.clear();
    return "chain joints missing from the controller's joint list: " + missing_names;
  }

  if (chain_joint_names_.size() != chain_.getNrOfJoints()) {
    return "chain joint bookkeeping mismatch";
  }

  dyn_param_ = std::make_unique<KDL::ChainDynParam>(
    chain_, KDL::Vector(gravity[0], gravity[1], gravity[2]));

  q_.resize(chain_.getNrOfJoints());
  gravity_torque_.resize(chain_.getNrOfJoints());
  KDL::SetToZero(q_);
  KDL::SetToZero(gravity_torque_);

  return "";
}

void GravityModel::foldOffChainMass(const KDL::Tree & tree)
{
  // getChain walks one path and drops everything hanging off it. On this arm
  // that silently threw away the gripper fingers and the wrist payload, all of
  // it bolted to the last link, on the longest lever there is. The masses are
  // real whether or not KDL's chain happens to pass through them, so fold each
  // one into the nearest chain link it is rigidly attached to.
  folded_mass_ = 0.0;
  folded_links_.clear();

  std::set<std::string> on_chain;
  for (unsigned int i = 0; i < chain_.getNrOfSegments(); ++i) {
    on_chain.insert(chain_.getSegment(i).getName());
  }

  // Descend from the root recording each segment's parent. The parent iterator
  // KDL stores on a TreeElement is not dereferenceable for the root, so the
  // tree is walked downwards, where the child lists are always well formed.
  const auto root = tree.getRootSegment();
  if (root == tree.getSegments().end()) {
    return;
  }

  std::map<std::string, std::string> parent_of;
  std::vector<KDL::SegmentMap::const_iterator> pending{root};
  while (!pending.empty()) {
    const auto element = pending.back();
    pending.pop_back();
    for (const auto & child : GetTreeElementChildren(element->second)) {
      parent_of[child->first] = element->first;
      pending.push_back(child);
    }
  }

  std::map<std::string, KDL::RigidBodyInertia> folded;
  for (const auto & entry : tree.getSegments()) {
    const std::string & name = entry.first;
    if (on_chain.count(name) != 0 || name == root->first) {
      continue;
    }

    const KDL::RigidBodyInertia & inertia = GetTreeElementSegment(entry.second).getInertia();
    if (inertia.getMass() <= 0.0) {
      continue;
    }

    // Walk up to the first ancestor the chain does pass through, accumulating
    // the transform on the way. Movable joints along that path are taken at
    // zero: the gripper fingers travel about a centimetre, which is nothing
    // against being absent from the model entirely.
    KDL::Frame transform = KDL::Frame::Identity();
    std::string current = name;
    while (true) {
      const auto element = tree.getSegments().find(current);
      if (element == tree.getSegments().end()) {
        break;
      }
      transform = GetTreeElementSegment(element->second).pose(0.0) * transform;

      const auto parent = parent_of.find(current);
      if (parent == parent_of.end()) {
        break;                                  // reached the root, nothing to anchor to
      }
      if (on_chain.count(parent->second) != 0) {
        const KDL::RigidBodyInertia contribution = transform * inertia;
        const auto existing = folded.find(parent->second);
        if (existing == folded.end()) {
          folded.emplace(parent->second, contribution);
        } else {
          existing->second = existing->second + contribution;
        }
        folded_mass_ += inertia.getMass();
        folded_links_.push_back(name);
        break;
      }
      current = parent->second;
    }
  }

  if (folded.empty()) {
    return;
  }

  KDL::Chain augmented;
  for (unsigned int i = 0; i < chain_.getNrOfSegments(); ++i) {
    const KDL::Segment & segment = chain_.getSegment(i);
    KDL::RigidBodyInertia inertia = segment.getInertia();
    const auto it = folded.find(segment.getName());
    if (it != folded.end()) {
      inertia = inertia + it->second;
    }
    augmented.addSegment(
      KDL::Segment(segment.getName(), segment.getJoint(), segment.getFrameToTip(), inertia));
  }
  chain_ = augmented;
}

void GravityModel::compute(const std::vector<double> & positions, std::vector<double> & torques)
{
  std::fill(torques.begin(), torques.end(), 0.0);
  if (!dyn_param_) {
    return;
  }

  for (size_t i = 0; i < chain_to_controller_.size(); ++i) {
    q_(i) = positions[chain_to_controller_[i]];
  }

  if (dyn_param_->JntToGravity(q_, gravity_torque_) < 0) {
    return;
  }

  for (size_t i = 0; i < chain_to_controller_.size(); ++i) {
    torques[chain_to_controller_[i]] = gravity_torque_(i);
  }
}

double GravityModel::friction(double velocity, const FrictionParameters & parameters)
{
  const double deadzone = std::max(parameters.deadzone, 1.0e-9);
  return parameters.coulomb * std::tanh(velocity / deadzone) + parameters.viscous * velocity;
}

}  // namespace om6dof_controllers
