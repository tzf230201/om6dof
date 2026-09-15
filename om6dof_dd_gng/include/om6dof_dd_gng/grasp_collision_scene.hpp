#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <map>
#include <memory>
#include <stdexcept>
#include <vector>

#include <Eigen/Geometry>
#include "moveit/planning_scene/planning_scene.h"
#include "moveit_msgs/msg/collision_object.hpp"
#include "om6dof_dd_gng/msg/environment_graph.hpp"
#include "shape_msgs/msg/solid_primitive.hpp"

namespace om6dof_dd_gng::grasp
{

inline constexpr const char * kTargetObjectId = "dd_gng_grasp_target";
inline constexpr const char * kObstacleObjectId = "dd_gng_grasp_obstacles";

// Masks follow the input graph's node/edge order. An edge belongs to the target
// only when both endpoints belong to the selected same-class component.
// Consumers can use the same partition for capsule and mesh collision scenes.
struct GraspTargetPartition
{
  std::vector<std::uint32_t> target_node_ids;
  std::vector<bool> node_is_target;
  std::vector<bool> edge_is_target;
  geometry_msgs::msg::Point center;
  std::int32_t class_id = -1;
};

struct GraspCollisionScene
{
  planning_scene::PlanningScenePtr scene;
  std::vector<std::uint32_t> target_node_ids;
  geometry_msgs::msg::Point center;
  std::int32_t class_id = -1;
  moveit_msgs::msg::CollisionObject target;
  moveit_msgs::msg::CollisionObject obstacles;
};

// Select one component, not every node of its semantic class. Validate the
// complete graph even when callers intend to omit the selected component.
inline GraspTargetPartition partitionGraspTarget(
  const om6dof_dd_gng::msg::EnvironmentGraph & environment,
  std::uint32_t target_id)
{
  std::map<std::uint32_t, std::size_t> indices;
  for (std::size_t i = 0; i < environment.nodes.size(); ++i) {
    const auto & node = environment.nodes[i];
    if (!std::isfinite(node.position.x) || !std::isfinite(node.position.y) ||
      !std::isfinite(node.position.z))
    {
      throw std::invalid_argument("grasp_scene_nonfinite_point");
    }
    if (!indices.emplace(node.id, i).second) {
      throw std::invalid_argument("grasp_scene_duplicate_node_id");
    }
  }
  const auto selected = indices.find(target_id);
  if (selected == indices.end() || environment.nodes[selected->second].class_id < 0) {
    throw std::invalid_argument("grasp_scene_target_node_missing_or_unlabelled");
  }
  std::vector<std::vector<std::size_t>> adjacency(environment.nodes.size());
  for (const auto & edge : environment.edges) {
    const auto a = indices.find(edge.source_id), b = indices.find(edge.target_id);
    if (a == indices.end() || b == indices.end()) {
      throw std::invalid_argument("grasp_scene_edge_references_missing_node");
    }
    if (environment.nodes[a->second].class_id == environment.nodes[b->second].class_id) {
      adjacency[a->second].push_back(b->second);
      adjacency[b->second].push_back(a->second);
    }
  }
  std::vector<bool> in_component(environment.nodes.size(), false);
  std::vector<std::size_t> component{selected->second};
  in_component[selected->second] = true;
  auto minimum = environment.nodes[selected->second].position, maximum = minimum;
  for (std::size_t cursor = 0; cursor < component.size(); ++cursor) {
    const auto index = component[cursor];
    const auto & point = environment.nodes[index].position;
    minimum.x = std::min(minimum.x, point.x); maximum.x = std::max(maximum.x, point.x);
    minimum.y = std::min(minimum.y, point.y); maximum.y = std::max(maximum.y, point.y);
    minimum.z = std::min(minimum.z, point.z); maximum.z = std::max(maximum.z, point.z);
    for (const auto next : adjacency[index]) {
      if (!in_component[next]) {in_component[next] = true; component.push_back(next);}
    }
  }

  GraspTargetPartition result;
  result.class_id = environment.nodes[selected->second].class_id;
  result.center.x = (minimum.x + maximum.x) * 0.5;
  result.center.y = (minimum.y + maximum.y) * 0.5;
  result.center.z = (minimum.z + maximum.z) * 0.5;
  for (const auto index : component) {result.target_node_ids.push_back(environment.nodes[index].id);}
  std::sort(result.target_node_ids.begin(), result.target_node_ids.end());
  result.node_is_target = std::move(in_component);
  result.edge_is_target.reserve(environment.edges.size());
  for (const auto & edge : environment.edges) {
    result.edge_is_target.push_back(
      result.node_is_target[indices.at(edge.source_id)] &&
      result.node_is_target[indices.at(edge.target_id)]);
  }
  return result;
}

// Construct an independent scene. By default the selected target remains a
// separate collision object. When explicitly requested, omit only its nodes
// and internal edges; preserve its identity/center for grasp planning. Other
// instances, unknown nodes, and component-boundary edges remain obstacles.
inline GraspCollisionScene buildGraspCollisionScene(
  const moveit::core::RobotModelConstPtr & model,
  const om6dof_dd_gng::msg::EnvironmentGraph & environment,
  std::uint32_t target_id,
  double point_radius = 0.012,
  double edge_radius = 0.006,
  bool exclude_selected_target = false)
{
  if (!model) {throw std::invalid_argument("grasp_scene_missing_robot_model");}
  if (!std::isfinite(point_radius) || point_radius <= 0.0 ||
    !std::isfinite(edge_radius) || edge_radius <= 0.0)
  {
    throw std::invalid_argument("grasp_scene_invalid_collision_radius");
  }
  if (!environment.header.frame_id.empty() &&
    environment.header.frame_id != model->getModelFrame())
  {
    throw std::invalid_argument("grasp_scene_environment_frame_mismatch");
  }
  const auto partition = partitionGraspTarget(environment, target_id);
  GraspCollisionScene result;
  result.target_node_ids = partition.target_node_ids;
  result.center = partition.center;
  result.class_id = partition.class_id;
  std::map<std::uint32_t, std::size_t> indices;
  for (std::size_t i = 0; i < environment.nodes.size(); ++i) {
    indices.emplace(environment.nodes[i].id, i);
  }
  for (auto * object : {&result.target, &result.obstacles}) {
    object->header = environment.header;
    object->header.frame_id = model->getModelFrame();
    object->operation = moveit_msgs::msg::CollisionObject::ADD;
  }
  result.target.id = kTargetObjectId;
  result.obstacles.id = kObstacleObjectId;

  for (std::size_t i = 0; i < environment.nodes.size(); ++i) {
    if (exclude_selected_target && partition.node_is_target[i]) {continue;}
    auto & object = partition.node_is_target[i] ? result.target : result.obstacles;
    shape_msgs::msg::SolidPrimitive sphere;
    sphere.type = shape_msgs::msg::SolidPrimitive::SPHERE;
    sphere.dimensions.resize(1U);
    sphere.dimensions[shape_msgs::msg::SolidPrimitive::SPHERE_RADIUS] = point_radius;
    geometry_msgs::msg::Pose pose;
    pose.position = environment.nodes[i].position;
    pose.orientation.w = 1.0;
    object.primitives.push_back(std::move(sphere));
    object.primitive_poses.push_back(std::move(pose));
  }
  for (std::size_t edge_index = 0; edge_index < environment.edges.size(); ++edge_index) {
    if (exclude_selected_target && partition.edge_is_target[edge_index]) {continue;}
    const auto & edge = environment.edges[edge_index];
    const auto a_index = indices.at(edge.source_id), b_index = indices.at(edge.target_id);
    const auto & pa = environment.nodes[a_index].position;
    const auto & pb = environment.nodes[b_index].position;
    const Eigen::Vector3d a(pa.x, pa.y, pa.z), b(pb.x, pb.y, pb.z);
    const auto direction = (b - a).eval();
    const double length = direction.norm();
    // Match the reachability planner's sphere/cylinder model. Coincident
    // endpoints are already protected by their spheres.
    if (length <= 1.0e-6) {continue;}
    auto & object = partition.edge_is_target[edge_index] ?
      result.target : result.obstacles;
    shape_msgs::msg::SolidPrimitive cylinder;
    cylinder.type = shape_msgs::msg::SolidPrimitive::CYLINDER;
    cylinder.dimensions.resize(2U);
    cylinder.dimensions[shape_msgs::msg::SolidPrimitive::CYLINDER_HEIGHT] = length;
    cylinder.dimensions[shape_msgs::msg::SolidPrimitive::CYLINDER_RADIUS] = edge_radius;
    const Eigen::Quaterniond orientation = Eigen::Quaterniond::FromTwoVectors(
      Eigen::Vector3d::UnitZ(), direction / length);
    geometry_msgs::msg::Pose pose;
    pose.position.x = (pa.x + pb.x) * 0.5;
    pose.position.y = (pa.y + pb.y) * 0.5;
    pose.position.z = (pa.z + pb.z) * 0.5;
    pose.orientation.x = orientation.x(); pose.orientation.y = orientation.y();
    pose.orientation.z = orientation.z(); pose.orientation.w = orientation.w();
    object.primitives.push_back(std::move(cylinder));
    object.primitive_poses.push_back(std::move(pose));
  }
  result.scene = std::make_shared<planning_scene::PlanningScene>(model);
  for (const auto * object : {&result.target, &result.obstacles}) {
    if (!object->primitives.empty() && !result.scene->processCollisionObjectMsg(*object)) {
      throw std::runtime_error("grasp_scene_collision_object_rejected:" + object->id);
    }
  }
  return result;
}
}  // namespace om6dof_dd_gng::grasp
