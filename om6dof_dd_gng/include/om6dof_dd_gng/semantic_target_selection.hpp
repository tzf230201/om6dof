#pragma once

#include <algorithm>
#include <map>
#include <utility>
#include <vector>
#include "om6dof_dd_gng/reachability_graph.hpp"

namespace om6dof_dd_gng::reachability
{
struct LabeledTarget
{
  Target target;
  int class_id;
};

// Select an observed surface node nearest the bounding-box center of each
// same-class connected component. Never synthesize an interior grasp point or
// average disconnected objects together. Geometry/collision data stay intact.
inline std::vector<Target> componentCenterTargets(
  const std::vector<LabeledTarget> & nodes,
  const std::vector<std::pair<std::uint32_t, std::uint32_t>> & edges)
{
  std::map<std::uint32_t, std::size_t> indices;
  for (std::size_t i = 0; i < nodes.size(); ++i) {
    indices.emplace(nodes[i].target.environment_node_id, i);
  }
  std::vector<std::vector<std::size_t>> adjacency(nodes.size());
  for (const auto & edge : edges) {
    const auto a = indices.find(edge.first), b = indices.find(edge.second);
    if (a != indices.end() && b != indices.end() &&
      nodes[a->second].class_id == nodes[b->second].class_id)
    {
      adjacency[a->second].push_back(b->second);
      adjacency[b->second].push_back(a->second);
    }
  }
  std::vector<bool> visited(nodes.size(), false);
  std::vector<Target> selected;
  for (std::size_t root = 0; root < nodes.size(); ++root) {
    if (visited[root] || nodes[root].class_id < 0) {continue;}
    std::vector<std::size_t> component{root};
    visited[root] = true;
    auto minimum = nodes[root].target.position, maximum = minimum;
    for (std::size_t cursor = 0; cursor < component.size(); ++cursor) {
      const auto i = component[cursor];
      const auto & p = nodes[i].target.position;
      minimum.x = std::min(minimum.x, p.x); maximum.x = std::max(maximum.x, p.x);
      minimum.y = std::min(minimum.y, p.y); maximum.y = std::max(maximum.y, p.y);
      minimum.z = std::min(minimum.z, p.z); maximum.z = std::max(maximum.z, p.z);
      for (const auto next : adjacency[i]) {
        if (!visited[next]) {visited[next] = true; component.push_back(next);}
      }
    }
    const Point3 center{(minimum.x + maximum.x) * 0.5,
      (minimum.y + maximum.y) * 0.5, (minimum.z + maximum.z) * 0.5};
    const auto best = std::min_element(component.begin(), component.end(),
      [&](std::size_t a, std::size_t b) {
        const auto da = distance(nodes[a].target.position, center);
        const auto db = distance(nodes[b].target.position, center);
        return da < db || (da == db &&
          nodes[a].target.environment_node_id < nodes[b].target.environment_node_id);
      });
    selected.push_back(nodes[*best].target);
  }
  std::sort(selected.begin(), selected.end(), [](const Target & a, const Target & b) {
    return a.environment_node_id < b.environment_node_id;
  });
  return selected;
}

// Retain several observed surface nodes nearest each component's bounding-box
// center.  This is a target *set*, not a synthetic interior point: the planner
// still accepts only a node and graph path that pass its full collision checks.
// It gives exact validation alternatives around a grasp-height surface when one
// centre-nearest node is blocked by the object's own geometry.
inline std::vector<Target> componentCenterNeighborhoodTargets(
  const std::vector<LabeledTarget> & nodes,
  const std::vector<std::pair<std::uint32_t, std::uint32_t>> & edges,
  std::size_t candidates_per_component)
{
  if (candidates_per_component == 0U) {return {};}
  std::map<std::uint32_t, std::size_t> indices;
  for (std::size_t i = 0; i < nodes.size(); ++i) {
    indices.emplace(nodes[i].target.environment_node_id, i);
  }
  std::vector<std::vector<std::size_t>> adjacency(nodes.size());
  for (const auto & edge : edges) {
    const auto a = indices.find(edge.first), b = indices.find(edge.second);
    if (a != indices.end() && b != indices.end() &&
      nodes[a->second].class_id == nodes[b->second].class_id)
    {
      adjacency[a->second].push_back(b->second);
      adjacency[b->second].push_back(a->second);
    }
  }
  std::vector<bool> visited(nodes.size(), false);
  std::vector<Target> selected;
  for (std::size_t root = 0; root < nodes.size(); ++root) {
    if (visited[root] || nodes[root].class_id < 0) {continue;}
    std::vector<std::size_t> component{root};
    visited[root] = true;
    auto minimum = nodes[root].target.position, maximum = minimum;
    for (std::size_t cursor = 0; cursor < component.size(); ++cursor) {
      const auto i = component[cursor];
      const auto & p = nodes[i].target.position;
      minimum.x = std::min(minimum.x, p.x); maximum.x = std::max(maximum.x, p.x);
      minimum.y = std::min(minimum.y, p.y); maximum.y = std::max(maximum.y, p.y);
      minimum.z = std::min(minimum.z, p.z); maximum.z = std::max(maximum.z, p.z);
      for (const auto next : adjacency[i]) {
        if (!visited[next]) {visited[next] = true; component.push_back(next);}
      }
    }
    const Point3 center{(minimum.x + maximum.x) * 0.5,
      (minimum.y + maximum.y) * 0.5, (minimum.z + maximum.z) * 0.5};
    std::sort(component.begin(), component.end(), [&](std::size_t a, std::size_t b) {
      const auto da = distance(nodes[a].target.position, center);
      const auto db = distance(nodes[b].target.position, center);
      return da < db || (da == db &&
        nodes[a].target.environment_node_id < nodes[b].target.environment_node_id);
    });
    const auto count = std::min(candidates_per_component, component.size());
    for (std::size_t i = 0; i < count; ++i) {
      selected.push_back(nodes[component[i]].target);
    }
  }
  std::sort(selected.begin(), selected.end(), [](const Target & a, const Target & b) {
    return a.environment_node_id < b.environment_node_id;
  });
  return selected;
}
}  // namespace om6dof_dd_gng::reachability
