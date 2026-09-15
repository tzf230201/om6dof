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

// Select one target per same-class connected component.  The target position
// is the component's 3-D bounding-box centre, while environment_node_id stays
// an observed centre-nearest node solely as a stable identity for snapshots
// and object tracking.  Collision geometry is built from every raw node
// before this selection, so using the centre as the grasp reference never
// makes the object disappear from collision checking.
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
    selected.push_back({nodes[*best].target.environment_node_id, center});
  }
  std::sort(selected.begin(), selected.end(), [](const Target & a, const Target & b) {
    return a.environment_node_id < b.environment_node_id;
  });
  return selected;
}

// Compatibility mode for callers that want multiple centre-nearest identities.
// Each identity deliberately references the same component centre: it must not
// turn a top or side surface node into the physical grasp reference.
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
      selected.push_back({nodes[component[i]].target.environment_node_id, center});
    }
  }
  std::sort(selected.begin(), selected.end(), [](const Target & a, const Target & b) {
    return a.environment_node_id < b.environment_node_id;
  });
  return selected;
}
}  // namespace om6dof_dd_gng::reachability
