#pragma once

// Conservative visual-only semantic completion for an already-labelled GNG
// component. A node can inherit a class only from multiple labelled direct
// graph neighbours in a small 3D radius. It never changes detector evidence,
// DD-GNG attention, or the complete collision point cloud.

#include "om6dof_dd_gng/ddgng.hpp"

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <map>
#include <utility>
#include <vector>

namespace om6dof_dd_gng
{

struct SemanticClusterPropagationParameters
{
  bool enabled = true;
  float max_edge_length_m = 0.055F;
  std::size_t min_same_class_neighbours = 2U;
  float min_neighbour_confidence = 0.15F;
};

// Updates are simultaneous, so a newly inherited node cannot cascade into a
// neighbouring object in the same frame. Returns the number of filled nodes.
inline std::size_t propagateUnknownClusterLabels(
  const std::vector<GngPoint3f> & nodes,
  const std::vector<std::pair<uint16_t, uint16_t>> & edges,
  std::vector<int16_t> & class_ids,
  std::vector<float> & confidences,
  const SemanticClusterPropagationParameters & parameters = {})
{
  if (!parameters.enabled || nodes.size() != class_ids.size() ||
    nodes.size() != confidences.size() || !std::isfinite(parameters.max_edge_length_m) ||
    parameters.max_edge_length_m <= 0.0F || parameters.min_same_class_neighbours == 0U ||
    !std::isfinite(parameters.min_neighbour_confidence) ||
    parameters.min_neighbour_confidence < 0.0F)
  {
    return 0U;
  }

  struct Support {std::size_t count = 0U; float confidence_sum = 0.0F;};
  std::vector<std::map<int16_t, Support>> support(nodes.size());
  const float max_squared = parameters.max_edge_length_m * parameters.max_edge_length_m;
  const auto validPoint = [](const GngPoint3f & point) {
      return std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z);
    };
  const auto addSupport = [&](std::size_t unknown, std::size_t labelled) {
      if (class_ids[labelled] < 0 || confidences[labelled] < parameters.min_neighbour_confidence) {
        return;
      }
      Support & entry = support[unknown][class_ids[labelled]];
      ++entry.count;
      entry.confidence_sum += confidences[labelled];
    };

  for (const auto & edge : edges) {
    const std::size_t a = edge.first;
    const std::size_t b = edge.second;
    if (a >= nodes.size() || b >= nodes.size() || !validPoint(nodes[a]) || !validPoint(nodes[b])) {
      continue;
    }
    const float dx = nodes[a].x - nodes[b].x;
    const float dy = nodes[a].y - nodes[b].y;
    const float dz = nodes[a].z - nodes[b].z;
    if (dx * dx + dy * dy + dz * dz > max_squared) {
      continue;
    }
    if (class_ids[a] < 0 && class_ids[b] >= 0) {
      addSupport(a, b);
    } else if (class_ids[b] < 0 && class_ids[a] >= 0) {
      addSupport(b, a);
    }
  }

  std::size_t inherited = 0U;
  for (std::size_t node = 0; node < nodes.size(); ++node) {
    if (class_ids[node] >= 0 || support[node].size() != 1U) {
      continue;  // competing semantic evidence stays UNKNOWN
    }
    const auto & entry = *support[node].begin();
    if (entry.second.count < parameters.min_same_class_neighbours) {
      continue;
    }
    class_ids[node] = entry.first;
    confidences[node] = entry.second.confidence_sum / static_cast<float>(entry.second.count);
    ++inherited;
  }
  return inherited;
}

}  // namespace om6dof_dd_gng
