#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <queue>
#include <stdexcept>
#include <utility>
#include <vector>

namespace om6dof_dd_gng::reachability
{

constexpr std::size_t kInvalidIndex = std::numeric_limits<std::size_t>::max();

inline std::uint64_t splitMix64(std::uint64_t value)
{
  value += 0x9e3779b97f4a7c15ULL;
  value = (value ^ (value >> 30U)) * 0xbf58476d1ce4e5b9ULL;
  value = (value ^ (value >> 27U)) * 0x94d049bb133111ebULL;
  return value ^ (value >> 31U);
}

// Generalized Halton digit permutation. Zero remains fixed so the radical
// inverse has a finite representation, while digits 1..base-1 are shuffled
// independently for every dimension and stream seed. All bases used by the
// OM6DOF sampler are prime, but this construction is a bijection for any base.
inline std::vector<unsigned int> haltonDigitPermutation(
  unsigned int base, std::uint64_t stream_seed, std::size_t dimension)
{
  if (base < 2U) {
    throw std::invalid_argument("Halton base must be at least two");
  }
  std::vector<unsigned int> permutation(base, 0U);
  for (unsigned int digit = 0U; digit < base; ++digit) {
    permutation[digit] = digit;
  }
  if (stream_seed == 0U) {
    return permutation;
  }
  std::uint64_t state = splitMix64(
    stream_seed ^ splitMix64(static_cast<std::uint64_t>(dimension) + 1U) ^
    static_cast<std::uint64_t>(base));
  for (unsigned int i = base - 1U; i > 1U; --i) {
    state = splitMix64(state);
    const unsigned int j = 1U + static_cast<unsigned int>(state % i);
    std::swap(permutation[i], permutation[j]);
  }
  return permutation;
}

inline double haltonRadicalInverse(
  std::uint64_t index, unsigned int base,
  const std::vector<unsigned int> & digit_permutation)
{
  if (base < 2U || digit_permutation.size() != base || digit_permutation[0] != 0U) {
    throw std::invalid_argument("invalid Halton digit permutation");
  }
  double result = 0.0;
  double fraction = 1.0 / static_cast<double>(base);
  while (index > 0U) {
    const unsigned int digit = static_cast<unsigned int>(index % base);
    result += fraction * static_cast<double>(digit_permutation[digit]);
    index /= base;
    fraction /= static_cast<double>(base);
  }
  return result;
}

struct Point3
{
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
};

struct Quaternion
{
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
  double w = 1.0;
};

struct Node
{
  std::uint32_t id = 0;
  Point3 position;
  Quaternion orientation;
  std::vector<double> joints;
};

struct Edge
{
  std::size_t a = 0;
  std::size_t b = 0;
  double cost = 0.0;
};

struct Segment
{
  Point3 a;
  Point3 b;
};

struct Capsule
{
  Segment axis;
  double radius = 0.0;
};

struct GngParameters
{
  std::size_t max_units = 100;
  std::size_t insertion_interval = 20;
  int max_edge_age = 50;
  double winner_learning_rate = 0.05;
  double neighbor_learning_rate = 0.0006;
  double error_reduction = 0.5;
  double error_decay = 0.995;
  std::size_t max_epochs = 8;
};

struct Target
{
  std::uint32_t environment_node_id = 0;
  Point3 position;
};

struct PlanResult
{
  bool has_start = false;
  bool has_intersection = false;
  bool success = false;
  std::size_t start = kInvalidIndex;
  std::size_t goal = kInvalidIndex;
  std::size_t target_index = kInvalidIndex;
  double target_distance = std::numeric_limits<double>::infinity();
  double graph_cost = std::numeric_limits<double>::infinity();
  std::vector<std::size_t> path;
};

inline Point3 operator+(const Point3 & a, const Point3 & b)
{
  return {a.x + b.x, a.y + b.y, a.z + b.z};
}

inline Point3 operator-(const Point3 & a, const Point3 & b)
{
  return {a.x - b.x, a.y - b.y, a.z - b.z};
}

inline Point3 operator*(const Point3 & a, double scale)
{
  return {a.x * scale, a.y * scale, a.z * scale};
}

inline double dot(const Point3 & a, const Point3 & b)
{
  return a.x * b.x + a.y * b.y + a.z * b.z;
}

inline double squaredDistance(const Point3 & a, const Point3 & b)
{
  const Point3 d = a - b;
  return dot(d, d);
}

inline double distance(const Point3 & a, const Point3 & b)
{
  return std::sqrt(squaredDistance(a, b));
}

inline double pointSegmentDistance(const Point3 & p, const Segment & segment)
{
  const Point3 ab = segment.b - segment.a;
  const double length_squared = dot(ab, ab);
  if (length_squared <= 1.0e-16) {
    return distance(p, segment.a);
  }
  const double t = std::clamp(dot(p - segment.a, ab) / length_squared, 0.0, 1.0);
  return distance(p, segment.a + ab * t);
}

// Closest distance between two finite 3-D segments. This is the robust
// degenerate-segment form from Real-Time Collision Detection, section 5.1.9.
inline double segmentSegmentDistance(const Segment & first, const Segment & second)
{
  constexpr double epsilon = 1.0e-16;
  const Point3 d1 = first.b - first.a;
  const Point3 d2 = second.b - second.a;
  const Point3 r = first.a - second.a;
  const double a = dot(d1, d1);
  const double e = dot(d2, d2);
  const double f = dot(d2, r);

  double s = 0.0;
  double t = 0.0;
  if (a <= epsilon && e <= epsilon) {
    return distance(first.a, second.a);
  }
  if (a <= epsilon) {
    t = std::clamp(f / e, 0.0, 1.0);
  } else {
    const double c = dot(d1, r);
    if (e <= epsilon) {
      s = std::clamp(-c / a, 0.0, 1.0);
    } else {
      const double b = dot(d1, d2);
      const double denominator = a * e - b * b;
      if (std::abs(denominator) > epsilon) {
        s = std::clamp((b * f - c * e) / denominator, 0.0, 1.0);
      }
      t = (b * s + f) / e;
      if (t < 0.0) {
        t = 0.0;
        s = std::clamp(-c / a, 0.0, 1.0);
      } else if (t > 1.0) {
        t = 1.0;
        s = std::clamp((b - c) / a, 0.0, 1.0);
      }
    }
  }
  const Point3 closest_first = first.a + d1 * s;
  const Point3 closest_second = second.a + d2 * t;
  return distance(closest_first, closest_second);
}

inline double normalizedJointDistance(
  const std::vector<double> & a,
  const std::vector<double> & b,
  const std::vector<double> & ranges)
{
  if (a.size() != b.size() || a.size() != ranges.size()) {
    return std::numeric_limits<double>::infinity();
  }
  double sum = 0.0;
  for (std::size_t i = 0; i < a.size(); ++i) {
    const double range = std::max(std::abs(ranges[i]), 1.0e-12);
    const double scaled = (a[i] - b[i]) / range;
    sum += scaled * scaled;
  }
  return std::sqrt(sum);
}

inline double jointPathCost(const std::vector<double> & a, const std::vector<double> & b)
{
  if (a.size() != b.size()) {
    return std::numeric_limits<double>::infinity();
  }
  double sum = 0.0;
  for (std::size_t i = 0; i < a.size(); ++i) {
    const double delta = a[i] - b[i];
    sum += delta * delta;
  }
  return std::sqrt(sum);
}

inline double squaredVectorDistance(
  const std::vector<double> & a, const std::vector<double> & b)
{
  if (a.size() != b.size()) {
    return std::numeric_limits<double>::infinity();
  }
  double result = 0.0;
  for (std::size_t i = 0; i < a.size(); ++i) {
    const double delta = a[i] - b[i];
    result += delta * delta;
  }
  return result;
}

// Select deterministic, evenly distributed representatives from an ordered
// sample stream. Mid-stratum indices avoid over-representing either end of the
// stream and remain unique whenever requested_count <= sample_count.
inline std::vector<std::size_t> stratifiedGuardIndices(
  std::size_t sample_count, std::size_t requested_count)
{
  const std::size_t count = std::min(sample_count, requested_count);
  std::vector<std::size_t> indices;
  indices.reserve(count);
  if (count == 0U) {
    return indices;
  }
  for (std::size_t i = 0U; i < count; ++i) {
    const long double numerator =
      (2.0L * static_cast<long double>(i) + 1.0L) *
      static_cast<long double>(sample_count);
    const long double denominator = 2.0L * static_cast<long double>(count);
    indices.push_back(std::min(
      sample_count - 1U, static_cast<std::size_t>(numerator / denominator)));
  }
  return indices;
}

struct GngNodeBudget
{
  std::size_t prototype_count = 0U;
  std::size_t guard_count = 0U;
};

// Allocate only the slots that remain after fixed start/home/measured anchors.
// Guarded GNG keeps at least two prototype slots because GNG is initialized
// with two units. Pure GNG receives every remaining slot as a prototype.
inline GngNodeBudget allocateGngNodeBudget(
  std::size_t total_node_count,
  std::size_t anchor_node_count,
  double guard_fraction,
  bool reserve_guard_budget)
{
  if (!std::isfinite(guard_fraction) || guard_fraction < 0.0 || guard_fraction > 1.0) {
    throw std::invalid_argument("GNG guard fraction must be within [0, 1]");
  }
  const std::size_t remaining = total_node_count > anchor_node_count ?
    total_node_count - anchor_node_count : 0U;
  if (!reserve_guard_budget || remaining <= 2U) {
    return {remaining, 0U};
  }
  const std::size_t requested_guard_count = static_cast<std::size_t>(std::llround(
      guard_fraction * static_cast<double>(remaining)));
  const std::size_t guard_count = std::min(requested_guard_count, remaining - 2U);
  return {remaining - guard_count, guard_count};
}

// Deterministic Growing Neural Gas over a caller-provided sample stream.
// The caller is responsible for feature normalization. A dense edge-age
// matrix keeps the implementation compact and deterministic for roadmap-sized
// networks (hundreds rather than millions of units).
inline std::vector<std::vector<double>> growingNeuralGas(
  const std::vector<std::vector<double>> & samples,
  const GngParameters & parameters)
{
  if (samples.size() < 2U || parameters.max_units < 2U ||
    parameters.insertion_interval == 0U || parameters.max_edge_age < 1 ||
    parameters.max_epochs == 0U || parameters.winner_learning_rate <= 0.0 ||
    parameters.neighbor_learning_rate < 0.0 || parameters.error_reduction <= 0.0 ||
    parameters.error_reduction >= 1.0 || parameters.error_decay <= 0.0 ||
    parameters.error_decay > 1.0)
  {
    throw std::invalid_argument("invalid Growing Neural Gas parameters");
  }
  const std::size_t dimensions = samples.front().size();
  if (dimensions == 0U) {
    throw std::invalid_argument("Growing Neural Gas samples must not be empty");
  }
  for (const auto & sample : samples) {
    if (sample.size() != dimensions ||
      !std::all_of(sample.begin(), sample.end(), [](double value) {return std::isfinite(value);}))
    {
      throw std::invalid_argument("Growing Neural Gas samples have inconsistent dimensions");
    }
  }

  std::size_t second_seed = 1U;
  double farthest = squaredVectorDistance(samples.front(), samples[second_seed]);
  for (std::size_t i = 2U; i < samples.size(); ++i) {
    const double candidate = squaredVectorDistance(samples.front(), samples[i]);
    if (candidate > farthest) {
      farthest = candidate;
      second_seed = i;
    }
  }

  std::vector<std::vector<double>> units{samples.front(), samples[second_seed]};
  std::vector<double> errors(2U, 0.0);
  std::vector<std::vector<int>> ages(2U, std::vector<int>(2U, -1));
  ages[0][1] = 0;
  ages[1][0] = 0;
  std::size_t update_count = 0U;

  for (std::size_t epoch = 0U; epoch < parameters.max_epochs; ++epoch) {
    for (const auto & sample : samples) {
      std::size_t winner = 0U;
      std::size_t runner_up = 1U;
      double winner_distance = squaredVectorDistance(sample, units[winner]);
      double runner_distance = squaredVectorDistance(sample, units[runner_up]);
      if (runner_distance < winner_distance) {
        std::swap(winner, runner_up);
        std::swap(winner_distance, runner_distance);
      }
      for (std::size_t i = 2U; i < units.size(); ++i) {
        const double candidate = squaredVectorDistance(sample, units[i]);
        if (candidate < winner_distance) {
          runner_up = winner;
          runner_distance = winner_distance;
          winner = i;
          winner_distance = candidate;
        } else if (candidate < runner_distance) {
          runner_up = i;
          runner_distance = candidate;
        }
      }

      errors[winner] += winner_distance;
      for (std::size_t neighbor = 0U; neighbor < units.size(); ++neighbor) {
        if (ages[winner][neighbor] >= 0) {
          ++ages[winner][neighbor];
          ages[neighbor][winner] = ages[winner][neighbor];
        }
      }
      for (std::size_t dimension = 0U; dimension < dimensions; ++dimension) {
        units[winner][dimension] += parameters.winner_learning_rate *
          (sample[dimension] - units[winner][dimension]);
      }
      for (std::size_t neighbor = 0U; neighbor < units.size(); ++neighbor) {
        if (neighbor == winner || ages[winner][neighbor] < 0) {
          continue;
        }
        for (std::size_t dimension = 0U; dimension < dimensions; ++dimension) {
          units[neighbor][dimension] += parameters.neighbor_learning_rate *
            (sample[dimension] - units[neighbor][dimension]);
        }
      }
      ages[winner][runner_up] = 0;
      ages[runner_up][winner] = 0;
      for (std::size_t neighbor = 0U; neighbor < units.size(); ++neighbor) {
        if (ages[winner][neighbor] > parameters.max_edge_age) {
          ages[winner][neighbor] = -1;
          ages[neighbor][winner] = -1;
        }
      }

      ++update_count;
      if (units.size() < parameters.max_units &&
        update_count % parameters.insertion_interval == 0U)
      {
        const std::size_t high_error = static_cast<std::size_t>(
          std::distance(errors.begin(), std::max_element(errors.begin(), errors.end())));
        std::size_t high_error_neighbor = kInvalidIndex;
        for (std::size_t neighbor = 0U; neighbor < units.size(); ++neighbor) {
          if (ages[high_error][neighbor] >= 0 &&
            (high_error_neighbor == kInvalidIndex || errors[neighbor] > errors[high_error_neighbor]))
          {
            high_error_neighbor = neighbor;
          }
        }
        if (high_error_neighbor == kInvalidIndex) {
          double maximum_distance = -1.0;
          for (std::size_t candidate = 0U; candidate < units.size(); ++candidate) {
            if (candidate == high_error) {
              continue;
            }
            const double distance = squaredVectorDistance(units[high_error], units[candidate]);
            if (distance > maximum_distance) {
              maximum_distance = distance;
              high_error_neighbor = candidate;
            }
          }
        }

        std::vector<double> inserted(dimensions, 0.0);
        for (std::size_t dimension = 0U; dimension < dimensions; ++dimension) {
          inserted[dimension] = 0.5 *
            (units[high_error][dimension] + units[high_error_neighbor][dimension]);
        }
        for (auto & row : ages) {
          row.push_back(-1);
        }
        ages.emplace_back(units.size() + 1U, -1);
        const std::size_t inserted_index = units.size();
        units.push_back(std::move(inserted));
        errors[high_error] *= parameters.error_reduction;
        errors[high_error_neighbor] *= parameters.error_reduction;
        errors.push_back(errors[high_error]);
        ages[high_error][high_error_neighbor] = -1;
        ages[high_error_neighbor][high_error] = -1;
        ages[high_error][inserted_index] = 0;
        ages[inserted_index][high_error] = 0;
        ages[high_error_neighbor][inserted_index] = 0;
        ages[inserted_index][high_error_neighbor] = 0;
      }
      for (double & error : errors) {
        error *= parameters.error_decay;
      }
    }
  }
  return units;
}

inline bool capsuleIntersectsEnvironment(
  const Capsule & capsule,
  const std::vector<Point3> & obstacle_points,
  const std::vector<Segment> & obstacle_segments,
  double clearance)
{
  const double threshold = std::max(0.0, capsule.radius + clearance);
  for (const Point3 & obstacle : obstacle_points) {
    if (pointSegmentDistance(obstacle, capsule.axis) < threshold) {
      return true;
    }
  }
  for (const Segment & obstacle : obstacle_segments) {
    if (segmentSegmentDistance(capsule.axis, obstacle) < threshold) {
      return true;
    }
  }
  return false;
}

inline std::vector<std::size_t> rankNodesByJointDistance(
  const std::vector<Node> & nodes,
  const std::vector<double> & joints,
  const std::vector<double> & ranges,
  const std::vector<bool> & blocked = {})
{
  std::vector<std::pair<double, std::size_t>> ranked;
  ranked.reserve(nodes.size());
  for (std::size_t i = 0; i < nodes.size(); ++i) {
    if (i < blocked.size() && blocked[i]) {
      continue;
    }
    const double d = normalizedJointDistance(nodes[i].joints, joints, ranges);
    if (std::isfinite(d)) {
      ranked.emplace_back(d, i);
    }
  }
  std::sort(ranked.begin(), ranked.end(), [](const auto & lhs, const auto & rhs) {
    if (lhs.first != rhs.first) {
      return lhs.first < rhs.first;
    }
    return lhs.second < rhs.second;
  });
  std::vector<std::size_t> result;
  result.reserve(ranked.size());
  for (const auto & entry : ranked) {
    result.push_back(entry.second);
  }
  return result;
}

inline std::vector<bool> targetIntersectionMask(
  const std::vector<Node> & nodes,
  const std::vector<Target> & targets,
  double radius)
{
  std::vector<bool> mask(nodes.size(), false);
  const double radius_squared = radius * radius;
  for (std::size_t i = 0; i < nodes.size(); ++i) {
    for (const Target & target : targets) {
      if (squaredDistance(nodes[i].position, target.position) <= radius_squared) {
        mask[i] = true;
        break;
      }
    }
  }
  return mask;
}

inline std::vector<bool> blockedNodes(
  const std::vector<Node> & nodes,
  const std::vector<Point3> & obstacle_points,
  const std::vector<Segment> & obstacle_segments,
  double clearance)
{
  std::vector<bool> result(nodes.size(), false);
  const double clearance_squared = clearance * clearance;
  for (std::size_t i = 0; i < nodes.size(); ++i) {
    for (const Point3 & obstacle : obstacle_points) {
      if (squaredDistance(nodes[i].position, obstacle) < clearance_squared) {
        result[i] = true;
        break;
      }
    }
    if (result[i]) {
      continue;
    }
    for (const Segment & obstacle : obstacle_segments) {
      if (pointSegmentDistance(nodes[i].position, obstacle) < clearance) {
        result[i] = true;
        break;
      }
    }
  }
  return result;
}

inline std::vector<bool> blockedEdges(
  const std::vector<Node> & nodes,
  const std::vector<Edge> & edges,
  const std::vector<bool> & blocked_nodes,
  const std::vector<Point3> & obstacle_points,
  const std::vector<Segment> & obstacle_segments,
  double clearance)
{
  std::vector<bool> result(edges.size(), false);
  for (std::size_t edge_index = 0; edge_index < edges.size(); ++edge_index) {
    const Edge & edge = edges[edge_index];
    if (edge.a >= nodes.size() || edge.b >= nodes.size() ||
      (edge.a < blocked_nodes.size() && blocked_nodes[edge.a]) ||
      (edge.b < blocked_nodes.size() && blocked_nodes[edge.b]))
    {
      result[edge_index] = true;
      continue;
    }
    const Segment motion{nodes[edge.a].position, nodes[edge.b].position};
    for (const Point3 & obstacle : obstacle_points) {
      if (pointSegmentDistance(obstacle, motion) < clearance) {
        result[edge_index] = true;
        break;
      }
    }
    if (result[edge_index]) {
      continue;
    }
    for (const Segment & obstacle : obstacle_segments) {
      if (segmentSegmentDistance(motion, obstacle) < clearance) {
        result[edge_index] = true;
        break;
      }
    }
  }
  return result;
}

inline PlanResult planToNearestTarget(
  const std::vector<Node> & nodes,
  const std::vector<Edge> & edges,
  const std::vector<bool> & blocked_nodes,
  const std::vector<bool> & blocked_edges,
  std::size_t start,
  const std::vector<Target> & targets,
  double intersection_radius)
{
  PlanResult result;
  result.start = start;
  result.has_start = start < nodes.size();
  if (!result.has_start || (start < blocked_nodes.size() && blocked_nodes[start])) {
    return result;
  }

  using QueueEntry = std::pair<double, std::size_t>;
  std::vector<std::vector<std::pair<std::size_t, double>>> adjacency(nodes.size());
  for (std::size_t edge_index = 0; edge_index < edges.size(); ++edge_index) {
    if (edge_index < blocked_edges.size() && blocked_edges[edge_index]) {
      continue;
    }
    const Edge & edge = edges[edge_index];
    if (edge.a >= nodes.size() || edge.b >= nodes.size() || !std::isfinite(edge.cost) || edge.cost <= 0.0) {
      continue;
    }
    adjacency[edge.a].emplace_back(edge.b, edge.cost);
    adjacency[edge.b].emplace_back(edge.a, edge.cost);
  }

  std::vector<double> cost(nodes.size(), std::numeric_limits<double>::infinity());
  std::vector<std::size_t> predecessor(nodes.size(), kInvalidIndex);
  std::priority_queue<QueueEntry, std::vector<QueueEntry>, std::greater<QueueEntry>> queue;
  cost[start] = 0.0;
  queue.emplace(0.0, start);
  while (!queue.empty()) {
    const auto [current_cost, current] = queue.top();
    queue.pop();
    if (current_cost > cost[current]) {
      continue;
    }
    for (const auto & [next, edge_cost] : adjacency[current]) {
      if (next < blocked_nodes.size() && blocked_nodes[next]) {
        continue;
      }
      const double candidate = current_cost + edge_cost;
      if (candidate + 1.0e-12 < cost[next]) {
        cost[next] = candidate;
        predecessor[next] = current;
        queue.emplace(candidate, next);
      }
    }
  }

  const double radius_squared = intersection_radius * intersection_radius;
  for (std::size_t target_index = 0; target_index < targets.size(); ++target_index) {
    for (std::size_t node_index = 0; node_index < nodes.size(); ++node_index) {
      const double d_squared = squaredDistance(nodes[node_index].position, targets[target_index].position);
      if (d_squared > radius_squared) {
        continue;
      }
      result.has_intersection = true;
      if (!std::isfinite(cost[node_index]) ||
        (node_index < blocked_nodes.size() && blocked_nodes[node_index]))
      {
        continue;
      }
      const double target_distance = std::sqrt(d_squared);
      const bool better_target = target_distance + 1.0e-12 < result.target_distance;
      const bool equal_target_shorter_path =
        std::abs(target_distance - result.target_distance) <= 1.0e-12 &&
        cost[node_index] < result.graph_cost;
      if (better_target || equal_target_shorter_path) {
        result.goal = node_index;
        result.target_index = target_index;
        result.target_distance = target_distance;
        result.graph_cost = cost[node_index];
      }
    }
  }

  if (result.goal == kInvalidIndex) {
    return result;
  }
  result.success = true;
  for (std::size_t cursor = result.goal; cursor != kInvalidIndex; cursor = predecessor[cursor]) {
    result.path.push_back(cursor);
    if (cursor == start) {
      break;
    }
  }
  if (result.path.empty() || result.path.back() != start) {
    result.success = false;
    result.path.clear();
    return result;
  }
  std::reverse(result.path.begin(), result.path.end());
  return result;
}

}  // namespace om6dof_dd_gng::reachability
