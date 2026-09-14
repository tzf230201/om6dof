// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <stdexcept>
#include <vector>

#include "om6dof_dd_gng/dynamic_density_gng.hpp"

namespace om6dof_dd_gng
{

struct SemanticDensityAttentionParameters
{
  double max_source_age_sec = 1.0;
  float radius_m = .05F;
  float max_strength = 3.F;
  std::size_t max_regions = 64;
};

// Stores attention from direct, accepted YOLO label matches, not held semantic labels.
// All timestamps are seconds from the same monotonic clock. Sequence zero denotes
// an unknown result. This class is owned by the topology callback, not its worker.
class SemanticDensityAttention
{
public:
  explicit SemanticDensityAttention(SemanticDensityAttentionParameters parameters = {})
  : parameters_(parameters)
  {
    if (!std::isfinite(parameters_.max_source_age_sec) ||
      parameters_.max_source_age_sec <= 0.0 || !std::isfinite(parameters_.radius_m) ||
      parameters_.radius_m <= 0.F || !std::isfinite(parameters_.max_strength) ||
      parameters_.max_strength < 1.F || parameters_.max_strength > 20.F ||
      parameters_.max_regions < 1 || parameters_.max_regions > 64)
    {
      throw std::invalid_argument("Invalid semantic density attention parameters");
    }
  }

  void observe(
    std::uint64_t result_sequence, double source_time_sec, double now_sec,
    bool pose_compatible, const std::vector<GngPoint3f> & nodes,
    const std::vector<float> & observed_scores)
  {
    // Consume a new result even if invalid. A later reuse must not resurrect it.
    const bool newer = result_sequence > last_sequence_;
    const bool older = result_sequence < last_sequence_;
    if (newer) {
      last_sequence_ = result_sequence;
    }
    if (!checkClock(now_sec) || result_sequence == 0 || older || !pose_compatible ||
      nodes.size() != observed_scores.size() || !std::isfinite(source_time_sec) ||
      source_time_sec < 0.0 || source_time_sec > now_sec ||
      now_sec - source_time_sec > parameters_.max_source_age_sec ||
      (have_source_ && source_time_sec < latest_source_time_sec_))
    {
      clear();
      return;
    }
    if (!newer) {
      // In particular, never follow moving nodes or replace the source timestamp
      // while processing a cached YOLO result in subsequent topology callbacks.
      expire(now_sec);
      return;
    }

    have_source_ = true;
    latest_source_time_sec_ = source_time_sec;
    active_source_time_sec_ = source_time_sec;
    clear();

    struct Candidate
    {
      std::size_t index;
      float score;
    };
    std::vector<Candidate> candidates;
    candidates.reserve(nodes.size());
    for (std::size_t i = 0; i < nodes.size(); ++i) {
      if (finitePoint(nodes[i]) && std::isfinite(observed_scores[i]) &&
        observed_scores[i] > 0.F)
      {
        candidates.push_back({i, std::min(observed_scores[i], 1.F)});
      }
    }
    std::sort(candidates.begin(), candidates.end(), [](const Candidate & a, const Candidate & b) {
      if (a.score != b.score) {
        return a.score > b.score;
      }
      return a.index < b.index;
    });

    const double separation = static_cast<double>(parameters_.radius_m) / 2.0;
    const double separation_squared = separation * separation;
    for (const auto & candidate : candidates) {
      const auto & center = nodes[candidate.index];
      const bool overlaps = std::any_of(
        active_regions_.begin(), active_regions_.end(),
        [&center, separation_squared](const DensityAttentionRegion & region) {
          return squaredDistance(center, region.center) <= separation_squared;
        });
      if (overlaps) {
        continue;
      }
      DensityAttentionRegion region;
      region.center = center;
      region.radius = parameters_.radius_m;
      region.strength = 1.F + (parameters_.max_strength - 1.F) * candidate.score;
      active_regions_.push_back(region);
      if (active_regions_.size() == parameters_.max_regions) {
        break;
      }
    }
  }

  std::vector<DensityAttentionRegion> regions(double now_sec)
  {
    if (!checkClock(now_sec)) {
      clear();
    } else {
      expire(now_sec);
    }
    return active_regions_;
  }

private:
  static bool finitePoint(const GngPoint3f & point)
  {
    return std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z);
  }

  static double squaredDistance(const GngPoint3f & a, const GngPoint3f & b)
  {
    const double dx = static_cast<double>(a.x) - static_cast<double>(b.x);
    const double dy = static_cast<double>(a.y) - static_cast<double>(b.y);
    const double dz = static_cast<double>(a.z) - static_cast<double>(b.z);
    return dx * dx + dy * dy + dz * dz;
  }

  bool checkClock(double now_sec)
  {
    if (!std::isfinite(now_sec) || now_sec < 0.0 ||
      (have_clock_ && now_sec < latest_now_sec_))
    {
      return false;
    }
    have_clock_ = true;
    latest_now_sec_ = now_sec;
    return true;
  }

  void expire(double now_sec)
  {
    if (now_sec < active_source_time_sec_ ||
      now_sec - active_source_time_sec_ > parameters_.max_source_age_sec)
    {
      clear();
    }
  }

  void clear()
  {
    active_regions_.clear();
  }

  SemanticDensityAttentionParameters parameters_;
  std::uint64_t last_sequence_ = 0;
  double latest_source_time_sec_ = 0.0;
  double active_source_time_sec_ = 0.0;
  double latest_now_sec_ = 0.0;
  bool have_source_ = false;
  bool have_clock_ = false;
  std::vector<DensityAttentionRegion> active_regions_;
};

}  // namespace om6dof_dd_gng
