#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <functional>
#include <limits>
#include <map>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "om6dof_dd_gng/dynamic_density_gng.hpp"

namespace density = om6dof_dd_gng;

namespace
{

struct GraphSnapshot
{
  std::vector<GngPoint3f> nodes;
  std::vector<std::uint32_t> ids;
  std::vector<std::pair<std::uint16_t, std::uint16_t>> edges;
  std::vector<float> strengths;
};

GraphSnapshot snapshot(const density::DynamicDensityGrowingNeuralGas & graph)
{
  GraphSnapshot result;
  graph.copyGraph(result.nodes, result.ids, result.edges);
  graph.copyNodeStrengths(result.strengths);
  return result;
}

// Equal-area, equally sampled planar patches. Neither patch obtains extra input
// points: only the explicitly configured attention rule may change its budget.
std::vector<GngPoint3f> equalPlanarPatches()
{
  std::vector<GngPoint3f> points;
  for (int y = -10; y <= 10; ++y) {
    for (int x = -10; x <= 10; ++x) {
      const float dx = static_cast<float>(x) * 0.025F;
      const float dy = static_cast<float>(y) * 0.025F;
      points.push_back({-1.0F + dx, dy, 0.0F});
      points.push_back({1.0F + dx, dy, 0.0F});
    }
  }
  return points;
}

double squaredDistance(const GngPoint3f & a, const GngPoint3f & b)
{
  const double dx = static_cast<double>(a.x) - static_cast<double>(b.x);
  const double dy = static_cast<double>(a.y) - static_cast<double>(b.y);
  const double dz = static_cast<double>(a.z) - static_cast<double>(b.z);
  return dx * dx + dy * dy + dz * dz;
}

void expectGraphInvariants(const GraphSnapshot & graph, std::size_t capacity)
{
  ASSERT_LE(graph.nodes.size(), capacity);
  ASSERT_EQ(graph.ids.size(), graph.nodes.size());
  ASSERT_EQ(graph.strengths.size(), graph.nodes.size());
  std::set<std::uint32_t> ids;
  for (std::size_t i = 0; i < graph.nodes.size(); ++i) {
    EXPECT_TRUE(std::isfinite(graph.nodes[i].x));
    EXPECT_TRUE(std::isfinite(graph.nodes[i].y));
    EXPECT_TRUE(std::isfinite(graph.nodes[i].z));
    EXPECT_GT(graph.ids[i], 0U);
    EXPECT_TRUE(ids.insert(graph.ids[i]).second) << "duplicate stable node ID";
    EXPECT_TRUE(std::isfinite(graph.strengths[i]));
    EXPECT_GE(graph.strengths[i], 1.0F);
    EXPECT_LE(graph.strengths[i], 20.0F);
  }
  std::set<std::pair<std::uint16_t, std::uint16_t>> undirected_edges;
  for (const auto & edge : graph.edges) {
    EXPECT_LT(static_cast<std::size_t>(edge.first), graph.nodes.size());
    EXPECT_LT(static_cast<std::size_t>(edge.second), graph.nodes.size());
    EXPECT_NE(edge.first, edge.second);
    const auto canonical = std::minmax(edge.first, edge.second);
    EXPECT_TRUE(undirected_edges.emplace(canonical.first, canonical.second).second)
      << "duplicate undirected edge";
  }
}

void expectIdentical(const GraphSnapshot & first, const GraphSnapshot & second)
{
  ASSERT_EQ(first.nodes.size(), second.nodes.size());
  EXPECT_EQ(first.ids, second.ids);
  EXPECT_EQ(first.edges, second.edges);
  EXPECT_EQ(first.strengths, second.strengths);
  for (std::size_t i = 0; i < first.nodes.size(); ++i) {
    EXPECT_EQ(first.nodes[i].x, second.nodes[i].x);
    EXPECT_EQ(first.nodes[i].y, second.nodes[i].y);
    EXPECT_EQ(first.nodes[i].z, second.nodes[i].z);
  }
}

double patchFraction(const GraphSnapshot & graph, bool left)
{
  if (graph.nodes.empty()) {
    return 0.0;
  }
  const GngPoint3f center{left ? -1.0F : 1.0F, 0.0F, 0.0F};
  const auto count = std::count_if(
    graph.nodes.begin(), graph.nodes.end(), [&center](const GngPoint3f & node) {
      return squaredDistance(node, center) <= 0.16;  // 0.4 m radius.
    });
  return static_cast<double>(count) / static_cast<double>(graph.nodes.size());
}

}  // namespace

TEST(DynamicDensityGng, MaintainsFiniteBoundedUndirectedGraphDuringLongUpdates)
{
  const auto points = equalPlanarPatches();
  for (const int capacity : {2, 32, 64}) {
    SCOPED_TRACE("capacity=" + std::to_string(capacity));
    density::DynamicDensityGngParameters parameters;
    parameters.max_nodes = capacity;
    parameters.insertion_interval = 3;
    parameters.max_edge_age = 7;
    parameters.random_seed = 121;
    density::DynamicDensityGrowingNeuralGas graph(parameters);
    graph.setAttentionRegions({{{-1.0F, 0.0F, 0.0F}, 0.4F, 20.0F}});
    for (int batch = 0; batch < 180; ++batch) {
      if (batch == 90) {
        graph.setAttentionRegions({{{1.0F, 0.0F, 0.0F}, 0.4F, 20.0F}});
      }
      graph.partialFit(points, 113);
      const auto current = snapshot(graph);
      expectGraphInvariants(current, static_cast<std::size_t>(capacity));
      EXPECT_GE(current.nodes.size(), 2U);
    }
    if (capacity == 2) {
      EXPECT_EQ(snapshot(graph).nodes.size(), 2U);
    } else {
      EXPECT_GT(graph.stats().recycled_nodes, 0U);
    }
  }
}

TEST(DynamicDensityGng, StableIdsSurviveCompactionAndRetiredIdsAreNotReused)
{
  density::DynamicDensityGngParameters parameters;
  parameters.max_nodes = 8;
  parameters.insertion_interval = 2;
  parameters.max_edge_age = 32766;
  // With motion disabled, an existing ID must retain its coordinates even when
  // the array slot changes. This checks identity, not merely ID uniqueness.
  parameters.winner_learning_rate = 0.0F;
  parameters.neighbor_learning_rate = 0.0F;
  density::DynamicDensityGrowingNeuralGas graph(parameters);
  const auto points = equalPlanarPatches();
  std::map<std::uint32_t, GngPoint3f> previous;
  std::set<std::uint32_t> retired;
  std::uint32_t maximum_issued = 0;
  for (int step = 0; step < 1400; ++step) {
    graph.partialFit(points, 1);
    const auto current = snapshot(graph);
    expectGraphInvariants(current, 8U);
    std::map<std::uint32_t, GngPoint3f> next;
    for (std::size_t i = 0; i < current.ids.size(); ++i) {
      const auto id = current.ids[i];
      EXPECT_EQ(retired.count(id), 0U) << "a retired semantic identity reappeared";
      const auto old = previous.find(id);
      if (old != previous.end()) {
        EXPECT_EQ(current.nodes[i].x, old->second.x);
        EXPECT_EQ(current.nodes[i].y, old->second.y);
        EXPECT_EQ(current.nodes[i].z, old->second.z);
      } else {
        EXPECT_GT(id, maximum_issued) << "new nodes must use fresh monotonic IDs";
      }
      next.emplace(id, current.nodes[i]);
    }
    for (const auto & entry : previous) {
      if (next.count(entry.first) == 0U) {
        retired.insert(entry.first);
      }
    }
    if (!current.ids.empty()) {
      maximum_issued = std::max(
        maximum_issued, *std::max_element(current.ids.begin(), current.ids.end()));
    }
    previous = std::move(next);
  }
  EXPECT_GT(retired.size(), 8U);
  EXPECT_GT(maximum_issued, 8U);
  EXPECT_GT(graph.stats().recycled_nodes, 0U);
}

TEST(DynamicDensityGng, SeedAndResetReproduceGraphAndSamplingCounters)
{
  density::DynamicDensityGngParameters parameters;
  parameters.max_nodes = 32;
  parameters.insertion_interval = 7;
  parameters.random_seed = 807;
  parameters.attention_sample_fraction = 0.37F;
  const std::vector<density::DensityAttentionRegion> regions{
    {{-1.0F, 0.0F, 0.0F}, 0.4F, 10.0F}};
  density::DynamicDensityGrowingNeuralGas first(parameters);
  density::DynamicDensityGrowingNeuralGas second(parameters);
  first.setAttentionRegions(regions);
  second.setAttentionRegions(regions);
  const auto points = equalPlanarPatches();
  for (const int updates : {17, 101, 509, 1703}) {
    first.partialFit(points, updates);
    second.partialFit(points, updates);
    expectIdentical(snapshot(first), snapshot(second));
  }
  const auto expected = snapshot(first);
  const auto expected_stats = first.stats();
  first.reset();
  EXPECT_TRUE(snapshot(first).nodes.empty());
  EXPECT_EQ(first.stats().active_attention_regions, 0U);
  EXPECT_EQ(first.stats().focused_nodes, 0U);
  EXPECT_EQ(first.stats().focused_samples, 0U);
  EXPECT_EQ(first.stats().uniform_samples, 0U);
  EXPECT_EQ(first.stats().insertions, 0U);
  EXPECT_EQ(first.stats().recycled_nodes, 0U);
  first.setAttentionRegions(regions);
  for (const int updates : {17, 101, 509, 1703}) {
    first.partialFit(points, updates);
  }
  expectIdentical(snapshot(first), expected);
  EXPECT_EQ(first.stats().focused_samples, expected_stats.focused_samples);
  EXPECT_EQ(first.stats().uniform_samples, expected_stats.uniform_samples);
  EXPECT_EQ(first.stats().insertions, expected_stats.insertions);
  EXPECT_EQ(first.stats().recycled_nodes, expected_stats.recycled_nodes);
}

TEST(DynamicDensityGng, IgnoresNonfinitePointsAndNonpositiveUpdateCountsSafely)
{
  density::DynamicDensityGngParameters parameters;
  parameters.max_nodes = 32;
  density::DynamicDensityGrowingNeuralGas graph(parameters);
  const float nan = std::numeric_limits<float>::quiet_NaN();
  const float inf = std::numeric_limits<float>::infinity();
  const std::vector<GngPoint3f> invalid{{nan, 0.0F, 0.0F}, {0.0F, inf, 0.0F},
    {0.0F, 0.0F, -inf}};
  EXPECT_NO_THROW(graph.partialFit(invalid, 100));
  EXPECT_TRUE(snapshot(graph).nodes.empty());
  EXPECT_NO_THROW(graph.partialFit({}, 100));
  EXPECT_NO_THROW(graph.partialFit({{0.0F, 0.0F, 0.0F}}, 100));
  EXPECT_TRUE(snapshot(graph).nodes.empty());
  auto mixed = equalPlanarPatches();
  mixed.insert(mixed.end(), invalid.begin(), invalid.end());
  EXPECT_NO_THROW(graph.partialFit(mixed, 2000));
  const auto before = snapshot(graph);
  ASSERT_GE(before.nodes.size(), 2U);
  expectGraphInvariants(before, 32U);
  graph.partialFit(mixed, 0);
  graph.partialFit(mixed, -7);
  graph.partialFit(invalid, 100);
  expectIdentical(snapshot(graph), before);
}

TEST(DynamicDensityGng, RejectsInvalidParametersBeforeAllocatingOrTraining)
{
  using Parameters = density::DynamicDensityGngParameters;
  const float nan = std::numeric_limits<float>::quiet_NaN();
  const std::vector<std::function<void(Parameters &)>> invalidators{
    [](Parameters & p) {p.max_nodes = 1;},
    [](Parameters & p) {p.max_nodes = 4097;},
    [](Parameters & p) {p.insertion_interval = 0;},
    [](Parameters & p) {p.max_edge_age = -1;},
    [](Parameters & p) {p.max_edge_age = 32767;},
    [](Parameters & p) {p.winner_learning_rate = -0.01F;},
    [](Parameters & p) {p.winner_learning_rate = 1.01F;},
    [nan](Parameters & p) {p.winner_learning_rate = nan;},
    [](Parameters & p) {p.neighbor_learning_rate = -0.01F;},
    [](Parameters & p) {p.neighbor_learning_rate = 1.01F;},
    [nan](Parameters & p) {p.neighbor_learning_rate = nan;},
    [](Parameters & p) {p.error_decay = -0.01F;},
    [](Parameters & p) {p.error_decay = 1.01F;},
    [nan](Parameters & p) {p.error_decay = nan;},
    [](Parameters & p) {p.utility_decay = -0.01F;},
    [](Parameters & p) {p.utility_decay = 1.01F;},
    [nan](Parameters & p) {p.utility_decay = nan;},
    [](Parameters & p) {p.attention_sample_fraction = -0.01F;},
    [](Parameters & p) {p.attention_sample_fraction = 0.5001F;},
    [nan](Parameters & p) {p.attention_sample_fraction = nan;}};
  for (std::size_t i = 0; i < invalidators.size(); ++i) {
    SCOPED_TRACE("invalid parameter case=" + std::to_string(i));
    Parameters parameters;
    invalidators[i](parameters);
    EXPECT_THROW(density::DynamicDensityGrowingNeuralGas graph(parameters),
      std::invalid_argument);
  }
}

TEST(DynamicDensityGng, FiniteFloatExtremesDoNotOverflowNodeCoordinates)
{
  density::DynamicDensityGngParameters parameters;
  parameters.max_nodes = 16;
  parameters.insertion_interval = 3;
  density::DynamicDensityGrowingNeuralGas graph(parameters);
  const float largest = std::numeric_limits<float>::max();
  const std::vector<GngPoint3f> points{
    {largest, largest, largest}, {-largest, -largest, -largest},
    {largest, -largest, 0.0F}, {-largest, largest, 0.0F}, {0.0F, 0.0F, 0.0F}};
  for (int batch = 0; batch < 20; ++batch) {
    EXPECT_NO_THROW(graph.partialFit(points, 100));
    expectGraphInvariants(snapshot(graph), 16U);
  }
}

TEST(DynamicDensityGng, InvalidRegionReplacementPreservesPreviousAttention)
{
  density::DynamicDensityGrowingNeuralGas graph(density::DynamicDensityGngParameters{});
  graph.partialFit(equalPlanarPatches(), 500);
  graph.setAttentionRegions({{{-1.0F, 0.0F, 0.0F}, 0.4F, 20.0F}});
  const auto before = snapshot(graph);
  const auto before_stats = graph.stats();
  const float nan = std::numeric_limits<float>::quiet_NaN();
  const float inf = std::numeric_limits<float>::infinity();
  const std::vector<density::DensityAttentionRegion> invalid{
    {{nan, 0.0F, 0.0F}, 0.4F, 2.0F},
    {{0.0F, inf, 0.0F}, 0.4F, 2.0F},
    {{0.0F, 0.0F, 0.0F}, 0.0F, 2.0F},
    {{0.0F, 0.0F, 0.0F}, -0.1F, 2.0F},
    {{0.0F, 0.0F, 0.0F}, inf, 2.0F},
    {{0.0F, 0.0F, 0.0F}, 0.4F, 0.9F},
    {{0.0F, 0.0F, 0.0F}, 0.4F, 20.1F},
    {{0.0F, 0.0F, 0.0F}, 0.4F, nan}};
  for (const auto & region : invalid) {
    EXPECT_THROW(graph.setAttentionRegions({region}), std::invalid_argument);
    expectIdentical(snapshot(graph), before);
    EXPECT_EQ(graph.stats().active_attention_regions, before_stats.active_attention_regions);
    EXPECT_EQ(graph.stats().focused_nodes, before_stats.focused_nodes);
  }
  const std::vector<density::DensityAttentionRegion> too_many(
    65, {{0.0F, 0.0F, 0.0F}, 0.4F, 2.0F});
  EXPECT_THROW(graph.setAttentionRegions(too_many), std::invalid_argument);
  expectIdentical(snapshot(graph), before);
}

TEST(DynamicDensityGng, NeutralStrengthIsIdenticalToNoAttention)
{
  density::DynamicDensityGngParameters parameters;
  parameters.max_nodes = 32;
  parameters.insertion_interval = 11;
  parameters.attention_sample_fraction = 0.5F;
  density::DynamicDensityGrowingNeuralGas baseline(parameters);
  density::DynamicDensityGrowingNeuralGas neutral(parameters);
  neutral.setAttentionRegions({{{-1.0F, 0.0F, 0.0F}, 0.4F, 1.0F}});
  const auto points = equalPlanarPatches();
  for (int batch = 0; batch < 20; ++batch) {
    baseline.partialFit(points, 257);
    neutral.partialFit(points, 257);
    expectIdentical(snapshot(baseline), snapshot(neutral));
  }
  EXPECT_EQ(neutral.stats().active_attention_regions, 0U);
  EXPECT_EQ(neutral.stats().focused_nodes, 0U);
  EXPECT_EQ(neutral.stats().focused_samples, 0U);
  EXPECT_EQ(neutral.stats().uniform_samples, 20U * 257U);
}

TEST(DynamicDensityGng, GeometricStrengthIsBoundedLocalAndClearable)
{
  density::DynamicDensityGngParameters parameters;
  parameters.max_nodes = 64;
  parameters.insertion_interval = 10;
  density::DynamicDensityGrowingNeuralGas graph(parameters);
  const auto points = equalPlanarPatches();
  graph.partialFit(points, 3000);
  const std::vector<density::DensityAttentionRegion> regions{
    {{-1.0F, 0.0F, 0.0F}, 0.4F, 3.0F},
    {{-1.0F, 0.0F, 0.0F}, 0.2F, 20.0F}};
  graph.setAttentionRegions(regions);
  for (int batch = 0; batch < 10; ++batch) {
    graph.partialFit(points, 101);
    const auto current = snapshot(graph);
    std::size_t expected_focused = 0;
    for (std::size_t i = 0; i < current.nodes.size(); ++i) {
      float expected_strength = 1.0F;
      for (const auto & region : regions) {
        if (squaredDistance(current.nodes[i], region.center) <=
          static_cast<double>(region.radius) * region.radius)
        {
          expected_strength = std::max(expected_strength, region.strength);
        }
      }
      EXPECT_FLOAT_EQ(current.strengths[i], expected_strength);
      expected_focused += expected_strength > 1.0F ? 1U : 0U;
    }
    EXPECT_EQ(graph.stats().focused_nodes, expected_focused);
  }
  // Clearing is synchronous and changes strengths, not node IDs or coordinates.
  // Expiration/TTL is intentionally a caller policy, not a clock in this core.
  const auto before_clear = snapshot(graph);
  graph.setAttentionRegions({});
  auto expected_cleared = before_clear;
  expected_cleared.strengths.assign(expected_cleared.nodes.size(), 1.0F);
  expectIdentical(snapshot(graph), expected_cleared);
  EXPECT_EQ(graph.stats().active_attention_regions, 0U);
  EXPECT_EQ(graph.stats().focused_nodes, 0U);
  const auto focused_before = graph.stats().focused_samples;
  graph.partialFit(points, 101);
  EXPECT_EQ(graph.stats().focused_samples, focused_before);
}

TEST(DynamicDensityGng, AttentionOutsideObservedGeometryDoesNotInventFocusedSamples)
{
  density::DynamicDensityGngParameters parameters;
  parameters.max_nodes = 32;
  parameters.attention_sample_fraction = 0.5F;
  density::DynamicDensityGrowingNeuralGas baseline(parameters);
  density::DynamicDensityGrowingNeuralGas far_away(parameters);
  far_away.setAttentionRegions({{{100.0F, 100.0F, 100.0F}, 0.1F, 20.0F}});
  baseline.partialFit(equalPlanarPatches(), 3000);
  far_away.partialFit(equalPlanarPatches(), 3000);
  expectIdentical(snapshot(baseline), snapshot(far_away));
  EXPECT_EQ(far_away.stats().focused_nodes, 0U);
  EXPECT_EQ(far_away.stats().focused_samples, 0U);
  EXPECT_EQ(far_away.stats().uniform_samples, 3000U);
}

TEST(DynamicDensityGng, ReservesAtLeastHalfThePerCallBudgetForGlobalSampling)
{
  const auto points = equalPlanarPatches();
  for (const float fraction : {0.0F, 0.37F, 0.5F}) {
    SCOPED_TRACE("attention_sample_fraction=" + std::to_string(fraction));
    density::DynamicDensityGngParameters parameters;
    parameters.max_nodes = 32;
    parameters.attention_sample_fraction = fraction;
    density::DynamicDensityGrowingNeuralGas graph(parameters);
    graph.setAttentionRegions({{{-1.0F, 0.0F, 0.0F}, 0.4F, 20.0F}});
    for (const int updates : {1, 2, 3, 7, 10, 101, 509}) {
      const auto before = graph.stats();
      graph.partialFit(points, updates);
      const auto after = graph.stats();
      const auto focused = after.focused_samples - before.focused_samples;
      const auto uniform = after.uniform_samples - before.uniform_samples;
      const auto expected_focused = static_cast<std::uint64_t>(
        std::floor(static_cast<double>(updates) * static_cast<double>(fraction)));
      EXPECT_EQ(focused + uniform, static_cast<std::uint64_t>(updates));
      EXPECT_EQ(focused, expected_focused);
      EXPECT_GE(uniform, static_cast<std::uint64_t>((updates + 1) / 2));
      // This is a global-draw budget guarantee, NOT a guarantee that half of the
      // nodes or samples fall outside the focused region: global draws may hit it.
    }
  }
}

namespace
{

void runDensitySwitchRegression(float attention_strength, bool use_stress_parameters)
{
  SCOPED_TRACE("attention_strength=" + std::to_string(attention_strength));
  const auto points = equalPlanarPatches();
  constexpr int seed_count = 5;
  constexpr int batches_per_phase = 40;
  constexpr int updates_per_batch = 400;
  double left_gain_sum = 0.0;
  double right_gain_sum = 0.0;
  double switching_gain_sum = 0.0;
  std::ostringstream distribution;
  distribution << "seed,neutral_left,focused_left,neutral_right,focused_right,right_before_switch;";
  for (int seed = 0; seed < seed_count; ++seed) {
    SCOPED_TRACE("seed=" + std::to_string(seed));
    density::DynamicDensityGngParameters parameters;
    parameters.max_nodes = 64;
    if (use_stress_parameters) {
      parameters.insertion_interval = 20;
      parameters.max_edge_age = 500;
    }
    parameters.random_seed = 401 + seed;
    density::DynamicDensityGrowingNeuralGas neutral(parameters);
    density::DynamicDensityGrowingNeuralGas focused(parameters);
    neutral.setAttentionRegions({{{-1.0F, 0.0F, 0.0F}, 0.4F, 1.0F}});
    focused.setAttentionRegions({{{-1.0F, 0.0F, 0.0F}, 0.4F, attention_strength}});
    bool neutral_reached_capacity = false;
    bool focused_reached_capacity = false;
    for (int batch = 0; batch < batches_per_phase; ++batch) {
      neutral.partialFit(points, updates_per_batch);
      focused.partialFit(points, updates_per_batch);
      neutral_reached_capacity |= snapshot(neutral).nodes.size() == 64U;
      focused_reached_capacity |= snapshot(focused).nodes.size() == 64U;
    }
    EXPECT_TRUE(neutral_reached_capacity);
    EXPECT_TRUE(focused_reached_capacity);
    const auto neutral_left_graph = snapshot(neutral);
    const auto focused_left_graph = snapshot(focused);
    const double neutral_left = patchFraction(neutral_left_graph, true);
    const double focused_left = patchFraction(focused_left_graph, true);
    const double right_before_switch = patchFraction(focused_left_graph, false);
    neutral.setAttentionRegions({{{1.0F, 0.0F, 0.0F}, 0.4F, 1.0F}});
    focused.setAttentionRegions({{{1.0F, 0.0F, 0.0F}, 0.4F, attention_strength}});
    for (int batch = 0; batch < batches_per_phase; ++batch) {
      neutral.partialFit(points, updates_per_batch);
      focused.partialFit(points, updates_per_batch);
    }
    const auto neutral_right_graph = snapshot(neutral);
    const auto focused_right_graph = snapshot(focused);
    expectGraphInvariants(neutral_right_graph, 64U);
    expectGraphInvariants(focused_right_graph, 64U);
    const double neutral_right = patchFraction(neutral_right_graph, false);
    const double focused_right = patchFraction(focused_right_graph, false);
    left_gain_sum += focused_left - neutral_left;
    right_gain_sum += focused_right - neutral_right;
    switching_gain_sum += focused_right - right_before_switch;
    distribution << 401 + seed << ',' << neutral_left << ',' << focused_left << ','
                 << neutral_right << ',' << focused_right << ',' << right_before_switch << ';';
    EXPECT_GT(focused.stats().recycled_nodes, 0U);
  }
  // Retain the actual per-seed distribution in gtest XML, not just a pass/fail.
  // This bounded synthetic regression checks an average directional response;
  // it is not a paper result, an optimal-density claim, or full DD-GNG replication.
  ::testing::Test::RecordProperty("attention_strength", std::to_string(attention_strength));
  ::testing::Test::RecordProperty("uses_core_defaults", use_stress_parameters ? "false" : "true");
  ::testing::Test::RecordProperty("max_nodes", "64");
  ::testing::Test::RecordProperty("density_switch_distribution", distribution.str());
  ::testing::Test::RecordProperty(
    "mean_left_fraction_gain", std::to_string(left_gain_sum / seed_count));
  ::testing::Test::RecordProperty(
    "mean_right_fraction_gain", std::to_string(right_gain_sum / seed_count));
  ::testing::Test::RecordProperty(
    "mean_switch_fraction_gain", std::to_string(switching_gain_sum / seed_count));
  EXPECT_GT(left_gain_sum / seed_count, 0.0) << distribution.str();
  EXPECT_GT(right_gain_sum / seed_count, 0.0) << distribution.str();
  EXPECT_GT(switching_gain_sum / seed_count, 0.0) << distribution.str();
}

}  // namespace

TEST(DynamicDensityGng, ReallocatesDensityWhenAttentionSwitchesAfterCapacity)
{
  runDensitySwitchRegression(20.0F, true);
}

TEST(DynamicDensityGng, ReallocatesDensityAtDeployedSemanticStrength)
{
  // The semantic configuration caps S at 3, rather than the core's allowed 20.
  // Exercise that cap with actual default learning/decay rates, insertion
  // interval, edge age and sampling fraction. Only the synthetic graph budget
  // (64 nodes) and explicit reproducibility seeds differ from core defaults.
  runDensitySwitchRegression(3.0F, false);
}
