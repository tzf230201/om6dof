#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <vector>

#include "om6dof_dd_gng/reachability_graph.hpp"

namespace reach = om6dof_dd_gng::reachability;

TEST(ReachabilityGeometry, PointAndSegmentDistancesAreFinite)
{
  const reach::Segment x_axis{{0.0, 0.0, 0.0}, {1.0, 0.0, 0.0}};
  EXPECT_NEAR(reach::pointSegmentDistance({0.5, 0.2, 0.0}, x_axis), 0.2, 1.0e-12);
  EXPECT_NEAR(reach::pointSegmentDistance({-0.5, 0.0, 0.0}, x_axis), 0.5, 1.0e-12);

  const reach::Segment crossing{{0.5, -1.0, 0.0}, {0.5, 1.0, 0.0}};
  const reach::Segment parallel{{0.0, 0.3, 0.0}, {1.0, 0.3, 0.0}};
  EXPECT_NEAR(reach::segmentSegmentDistance(x_axis, crossing), 0.0, 1.0e-12);
  EXPECT_NEAR(reach::segmentSegmentDistance(x_axis, parallel), 0.3, 1.0e-12);
}

TEST(ReachabilityGeometry, CapsuleChecksPointsAndObstacleEdges)
{
  const reach::Capsule capsule{{{0.0, 0.0, 0.0}, {1.0, 0.0, 0.0}}, 0.1};
  EXPECT_TRUE(reach::capsuleIntersectsEnvironment(
    capsule, {{0.5, 0.14, 0.0}}, {}, 0.05));
  EXPECT_TRUE(reach::capsuleIntersectsEnvironment(
    capsule, {}, {{{0.5, -1.0, 0.0}, {0.5, 1.0, 0.0}}}, 0.0));
  EXPECT_FALSE(reach::capsuleIntersectsEnvironment(
    capsule, {{0.5, 0.3, 0.0}}, {}, 0.05));
}

TEST(ReachabilityGng, IsDeterministicFiniteAndHonorsNodeBudget)
{
  std::vector<std::vector<double>> samples;
  for (int i = 0; i < 40; ++i) {
    const double t = static_cast<double>(i) / 39.0;
    samples.push_back({t, 0.25 + 0.5 * t});
  }
  reach::GngParameters parameters;
  parameters.max_units = 12;
  parameters.insertion_interval = 2;
  parameters.max_epochs = 2;
  const auto first = reach::growingNeuralGas(samples, parameters);
  const auto second = reach::growingNeuralGas(samples, parameters);
  ASSERT_EQ(first.size(), 12U);
  ASSERT_EQ(first, second);
  for (const auto & unit : first) {
    ASSERT_EQ(unit.size(), 2U);
    EXPECT_TRUE(std::isfinite(unit[0]));
    EXPECT_TRUE(std::isfinite(unit[1]));
    EXPECT_GE(unit[0], 0.0);
    EXPECT_LE(unit[0], 1.0);
    EXPECT_GE(unit[1], 0.25);
    EXPECT_LE(unit[1], 0.75);
  }
}

TEST(ReachabilityGng, RejectsInconsistentTrainingSamples)
{
  reach::GngParameters parameters;
  EXPECT_THROW(
    reach::growingNeuralGas({{0.0, 1.0}, {0.5}}, parameters),
    std::invalid_argument);
}

TEST(ReachabilityGng, GuardIndicesAreDeterministicStratifiedAndUnique)
{
  const auto first = reach::stratifiedGuardIndices(10U, 4U);
  const auto second = reach::stratifiedGuardIndices(10U, 4U);
  EXPECT_EQ(first, (std::vector<std::size_t>{1U, 3U, 6U, 8U}));
  EXPECT_EQ(first, second);
  EXPECT_TRUE(std::adjacent_find(first.begin(), first.end()) == first.end());
}

TEST(ReachabilityGng, GuardIndicesClampToAvailableSamples)
{
  EXPECT_TRUE(reach::stratifiedGuardIndices(0U, 5U).empty());
  EXPECT_TRUE(reach::stratifiedGuardIndices(5U, 0U).empty());
  EXPECT_EQ(
    reach::stratifiedGuardIndices(5U, 8U),
    (std::vector<std::size_t>{0U, 1U, 2U, 3U, 4U}));
}

TEST(ReachabilityGng, NodeBudgetExcludesFixedAnchors)
{
  const auto pure = reach::allocateGngNodeBudget(800U, 2U, 0.0, false);
  EXPECT_EQ(pure.prototype_count, 798U);
  EXPECT_EQ(pure.guard_count, 0U);

  const auto guarded = reach::allocateGngNodeBudget(800U, 2U, 0.75, true);
  EXPECT_EQ(guarded.prototype_count, 199U);
  EXPECT_EQ(guarded.guard_count, 599U);

  const auto small = reach::allocateGngNodeBudget(4U, 2U, 0.75, true);
  EXPECT_EQ(small.prototype_count, 2U);
  EXPECT_EQ(small.guard_count, 0U);
  EXPECT_THROW(
    reach::allocateGngNodeBudget(800U, 2U, 1.1, true),
    std::invalid_argument);
}

TEST(ReachabilitySampling, DigitScrambledHaltonIsDeterministicAndBijective)
{
  const auto legacy = reach::haltonDigitPermutation(5U, 0U, 0U);
  EXPECT_EQ(legacy, (std::vector<unsigned int>{0U, 1U, 2U, 3U, 4U}));
  EXPECT_NEAR(reach::haltonRadicalInverse(1U, 2U, {0U, 1U}), 0.5, 1.0e-12);
  EXPECT_NEAR(reach::haltonRadicalInverse(2U, 2U, {0U, 1U}), 0.25, 1.0e-12);

  const auto first = reach::haltonDigitPermutation(37U, 42U, 3U);
  const auto repeated = reach::haltonDigitPermutation(37U, 42U, 3U);
  const auto other_stream = reach::haltonDigitPermutation(37U, 43U, 3U);
  EXPECT_EQ(first, repeated);
  EXPECT_NE(first, other_stream);
  EXPECT_EQ(first.front(), 0U);
  auto sorted = first;
  std::sort(sorted.begin(), sorted.end());
  for (unsigned int digit = 0U; digit < sorted.size(); ++digit) {
    EXPECT_EQ(sorted[digit], digit);
  }
  const double value = reach::haltonRadicalInverse(12345U, 37U, first);
  EXPECT_GE(value, 0.0);
  EXPECT_LT(value, 1.0);

  EXPECT_THROW(reach::haltonDigitPermutation(1U, 42U, 0U), std::invalid_argument);
  EXPECT_THROW(
    reach::haltonRadicalInverse(1U, 5U, {0U, 1U}),
    std::invalid_argument);
}

TEST(ReachabilityGraph, JointRankingUsesNormalizedJointSpace)
{
  std::vector<reach::Node> nodes(3);
  nodes[0].joints = {0.0, 0.0};
  nodes[1].joints = {0.5, 0.0};
  nodes[2].joints = {0.0, 1.0};
  const std::vector<double> query{0.45, 0.0};
  const std::vector<double> ranges{2.0, 4.0};
  const auto ranked = reach::rankNodesByJointDistance(nodes, query, ranges);
  ASSERT_EQ(ranked.size(), 3U);
  EXPECT_EQ(ranked[0], 1U);
  EXPECT_EQ(ranked[1], 0U);
}

TEST(ReachabilityGraph, ObstacleBlocksNodeAndCrossingEdge)
{
  std::vector<reach::Node> nodes(3);
  nodes[0].position = {0.0, 0.0, 0.0};
  nodes[1].position = {1.0, 0.0, 0.0};
  nodes[2].position = {2.0, 0.0, 0.0};
  const std::vector<reach::Edge> edges{{0, 1, 1.0}, {1, 2, 1.0}};
  const std::vector<reach::Point3> obstacle_points{{0.5, 0.01, 0.0}};
  const std::vector<reach::Segment> obstacle_segments;
  const auto blocked_nodes = reach::blockedNodes(nodes, obstacle_points, obstacle_segments, 0.05);
  const auto blocked_edges = reach::blockedEdges(
    nodes, edges, blocked_nodes, obstacle_points, obstacle_segments, 0.05);
  EXPECT_FALSE(blocked_nodes[0]);
  EXPECT_FALSE(blocked_nodes[1]);
  ASSERT_EQ(blocked_edges.size(), 2U);
  EXPECT_TRUE(blocked_edges[0]);
  EXPECT_FALSE(blocked_edges[1]);
}

TEST(ReachabilityGraph, DijkstraChoosesClosestReachableIntersection)
{
  std::vector<reach::Node> nodes(4);
  nodes[0].id = 10;
  nodes[0].position = {0.0, 0.0, 0.0};
  nodes[1].id = 11;
  nodes[1].position = {1.0, 0.0, 0.0};
  nodes[2].id = 12;
  nodes[2].position = {2.0, 0.0, 0.0};
  nodes[3].id = 13;
  nodes[3].position = {1.0, 1.0, 0.0};
  const std::vector<reach::Edge> edges{{0, 1, 1.0}, {1, 2, 1.0}, {0, 3, 1.5}};
  const std::vector<bool> blocked_nodes(nodes.size(), false);
  const std::vector<bool> blocked_edges(edges.size(), false);
  const std::vector<reach::Target> targets{
    {100, {2.01, 0.0, 0.0}},
    {101, {1.04, 1.0, 0.0}}};

  const auto plan = reach::planToNearestTarget(
    nodes, edges, blocked_nodes, blocked_edges, 0, targets, 0.05);
  ASSERT_TRUE(plan.success);
  EXPECT_EQ(plan.goal, 2U);
  EXPECT_EQ(plan.target_index, 0U);
  EXPECT_NEAR(plan.target_distance, 0.01, 1.0e-12);
  EXPECT_EQ(plan.path, (std::vector<std::size_t>{0, 1, 2}));
}

TEST(ReachabilityGraph, BlockedRouteAndOutOfRangeTargetFailCleanly)
{
  std::vector<reach::Node> nodes(3);
  nodes[0].position = {0.0, 0.0, 0.0};
  nodes[1].position = {1.0, 0.0, 0.0};
  nodes[2].position = {2.0, 0.0, 0.0};
  const std::vector<reach::Edge> edges{{0, 1, 1.0}, {1, 2, 1.0}};
  const std::vector<bool> blocked_nodes(nodes.size(), false);
  const std::vector<bool> one_blocked_edge{false, true};
  const std::vector<reach::Target> near_target{{200, {2.0, 0.0, 0.0}}};
  const auto disconnected = reach::planToNearestTarget(
    nodes, edges, blocked_nodes, one_blocked_edge, 0, near_target, 0.01);
  EXPECT_TRUE(disconnected.has_intersection);
  EXPECT_FALSE(disconnected.success);

  const std::vector<bool> clear_edges(edges.size(), false);
  const std::vector<reach::Target> far_target{{201, {5.0, 5.0, 5.0}}};
  const auto out_of_range = reach::planToNearestTarget(
    nodes, edges, blocked_nodes, clear_edges, 0, far_target, 0.05);
  EXPECT_FALSE(out_of_range.has_intersection);
  EXPECT_FALSE(out_of_range.success);
}

TEST(ReachabilityGraph, IntersectionRadiusIncludesBoundary)
{
  std::vector<reach::Node> nodes(1);
  nodes[0].position = {0.0, 0.0, 0.0};
  const std::vector<reach::Target> targets{{1, {0.05, 0.0, 0.0}}};
  const auto mask = reach::targetIntersectionMask(nodes, targets, 0.05);
  ASSERT_EQ(mask.size(), 1U);
  EXPECT_TRUE(mask[0]);
}
