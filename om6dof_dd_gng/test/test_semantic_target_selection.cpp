#include <gtest/gtest.h>
#include "om6dof_dd_gng/semantic_target_selection.hpp"

namespace reach = om6dof_dd_gng::reachability;

TEST(SemanticTargetSelection, UsesClusterCenterWithObservedMiddleIdentity)
{
  std::vector<reach::LabeledTarget> nodes{
    {{10, {.1, .0, .0}}, 39}, {{20, {.1, .0, .1}}, 39},
    {{30, {.1, .0, .2}}, 39}, {{31, {.1, .0, .19}}, 39},
    {{32, {.1, .0, .18}}, 39}};
  const auto selected = reach::componentCenterTargets(nodes,
    {{10, 20}, {20, 30}, {30, 31}, {31, 32}});
  ASSERT_EQ(selected.size(), 1U);
  EXPECT_EQ(selected[0].environment_node_id, 20U);
  EXPECT_DOUBLE_EQ(selected[0].position.z, .1);
  EXPECT_DOUBLE_EQ(selected[0].position.x, .1);
}

TEST(SemanticTargetSelection, KeepsDisconnectedObjectsAndDifferentClassesSeparate)
{
  std::vector<reach::LabeledTarget> nodes{
    {{1, {0, 0, 0}}, 39}, {{2, {0, 0, .1}}, 39}, {{3, {0, 0, .2}}, 39},
    {{4, {1, 0, 0}}, 39}, {{5, {1, 0, .1}}, 39}, {{6, {1, 0, .2}}, 39},
    {{7, {0, 0, 1}}, 41}, {{8, {0, 0, 2}}, -1}};
  const auto selected = reach::componentCenterTargets(nodes,
    {{1, 2}, {2, 3}, {4, 5}, {5, 6}, {3, 7}, {7, 8}, {7, 999}});
  ASSERT_EQ(selected.size(), 3U);
  EXPECT_EQ(selected[0].environment_node_id, 2U);
  EXPECT_EQ(selected[1].environment_node_id, 5U);
  EXPECT_EQ(selected[2].environment_node_id, 7U);
}

TEST(SemanticTargetSelection, TiesAreIndependentOfInputOrderAndEmptyIsValid)
{
  std::vector<reach::LabeledTarget> nodes{
    {{8, {0, 0, .2}}, 39}, {{2, {0, 0, 0}}, 39}};
  EXPECT_EQ(reach::componentCenterTargets(nodes, {{8, 2}})[0].environment_node_id, 2U);
  std::reverse(nodes.begin(), nodes.end());
  EXPECT_EQ(reach::componentCenterTargets(nodes, {{8, 2}})[0].environment_node_id, 2U);
  EXPECT_TRUE(reach::componentCenterTargets({}, {}).empty());
}

TEST(SemanticTargetSelection, CenterNeighborhoodKeepsCentralSurfaceAlternatives)
{
  std::vector<reach::LabeledTarget> nodes{
    {{10, {.0, .0, .00}}, 39}, {{20, {.0, .0, .10}}, 39},
    {{30, {.0, .0, .20}}, 39}, {{31, {.03, .0, .10}}, 39},
    {{32, {-.03, .0, .10}}, 39}, {{99, {2.0, .0, .0}}, 41}};
  const auto selected = reach::componentCenterNeighborhoodTargets(nodes,
    {{10, 20}, {20, 30}, {20, 31}, {20, 32}}, 3U);
  ASSERT_EQ(selected.size(), 4U);  // Three bottle alternatives plus class-41 component.
  EXPECT_EQ(selected[0].environment_node_id, 20U);
  EXPECT_EQ(selected[1].environment_node_id, 31U);
  EXPECT_EQ(selected[2].environment_node_id, 32U);
  EXPECT_EQ(selected[3].environment_node_id, 99U);
  // All representatives refer to their cluster centre, not their individual
  // surface locations.
  EXPECT_DOUBLE_EQ(selected[0].position.z, .10);
  EXPECT_DOUBLE_EQ(selected[1].position.z, .10);
  EXPECT_DOUBLE_EQ(selected[2].position.z, .10);
  EXPECT_TRUE(reach::componentCenterNeighborhoodTargets(nodes, {}, 0U).empty());
}
