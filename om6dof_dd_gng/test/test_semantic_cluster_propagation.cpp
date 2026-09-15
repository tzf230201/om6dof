// SPDX-License-Identifier: Apache-2.0
#include <gtest/gtest.h>

#include "om6dof_dd_gng/semantic_cluster_propagation.hpp"

using namespace om6dof_dd_gng;

TEST(SemanticClusterPropagation, FillsUnknownNodeBetweenTwoBottleNodes)
{
  const std::vector<GngPoint3f> nodes{{0.F, 0.F, 0.F}, {.02F, 0.F, 0.F}, {.04F, 0.F, 0.F}};
  const std::vector<std::pair<uint16_t, uint16_t>> edges{{0U, 1U}, {1U, 2U}};
  std::vector<int16_t> classes{39, -1, 39};
  std::vector<float> confidence{.8F, 0.F, .6F};
  EXPECT_EQ(propagateUnknownClusterLabels(nodes, edges, classes, confidence), 1U);
  EXPECT_EQ(classes[1], 39);
  EXPECT_FLOAT_EQ(confidence[1], .7F);
}

TEST(SemanticClusterPropagation, RefusesCompetingClassOrLongEdge)
{
  const std::vector<GngPoint3f> nodes{{0.F, 0.F, 0.F}, {.02F, 0.F, 0.F}, {.04F, 0.F, 0.F}};
  const std::vector<std::pair<uint16_t, uint16_t>> edges{{0U, 1U}, {1U, 2U}};
  std::vector<int16_t> classes{39, -1, 66};
  std::vector<float> confidence{.8F, 0.F, .8F};
  EXPECT_EQ(propagateUnknownClusterLabels(nodes, edges, classes, confidence), 0U);
  EXPECT_EQ(classes[1], -1);
  classes = {39, -1, 39};
  confidence = {.8F, 0.F, .8F};
  SemanticClusterPropagationParameters parameters;
  parameters.max_edge_length_m = .01F;
  EXPECT_EQ(propagateUnknownClusterLabels(nodes, edges, classes, confidence, parameters), 0U);
  EXPECT_EQ(classes[1], -1);
}

TEST(SemanticClusterPropagation, DoesNotCascadeInheritedLabels)
{
  const std::vector<GngPoint3f> nodes{{0.F, 0.F, 0.F}, {.02F, 0.F, 0.F}, {.04F, 0.F, 0.F}, {.06F, 0.F, 0.F}};
  const std::vector<std::pair<uint16_t, uint16_t>> edges{{0U, 1U}, {1U, 2U}, {2U, 3U}};
  std::vector<int16_t> classes{39, -1, -1, 39};
  std::vector<float> confidence{.8F, 0.F, 0.F, .8F};
  EXPECT_EQ(propagateUnknownClusterLabels(nodes, edges, classes, confidence), 0U);
  EXPECT_EQ(classes[1], -1);
  EXPECT_EQ(classes[2], -1);
}
