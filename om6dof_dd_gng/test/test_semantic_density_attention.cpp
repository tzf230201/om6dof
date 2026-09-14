// SPDX-License-Identifier: Apache-2.0
#include <gtest/gtest.h>

#include <limits>
#include <vector>

#include "om6dof_dd_gng/semantic_density_attention.hpp"

using namespace om6dof_dd_gng;

namespace
{
const std::vector<GngPoint3f> kNode{{0.F, 0.F, 1.F}};
const std::vector<float> kScore{1.F};
constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();
constexpr float kFloatNaN = std::numeric_limits<float>::quiet_NaN();
constexpr float kFloatInfinity = std::numeric_limits<float>::infinity();
}  // namespace

TEST(SemanticDensityAttention, ExpiresFromSourceCaptureNotInferenceCompletion)
{
  SemanticDensityAttention attention;
  attention.observe(1, 10.0, 10.8, true, kNode, kScore);
  ASSERT_EQ(attention.regions(10.8).size(), 1U);
  EXPECT_EQ(attention.regions(11.0).size(), 1U);
  EXPECT_TRUE(attention.regions(11.00001).empty());
}

TEST(SemanticDensityAttention, DuplicateResultDoesNotRefreshOrMoveAttention)
{
  SemanticDensityAttention attention;
  attention.observe(1, 10.0, 10.1, true, kNode, kScore);
  attention.observe(1, 10.9, 10.9, true, {{2.F, 0.F, 1.F}}, {.5F});
  const auto held = attention.regions(10.9);
  ASSERT_EQ(held.size(), 1U);
  EXPECT_FLOAT_EQ(held[0].center.x, 0.F);
  EXPECT_FLOAT_EQ(held[0].strength, 3.F);
  EXPECT_TRUE(attention.regions(11.01).empty());
  attention.observe(1, 11.1, 11.1, true, kNode, kScore);
  EXPECT_TRUE(attention.regions(11.1).empty());
}

TEST(SemanticDensityAttention, NewUniqueResultReplacesAndEmptyResultClears)
{
  SemanticDensityAttention attention;
  attention.observe(1, 10.0, 10.1, true, kNode, kScore);
  attention.observe(2, 10.2, 10.3, true, {{2.F, 0.F, 1.F}}, {.5F});
  const auto next = attention.regions(10.3);
  ASSERT_EQ(next.size(), 1U);
  EXPECT_FLOAT_EQ(next[0].center.x, 2.F);
  EXPECT_FLOAT_EQ(next[0].strength, 2.F);
  attention.observe(3, 10.4, 10.4, true, {}, {});
  EXPECT_TRUE(attention.regions(10.4).empty());
  attention.observe(3, 10.5, 10.5, true, kNode, kScore);
  EXPECT_TRUE(attention.regions(10.5).empty());
}

TEST(SemanticDensityAttention, PoseMismatchClearsAndCannotResurrectSameResult)
{
  SemanticDensityAttention attention;
  attention.observe(1, 10.0, 10.1, true, kNode, kScore);
  attention.observe(2, 10.2, 10.3, false, kNode, kScore);
  EXPECT_TRUE(attention.regions(10.3).empty());
  attention.observe(2, 10.2, 10.4, true, kNode, kScore);
  EXPECT_TRUE(attention.regions(10.4).empty());
  attention.observe(3, 10.5, 10.6, true, kNode, kScore);
  EXPECT_EQ(attention.regions(10.6).size(), 1U);
}

TEST(SemanticDensityAttention, InvalidDuplicateClearsPreviousAttention)
{
  SemanticDensityAttention attention;
  attention.observe(1, 10.0, 10.0, true, kNode, kScore);
  attention.observe(1, 10.0, 10.1, false, kNode, kScore);
  EXPECT_TRUE(attention.regions(10.1).empty());
  attention.observe(1, 10.0, 10.2, true, kNode, kScore);
  EXPECT_TRUE(attention.regions(10.2).empty());
}

TEST(SemanticDensityAttention, StaleUnknownAndFutureSourceClearAttention)
{
  for (const auto bad_source : {8.0, -1.0, kNaN, 11.0}) {
    SemanticDensityAttention attention;
    attention.observe(1, 10.0, 10.0, true, kNode, kScore);
    attention.observe(2, bad_source, 10.2, true, kNode, kScore);
    EXPECT_TRUE(attention.regions(10.2).empty());
    attention.observe(2, 10.2, 10.3, true, kNode, kScore);
    EXPECT_TRUE(attention.regions(10.3).empty());
  }
}

TEST(SemanticDensityAttention, SequenceZeroAndOutOfOrderFailClosed)
{
  SemanticDensityAttention attention;
  attention.observe(2, 10.0, 10.0, true, kNode, kScore);
  attention.observe(0, 10.0, 10.1, true, kNode, kScore);
  EXPECT_TRUE(attention.regions(10.1).empty());
  attention.observe(3, 10.2, 10.2, true, kNode, kScore);
  EXPECT_EQ(attention.regions(10.2).size(), 1U);
  attention.observe(2, 10.1, 10.3, true, kNode, kScore);
  EXPECT_TRUE(attention.regions(10.3).empty());
  attention.observe(3, 10.2, 10.4, true, kNode, kScore);
  EXPECT_TRUE(attention.regions(10.4).empty());
}

TEST(SemanticDensityAttention, NewSequenceWithRegressingCaptureTimeFailsClosed)
{
  SemanticDensityAttention attention;
  attention.observe(1, 10.1, 10.2, true, kNode, kScore);
  attention.observe(2, 10.0, 10.3, true, kNode, kScore);
  EXPECT_TRUE(attention.regions(10.3).empty());
  attention.observe(2, 10.2, 10.4, true, kNode, kScore);
  EXPECT_TRUE(attention.regions(10.4).empty());
}

TEST(SemanticDensityAttention, InvalidOrRegressingReadClockClears)
{
  for (const auto bad_now : {9.9, -1.0, kNaN, std::numeric_limits<double>::infinity()}) {
    SemanticDensityAttention attention;
    attention.observe(1, 10.0, 10.0, true, kNode, kScore);
    EXPECT_TRUE(attention.regions(bad_now).empty());
    attention.observe(1, 10.0, 10.1, true, kNode, kScore);
    EXPECT_TRUE(attention.regions(10.1).empty());
  }
}

TEST(SemanticDensityAttention, InvalidOrRegressingObservationClockConsumesResult)
{
  for (const auto bad_now : {9.9, -1.0, kNaN, std::numeric_limits<double>::infinity()}) {
    SemanticDensityAttention attention;
    attention.observe(1, 10.0, 10.0, true, kNode, kScore);
    attention.observe(2, 10.0, bad_now, true, kNode, kScore);
    EXPECT_TRUE(attention.regions(10.1).empty());
    attention.observe(2, 10.0, 10.2, true, kNode, kScore);
    EXPECT_TRUE(attention.regions(10.2).empty());
  }
}

TEST(SemanticDensityAttention, LengthMismatchClearsInsteadOfReadingOutOfBounds)
{
  SemanticDensityAttention attention;
  attention.observe(1, 10.0, 10.0, true, kNode, kScore);
  attention.observe(2, 10.1, 10.1, true, kNode, {});
  EXPECT_TRUE(attention.regions(10.1).empty());
  attention.observe(2, 10.1, 10.2, true, kNode, kScore);
  EXPECT_TRUE(attention.regions(10.2).empty());
}

TEST(SemanticDensityAttention, RejectsInvalidPointsAndScoresAndClampsConfidence)
{
  SemanticDensityAttention attention;
  const std::vector<GngPoint3f> nodes{
    {0.F, 0.F, 1.F}, {1.F, 0.F, 1.F}, {2.F, 0.F, 1.F},
    {3.F, 0.F, 1.F}, {4.F, 0.F, 1.F}, {5.F, 0.F, 1.F},
    {kFloatNaN, 0.F, 1.F}, {6.F, kFloatInfinity, 1.F},
    {7.F, 0.F, kFloatNaN}};
  attention.observe(
    1, 10.0, 10.0, true, nodes,
    {2.F, .25F, 0.F, -.5F, kFloatNaN, kFloatInfinity, 1.F, 1.F, 1.F});
  const auto result = attention.regions(10.0);
  ASSERT_EQ(result.size(), 2U);
  EXPECT_FLOAT_EQ(result[0].center.x, 0.F);
  EXPECT_FLOAT_EQ(result[0].strength, 3.F);
  EXPECT_FLOAT_EQ(result[1].center.x, 1.F);
  EXPECT_FLOAT_EQ(result[1].strength, 1.5F);
  EXPECT_FLOAT_EQ(result[1].radius, .05F);
}

TEST(SemanticDensityAttention, ZeroDirectMatchesDoNotCreateAttention)
{
  SemanticDensityAttention attention;
  attention.observe(1, 10.0, 10.0, true, kNode, kScore);
  attention.observe(2, 10.1, 10.1, true, kNode, {0.F});
  EXPECT_TRUE(attention.regions(10.1).empty());
}

TEST(SemanticDensityAttention, GreedySelectionUsesScoreThenIndexAndHalfRadius)
{
  SemanticDensityAttentionParameters parameters;
  parameters.radius_m = .25F;
  parameters.max_regions = 3;
  SemanticDensityAttention attention(parameters);
  attention.observe(
    1, 10.0, 10.0, true,
    {{0.F, 0.F, 1.F}, {.0625F, 0.F, 1.F}, {.1875F, 0.F, 1.F},
      {1.F, 0.F, 1.F}, {2.F, 0.F, 1.F}, {3.F, 0.F, 1.F}},
    {.8F, .9F, .8F, .8F, .8F, .7F});
  const auto result = attention.regions(10.0);
  ASSERT_EQ(result.size(), 3U);
  EXPECT_FLOAT_EQ(result[0].center.x, .0625F);
  EXPECT_FLOAT_EQ(result[1].center.x, 1.F);
  EXPECT_FLOAT_EQ(result[2].center.x, 2.F);
}

TEST(SemanticDensityAttention, MaximumRegionBudgetIsBoundedAndDeterministic)
{
  std::vector<GngPoint3f> nodes;
  for (int i = 0; i < 100; ++i) {
    nodes.push_back({static_cast<float>(i), 0.F, 1.F});
  }
  const std::vector<float> scores(100, .5F);
  SemanticDensityAttention a;
  SemanticDensityAttention b;
  a.observe(1, 10.0, 10.0, true, nodes, scores);
  b.observe(1, 10.0, 10.0, true, nodes, scores);
  const auto ra = a.regions(10.0);
  const auto rb = b.regions(10.0);
  ASSERT_EQ(ra.size(), 64U);
  ASSERT_EQ(ra.size(), rb.size());
  for (std::size_t i = 0; i < ra.size(); ++i) {
    EXPECT_FLOAT_EQ(ra[i].center.x, static_cast<float>(i));
    EXPECT_FLOAT_EQ(ra[i].center.x, rb[i].center.x);
    EXPECT_FLOAT_EQ(ra[i].strength, rb[i].strength);
  }
}

TEST(SemanticDensityAttention, ReturnedSnapshotCannotMutateInternalRegions)
{
  SemanticDensityAttention attention;
  attention.observe(1, 10.0, 10.0, true, kNode, kScore);
  auto result = attention.regions(10.0);
  ASSERT_EQ(result.size(), 1U);
  result[0].center.x = 999.F;
  result.clear();
  const auto again = attention.regions(10.0);
  ASSERT_EQ(again.size(), 1U);
  EXPECT_FLOAT_EQ(again[0].center.x, 0.F);
}

TEST(SemanticDensityAttention, ValidatesParameterBounds)
{
  for (const double age : {0.0, -1.0, kNaN, std::numeric_limits<double>::infinity()}) {
    SemanticDensityAttentionParameters parameters;
    parameters.max_source_age_sec = age;
    EXPECT_THROW((SemanticDensityAttention{parameters}), std::invalid_argument);
  }
  for (const float radius : {0.F, -1.F, kFloatNaN, kFloatInfinity}) {
    SemanticDensityAttentionParameters parameters;
    parameters.radius_m = radius;
    EXPECT_THROW((SemanticDensityAttention{parameters}), std::invalid_argument);
  }
  for (const float strength : {.99F, 20.01F, kFloatNaN, kFloatInfinity}) {
    SemanticDensityAttentionParameters parameters;
    parameters.max_strength = strength;
    EXPECT_THROW((SemanticDensityAttention{parameters}), std::invalid_argument);
  }
  for (const std::size_t count : {0U, 65U}) {
    SemanticDensityAttentionParameters parameters;
    parameters.max_regions = count;
    EXPECT_THROW((SemanticDensityAttention{parameters}), std::invalid_argument);
  }
  SemanticDensityAttentionParameters parameters;
  parameters.max_strength = 1.F;
  parameters.max_regions = 1;
  EXPECT_NO_THROW((SemanticDensityAttention{parameters}));
  parameters.max_strength = 20.F;
  parameters.max_regions = 64;
  EXPECT_NO_THROW((SemanticDensityAttention{parameters}));
}
