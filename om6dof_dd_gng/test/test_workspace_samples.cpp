#include <set>
#include <gtest/gtest.h>
#include "om6dof_dd_gng/workspace_samples.hpp"
namespace reach = om6dof_dd_gng::reachability;
const std::string header = "x_mm,y_mm,z_mm,position_found,q1_rad,q2_rad,q3_rad,q4_rad,q5_rad,q6_rad\n";
TEST(WorkspaceSamples, KeepsPositionWitnessesAndConvertsMillimeters)
{
  std::istringstream input(header + "100,200,300,1,0,0.1,0.2,0.3,0.4,0.5\n0,0,0,0,,,,,,\n");
  auto rows = reach::readWorkspaceSamples(input);
  ASSERT_EQ(rows.size(), 1U);
  EXPECT_DOUBLE_EQ(rows[0].requested_position.x, .1);
  EXPECT_DOUBLE_EQ(rows[0].requested_position.z, .3);
  EXPECT_DOUBLE_EQ(rows[0].joints[5], .5);
}
TEST(WorkspaceSamples, RejectsMalformedAndNonfiniteWitnesses)
{
  for (const auto & row : {"0,0,0,1,nan,0,0,0,0,0\n", "0,0,0,1,0,0,0\n",
    "0,0,0,2,0,0,0,0,0,0\n", "0,0,0,1,1junk,0,0,0,0,0\n"})
  {
    std::istringstream input(header + row);
    EXPECT_THROW(reach::readWorkspaceSamples(input), std::exception);
  }
  std::istringstream missing("x_mm,y_mm\n0,0\n");
  EXPECT_THROW(reach::readWorkspaceSamples(missing), std::runtime_error);
}
TEST(WorkspaceSamples, SpatialBucketsMatchBruteForceJointNeighbors)
{
  std::vector<reach::Node> nodes(60);
  for (std::size_t i = 0; i < nodes.size(); ++i) {
    nodes[i].position = {double(i % 5) * .03 - .07, double(i % 7) * .02 - .09, double(i % 3) * .02};
    nodes[i].joints = {double(i) * .01, double(i % 8) * .08};
  }
  const std::vector<double> ranges{1, 1};
  auto actual = reach::workspaceCandidateEdges(nodes, ranges, .08, .3, 3);
  std::set<std::pair<std::size_t, std::size_t>> expected;
  for (std::size_t i = 0; i < nodes.size(); ++i) {
    std::vector<std::pair<double, std::size_t>> ranked;
    for (std::size_t j = 0; j < nodes.size(); ++j) {
      double d = reach::normalizedJointDistance(nodes[i].joints, nodes[j].joints, ranges);
      if (i != j && reach::distance(nodes[i].position, nodes[j].position) <= .08 && d > 1e-9 && d <= .3) {
        ranked.emplace_back(d, j);
      }
    }
    std::sort(ranked.begin(), ranked.end());
    for (std::size_t k = 0; k < std::min<std::size_t>(3, ranked.size()); ++k) {
      expected.insert(std::minmax(i, ranked[k].second));
    }
  }
  std::set<std::pair<std::size_t, std::size_t>> found;
  for (const auto & edge : actual) {found.insert({edge.a, edge.b}); EXPECT_GT(edge.cost, 0);}
  EXPECT_EQ(found, expected);
}
TEST(WorkspaceSamples, CoincidentXyzDoesNotMergeDifferentJointConfigurations)
{
  std::vector<reach::Node> nodes(2);
  nodes[0].joints = {0.0}; nodes[1].joints = {.1};
  EXPECT_EQ(reach::workspaceCandidateEdges(nodes, {1.0}, .05, .5, 1).size(), 1U);
  EXPECT_THROW(reach::workspaceCandidateEdges(nodes, {1.0}, 0, .5, 1), std::invalid_argument);
}
