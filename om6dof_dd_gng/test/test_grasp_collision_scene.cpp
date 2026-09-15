#include <gtest/gtest.h>
#include <limits>
#include <srdfdom/model.h>
#include <urdf_parser/urdf_parser.h>
#include "om6dof_dd_gng/grasp_collision_scene.hpp"
#include "om6dof_dd_gng/semantic_target_selection.hpp"

namespace grasp = om6dof_dd_gng::grasp;
namespace reach = om6dof_dd_gng::reachability;

namespace
{
moveit::core::RobotModelPtr robotModel()
{
  const auto urdf = urdf::parseURDF(R"(
    <robot name="grasp_scene_fixture">
      <link name="world"/>
      <link name="finger"><collision><geometry><sphere radius="0.01"/></geometry></collision></link>
      <joint name="slide" type="prismatic">
        <parent link="world"/><child link="finger"/><axis xyz="1 0 0"/>
        <limit lower="-2" upper="2" effort="1" velocity="1"/>
      </joint>
    </robot>)");
  auto srdf = std::make_shared<srdf::Model>();
  if (!urdf || !srdf->initString(*urdf, "<robot name=\"grasp_scene_fixture\"></robot>")) {
    throw std::runtime_error("fixture_model_invalid");
  }
  return std::make_shared<moveit::core::RobotModel>(urdf, srdf);
}

om6dof_dd_gng::msg::EnvironmentNode node(std::uint32_t id, double x, double z, int label)
{
  om6dof_dd_gng::msg::EnvironmentNode result;
  result.id = id; result.position.x = x; result.position.z = z; result.class_id = label;
  return result;
}

om6dof_dd_gng::msg::TopologyEdge edge(std::uint32_t a, std::uint32_t b)
{
  om6dof_dd_gng::msg::TopologyEdge result;
  result.source_id = a; result.target_id = b;
  return result;
}

om6dof_dd_gng::msg::EnvironmentGraph environment()
{
  om6dof_dd_gng::msg::EnvironmentGraph graph;
  graph.header.frame_id = "world";
  graph.nodes = {node(1, -.3, 0, 39), node(2, -.2, 0, 39), node(3, -.1, 0, 39),
    node(4, .2, 0, 39), node(5, .3, 0, -1)};
  graph.edges = {edge(1, 2), edge(2, 3), edge(3, 5), edge(4, 5)};
  return graph;
}
}  // namespace

TEST(GraspCollisionScene, OnlySelectedSameClassComponentBecomesTarget)
{
  const auto graph = environment();
  const auto result = grasp::buildGraspCollisionScene(robotModel(), graph, 2);
  EXPECT_EQ(result.target_node_ids, (std::vector<std::uint32_t>{1, 2, 3}));
  EXPECT_EQ(result.class_id, 39);
  EXPECT_EQ(result.target.primitives.size(), 5U);  // Three nodes, two internal edges.
  EXPECT_EQ(result.obstacles.primitives.size(), 4U);  // Two nodes, both crossing edges.
  EXPECT_DOUBLE_EQ(result.obstacles.primitive_poses[0].position.x, .2);
  EXPECT_DOUBLE_EQ(result.obstacles.primitives[0].dimensions[0], .012);
  EXPECT_EQ(result.obstacles.primitives[2].type, shape_msgs::msg::SolidPrimitive::CYLINDER);
  EXPECT_DOUBLE_EQ(result.obstacles.primitives[2].dimensions[1], .006);
  EXPECT_DOUBLE_EQ(result.obstacles.primitives[2].dimensions[0], .4);
}

TEST(GraspCollisionScene, CenterExactlyMatchesSemanticTargetSelection)
{
  auto graph = environment();
  graph.nodes.push_back(node(6, -.28, .2, 39));
  graph.edges.push_back(edge(1, 6));
  const auto result = grasp::buildGraspCollisionScene(robotModel(), graph, 2);
  std::vector<reach::LabeledTarget> nodes;
  std::vector<std::pair<std::uint32_t, std::uint32_t>> edges;
  for (const auto & n : graph.nodes) {
    nodes.push_back({{n.id, {n.position.x, n.position.y, n.position.z}}, n.class_id});
  }
  for (const auto & e : graph.edges) {edges.emplace_back(e.source_id, e.target_id);}
  const auto centers = reach::componentCenterTargets(nodes, edges);
  ASSERT_EQ(centers.size(), 2U);
  EXPECT_DOUBLE_EQ(result.center.x, centers[0].position.x);
  EXPECT_DOUBLE_EQ(result.center.y, centers[0].position.y);
  EXPECT_DOUBLE_EQ(result.center.z, centers[0].position.z);
}

TEST(GraspCollisionScene, TargetContactDoesNotExemptOtherBottleOrBoundaryEdges)
{
  const auto result = grasp::buildGraspCollisionScene(robotModel(), environment(), 2);
  auto & state = result.scene->getCurrentStateNonConst();
  state.setToDefaultValues();
  state.setVariablePosition("slide", -.2); state.update();
  EXPECT_TRUE(result.scene->isStateColliding(state));  // Target protected by default.
  result.scene->getAllowedCollisionMatrixNonConst().setEntry("finger", grasp::kTargetObjectId, true);
  EXPECT_FALSE(result.scene->isStateColliding(state));  // Only selected target contact allowed.
  state.setVariablePosition("slide", .2); state.update();
  EXPECT_TRUE(result.scene->isStateColliding(state));  // Disconnected bottle still obstacle.
  state.setVariablePosition("slide", 0); state.update();
  EXPECT_TRUE(result.scene->isStateColliding(state));  // Crossing edge remains protected.
  state.setVariablePosition("slide", 1); state.update();
  EXPECT_FALSE(result.scene->isStateColliding(state));
}

TEST(GraspCollisionScene, ExcludingSelectedTargetPreservesCenterIdentityAndAllOtherGeometry)
{
  const auto model = robotModel();
  const auto graph = environment();
  const auto protected_scene = grasp::buildGraspCollisionScene(model, graph, 2);
  const auto result = grasp::buildGraspCollisionScene(model, graph, 2, .012, .006, true);
  EXPECT_EQ(result.target_node_ids, protected_scene.target_node_ids);
  EXPECT_EQ(result.class_id, protected_scene.class_id);
  EXPECT_EQ(result.center, protected_scene.center);
  EXPECT_EQ(result.target.id, grasp::kTargetObjectId);
  EXPECT_EQ(result.target.header.frame_id, "world");
  EXPECT_TRUE(result.target.primitives.empty());
  EXPECT_TRUE(result.target.primitive_poses.empty());
  EXPECT_FALSE(result.scene->getWorld()->hasObject(grasp::kTargetObjectId));
  EXPECT_TRUE(result.scene->getWorld()->hasObject(grasp::kObstacleObjectId));
  EXPECT_EQ(result.obstacles, protected_scene.obstacles);
  ASSERT_EQ(result.obstacles.primitives.size(), 4U);  // Other bottle, unknown, both boundary edges.

  auto & state = result.scene->getCurrentStateNonConst();
  state.setToDefaultValues();
  state.setVariablePosition("slide", -.2); state.update();
  EXPECT_FALSE(result.scene->isStateColliding(state));  // Selected target omitted with no ACM exemption.
  state.setVariablePosition("slide", .2); state.update();
  EXPECT_TRUE(result.scene->isStateColliding(state));  // Another bottle remains an obstacle.
  state.setVariablePosition("slide", .3); state.update();
  EXPECT_TRUE(result.scene->isStateColliding(state));  // Unknown node remains an obstacle.
  state.setVariablePosition("slide", 0); state.update();
  EXPECT_TRUE(result.scene->isStateColliding(state));  // Boundary edge crossing free space retained.
  state.setVariablePosition("slide", 1); state.update();
  EXPECT_FALSE(result.scene->isStateColliding(state));
}

TEST(GraspCollisionScene, ReusablePartitionKeepsMasksAlignedAndDoesNotExcludeEveryBottle)
{
  auto graph = environment();
  const auto selected = grasp::partitionGraspTarget(graph, 3);
  EXPECT_EQ(selected.target_node_ids, (std::vector<std::uint32_t>{1, 2, 3}));
  EXPECT_EQ(selected.node_is_target, (std::vector<bool>{true, true, true, false, false}));
  EXPECT_EQ(selected.edge_is_target, (std::vector<bool>{true, true, false, false}));
  EXPECT_DOUBLE_EQ(selected.center.x, -.2);

  const auto other_bottle = grasp::partitionGraspTarget(graph, 4);
  EXPECT_EQ(other_bottle.target_node_ids, (std::vector<std::uint32_t>{4}));
  EXPECT_EQ(other_bottle.node_is_target, (std::vector<bool>{false, false, false, true, false}));
  EXPECT_EQ(other_bottle.edge_is_target, (std::vector<bool>{false, false, false, false}));
  const auto other_scene = grasp::buildGraspCollisionScene(robotModel(), graph, 4, .012, .006, true);
  EXPECT_TRUE(other_scene.target.primitives.empty());
  EXPECT_EQ(other_scene.obstacles.primitives.size(), 8U);  // Four remaining nodes and all four edges.

  std::reverse(graph.nodes.begin(), graph.nodes.end());
  std::reverse(graph.edges.begin(), graph.edges.end());
  const auto reordered = grasp::partitionGraspTarget(graph, 1);
  EXPECT_EQ(reordered.target_node_ids, selected.target_node_ids);
  EXPECT_EQ(reordered.node_is_target, (std::vector<bool>{false, false, true, true, true}));
  EXPECT_EQ(reordered.edge_is_target, (std::vector<bool>{false, false, true, true}));
  EXPECT_EQ(reordered.center, selected.center);
}

TEST(GraspCollisionScene, ExplicitFalseRetainsDefaultProtection)
{
  const auto model = robotModel();
  const auto original = grasp::buildGraspCollisionScene(model, environment(), 2);
  const auto explicit_false = grasp::buildGraspCollisionScene(model, environment(), 2, .012, .006, false);
  EXPECT_EQ(original.target, explicit_false.target);
  EXPECT_EQ(original.obstacles, explicit_false.obstacles);
  EXPECT_TRUE(explicit_false.scene->getWorld()->hasObject(grasp::kTargetObjectId));
}

TEST(GraspCollisionScene, InvalidGraphFailsBeforeBuildingAnIncompleteScene)
{
  const auto model = robotModel();
  auto graph = environment();
  graph.edges.push_back(edge(2, 99));
  EXPECT_THROW(grasp::buildGraspCollisionScene(model, graph, 2), std::invalid_argument);
  graph = environment(); graph.nodes.push_back(graph.nodes.front());
  EXPECT_THROW(grasp::buildGraspCollisionScene(model, graph, 2), std::invalid_argument);
  graph = environment(); graph.nodes[0].position.x = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(grasp::buildGraspCollisionScene(model, graph, 2), std::invalid_argument);
  graph = environment(); graph.header.frame_id = "camera";
  EXPECT_THROW(grasp::buildGraspCollisionScene(model, graph, 2), std::invalid_argument);
  EXPECT_THROW(grasp::buildGraspCollisionScene(model, environment(), 5), std::invalid_argument);
  EXPECT_THROW(grasp::buildGraspCollisionScene(model, environment(), 2, -.1), std::invalid_argument);
  graph = environment(); graph.edges.push_back(edge(2, 99));
  EXPECT_THROW(grasp::partitionGraspTarget(graph, 2), std::invalid_argument);
  EXPECT_THROW(grasp::buildGraspCollisionScene(model, graph, 2, .012, .006, true), std::invalid_argument);
  EXPECT_THROW(grasp::partitionGraspTarget(environment(), 5), std::invalid_argument);
}
