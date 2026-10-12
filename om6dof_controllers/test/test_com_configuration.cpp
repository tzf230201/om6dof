// Copyright 2026 OM6DOF maintainers.
// SPDX-License-Identifier: Apache-2.0

#include <memory>
#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <vector>

#include "gtest/gtest.h"
#include "lifecycle_msgs/msg/state.hpp"
#include "om6dof_controllers/gravity_compensation_controller.hpp"
#include "rclcpp/rclcpp.hpp"

namespace
{
const char * kDescription = R"(
<robot name="com_parameter_test">
  <link name="base"/>
  <link name="arm">
    <inertial><origin xyz="0.1 0 0"/><mass value="1"/>
      <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/>
    </inertial>
  </link>
  <joint name="hinge" type="revolute">
    <parent link="base"/><child link="arm"/><axis xyz="0 1 0"/>
    <limit lower="-3" upper="3" effort="1" velocity="1"/>
  </joint>
</robot>)";

class InspectableController : public om6dof_controllers::GravityCompensationController
{
public:
  bool publishes_every_update() const {return publish_every_update_;}

  bool publication_due_after(double seconds) const
  {
    return publicationDue(last_publish_time_ + rclcpp::Duration::from_seconds(seconds));
  }

  double gravity_at_zero()
  {
    std::vector<double> torques(1);
    model_.compute({0.0}, torques);
    return torques[0];
  }

  double command_for_velocity(double v)
  {
    // Only stack-backed mock interfaces. No hardware/system plugin is loaded.
    double q = 0.0, command = 0.0;
    hardware_interface::CommandInterface cmd("hinge", "effort", &command);
    hardware_interface::StateInterface pos("hinge", "position", &q);
    hardware_interface::StateInterface vel("hinge", "velocity", &v);
    std::vector<hardware_interface::LoanedCommandInterface> commands;
    std::vector<hardware_interface::LoanedStateInterface> states;
    commands.emplace_back(cmd);
    states.emplace_back(pos);
    states.emplace_back(vel);
    assign_interfaces(std::move(commands), std::move(states));
    EXPECT_EQ(controller_interface::CallbackReturn::SUCCESS,
      on_activate(rclcpp_lifecycle::State()));
    EXPECT_EQ(controller_interface::return_type::OK,
      update(get_node()->now(), rclcpp::Duration::from_seconds(1.0)));
    release_interfaces();
    return command;
  }

  void expect_urdf_only_defaults()
  {
    EXPECT_TRUE(model_.com_adjustments().empty());
    ASSERT_EQ(6u, gain_.size());
    for (size_t i = 0; i < gain_.size(); ++i) {
      EXPECT_DOUBLE_EQ(1.0, gain_[i]);
      EXPECT_DOUBLE_EQ(0.0, bias_[i]);
      EXPECT_DOUBLE_EQ(i == 0 ? 0.2 : 0.0, friction_[i].coulomb);
      EXPECT_DOUBLE_EQ(0.1, friction_[i].viscous);
    }
    EXPECT_EQ((std::vector<double>{30.0, 360.0, 260.0, 30.0, 220.0, 30.0}), max_effort_);
    EXPECT_EQ((std::vector<double>{208.541119, 208.541119, 208.541119,
        285.006196, 208.541119, 285.006196}), effort_scale_);
  }
};

class ComConfiguration : public ::testing::Test
{
protected:
  void SetUp() override
  {
    context_ = std::make_shared<rclcpp::Context>();
    rclcpp::InitOptions init_options;
    init_options.set_domain_id(192);  // isolated from the hardware ROS domain
    context_->init(0, nullptr, init_options);
  }

  void TearDown() override {context_->shutdown("test finished");}

  rclcpp::NodeOptions options(const std::vector<double> & xyz)
  {
    return rclcpp::NodeOptions().context(context_).parameter_overrides({
      rclcpp::Parameter("joints", std::vector<std::string>{"hinge"}),
      rclcpp::Parameter("base_link", "base"),
      rclcpp::Parameter("tip_link", "arm"),
      rclcpp::Parameter("robot_description", kDescription),
      rclcpp::Parameter("com_offsets.links", std::vector<std::string>{"arm"}),
      rclcpp::Parameter("com_offsets.xyz_m", xyz)});
  }

  std::shared_ptr<rclcpp::Context> context_;
};
}  // namespace

TEST_F(ComConfiguration, StartupOverridesReachTheGravitySolver)
{
  InspectableController controller;
  ASSERT_EQ(controller_interface::return_type::OK,
    controller.init("com_test", "/offline", options({0.01, 0.0, 0.0})));
  ASSERT_EQ(controller_interface::CallbackReturn::SUCCESS,
    controller.on_configure(rclcpp_lifecycle::State()));
  EXPECT_NEAR(-0.11 * 9.80665, controller.gravity_at_zero(), 1e-9);
}

TEST_F(ComConfiguration, RuntimeEditsAreRejectedWithoutChangingParameterOrModel)
{
  InspectableController controller;
  ASSERT_EQ(controller_interface::return_type::OK,
    controller.init("com_test", "/offline", options({0.01, 0.0, 0.0})));
  ASSERT_EQ(controller_interface::CallbackReturn::SUCCESS,
    controller.on_configure(rclcpp_lifecycle::State()));
  const auto node = controller.get_node();
  const auto result = node->set_parameters_atomically({
    rclcpp::Parameter("com_offsets.xyz_m", std::vector<double>{0.02, 0.0, 0.0})});
  EXPECT_FALSE(result.successful);
  EXPECT_NE(std::string::npos, result.reason.find("startup-only"));
  EXPECT_DOUBLE_EQ(0.01, node->get_parameter("com_offsets.xyz_m").as_double_array()[0]);
  EXPECT_FALSE(node->set_parameter(rclcpp::Parameter(
      "com_offsets.links", std::vector<std::string>{"base"})).successful);
  EXPECT_NEAR(-0.11 * 9.80665, controller.gravity_at_zero(), 1e-9);
}

TEST_F(ComConfiguration, InvalidStartupOffsetFailsConfiguration)
{
  InspectableController controller;
  ASSERT_EQ(controller_interface::return_type::OK,
    controller.init("com_test", "/offline", options({0.1, 0.0, 0.0})));
  EXPECT_EQ(controller_interface::CallbackReturn::ERROR,
    controller.on_configure(rclcpp_lifecycle::State()));
}

TEST_F(ComConfiguration, ShippedTeachingYamlUsesUrdfOnlyWithMotorLimitsPreserved)
{
  // Load the actual ROS YAML with the same node name as the spawner. Replace
  // only robot geometry/joint names with an analytic fixture; no subscriptions,
  // hardware interfaces, controller activation or motor commands are involved.
  std::string description(kDescription);
  description.insert(description.find("</joint>"), "<dynamics friction=\"0.2\" damping=\"0.1\"/>");
  for (int i = 2; i <= 6; ++i) {
    const std::string name = "extra" + std::to_string(i);
    description.insert(description.find("</robot>"),
      "<link name=\"" + name + "_link\"/><joint name=\"" + name + "\" type=\"continuous\">"
      "<parent link=\"base\"/><child link=\"" + name + "_link\"/>"
      "<axis xyz=\"0 0 1\"/><dynamics damping=\"0.1\"/></joint>");
  }
  auto node_options = rclcpp::NodeOptions().context(context_)
    .allow_undeclared_parameters(true)
    .automatically_declare_parameters_from_overrides(true)
    .arguments({"--ros-args", "--params-file", TEST_CONTROLLER_CONFIG_PATH})
    .parameter_overrides({
      rclcpp::Parameter("robot_description", description),
      rclcpp::Parameter("base_link", "base"),
      rclcpp::Parameter("tip_link", "arm"),
      rclcpp::Parameter("joints", std::vector<std::string>{
        "hinge", "extra2", "extra3", "extra4", "extra5", "extra6"})});
  InspectableController controller;
  ASSERT_EQ(controller_interface::return_type::OK,
    controller.init("om6dof_torque_teaching_controller", "/offline", node_options));
  ASSERT_EQ(controller_interface::CallbackReturn::SUCCESS,
    controller.on_configure(rclcpp_lifecycle::State()));
  controller.expect_urdf_only_defaults();
  EXPECT_EQ(100, controller.get_node()->get_parameter("update_rate").as_int());
  EXPECT_DOUBLE_EQ(100.0, controller.get_node()->get_parameter("publish_rate").as_double());
  EXPECT_NEAR(-0.1 * 9.80665, controller.gravity_at_zero(), 1e-9);
}

TEST_F(ComConfiguration, FullRatePublicationDoesNotSkipShortJitteredCycles)
{
  for (double rate : {0.0, 50.0, 100.0, 200.0}) {
    auto opts = options({0.0, 0.0, 0.0});
    opts.append_parameter_override("update_rate", 100);
    opts.append_parameter_override("publish_rate", rate);
    InspectableController controller;
    ASSERT_EQ(controller_interface::return_type::OK,
      controller.init("rate_test", "/offline", opts));
    // Use the actual lifecycle wrapper, which resolves update_rate.
    ASSERT_EQ(lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE, controller.configure().id());
    ASSERT_EQ(100u, controller.get_update_rate());
    controller.command_for_velocity(0.0);  // mock interface activation, no robot
    EXPECT_EQ(rate >= 100.0, controller.publishes_every_update());
    for (double interval : {0.008, 0.0098, 0.01, 0.0102, 0.012}) {
      EXPECT_EQ(rate >= 100.0, controller.publication_due_after(interval));
    }
    EXPECT_EQ(rate > 0.0, controller.publication_due_after(0.021));
    EXPECT_FALSE(controller.get_node()->set_parameter(
        rclcpp::Parameter("publish_rate", 10.0)).successful);
  }
}

TEST_F(ComConfiguration, InvalidPublishRatesAreRejected)
{
  for (double rate : {-1.0, std::numeric_limits<double>::infinity(),
      std::numeric_limits<double>::quiet_NaN()})
  {
    auto opts = options({0.0, 0.0, 0.0});
    opts.append_parameter_override("publish_rate", rate);
    InspectableController controller;
    ASSERT_EQ(controller_interface::return_type::OK,
      controller.init("rate_test", "/offline", opts));
    EXPECT_EQ(controller_interface::CallbackReturn::ERROR,
      controller.on_configure(rclcpp_lifecycle::State()));
  }
}

TEST_F(ComConfiguration, UrdfFrictionReachesSignedCurrentAndClamp)
{
  std::string description(kDescription);
  description.insert(description.find("</joint>"), "<dynamics friction=\"0.3\" damping=\"0.4\"/>");
  for (double limit : {1.0, 0.5}) {
    auto opts = options({0.0, 0.0, 0.0});
    opts.append_parameter_override("robot_description", description);
    opts.append_parameter_override("friction.source", "urdf");
    opts.append_parameter_override("gravity", std::vector<double>{0.0, 0.0, 0.0});
    opts.append_parameter_override("effort_scale", std::vector<double>{2.0});
    opts.append_parameter_override("max_effort", std::vector<double>{limit});
    InspectableController controller;
    ASSERT_EQ(controller_interface::return_type::OK, controller.init("friction_test", "/offline", opts));
    ASSERT_EQ(controller_interface::CallbackReturn::SUCCESS,
      controller.on_configure(rclcpp_lifecycle::State()));
    const double expected = std::min(limit, 2.0 * (0.3 * std::tanh(4.0) + 0.4 * 0.2));
    EXPECT_NEAR(expected, controller.command_for_velocity(0.2), 1e-12);
    EXPECT_NEAR(-expected, controller.command_for_velocity(-0.2), 1e-12);
    EXPECT_DOUBLE_EQ(0.0, controller.command_for_velocity(0.0));
    EXPECT_FALSE(controller.get_node()->set_parameter(
        rclcpp::Parameter("friction.source", "parameters")).successful);
    EXPECT_FALSE(controller.get_node()->set_parameter(
        rclcpp::Parameter("friction.viscous", std::vector<double>{0.8})).successful);
  }
}

TEST_F(ComConfiguration, UrdfSourceRejectsConflictingYamlOrUnknownSource)
{
  for (const std::string source : {"urdf", "typo"}) {
    auto opts = options({0.0, 0.0, 0.0});
    opts.append_parameter_override("friction.source", source);
    opts.append_parameter_override("friction.coulomb", std::vector<double>{0.2});
    InspectableController controller;
    ASSERT_EQ(controller_interface::return_type::OK, controller.init("friction_test", "/offline", opts));
    EXPECT_EQ(controller_interface::CallbackReturn::ERROR,
      controller.on_configure(rclcpp_lifecycle::State()));
  }
}
