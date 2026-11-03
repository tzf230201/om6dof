// Offline only: stack-backed interfaces, isolated ROS domain, no hardware plugin.
#include <cmath>
#include <cstdio>
#include <limits>
#include <memory>
#include <gtest/gtest.h>
#include "hardware_interface/handle.hpp"
#include "pluginlib/class_loader.hpp"
#include "om6dof_controllers/position_impedance_controller.hpp"

using controller_interface::CallbackReturn;
using om6dof_controllers::impedanceBudget;
namespace
{
const char * kUrdf = R"(<robot name="test">
<link name="base"/><link name="arm"><inertial><origin xyz="0.1 0 0"/>
<mass value="1"/><inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/>
</inertial></link><joint name="hinge" type="revolute"><parent link="base"/>
<child link="arm"/><axis xyz="0 1 0"/><limit lower="-2" upper="2" effort="2" velocity="1"/>
<dynamics friction="0.3" damping="0.2"/></joint>
<ros2_control name="fake" type="system"><hardware><plugin>mock_components/GenericSystem</plugin></hardware>
<joint name="hinge"><command_interface name="effort"/><state_interface name="position"/>
<state_interface name="velocity"/></joint></ros2_control></robot>)";

class Inspectable : public om6dof_controllers::PositionImpedanceController
{
public:
  using PositionImpedanceController::receiveGoal;
  using PositionImpedanceController::receiveHealth;
  double g() const {return gravity_[0];}
  double residual() const {return residual_[0];}
  double budget() const {return budget_[0];}
  double reference() const {return reference_[0];}
  double target() const {return target_[0];}
  bool fault() const {return fault_;}
  int status() const {return status_;}
  std::array<double, 6> limitEvent() const {return limit_event_;}
  bool currentLimitsOk() const {return current_limits_ok_;}
  uint32_t flags() const {return fault_flags_;}
  void expireGoal() {
    auto g = *goal_buffer_.readFromRT(); g.received_ns = steadyNow() - 500000000;
    goal_buffer_.writeFromNonRT(g);
  }
  void mockHealth(uint64_t faults = 0, bool stale = false) {
    require_health_ = true;
    Health h; h.healthy = true; h.faults = faults;
    h.received_ns = steadyNow() - (stale ? 500000000 : 0);
    h.clean_since_ns = steadyNow() - 2000000000;
    health_buffer_.writeFromNonRT(h);
  }
  uint64_t healthFaults() {return health_buffer_.readFromRT()->faults;}
  bool reportedHealthy() {return health_buffer_.readFromRT()->healthy;}
};

class Impedance : public ::testing::Test
{
protected:
  void SetUp() override {
    context = std::make_shared<rclcpp::Context>();
    rclcpp::InitOptions init; init.set_domain_id(192); context->init(0, nullptr, init);
  }
  void TearDown() override {
    if (controller) {controller->release_interfaces(); controller.reset();}
    context->shutdown("offline test complete");
  }
  CallbackReturn configure(std::vector<rclcpp::Parameter> overrides = {}, std::string xml = kUrdf) {
    std::vector<rclcpp::Parameter> p{
      rclcpp::Parameter("joints", std::vector<std::string>{"hinge"}),
      rclcpp::Parameter("base_link", "base"), rclcpp::Parameter("tip_link", "arm"),
      rclcpp::Parameter("robot_description", xml),
      rclcpp::Parameter("enable_on_activation", true), rclcpp::Parameter("allow_mock_hardware", true),
      rclcpp::Parameter("require_bus_health", false), rclcpp::Parameter("ramp_seconds", 0.1)};
    for (const auto & entry : std::vector<std::pair<std::string, double>>{
      {"stiffness", 2.0}, {"damping", 0.2}, {"effort_scale", 100.0}, {"max_effort", 150.0},
      {"extra_torque_base", 0.03}, {"extra_torque_ratio", 0.10}, {"extra_torque_max", 0.2},
      {"reference_velocity", 0.1}, {"max_reference_error", 0.1}, {"residual_torque_rate", 1.0},
      {"max_velocity", 0.4}}) {p.emplace_back(entry.first, std::vector<double>{entry.second});}
    for (const auto & override : overrides) {
      p.erase(std::remove_if(p.begin(), p.end(), [&](const auto & item) {
        return item.get_name() == override.get_name();}), p.end());
      p.push_back(override);
    }
    controller = std::make_unique<Inspectable>();
    EXPECT_EQ(controller_interface::return_type::OK, controller->init("impedance", "/offline",
      rclcpp::NodeOptions().context(context).parameter_overrides(p)));
    return controller->on_configure(rclcpp_lifecycle::State());
  }
  CallbackReturn activate() {
    std::vector<hardware_interface::LoanedCommandInterface> cmds;
    std::vector<hardware_interface::LoanedStateInterface> states;
    cmds.emplace_back(effort);
    // Deliberately reverse requested ordering: controller must match by name.
    states.emplace_back(velocity); states.emplace_back(position);
    controller->assign_interfaces(std::move(cmds), std::move(states));
    return controller->on_activate(rclcpp_lifecycle::State());
  }
  void step(double dt = 0.01) {
    EXPECT_EQ(controller_interface::return_type::OK, controller->update(
      controller->get_node()->now(), rclcpp::Duration::from_seconds(dt)));
  }
  void ramp() {for (int i = 0; i < 12; ++i) {step();}}
  sensor_msgs::msg::JointState message(double goal = 1.0) {
    sensor_msgs::msg::JointState msg; msg.header.stamp = controller->get_node()->now();
    msg.name = {"hinge"}; msg.position = {goal}; return msg;
  }
  diagnostic_msgs::msg::DiagnosticArray health(uint64_t read_errors = 0) {
    diagnostic_msgs::msg::DiagnosticArray msg; msg.header.stamp = controller->get_node()->now();
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = "dynamixel_hardware_interface/BusHealth";
    const std::vector<std::pair<std::string, std::string>> fields{
      {"schema_version", "1"}, {"driver_instance_id", "test"},
      {"read_failure_count", std::to_string(read_errors)}, {"write_failure_count", "0"},
      {"consecutive_read_failures", "0"}, {"consecutive_write_failures", "0"},
      {"current_read_error", "0"}, {"current_write_error", "0"}, {"hardware_error_mask", "0"},
      {"fail_safe_triggered", "false"}, {"torque_all_enabled", "true"},
      {"torque_enable_inhibited", "false"}, {"hardware_error_monitoring_complete", "true"},
      {"input_voltage_monitoring_complete", "true"}};
    for (const auto & pair : fields) {
      diagnostic_msgs::msg::KeyValue value; value.key = pair.first; value.value = pair.second;
      status.values.push_back(value);
    }
    msg.status.push_back(status); return msg;
  }
  std::shared_ptr<rclcpp::Context> context;
  std::unique_ptr<Inspectable> controller;
  double q{0.0}, v{0.0}, command{0.0};
  hardware_interface::CommandInterface effort{"hinge", "effort", &command};
  hardware_interface::StateInterface position{"hinge", "position", &q};
  hardware_interface::StateInterface velocity{"hinge", "velocity", &v};
};
}

TEST(ImpedanceBudget, SignIndependentPoseDependentAndAbsolutelyBounded) {
  for (double g = -2; g <= 2; g += 0.01) {
    const double b = impedanceBudget(g, .03, .10, .2, 1.5);
    EXPECT_GE(b, 0); EXPECT_LE(b, .2);
    EXPECT_LE(b, std::max(0.0, 1.5 - std::abs(g)) + 1e-12);
    EXPECT_NEAR(b, impedanceBudget(-g, .03, .10, .2, 1.5), 1e-12);
  }
  EXPECT_GT(impedanceBudget(1, .03, .1, .2, 1.5), impedanceBudget(0, .03, .1, .2, 1.5));
  EXPECT_DOUBLE_EQ(0, impedanceBudget(1, .03, .1, 0, 1.5));
  EXPECT_DOUBLE_EQ(0, impedanceBudget(NAN, .03, .1, .2, 1.5));
}

TEST_F(Impedance, ActivationCapturesMeasuredPoseAndIgnoresOldTargets) {
  ASSERT_EQ(CallbackReturn::SUCCESS, configure()); q = .2;
  controller->receiveGoal(message(1));
  ASSERT_EQ(CallbackReturn::SUCCESS, activate()); EXPECT_EQ(0, command); ramp();
  EXPECT_DOUBLE_EQ(.2, controller->target()); EXPECT_DOUBLE_EQ(.2, controller->reference());
  EXPECT_NEAR(controller->g() * 100, command, 1e-9);
  EXPECT_EQ(0, controller->residual());
}

TEST_F(Impedance, FirstSpeedTripReportsMeasuredValueAndDoesNotAutoClear) {
  ASSERT_EQ(CallbackReturn::SUCCESS, configure());
  ASSERT_EQ(CallbackReturn::SUCCESS, activate()); ramp();
  v = .45; step();
  EXPECT_FALSE(controller->currentLimitsOk());
  const auto event = controller->limitEvent();
  EXPECT_EQ((std::array<double, 6>{{1, 2, 0, .45, -.4, .4}}), event);
  EXPECT_TRUE(controller->fault()); EXPECT_EQ(3, controller->status());
  v = 0; q = .1; step();
  EXPECT_TRUE(controller->currentLimitsOk());
  controller->receiveGoal(message(.2)); step();
  EXPECT_EQ(event, controller->limitEvent());
  EXPECT_TRUE(controller->fault()); EXPECT_EQ(0, controller->residual());
  EXPECT_NEAR(controller->g() * 100, command, 1e-9);
  q = 1.98; step();  // A later position violation must not overwrite the first trip.
  EXPECT_FALSE(controller->currentLimitsOk());
  EXPECT_EQ(event, controller->limitEvent());
}

TEST_F(Impedance, PositionTripRecordsSoftBoundsAndNewActivationClearsOnlyHistory) {
  ASSERT_EQ(CallbackReturn::SUCCESS, configure());
  ASSERT_EQ(CallbackReturn::SUCCESS, activate()); ramp();
  q = -1.98; step();
  EXPECT_EQ((std::array<double, 6>{{1, 1, 0, -1.98, -1.95, 1.95}}), controller->limitEvent());
  ASSERT_EQ(CallbackReturn::SUCCESS, controller->on_deactivate(rclcpp_lifecycle::State()));
  EXPECT_DOUBLE_EQ(0, command);
  controller->release_interfaces(); q = 0;
  ASSERT_EQ(CallbackReturn::SUCCESS, activate());
  EXPECT_EQ((std::array<double, 6>{{1, 0, -1, 0, 0, 0}}), controller->limitEvent());
  EXPECT_EQ(0, controller->residual());
}

TEST_F(Impedance, StackReadyPoseGateRejectsMovementDuringModeSwitch) {
  ASSERT_EQ(CallbackReturn::SUCCESS, configure({
    rclcpp::Parameter("activation_pose", std::vector<double>{0.2})}));
  q = 0.3;
  EXPECT_EQ(CallbackReturn::ERROR, activate());
  EXPECT_DOUBLE_EQ(0, command);
  controller->release_interfaces();
  q = 0.21;
  EXPECT_EQ(CallbackReturn::SUCCESS, activate());
}

TEST_F(Impedance, StackReadyPoseGateRejectsMalformedConfiguration) {
  EXPECT_EQ(CallbackReturn::ERROR, configure({
    rclcpp::Parameter("activation_pose", std::vector<double>{0., 0.})}));
  EXPECT_EQ(CallbackReturn::ERROR, configure({
    rclcpp::Parameter("activation_pose_tolerance", -0.1)}));
}

TEST_F(Impedance, SupportedPoseIsReadAtActivationWithoutSendingAnOldReadyTarget) {
  ASSERT_EQ(CallbackReturn::SUCCESS, configure({
    rclcpp::Parameter("require_activation_pose", true),
    rclcpp::Parameter("enable_on_activation", false),
    rclcpp::Parameter("activation_pose", std::vector<double>{0.0})}));
  q = .4;
  EXPECT_FALSE(controller->get_node()->get_parameter("enable_on_activation").as_bool());
  ASSERT_TRUE(controller->get_node()->set_parameters_atomically({
    rclcpp::Parameter("activation_pose", std::vector<double>{.4}),
    rclcpp::Parameter("enable_on_activation", true)}).successful);
  ASSERT_EQ(CallbackReturn::SUCCESS, activate());
  EXPECT_DOUBLE_EQ(.4, controller->target());
  EXPECT_DOUBLE_EQ(.4, controller->reference());
  EXPECT_DOUBLE_EQ(0., command);
  ramp();
  EXPECT_DOUBLE_EQ(0., controller->residual());
  EXPECT_NEAR(command, controller->g() * 100., 1e-9);
}

TEST_F(Impedance, SupportedPoseCannotRemoveGateOrLoosenConfiguredToleranceAtRuntime) {
  ASSERT_EQ(CallbackReturn::SUCCESS, configure({
    rclcpp::Parameter("require_activation_pose", true),
    rclcpp::Parameter("activation_pose", std::vector<double>{0.0})}));
  EXPECT_FALSE(controller->get_node()->set_parameters_atomically({
    rclcpp::Parameter("require_activation_pose", false),
    rclcpp::Parameter("activation_pose", std::vector<double>{})}).successful);
  EXPECT_TRUE(controller->get_node()->get_parameter("require_activation_pose").as_bool());
  q = .1;
  EXPECT_EQ(CallbackReturn::ERROR, activate());
  controller->release_interfaces();
  EXPECT_FALSE(controller->get_node()->set_parameters_atomically({
    rclcpp::Parameter("activation_pose", std::vector<double>{0.0}),
    rclcpp::Parameter("activation_pose_tolerance", 1.0)}).successful);
  q = .1;
  EXPECT_EQ(CallbackReturn::ERROR, activate());
  EXPECT_DOUBLE_EQ(0., command);
}

TEST_F(Impedance, ConfirmedSupportedPoseStillRequiresLimitsSpeedHealthAndGravityCapacity) {
  ASSERT_EQ(CallbackReturn::SUCCESS, configure({
    rclcpp::Parameter("require_activation_pose", true),
    rclcpp::Parameter("activation_pose", std::vector<double>{0.0})}));
  v = .06;
  EXPECT_EQ(CallbackReturn::ERROR, activate());
  controller->release_interfaces(); v = 0.; q = 2.0;
  controller->get_node()->set_parameter(rclcpp::Parameter("activation_pose", std::vector<double>{2.0}));
  EXPECT_EQ(CallbackReturn::ERROR, activate());
  controller->release_interfaces(); q = 0.;
  controller->get_node()->set_parameter(rclcpp::Parameter("activation_pose", std::vector<double>{0.0}));
  controller->mockHealth(0, true);
  EXPECT_EQ(CallbackReturn::ERROR, activate());
  EXPECT_DOUBLE_EQ(0., command);
}

TEST_F(Impedance, BlockedArmKeepsGoalWithoutWindupAndResumesWithinBudget) {
  ASSERT_EQ(CallbackReturn::SUCCESS, configure()); ASSERT_EQ(CallbackReturn::SUCCESS, activate()); ramp();
  for (int i = 0; i < 500; ++i) {
    controller->receiveGoal(message(1.0)); step();
    EXPECT_LE(std::abs(command / 100 - controller->g()), controller->budget() + 1e-9);
  }
  EXPECT_DOUBLE_EQ(1, controller->target()); EXPECT_NEAR(.1, controller->reference(), 1e-9);
  EXPECT_FALSE(controller->fault());
  q = .05; controller->receiveGoal(message()); step();
  EXPECT_GT(controller->reference(), .1); EXPECT_LE(controller->reference() - q, .1);
  EXPECT_LE(std::abs(command), 150);
}

TEST_F(Impedance, TimeoutReturnsToGravityAndNewCommandMustBeFresh) {
  ASSERT_EQ(CallbackReturn::SUCCESS, configure()); ASSERT_EQ(CallbackReturn::SUCCESS, activate()); ramp();
  controller->receiveGoal(message()); step(); EXPECT_GT(controller->residual(), 0);
  controller->expireGoal(); step(); EXPECT_EQ(2, controller->status());
  EXPECT_DOUBLE_EQ(0, controller->residual()); EXPECT_DOUBLE_EQ(1, controller->target());
  EXPECT_NEAR(controller->g() * 100, command, 1e-9);
  controller->receiveGoal(message()); step(); EXPECT_EQ(1, controller->status());
}

TEST_F(Impedance, WaitingDampsBothDirectionsWithoutSpringOrFriction) {
  ASSERT_EQ(CallbackReturn::SUCCESS, configure({rclcpp::Parameter("friction_compensation", true)}));
  ASSERT_EQ(CallbackReturn::SUCCESS, activate()); ramp();
  for (double velocity : {.2, -.2, .001, -.001, 0.}) {
    q = .5; v = velocity;  // Deliberately displaced from captured target=0.
    for (int i = 0; i < 10; ++i) {
      step();
      EXPECT_EQ(0, controller->status()); EXPECT_FALSE(controller->fault());
      EXPECT_LE(controller->residual() * v, 0.);
      EXPECT_LE(std::abs(controller->residual()), controller->budget());
      EXPECT_LE(std::abs(controller->residual()), .2 * std::abs(v) + 1e-12);
      EXPECT_DOUBLE_EQ(q, controller->reference());
      EXPECT_DOUBLE_EQ(0., controller->target());
    }
    EXPECT_NEAR(-.2 * v, controller->residual(), 1e-12);
    EXPECT_NEAR((controller->g() + controller->residual()) * 100, command, 1e-9);
  }
}

TEST_F(Impedance, IdleDampingRampsAndIncreasesWithinSlewLimit) {
  ASSERT_EQ(CallbackReturn::SUCCESS, configure());
  ASSERT_EQ(CallbackReturn::SUCCESS, activate()); v = .2;
  double previous = 0;
  for (int i = 1; i <= 12; ++i) {
    step();
    const double fraction = std::min(1., .1 * i);
    EXPECT_LE(std::abs(controller->residual()), fraction * .2 * std::abs(v) + 1e-12);
    EXPECT_LE(std::abs(controller->residual() - previous), .01 + 1e-12);
    EXPECT_LE(std::abs(controller->residual()), fraction * controller->budget() + 1e-12);
    EXPECT_LE(controller->residual() * v, 0.);
    previous = controller->residual();
  }
  EXPECT_NEAR(-.04, controller->residual(), 1e-12);
}

TEST_F(Impedance, TimeoutDiscardsSpringImmediatelyAndRetainsOnlyOpposingDamping) {
  ASSERT_EQ(CallbackReturn::SUCCESS, configure()); ASSERT_EQ(CallbackReturn::SUCCESS, activate()); ramp();
  for (int i = 0; i < 30; ++i) {controller->receiveGoal(message()); step();}
  ASSERT_GT(controller->residual(), 0.);
  v = .2; controller->expireGoal(); step();
  EXPECT_EQ(2, controller->status()); EXPECT_LT(controller->residual(), 0.);
  EXPECT_GE(controller->residual(), -.01 - 1e-12);
  EXPECT_DOUBLE_EQ(1., controller->target()); EXPECT_DOUBLE_EQ(q, controller->reference());
  ramp(); EXPECT_NEAR(-.04, controller->residual(), 1e-12);
  v = -.2; step(); EXPECT_GT(controller->residual(), 0.);
  EXPECT_LE(controller->residual(), .01 + 1e-12);
  v = 0.; step(); EXPECT_DOUBLE_EQ(0., controller->residual());
  EXPECT_EQ(2, controller->status());  // No replay or auto-resume of old goal.
}

TEST_F(Impedance, IdleDampingRespectsZeroBudgetAndAbsoluteCaps) {
  for (double extra : {0., .01}) {
    if (controller) {controller->release_interfaces(); controller.reset();}
    ASSERT_EQ(CallbackReturn::SUCCESS, configure({
      rclcpp::Parameter("extra_torque_max", std::vector<double>{extra}),
      rclcpp::Parameter("damping", std::vector<double>{100.})}));
    ASSERT_EQ(CallbackReturn::SUCCESS, activate()); ramp(); v = .2;
    for (int i = 0; i < 10; ++i) {
      step(); EXPECT_LE(std::abs(controller->residual()), extra + 1e-12);
      EXPECT_LE(std::abs(command), 150.); EXPECT_FALSE(controller->fault());
    }
    EXPECT_NEAR(-extra, controller->residual(), 1e-12);
    v = 0.; step(); EXPECT_DOUBLE_EQ(0., controller->residual());
  }
}

TEST_F(Impedance, IdleDampingCannotBypassLatchedFaultOrDeactivation) {
  ASSERT_EQ(CallbackReturn::SUCCESS, configure()); ASSERT_EQ(CallbackReturn::SUCCESS, activate()); ramp();
  v = .2; ramp(); ASSERT_LT(controller->residual(), 0.);
  v = .5; step(); EXPECT_TRUE(controller->fault()); EXPECT_EQ(0, controller->residual());
  v = .2; step(); EXPECT_EQ(3, controller->status()); EXPECT_EQ(0, controller->residual());
  EXPECT_NEAR(controller->g() * 100, command, 1e-9);
  controller->on_deactivate(rclcpp_lifecycle::State()); EXPECT_EQ(0, command);
}

TEST_F(Impedance, IdleDampingShrinkingBudgetOverridesSlewAndZeroGainMeansZero) {
  ASSERT_EQ(CallbackReturn::SUCCESS, configure({
    rclcpp::Parameter("damping", std::vector<double>{100.})}));
  ASSERT_EQ(CallbackReturn::SUCCESS, activate()); ramp(); v = .2;
  for (int i = 0; i < 30; ++i) {step();}
  const double previous = std::abs(controller->residual());
  q = 1.8; step();
  EXPECT_LT(controller->budget(), previous - .01);
  EXPECT_LE(std::abs(controller->residual()), controller->budget() + 1e-12);
  EXPECT_LE(controller->residual() * v, 0.);
  controller->release_interfaces(); controller.reset(); v = 0.; q = 0.;
  ASSERT_EQ(CallbackReturn::SUCCESS, configure({
    rclcpp::Parameter("damping", std::vector<double>{0.})}));
  ASSERT_EQ(CallbackReturn::SUCCESS, activate()); ramp(); v = .2; step();
  EXPECT_DOUBLE_EQ(0., controller->residual());
}

TEST_F(Impedance, IdleDampingStopsOnStaleHealthAndDoesNotReenergize) {
  ASSERT_EQ(CallbackReturn::SUCCESS, configure()); controller->mockHealth();
  ASSERT_EQ(CallbackReturn::SUCCESS, activate()); ramp(); v = .2; step();
  ASSERT_LT(controller->residual(), 0.);
  controller->mockHealth(0, true); step(); EXPECT_EQ(0, command);
  controller->mockHealth(); step(); EXPECT_EQ(0, command);
  EXPECT_EQ(0, controller->residual()); EXPECT_TRUE(controller->fault());
}

TEST_F(Impedance, InvalidTargetLatchesFaultEvenWhenFollowedByValidOne) {
  ASSERT_EQ(CallbackReturn::SUCCESS, configure()); ASSERT_EQ(CallbackReturn::SUCCESS, activate()); ramp();
  controller->receiveGoal(message(2)); controller->receiveGoal(message(1)); step();
  EXPECT_TRUE(controller->fault()); EXPECT_EQ(0, controller->residual());
}

TEST_F(Impedance, RejectsStaleFutureNanAndWrongJointCommands) {
  for (int kind = 0; kind < 5; ++kind) {
    if (controller) {controller->release_interfaces(); controller.reset();}
    ASSERT_EQ(CallbackReturn::SUCCESS, configure()); ASSERT_EQ(CallbackReturn::SUCCESS, activate()); ramp();
    auto msg = message();
    if (kind == 0) {msg.header.stamp = rclcpp::Time(1);}
    if (kind == 1) {msg.header.stamp = controller->get_node()->now() + rclcpp::Duration::from_seconds(1);}
    if (kind == 2) {msg.position[0] = NAN;}
    if (kind == 3) {msg.name[0] = "wrong";}
    if (kind == 4) {msg.velocity = {1.0};}
    controller->receiveGoal(msg); step(); EXPECT_TRUE(controller->fault());
  }
}

TEST_F(Impedance, StateNanWritesZeroAndNeverAutomaticallyReenergizes) {
  ASSERT_EQ(CallbackReturn::SUCCESS, configure()); ASSERT_EQ(CallbackReturn::SUCCESS, activate()); ramp();
  q = NAN; step(); EXPECT_EQ(0, command); EXPECT_TRUE(controller->fault());
  q = 0; controller->receiveGoal(message()); step(); EXPECT_EQ(0, command);
}

TEST_F(Impedance, OverspeedAndUpdateOverrunRemoveMotionTorque) {
  ASSERT_EQ(CallbackReturn::SUCCESS, configure()); ASSERT_EQ(CallbackReturn::SUCCESS, activate()); ramp();
  controller->receiveGoal(message()); v = .5; step(.1);
  EXPECT_TRUE(controller->fault()); EXPECT_EQ(0, controller->residual());
  EXPECT_NEAR(controller->g() * 100, command, 1e-9);
}

TEST_F(Impedance, FrictionCannotBypassResidualLimitAndZeroMeansZero) {
  ASSERT_EQ(CallbackReturn::SUCCESS, configure({rclcpp::Parameter("friction_compensation", true),
    rclcpp::Parameter("extra_torque_max", std::vector<double>{0.0})}));
  ASSERT_EQ(CallbackReturn::SUCCESS, activate()); ramp(); v = .2;
  controller->receiveGoal(message()); step(); EXPECT_EQ(0, controller->residual());
}

TEST_F(Impedance, ActivationGateAndMockOptInAreMandatory) {
  ASSERT_EQ(CallbackReturn::SUCCESS, configure({rclcpp::Parameter("enable_on_activation", false)}));
  EXPECT_EQ(CallbackReturn::ERROR, activate());
  controller->release_interfaces(); controller.reset();
  EXPECT_EQ(CallbackReturn::ERROR, configure({rclcpp::Parameter("allow_mock_hardware", false)}));
}

TEST_F(Impedance, InvalidGainScaleLimitAndRateFailConfiguration) {
  for (const auto & key : {"effort_scale", "max_effort", "reference_velocity", "residual_torque_rate"}) {
    controller.reset();
    EXPECT_EQ(CallbackReturn::ERROR, configure({rclcpp::Parameter(key, std::vector<double>{0.0})}));
  }
  controller.reset();
  EXPECT_EQ(CallbackReturn::ERROR, configure({rclcpp::Parameter("damping", std::vector<double>{NAN})}));
}

TEST_F(Impedance, CannotChangeGainsLive) {
  ASSERT_EQ(CallbackReturn::SUCCESS, configure());
  EXPECT_FALSE(controller->get_node()->set_parameter(rclcpp::Parameter("stiffness", std::vector<double>{200.0})).successful);
}

TEST_F(Impedance, ReactivationCannotReplayLastSessionTarget) {
  ASSERT_EQ(CallbackReturn::SUCCESS, configure()); ASSERT_EQ(CallbackReturn::SUCCESS, activate()); ramp();
  controller->receiveGoal(message()); step();
  controller->on_deactivate(rclcpp_lifecycle::State()); EXPECT_EQ(0, command);
  controller->release_interfaces(); q = .3;
  ASSERT_EQ(CallbackReturn::SUCCESS, activate()); ramp();
  EXPECT_DOUBLE_EQ(.3, controller->target()); EXPECT_EQ(0, controller->residual());
}

TEST_F(Impedance, HealthParserRejectsReplayMissingFieldsAndPersistsFailureCounters) {
  ASSERT_EQ(CallbackReturn::SUCCESS, configure());
  auto msg = health(); controller->receiveHealth(msg); EXPECT_TRUE(controller->reportedHealthy());
  auto faults = controller->healthFaults();
  controller->receiveHealth(msg); EXPECT_FALSE(controller->reportedHealthy()); EXPECT_GT(controller->healthFaults(), faults);
  msg = health(); msg.status[0].values.pop_back(); controller->receiveHealth(msg); EXPECT_FALSE(controller->reportedHealthy());
  controller->receiveHealth(health()); faults = controller->healthFaults();
  controller->receiveHealth(health(1)); EXPECT_GT(controller->healthFaults(), faults);
}

TEST_F(Impedance, StaleHealthLatchesZeroAndCounterChangeLatchesMotionFault) {
  ASSERT_EQ(CallbackReturn::SUCCESS, configure()); controller->mockHealth();
  ASSERT_EQ(CallbackReturn::SUCCESS, activate()); ramp();
  controller->mockHealth(1); controller->receiveGoal(message()); step(); EXPECT_TRUE(controller->fault());
  controller->mockHealth(1, true); step(); EXPECT_EQ(0, command);
  controller->mockHealth(1); step(); EXPECT_EQ(0, command);
}

TEST_F(Impedance, CannotActivateWithoutHealthOrWithInsufficientGravityCapacity) {
  ASSERT_EQ(CallbackReturn::SUCCESS, configure({rclcpp::Parameter("require_bus_health", true)}));
  EXPECT_EQ(CallbackReturn::ERROR, activate()); controller->release_interfaces(); controller.reset();
  ASSERT_EQ(CallbackReturn::SUCCESS, configure({rclcpp::Parameter("max_effort", std::vector<double>{1.0})}));
  EXPECT_EQ(CallbackReturn::ERROR, activate());
}

TEST_F(Impedance, RealV2DescriptionAcceptsModeZeroAndRejectsModeFive) {
  for (const auto & mode : {"0", "5"}) {
    const std::string cmd = std::string("xacro '") + TEST_BRINGUP_XACRO +
      "' current_control:=true use_fake_hardware:=false arm_operating_mode:=" + mode;
    FILE * pipe = popen(cmd.c_str(), "r"); ASSERT_NE(nullptr, pipe);
    std::string xml; char chunk[4096];
    while (const auto count = fread(chunk, 1, sizeof(chunk), pipe)) {xml.append(chunk, count);}
    ASSERT_EQ(0, pclose(pipe)); ASSERT_FALSE(xml.empty());
    controller = std::make_unique<Inspectable>();
    auto options = rclcpp::NodeOptions().context(context)
      .allow_undeclared_parameters(true).automatically_declare_parameters_from_overrides(true)
      .arguments({"--ros-args", "--params-file", TEST_IMPEDANCE_CONFIG})
      .parameter_overrides({rclcpp::Parameter("robot_description", xml)});
    ASSERT_EQ(controller_interface::return_type::OK,
      controller->init("om6dof_position_impedance_controller", "/offline", options));
    EXPECT_EQ(std::string(mode) == "0" ? CallbackReturn::SUCCESS : CallbackReturn::ERROR,
      controller->on_configure(rclcpp_lifecycle::State()));
    controller.reset();
  }
}

TEST(ImpedancePlugin, ExportCanBeDiscoveredAndConstructedWithoutHardware) {
  pluginlib::ClassLoader<controller_interface::ControllerInterface> loader(
    "controller_interface", "controller_interface::ControllerInterface");
  EXPECT_TRUE(loader.isClassAvailable("om6dof_controllers/PositionImpedanceController"));
  EXPECT_NE(nullptr, loader.createSharedInstance("om6dof_controllers/PositionImpedanceController"));
}

TEST_F(Impedance, NoResidualUntilRampCompleteAndTorqueRateIsBounded) {
  ASSERT_EQ(CallbackReturn::SUCCESS, configure()); ASSERT_EQ(CallbackReturn::SUCCESS, activate());
  controller->receiveGoal(message()); step(); EXPECT_EQ(0, controller->residual());
  ramp(); double previous = controller->residual();
  for (int i = 0; i < 50; ++i) {
    controller->receiveGoal(message()); step();
    EXPECT_LE(std::abs(controller->residual() - previous), .01 + 1e-12);
    previous = controller->residual();
  }
}

TEST_F(Impedance, GravityCapacityLossTripsAndStillHonorsAbsoluteCap) {
  ASSERT_EQ(CallbackReturn::SUCCESS, configure({rclcpp::Parameter("max_effort", std::vector<double>{50.0})}));
  q = 1.8; ASSERT_EQ(CallbackReturn::SUCCESS, activate()); ramp();
  q = 0; controller->receiveGoal(message()); step();
  EXPECT_TRUE(controller->fault()); EXPECT_TRUE(controller->flags() & 64);
  EXPECT_DOUBLE_EQ(0, controller->residual()); EXPECT_LE(std::abs(command), 50.0);
}
