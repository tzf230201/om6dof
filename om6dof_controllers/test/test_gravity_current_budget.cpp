// Copyright 2026 OM6DOF maintainers.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// Offline: only the controller and stack-backed doubles; no hardware plugin or serial access.
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <limits>
#include <thread>
#include <gtest/gtest.h>
#include "hardware_interface/handle.hpp"
#include "om6dof_controllers/gravity_current_budget_controller.hpp"

using controller_interface::CallbackReturn;
using om6dof_controllers::GravityCurrentBudgetController;

class TestBudget : public GravityCurrentBudgetController
{
public:
  using GravityCurrentBudgetController::receiveReference;
  const auto & packet() const {return observer_snapshot_;}
};

class GravityBudget : public ::testing::Test
{
protected:
  void SetUp() override {
    context = std::make_shared<rclcpp::Context>();
    rclcpp::InitOptions init; init.set_domain_id(194); context->init(0, nullptr, init);
    const std::string command = std::string("xacro '") + TEST_BRINGUP_XACRO +
      "' gravity_current_budget:=true";
    FILE * pipe = popen(command.c_str(), "r"); ASSERT_NE(nullptr, pipe);
    char buffer[4096];
    while (const auto size = fread(buffer, 1, sizeof(buffer), pipe)) {xml.append(buffer, size);}
    ASSERT_EQ(0, pclose(pipe));
    controller = std::make_unique<TestBudget>();
    ASSERT_EQ(controller_interface::return_type::OK, controller->init("budget", "/offline",
      rclcpp::NodeOptions().context(context).parameter_overrides({
        rclcpp::Parameter("robot_description", xml),
        rclcpp::Parameter("current_slew_raw_per_second", std::vector<double>(6, 1.0e6))})));
    ASSERT_EQ(CallbackReturn::SUCCESS, controller->on_configure(rclcpp_lifecycle::State()));
  }
  void TearDown() override {
    controller->release_interfaces(); controller.reset(); context->shutdown("offline complete");
  }
  CallbackReturn activate() {
    command_handles.reserve(6); state_handles.reserve(25);
    for (int i = 0; i < 6; ++i) {
      command_handles.emplace_back("joint" + std::to_string(i + 1), "effort", &current[i]);
      state_handles.emplace_back("joint" + std::to_string(i + 1), "position", &q[i]);
      state_handles.emplace_back("joint" + std::to_string(i + 1), "velocity", &velocity[i]);
      state_handles.emplace_back("joint" + std::to_string(i + 1), "effort", &measured_current[i]);
      state_handles.emplace_back("dxl" + std::to_string(i + 1), "stored_current_limit", &limits[i]);
    }
    state_handles.emplace_back("gravity_budget", "read_ok", &healthy);
    std::vector<hardware_interface::LoanedCommandInterface> commands;
    std::vector<hardware_interface::LoanedStateInterface> states;
    // Reverse ordering verifies name-based mapping.
    for (auto it = command_handles.rbegin(); it != command_handles.rend(); ++it) {commands.emplace_back(*it);}
    for (auto it = state_handles.rbegin(); it != state_handles.rend(); ++it) {states.emplace_back(*it);}
    controller->assign_interfaces(std::move(commands), std::move(states));
    return controller->on_activate(rclcpp_lifecycle::State());
  }
  void step(double dt = 0.01) {
    EXPECT_EQ(controller_interface::return_type::OK, controller->update(
      controller->get_node()->now(), rclcpp::Duration::from_seconds(dt)));
  }
  void ramp() {for (int i = 0; i < 105; ++i) {step();}}
  void expectZero() {for (auto value : current) {EXPECT_EQ(0.0, value);}}
  void configure(const std::vector<rclcpp::Parameter> & parameters) {
    for (const auto & result : controller->get_node()->set_parameters(parameters)) {ASSERT_TRUE(result.successful);}
    ASSERT_EQ(CallbackReturn::SUCCESS, controller->on_configure(rclcpp_lifecycle::State()));
  }
  control_msgs::msg::JointTrajectoryControllerState reference(double velocity = 0.0, double acceleration = 0.0) {
    control_msgs::msg::JointTrajectoryControllerState message;
    message.header.stamp = controller->get_node()->now();
    message.joint_names = {"joint6", "joint5", "joint4", "joint3", "joint2", "joint1"};
    message.reference.positions.assign(6, 0.0);
    message.reference.velocities.assign(6, velocity);
    message.reference.accelerations.assign(6, acceleration);
    return message;
  }
  std::shared_ptr<rclcpp::Context> context;
  std::unique_ptr<TestBudget> controller;
  std::string xml;
  std::array<double, 6> current{}, q{{0.0, -1.0472, 0.2618, 0.0, 1.0472, 0.0}};
  std::array<double, 6> velocity{}, measured_current{};
  std::array<double, 6> limits{{93, 297, 260, 112, 130, 74}};
  double healthy{1.0};
  std::vector<hardware_interface::CommandInterface> command_handles;
  std::vector<hardware_interface::StateInterface> state_handles;
};

TEST_F(GravityBudget, TracksMeasuredPoseWithGravityMagnitudeAndStoredCaps) {
  ASSERT_EQ(CallbackReturn::SUCCESS, activate()); expectZero(); ramp();
  const auto before = current;
  om6dof_controllers::GravityModel model;
  ASSERT_TRUE(model.configure(xml, "link1", "end_effector_link",
    {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6"}, {0, 0, -9.80665}).empty());
  for (double angle : {-0.7, -0.4}) {
    q[1] = angle; step();
    std::vector<double> g(6); ASSERT_TRUE(model.computeChecked({q.begin(), q.end()}, g));
    const std::array<double, 6> scale{{208.541119, 208.541119, 208.541119,
      285.006196, 208.541119, 285.006196}};
    for (size_t i = 0; i < 6; ++i) {
      EXPECT_GE(current[i], 0); EXPECT_LE(current[i], limits[i]);
      EXPECT_EQ(std::floor(std::min(limits[i], std::abs(g[i]) * scale[i])), current[i]);
    }
  }
  EXPECT_NE(before, current);
  EXPECT_EQ(0.0, current[0]);  // Vertical gravity axis: no invented current floor.
  controller->on_deactivate(rclcpp_lifecycle::State()); expectZero();
}

TEST_F(GravityBudget, FailedReadLatchesZeroAndDoesNotResumeAutomatically) {
  ASSERT_EQ(CallbackReturn::SUCCESS, activate()); ramp(); healthy = 0; step(); expectZero();
  healthy = 1; ramp(); expectZero();
}

TEST_F(GravityBudget, InvalidEncoderLatchesZero) {
  ASSERT_EQ(CallbackReturn::SUCCESS, activate()); ramp();
  q[2] = std::numeric_limits<double>::quiet_NaN(); step(); expectZero();
  q[2] = 0.3; step(); expectZero();
}

TEST_F(GravityBudget, OutOfBoundsPoseAndDelayedCycleFailClosed) {
  ASSERT_EQ(CallbackReturn::SUCCESS, activate()); ramp(); q[1] = 100; step(); expectZero();
}

TEST_F(GravityBudget, DelayedCycleFailsClosed) {
  ASSERT_EQ(CallbackReturn::SUCCESS, activate()); ramp(); step(0.2); expectZero();
}

TEST_F(GravityBudget, MissingCeilingRefusesActivation) {
  limits[2] = 0; ASSERT_EQ(CallbackReturn::ERROR, activate()); expectZero();
}

TEST_F(GravityBudget, OnlyClaimsCurrentNotPosition) {
  for (const auto & name : controller->command_interface_configuration().names) {
    EXPECT_NE(std::string::npos, name.find("/effort"));
    EXPECT_EQ(std::string::npos, name.find("position"));
  }
}

TEST_F(GravityBudget, HoldingReserveEnablesVerticalAxisAndNeverExceedsUserCap) {
  configure({rclcpp::Parameter("holding_current_raw", std::vector<double>(6, 25.0)),
    rclcpp::Parameter("max_current_raw", std::vector<double>(6, 20.0))});
  ASSERT_EQ(CallbackReturn::SUCCESS, activate()); expectZero(); ramp();
  EXPECT_EQ(20.0, current[0]);
  for (const auto raw : current) {EXPECT_LE(raw, 20.0);}
}

TEST_F(GravityBudget, ScaledGravityAndHoldingRespectUrdfTorqueLimitAsWellAsStoredLimit) {
  configure({rclcpp::Parameter("gravity_scale", 1.2),
    rclcpp::Parameter("holding_current_raw", std::vector<double>(6, 20.0))});
  ASSERT_EQ(CallbackReturn::SUCCESS, activate()); ramp();
  om6dof_controllers::GravityModel model;
  ASSERT_TRUE(model.configure(xml, "link1", "end_effector_link",
    {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6"}, {0, 0, -9.80665}).empty());
  std::vector<double> g(6); ASSERT_TRUE(model.computeChecked({q.begin(), q.end()}, g));
  const std::array<double, 6> scale{{208.541119, 208.541119, 208.541119,
    285.006196, 208.541119, 285.006196}};
  for (size_t i = 0; i < 6; ++i) {
    // V2 URDF declares 1 Nm per arm joint. Respect it even when the stored
    // current register and commissioning cap would permit more.
    const double expected = std::floor(std::min({limits[i], scale[i],
      1.2 * std::abs(g[i]) * scale[i] + 20.0}));
    EXPECT_EQ(expected, current[i]);
  }
}

TEST_F(GravityBudget, ReferenceSpeedAndAccelerationAddBoundedReserveInEitherDirection) {
  configure({rclcpp::Parameter("holding_current_raw", std::vector<double>(6, 10.0)),
    rclcpp::Parameter("velocity_current_gain", std::vector<double>(6, 100.0)),
    rclcpp::Parameter("acceleration_current_gain", std::vector<double>(6, 20.0))});
  ASSERT_EQ(CallbackReturn::SUCCESS, activate()); ramp(); EXPECT_EQ(10.0, current[0]);
  for (double direction : {-1.0, 1.0}) {
    controller->receiveReference(reference(0.1 * direction, 0.5 * direction)); step();
    EXPECT_EQ(30.0, current[0]);
  }
  controller->receiveReference(reference(100.0, 100.0)); step();
  EXPECT_EQ(limits[0], current[0]);
  controller->receiveReference(reference()); step(); EXPECT_EQ(10.0, current[0]);
}

TEST_F(GravityBudget, MapsNamedReferenceAndAcceptsLegacyDesiredWithoutAcceleration) {
  configure({rclcpp::Parameter("velocity_current_gain", std::vector<double>(6, 100.0))});
  ASSERT_EQ(CallbackReturn::SUCCESS, activate()); ramp();
  auto msg = reference(); msg.desired = msg.reference;
  msg.reference = trajectory_msgs::msg::JointTrajectoryPoint();
  msg.desired.accelerations.clear(); msg.desired.velocities[5] = 0.2;
  controller->receiveReference(msg); step(); EXPECT_EQ(20.0, current[0]);
}

TEST_F(GravityBudget, FreshDirectReferenceHasPriorityOverAlternatingJtcPackets) {
  configure({rclcpp::Parameter("holding_current_raw", std::vector<double>(6, 10.0)),
    rclcpp::Parameter("velocity_current_gain", std::vector<double>(6, 100.0))});
  ASSERT_EQ(CallbackReturn::SUCCESS, activate()); ramp();
  controller->receiveReference(reference(0.2), true); step(); EXPECT_EQ(30.0, current[0]);
  controller->receiveReference(reference()); step(); EXPECT_EQ(30.0, current[0]);
  controller->receiveReference(reference(0.15), true); step(); EXPECT_EQ(25.0, current[0]);
  auto invalid_jtc = reference(); invalid_jtc.reference.velocities.clear();
  controller->receiveReference(invalid_jtc); step(); EXPECT_EQ(25.0, current[0]);
  controller->receiveReference(reference()); step(); EXPECT_EQ(25.0, current[0]);
}

TEST_F(GravityBudget, InvalidDirectReferenceFallsBackToFreshJtcOnly) {
  configure({rclcpp::Parameter("holding_current_raw", std::vector<double>(6, 10.0)),
    rclcpp::Parameter("velocity_current_gain", std::vector<double>(6, 100.0))});
  ASSERT_EQ(CallbackReturn::SUCCESS, activate()); ramp();
  controller->receiveReference(reference(0.1));
  controller->receiveReference(reference(0.2), true); step(); EXPECT_EQ(30.0, current[0]);
  auto invalid_direct = reference(0.2); invalid_direct.header.stamp.sec = -1;
  controller->receiveReference(invalid_direct, true); step(); EXPECT_EQ(20.0, current[0]);
  controller->receiveReference(reference(0.2), true); step(); EXPECT_EQ(30.0, current[0]);
  invalid_direct = reference(0.2); invalid_direct.reference.velocities.clear();
  controller->receiveReference(invalid_direct, true); step(); EXPECT_EQ(20.0, current[0]);
}

TEST_F(GravityBudget, ExpiredDirectReferenceFallsBackWithoutLeakingReserve) {
  configure({rclcpp::Parameter("holding_current_raw", std::vector<double>(6, 10.0)),
    rclcpp::Parameter("velocity_current_gain", std::vector<double>(6, 100.0)),
    rclcpp::Parameter("reference_timeout_seconds", 0.005)});
  ASSERT_EQ(CallbackReturn::SUCCESS, activate()); ramp();
  controller->receiveReference(reference(0.2), true); step(); EXPECT_EQ(30.0, current[0]);
  std::this_thread::sleep_for(std::chrono::milliseconds(10));
  controller->receiveReference(reference(0.1)); step(); EXPECT_EQ(20.0, current[0]);
  std::this_thread::sleep_for(std::chrono::milliseconds(10)); step(); EXPECT_EQ(10.0, current[0]);
}

TEST_F(GravityBudget, InvalidAndOldReferencesRemoveOnlyMotionReserve) {
  configure({rclcpp::Parameter("holding_current_raw", std::vector<double>(6, 10.0)),
    rclcpp::Parameter("velocity_current_gain", std::vector<double>(6, 100.0))});
  ASSERT_EQ(CallbackReturn::SUCCESS, activate()); ramp();
  for (int invalid = 0; invalid < 8; ++invalid) {
    controller->receiveReference(reference(0.2)); step(); EXPECT_EQ(30.0, current[0]);
    auto msg = reference(0.2);
    if (invalid == 0) {msg.joint_names[0] = "joint1";}
    if (invalid == 1) {msg.reference.velocities.clear();}
    if (invalid == 2) {msg.reference.velocities[0] = std::numeric_limits<double>::quiet_NaN();}
    if (invalid == 3) {msg.reference.accelerations.pop_back();}
    if (invalid == 4) {msg.header.stamp.sec -= 2;}
    if (invalid == 5) {msg.header.stamp.sec += 2;}
    if (invalid == 6) {msg.header.stamp.sec = -1;}
    if (invalid == 7) {msg.header.stamp.nanosec = 1000000000U;}
    controller->receiveReference(msg); step(); EXPECT_EQ(10.0, current[0]);
  }
}

TEST_F(GravityBudget, ReferenceExpiresAndPreActivationReferenceIsDiscarded) {
  configure({rclcpp::Parameter("holding_current_raw", std::vector<double>(6, 10.0)),
    rclcpp::Parameter("velocity_current_gain", std::vector<double>(6, 100.0)),
    rclcpp::Parameter("reference_timeout_seconds", 0.005)});
  controller->receiveReference(reference(0.2));
  ASSERT_EQ(CallbackReturn::SUCCESS, activate()); ramp(); EXPECT_EQ(10.0, current[0]);
  controller->receiveReference(reference(0.2)); step(); EXPECT_EQ(30.0, current[0]);
  std::this_thread::sleep_for(std::chrono::milliseconds(10)); step();
  EXPECT_EQ(10.0, current[0]);
}

TEST_F(GravityBudget, SlewAccumulatesFractionalTicksAndReducedCeilingAppliesImmediately) {
  configure({rclcpp::Parameter("holding_current_raw", std::vector<double>(6, 100.0)),
    rclcpp::Parameter("current_slew_raw_per_second", std::vector<double>(6, 10.0))});
  ASSERT_EQ(CallbackReturn::SUCCESS, activate());
  for (int i = 0; i < 105; ++i) {step(0.001);}
  EXPECT_EQ(1.0, current[0]);
  for (int i = 0; i < 100; ++i) {step(0.01);}
  EXPECT_GE(current[0], 10.0); EXPECT_LE(current[0], 12.0);
  limits[0] = 5; step(); EXPECT_EQ(5.0, current[0]);
  healthy = 0; step(); expectZero();
}

TEST_F(GravityBudget, InvalidRuntimeStoredLimitLatchesZero) {
  ASSERT_EQ(CallbackReturn::SUCCESS, activate()); ramp();
  limits[2] = std::numeric_limits<double>::quiet_NaN(); step(); expectZero();
  limits[2] = 260; ramp(); expectZero();
}

TEST_F(GravityBudget, RejectsInvalidMarginParameters) {
  for (const auto & parameter : {
      rclcpp::Parameter("holding_current_raw", std::vector<double>(5, 10.0)),
      rclcpp::Parameter("holding_current_raw", std::vector<double>(6, -1.0)),
      rclcpp::Parameter("gravity_scale", 3.0),
      rclcpp::Parameter("reference_timeout_seconds", 0.0),
      rclcpp::Parameter("current_slew_raw_per_second", std::vector<double>(6, 0.0))})
  {
    controller->get_node()->set_parameter(parameter);
    EXPECT_EQ(CallbackReturn::ERROR, controller->on_configure(rclcpp_lifecycle::State()));
    // Restore so each test case is independent.
    controller->get_node()->set_parameters({
      rclcpp::Parameter("holding_current_raw", std::vector<double>(6, 0.0)),
      rclcpp::Parameter("gravity_scale", 1.0),
      rclcpp::Parameter("reference_timeout_seconds", 0.25),
      rclcpp::Parameter("current_slew_raw_per_second", std::vector<double>(6, 1000.0))});
  }
}

TEST_F(GravityBudget, ObserverUsesPhysicalUnscaledDynamicsAndSignedRawCurrent) {
  configure({rclcpp::Parameter("holding_current_raw", std::vector<double>(6, 20.0)),
    rclcpp::Parameter("gravity_scale", 1.2)});
  ASSERT_EQ(CallbackReturn::SUCCESS, activate()); step();
  EXPECT_EQ(1.0, controller->packet()[1]);  // Startup is not force-valid.
  ramp(); EXPECT_EQ(0.0, controller->packet()[1]);
  om6dof_controllers::GravityModel physical;
  ASSERT_TRUE(physical.configure(xml, "link1", "end_effector_link",
    {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6"}, {0, 0, -9.80665}).empty());
  std::vector<double> g(6); ASSERT_TRUE(physical.computeChecked({q.begin(), q.end()}, g));
  for (size_t i = 0; i < 6; ++i) {EXPECT_NEAR(g[i], controller->packet()[20 + i], 1e-10);}
  measured_current[0] = -5.0; step();
  EXPECT_EQ(-5.0, controller->packet()[14]);  // Raw signed count, no mA reinterpretation.
  EXPECT_EQ(q[0], controller->packet()[2]);
  healthy = 0; step(); EXPECT_EQ(2.0, controller->packet()[1]); expectZero();
}

TEST_F(GravityBudget, ObserverRejectsInvalidCurrentAndMarksSaturationWithoutInventingContact) {
  configure({rclcpp::Parameter("holding_current_raw", std::vector<double>(6, 20.0))});
  ASSERT_EQ(CallbackReturn::SUCCESS, activate()); ramp();
  measured_current[2] = std::numeric_limits<double>::quiet_NaN(); step();
  EXPECT_EQ(3.0, controller->packet()[1]); EXPECT_GT(current[2], 0.0);
  measured_current[2] = 0.0; step(); EXPECT_EQ(0.0, controller->packet()[1]);
  measured_current[0] = current[0]; step(); EXPECT_EQ(5.0, controller->packet()[1]);
}
