// Copyright 2026 KUBOTA Lab
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

#include <cstdint>
#include <limits>
#include <map>
#include <utility>
#include <vector>

#include "gtest/gtest.h"

#include "dynamixel_hardware_interface/bus_health.hpp"

namespace dynamixel_hardware_interface
{

TEST(BusHealthState, FailureRemainsObservableAfterRecovery)
{
  BusHealthState state;

  state.RecordReadFailure(-7, 1234);
  EXPECT_FALSE(state.CommunicationHealthy());
  EXPECT_EQ(state.read_failure_count, 1u);
  EXPECT_EQ(state.consecutive_read_failures, 1u);
  EXPECT_EQ(state.current_read_error, -7);
  EXPECT_EQ(state.last_comm_error, -7);
  EXPECT_EQ(state.last_failure_stamp_ns, 1234);

  state.RecordReadSuccess();
  EXPECT_TRUE(state.CommunicationHealthy());
  EXPECT_EQ(state.read_failure_count, 1u);
  EXPECT_EQ(state.consecutive_read_failures, 0u);
  EXPECT_EQ(state.current_read_error, 0);
  EXPECT_EQ(state.last_comm_error, -7);
  EXPECT_EQ(state.last_failure_stamp_ns, 1234);
}

TEST(BusHealthState, ReadAndWriteFailureChannelsAreIndependent)
{
  BusHealthState state;

  state.RecordReadFailure(-7, 100);
  state.RecordWriteFailure(-6, 200);
  state.RecordReadSuccess();

  EXPECT_FALSE(state.CommunicationHealthy());
  EXPECT_EQ(state.read_failure_count, 1u);
  EXPECT_EQ(state.write_failure_count, 1u);
  EXPECT_EQ(state.current_read_error, 0);
  EXPECT_EQ(state.current_write_error, -6);
  EXPECT_EQ(state.last_comm_error, -6);
  EXPECT_EQ(state.last_failure_stamp_ns, 200);

  state.RecordWriteSuccess();
  EXPECT_TRUE(state.CommunicationHealthy());
  EXPECT_EQ(state.write_failure_count, 1u);
}

TEST(BusHealthState, FailSafeLatchCannotLookHealthy)
{
  BusHealthState state;
  state.fail_safe_triggered = true;

  EXPECT_FALSE(state.CommunicationHealthy());
  state.RecordReadSuccess();
  state.RecordWriteSuccess();
  EXPECT_FALSE(state.CommunicationHealthy());
}

TEST(BusHealthState, CountersSaturateInsteadOfWrapping)
{
  BusHealthState state;
  state.read_failure_count = std::numeric_limits<std::uint64_t>::max();
  state.consecutive_read_failures = std::numeric_limits<std::uint32_t>::max();

  state.RecordReadFailure(-7, 1);

  EXPECT_EQ(state.read_failure_count, std::numeric_limits<std::uint64_t>::max());
  EXPECT_EQ(
    state.consecutive_read_failures, std::numeric_limits<std::uint32_t>::max());
}

TEST(HardwareErrorStatus, AggregatesEveryActuatorBit)
{
  const std::map<std::uint8_t, std::uint8_t> statuses = {
    {31, 0x00}, {32, 0x04}, {33, 0x20}, {24, 0x00},
    {35, 0x80}, {26, 0x00}, {37, 0x00}};

  EXPECT_EQ(AggregateHardwareErrorStatus(statuses), 0xA4u);
}

TEST(HardwareErrorStatus, EmptyOrPartialCoverageIsNotComplete)
{
  const std::vector<std::pair<std::uint8_t, std::uint8_t>> actuators = {
    {31, 31}, {32, 32}, {33, 33}, {24, 24}, {35, 35}, {26, 26}, {37, 37}};
  std::map<std::uint8_t, std::uint8_t> statuses;

  EXPECT_EQ(HardwareErrorMonitoredCount(actuators, statuses), 0u);
  EXPECT_FALSE(HardwareErrorMonitoringComplete(actuators, statuses));

  for (const auto & actuator : actuators) {
    statuses[actuator.second] = 0;
  }
  EXPECT_EQ(HardwareErrorMonitoredCount(actuators, statuses), actuators.size());
  EXPECT_TRUE(HardwareErrorMonitoringComplete(actuators, statuses));
  EXPECT_EQ(AggregateHardwareErrorStatus(statuses), 0u);
}

TEST(InputVoltage, ReportsCoverageAndMinimumInEngineeringUnits)
{
  const std::vector<std::pair<std::uint8_t, std::uint8_t>> actuators = {
    {31, 31}, {32, 32}, {37, 37}};
  const std::map<std::uint8_t, double> complete = {
    {31, 12.1}, {32, 11.8}, {37, 12.0}};

  const InputVoltageSummary summary = SummarizeInputVoltages(actuators, complete);
  EXPECT_EQ(summary.expected_count, 3u);
  EXPECT_EQ(summary.monitored_count, 3u);
  EXPECT_TRUE(summary.monitoring_complete);
  EXPECT_DOUBLE_EQ(summary.minimum_voltage, 11.8);
  EXPECT_EQ(summary.minimum_voltage_id, 32);

  const std::map<std::uint8_t, double> partial = {{31, 12.1}, {37, 12.0}};
  const InputVoltageSummary partial_summary =
    SummarizeInputVoltages(actuators, partial);
  EXPECT_EQ(partial_summary.monitored_count, 2u);
  EXPECT_FALSE(partial_summary.monitoring_complete);
}

}  // namespace dynamixel_hardware_interface
