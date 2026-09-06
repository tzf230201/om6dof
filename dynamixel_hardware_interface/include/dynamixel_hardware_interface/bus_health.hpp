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

#ifndef DYNAMIXEL_HARDWARE_INTERFACE__BUS_HEALTH_HPP_
#define DYNAMIXEL_HARDWARE_INTERFACE__BUS_HEALTH_HPP_

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <map>
#include <utility>
#include <vector>

namespace dynamixel_hardware_interface
{

/**
 * @brief Bitwise OR of the latest Hardware Error Status byte from each actuator.
 */
inline std::uint32_t AggregateHardwareErrorStatus(
  const std::map<std::uint8_t, std::uint8_t> & statuses)
{
  std::uint32_t result = 0;
  for (const auto & entry : statuses) {
    result |= static_cast<std::uint32_t>(entry.second);
  }
  return result;
}

/**
 * @brief Whether every physical Dynamixel has a Hardware Error Status sample slot.
 *
 * The driver populates the slots while building its configured cyclic state
 * interfaces, before the health publisher is created.  Treating an absent
 * slot as healthy would make an empty map indistinguishable from seven clear
 * status registers.
 */
inline std::size_t HardwareErrorMonitoredCount(
  const std::vector<std::pair<std::uint8_t, std::uint8_t>> & actuators,
  const std::map<std::uint8_t, std::uint8_t> & statuses)
{
  std::size_t result = 0;
  for (const auto & actuator : actuators) {
    if (statuses.find(actuator.second) != statuses.end()) {
      ++result;
    }
  }
  return result;
}

inline bool HardwareErrorMonitoringComplete(
  const std::vector<std::pair<std::uint8_t, std::uint8_t>> & actuators,
  const std::map<std::uint8_t, std::uint8_t> & statuses)
{
  return !actuators.empty() &&
         HardwareErrorMonitoredCount(actuators, statuses) == actuators.size();
}

struct InputVoltageSummary
{
  std::size_t expected_count{0};
  std::size_t monitored_count{0};
  bool monitoring_complete{false};
  double minimum_voltage{std::numeric_limits<double>::quiet_NaN()};
  int minimum_voltage_id{-1};
};

/**
 * @brief Summarize finite, engineering-unit input-voltage feedback.
 */
inline InputVoltageSummary SummarizeInputVoltages(
  const std::vector<std::pair<std::uint8_t, std::uint8_t>> & actuators,
  const std::map<std::uint8_t, double> & voltages)
{
  InputVoltageSummary summary;
  summary.expected_count = actuators.size();
  for (const auto & actuator : actuators) {
    const auto value = voltages.find(actuator.second);
    if (value == voltages.end() || !std::isfinite(value->second)) {
      continue;
    }
    ++summary.monitored_count;
    if (summary.minimum_voltage_id < 0 || value->second < summary.minimum_voltage) {
      summary.minimum_voltage = value->second;
      summary.minimum_voltage_id = static_cast<int>(actuator.second);
    }
  }
  summary.monitoring_complete = !actuators.empty() &&
    summary.monitored_count == actuators.size();
  return summary;
}

/**
 * @brief Persistent communication counters exported on the bus-health topic.
 *
 * A transient error must remain observable after communication recovers.  A
 * consumer can therefore compare these counters over a clean window instead
 * of depending on one short-lived error sample reaching it.
 */
struct BusHealthState
{
  std::uint64_t read_failure_count{0};
  std::uint64_t write_failure_count{0};
  std::uint32_t consecutive_read_failures{0};
  std::uint32_t consecutive_write_failures{0};
  std::int32_t current_read_error{0};
  std::int32_t current_write_error{0};
  std::int32_t last_comm_error{0};
  std::int64_t last_failure_stamp_ns{0};
  bool fail_safe_triggered{false};

  void RecordReadFailure(std::int32_t error, std::int64_t stamp_ns)
  {
    IncrementSaturated(read_failure_count);
    IncrementSaturated(consecutive_read_failures);
    current_read_error = error;
    last_comm_error = error;
    last_failure_stamp_ns = stamp_ns;
  }

  void RecordWriteFailure(std::int32_t error, std::int64_t stamp_ns)
  {
    IncrementSaturated(write_failure_count);
    IncrementSaturated(consecutive_write_failures);
    current_write_error = error;
    last_comm_error = error;
    last_failure_stamp_ns = stamp_ns;
  }

  void RecordReadSuccess()
  {
    consecutive_read_failures = 0;
    current_read_error = 0;
  }

  void RecordWriteSuccess()
  {
    consecutive_write_failures = 0;
    current_write_error = 0;
  }

  bool CommunicationHealthy() const
  {
    return !fail_safe_triggered &&
           consecutive_read_failures == 0 &&
           consecutive_write_failures == 0 &&
           current_read_error == 0 && current_write_error == 0;
  }

private:
  template<typename CounterT>
  static void IncrementSaturated(CounterT & counter)
  {
    if (counter < std::numeric_limits<CounterT>::max()) {
      ++counter;
    }
  }
};

}  // namespace dynamixel_hardware_interface

#endif  // DYNAMIXEL_HARDWARE_INTERFACE__BUS_HEALTH_HPP_
