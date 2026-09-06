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

#ifndef DYNAMIXEL_HARDWARE_INTERFACE__READ_PACKET_TIMEOUT_HPP_
#define DYNAMIXEL_HARDWARE_INTERFACE__READ_PACKET_TIMEOUT_HPP_

#include <cmath>
#include <cstddef>
#include <string>

namespace dynamixel_hardware_interface
{

// A failed read may block the ros2_control loop for this long, so reject
// accidental values that would either be physically implausible or hide a
// dead bus for an excessive time.
constexpr double kDefaultReadPacketTimeoutMs = 30.0;
constexpr double kMinimumReadPacketTimeoutMs = 5.0;
constexpr double kMaximumReadPacketTimeoutMs = 100.0;

inline bool ParseReadPacketTimeoutMs(const std::string & text, double & value)
{
  try {
    std::size_t parsed = 0;
    const double candidate = std::stod(text, &parsed);
    if (text.find_first_not_of(" \t\r\n", parsed) != std::string::npos) {
      return false;
    }
    if (!std::isfinite(candidate) || candidate < kMinimumReadPacketTimeoutMs ||
      candidate > kMaximumReadPacketTimeoutMs)
    {
      return false;
    }
    value = candidate;
    return true;
  } catch (...) {
    return false;
  }
}

}  // namespace dynamixel_hardware_interface

#endif  // DYNAMIXEL_HARDWARE_INTERFACE__READ_PACKET_TIMEOUT_HPP_
