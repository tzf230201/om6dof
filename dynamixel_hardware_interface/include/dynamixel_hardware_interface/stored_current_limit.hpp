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

#ifndef DYNAMIXEL_HARDWARE_INTERFACE__STORED_CURRENT_LIMIT_HPP_
#define DYNAMIXEL_HARDWARE_INTERFACE__STORED_CURRENT_LIMIT_HPP_

#include <cstdint>
#include <algorithm>
#include <cmath>

namespace dynamixel_hardware_interface
{
inline double bounded_mode5_current(double command, double stored_limit, bool fresh)
{
  if (!fresh || !std::isfinite(command) || !std::isfinite(stored_limit) ||
    stored_limit < 1.0 || stored_limit > 32767.0) {return 0.0;}
  return std::floor(std::max(0.0, std::min(command, stored_limit)));
}

// Call only during torque-off initialization. Never writes an EEPROM limit.
// Read/write callbacks return true only on successful register transactions.
template<typename Read, typename Write>
bool apply_stored_current_limit(Read read, Write write)
{
  uint32_t mode = 0;
  uint32_t limit = 0;
  uint32_t verified = 0;
  if (!read("Operating Mode", mode) || mode != 5 ||
    !read("Current Limit", limit) || limit == 0 || limit > 32767)
  {
    return false;
  }
  return write("Goal Current", limit) && read("Goal Current", verified) && verified == limit;
}
}  // namespace dynamixel_hardware_interface

#endif  // DYNAMIXEL_HARDWARE_INTERFACE__STORED_CURRENT_LIMIT_HPP_
