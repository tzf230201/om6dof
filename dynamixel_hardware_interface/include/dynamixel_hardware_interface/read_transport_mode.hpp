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

#ifndef DYNAMIXEL_HARDWARE_INTERFACE__READ_TRANSPORT_MODE_HPP_
#define DYNAMIXEL_HARDWARE_INTERFACE__READ_TRANSPORT_MODE_HPP_

#include <string>

namespace dynamixel_hardware_interface
{

enum class ReadTransportMode
{
  MULTI_SYNC,
  SEQUENTIAL_SINGLE_SYNC,
};

inline const char * ReadTransportModeName(ReadTransportMode mode)
{
  switch (mode) {
    case ReadTransportMode::MULTI_SYNC:
      return "multi_sync";
    case ReadTransportMode::SEQUENTIAL_SINGLE_SYNC:
      return "sequential_single_sync";
    default:
      return "unknown";
  }
}

inline bool ParseReadTransportMode(const std::string & text, ReadTransportMode & mode)
{
  const auto first = text.find_first_not_of(" \t\r\n");
  if (first == std::string::npos) {
    return false;
  }
  const auto last = text.find_last_not_of(" \t\r\n");
  const std::string normalized = text.substr(first, last - first + 1);

  if (normalized == "multi_sync") {
    mode = ReadTransportMode::MULTI_SYNC;
    return true;
  }
  if (normalized == "sequential_single_sync") {
    mode = ReadTransportMode::SEQUENTIAL_SINGLE_SYNC;
    return true;
  }
  return false;
}

}  // namespace dynamixel_hardware_interface

#endif  // DYNAMIXEL_HARDWARE_INTERFACE__READ_TRANSPORT_MODE_HPP_
