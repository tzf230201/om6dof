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

#include <gtest/gtest.h>

#include <array>
#include <string>

#include "dynamixel_hardware_interface/read_transport_mode.hpp"

namespace dhi = dynamixel_hardware_interface;

TEST(ReadTransportMode, ParsesOnlySupportedModes)
{
  dhi::ReadTransportMode mode = dhi::ReadTransportMode::SEQUENTIAL_SINGLE_SYNC;
  EXPECT_TRUE(dhi::ParseReadTransportMode("multi_sync", mode));
  EXPECT_EQ(mode, dhi::ReadTransportMode::MULTI_SYNC);

  EXPECT_TRUE(dhi::ParseReadTransportMode("  sequential_single_sync\n", mode));
  EXPECT_EQ(mode, dhi::ReadTransportMode::SEQUENTIAL_SINGLE_SYNC);
}

TEST(ReadTransportMode, InvalidConfigurationFailsClosed)
{
  dhi::ReadTransportMode mode = dhi::ReadTransportMode::MULTI_SYNC;
  const std::array<std::string, 5> invalid_modes{
    "", "sync", "sequential", "SEQUENTIAL_SINGLE_SYNC",
    "sequential_single_sync, multi_sync"};
  for (const auto & invalid : invalid_modes) {
    EXPECT_FALSE(dhi::ParseReadTransportMode(invalid, mode)) << invalid;
    EXPECT_EQ(mode, dhi::ReadTransportMode::MULTI_SYNC);
  }
}

TEST(ReadTransportMode, StableNamesMatchHardwareParameterContract)
{
  EXPECT_STREQ(
    dhi::ReadTransportModeName(dhi::ReadTransportMode::MULTI_SYNC),
    "multi_sync");
  EXPECT_STREQ(
    dhi::ReadTransportModeName(dhi::ReadTransportMode::SEQUENTIAL_SINGLE_SYNC),
    "sequential_single_sync");
}
