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

#include <cstdint>
#include <map>
#include <vector>

#include "dynamixel_hardware_interface/sequential_read_transaction.hpp"

namespace dhi = dynamixel_hardware_interface;

TEST(SequentialReadTransaction, CommitsIdKeyedPayloadsOnlyAfterAllAcquisitions)
{
  const std::vector<uint8_t> ids{31, 32, 33, 24, 35, 26, 37};
  std::map<uint8_t, int> staged;
  std::map<uint8_t, int> exported;
  std::vector<uint8_t> commit_order;

  const bool complete = dhi::AcquireAllThenCommit(
    ids,
    [&staged](uint8_t id) {
      staged[id] = 1000 + id;
      return true;
    },
    [&staged, &exported, &commit_order](uint8_t id) {
      exported[id] = staged.at(id);
      commit_order.push_back(id);
    });

  EXPECT_TRUE(complete);
  EXPECT_EQ(commit_order, ids);
  for (const uint8_t id : ids) {
    EXPECT_EQ(exported.at(id), 1000 + id);
  }
}

TEST(SequentialReadTransaction, AFailedResponseCausesZeroPartialCommits)
{
  const std::vector<uint8_t> ids{31, 32, 33, 24, 35, 26, 37};
  std::vector<uint8_t> acquired;
  std::vector<uint8_t> committed;

  const bool complete = dhi::AcquireAllThenCommit(
    ids,
    [&acquired](uint8_t id) {
      acquired.push_back(id);
      return id != 24;
    },
    [&committed](uint8_t id) {committed.push_back(id);});

  EXPECT_FALSE(complete);
  EXPECT_EQ(acquired, (std::vector<uint8_t>{31, 32, 33, 24}));
  EXPECT_TRUE(committed.empty());
}
