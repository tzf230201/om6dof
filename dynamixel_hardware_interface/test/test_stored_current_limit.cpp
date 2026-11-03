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

#include <gtest/gtest.h>
#include <cstdint>
#include <map>
#include <string>
#include <vector>
#include <limits>
#include "dynamixel_hardware_interface/stored_current_limit.hpp"

using dynamixel_hardware_interface::apply_stored_current_limit;
using dynamixel_hardware_interface::bounded_mode5_current;

TEST(Mode5Budget, ClampRawCeilingsAndFailClosed)
{
  EXPECT_EQ(0.0, bounded_mode5_current(-30, 93, true));
  EXPECT_EQ(12.0, bounded_mode5_current(12.99, 93, true));
  EXPECT_EQ(93.0, bounded_mode5_current(1193, 93, true));
  EXPECT_EQ(0.0, bounded_mode5_current(93, 93, false));
  EXPECT_EQ(0.0, bounded_mode5_current(std::numeric_limits<double>::quiet_NaN(), 93, true));
  EXPECT_EQ(0.0, bounded_mode5_current(93, 0, true));
  EXPECT_EQ(0.0, bounded_mode5_current(93, 32768, true));
}

struct Registers
{
  std::map<std::string, uint32_t> values{
    {"Operating Mode", 5}, {"Current Limit", 321}, {"Goal Current", 0}};
  std::string fail_read;
  bool fail_write = false;
  bool mismatch = false;
  std::vector<std::string> writes;
  bool apply()
  {
    return apply_stored_current_limit(
      [&](const char * key, uint32_t & value) {
        if (key == fail_read) {return false;}
        value = values.at(key);
        return true;
      },
      [&](const char * key, uint32_t value) {
        writes.push_back(key);
        if (fail_write) {return false;}
        values[key] = mismatch ? value - 1 : value;
        return true;
      });
  }
};

TEST(StoredCurrentLimit, FollowsEachStoredLimitWithoutEEPROMWrite)
{
  for (uint32_t limit : {1U, 45U, 321U, 1193U}) {
    Registers r;
    r.values["Current Limit"] = limit;
    ASSERT_TRUE(r.apply());
    EXPECT_EQ(r.values["Current Limit"], limit);
    EXPECT_EQ(r.values["Goal Current"], limit);
    EXPECT_EQ(r.writes, std::vector<std::string>{"Goal Current"});
  }
}

TEST(StoredCurrentLimit, RefusesWrongModeOrInvalidLimitBeforeWriting)
{
  for (uint32_t mode : {0U, 1U, 3U, 4U}) {
    Registers r;
    r.values["Operating Mode"] = mode;
    EXPECT_FALSE(r.apply());
    EXPECT_TRUE(r.writes.empty());
  }
  for (uint32_t limit : {0U, 32768U, 0xffffffffU}) {
    Registers r;
    r.values["Current Limit"] = limit;
    EXPECT_FALSE(r.apply());
    EXPECT_TRUE(r.writes.empty());
  }
}

TEST(StoredCurrentLimit, FailsClosedOnTransactionAndReadbackFailures)
{
  for (const char * key : {"Operating Mode", "Current Limit", "Goal Current"}) {
    Registers r;
    r.fail_read = key;
    EXPECT_FALSE(r.apply());
  }
  Registers r;
  r.fail_write = true;
  EXPECT_FALSE(r.apply());
  Registers mismatch;
  mismatch.mismatch = true;
  EXPECT_FALSE(mismatch.apply());
}
