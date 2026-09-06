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

#ifndef DYNAMIXEL_HARDWARE_INTERFACE__SEQUENTIAL_READ_TRANSACTION_HPP_
#define DYNAMIXEL_HARDWARE_INTERFACE__SEQUENTIAL_READ_TRANSACTION_HPP_

namespace dynamixel_hardware_interface
{

// Acquire every response before exposing any of them to ros2_control. This
// retains the all-or-nothing state snapshot provided by a multi-ID SyncRead.
template<typename ContainerT, typename AcquireT, typename CommitT>
bool AcquireAllThenCommit(
  const ContainerT & items, AcquireT acquire, CommitT commit)
{
  for (const auto & item : items) {
    if (!acquire(item)) {
      return false;
    }
  }
  for (const auto & item : items) {
    commit(item);
  }
  return true;
}

}  // namespace dynamixel_hardware_interface

#endif  // DYNAMIXEL_HARDWARE_INTERFACE__SEQUENTIAL_READ_TRANSACTION_HPP_
