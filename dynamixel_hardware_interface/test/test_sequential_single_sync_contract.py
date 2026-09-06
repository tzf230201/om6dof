# Copyright 2026 KUBOTA Lab
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Offline contracts for the commissioned single-responder read transport."""

from pathlib import Path


PACKAGE = Path(__file__).resolve().parents[1]
SOURCE = PACKAGE / "src" / "dynamixel" / "dynamixel.cpp"
HEADER = (
    PACKAGE
    / "include"
    / "dynamixel_hardware_interface"
    / "dynamixel"
    / "dynamixel.hpp"
)
HARDWARE = PACKAGE / "src" / "dynamixel_hardware_interface.cpp"
BRINGUP = PACKAGE.parent / "om6dof_bringup"


def test_library_default_is_backward_compatible_and_invalid_mode_aborts_init():
    header = HEADER.read_text(encoding="utf-8")
    hardware = HARDWARE.read_text(encoding="utf-8")

    assert (
        "ReadTransportMode read_transport_mode_"
        "{ReadTransportMode::MULTI_SYNC};"
    ) in header
    assert "if (!ParseReadTransportMode(" in hardware
    assert "refusing hardware initialization" in hardware
    assert "return hardware_interface::CallbackReturn::ERROR;" in hardware


def test_one_persistent_handler_per_comm_id_and_atomic_commit_contract():
    header = HEADER.read_text(encoding="utf-8")
    source = SOURCE.read_text(encoding="utf-8")

    assert (
        "std::map<uint8_t, std::unique_ptr<dynamixel::GroupSyncRead>>"
        in header
    )
    assert "new dynamixel::GroupSyncRead(" in source
    assert "handler->addParam(comm_id)" in source
    assert (
        "indirect_info_read_[address.first].indirect_data_addr = "
        "address.second;"
    ) in source
    assert "handler->isAvailable(comm_id, current_addr, item_size)" in source
    assert "handler->getError(comm_id, &device_error)" in source
    assert "AcquireAllThenCommit(" in source
    assert "No exported pointer is touched before this" in source
    assert "sequential_single_sync_read_handlers_.clear();" in source


def test_om6dof_selects_sequential_mode_without_changing_generic_macro_default():
    safety = (BRINGUP / "config" / "hardware_safety.yaml").read_text(
        encoding="utf-8"
    )
    wrapper = (BRINGUP / "urdf" / "om6dof.urdf.xacro").read_text(
        encoding="utf-8"
    )
    assert "read_transport_mode: sequential_single_sync" in safety
    assert (
        'read_transport_mode="'
        "${hardware_safety['read_transport_mode']}"
        '"'
    ) in wrapper

    for filename in (
        "om6dof.ros2_control.xacro",
        "om6dof.ros2_control.current.xacro",
    ):
        profile = (BRINGUP / "urdf" / filename).read_text(encoding="utf-8")
        assert "read_transport_mode:=multi_sync" in profile
        assert (
            '<param name="read_transport_mode">'
            "${read_transport_mode}</param>"
        ) in profile


def test_write_path_has_no_read_transport_branch():
    source = SOURCE.read_text(encoding="utf-8")
    start = source.index("DxlError Dynamixel::SetDxlValueToSyncWrite()")
    end = source.index("DxlError Dynamixel::SetBulkWriteItemAndHandler()", start)
    write_path = source[start:end]

    assert "read_transport_mode" not in write_path
    assert "for (auto it_write_data : write_data_list_)" in write_path
