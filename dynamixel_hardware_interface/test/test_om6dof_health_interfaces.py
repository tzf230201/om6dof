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

"""Contracts connecting OM6DOF state interfaces to live health diagnostics."""

from pathlib import Path
import xml.etree.ElementTree as ET

import pytest


PACKAGE = Path(__file__).resolve().parents[1]
BRINGUP = PACKAGE.parent / "om6dof_bringup"
DRIVER = PACKAGE / "src" / "dynamixel_hardware_interface.cpp"
PROBE = PACKAGE / "scripts" / "dxl_read_diagnostic"

EXPECTED_DYNAMIXELS = {
    "dxl1": 31,
    "dxl2": 32,
    "dxl3": 33,
    "dxl4": 24,
    "dxl5": 35,
    "dxl6": 26,
    "dxl7": 37,
}
CYCLIC_STATE_ITEMS = (
    "Present Position",
    "Present Velocity",
    "Present Current",
    "Torque Enable",
    "Present Input Voltage",
    "Hardware Error Status",
)
CONTROL_ITEM_BYTES = {
    "Present Position": 4,
    "Present Velocity": 4,
    "Present Current": 2,
    "Torque Enable": 1,
    "Present Input Voltage": 2,
    "Hardware Error Status": 1,
}


def _dynamixel_interfaces(profile_name):
    root = ET.parse(BRINGUP / "urdf" / profile_name).getroot()
    result = {}
    for gpio in root.findall(".//gpio"):
        parameters = {
            param.attrib.get("name"): (param.text or "").strip()
            for param in gpio.findall("param")
        }
        if parameters.get("type") != "dxl":
            continue
        result[gpio.attrib["name"]] = {
            "id": int(parameters["ID"]),
            "states": tuple(
                item.attrib["name"]
                for item in gpio.findall("state_interface")
            ),
        }
    return result


@pytest.mark.parametrize(
    "profile_name",
    (
        "om6dof.ros2_control.xacro",
        "om6dof.ros2_control.current.xacro",
    ),
)
def test_every_live_om6dof_actuator_reads_hardware_error_and_voltage(profile_name):
    interfaces = _dynamixel_interfaces(profile_name)

    assert {name: data["id"] for name, data in interfaces.items()} == (
        EXPECTED_DYNAMIXELS
    )
    for data in interfaces.values():
        assert data["states"] == CYCLIC_STATE_ITEMS
        assert data["states"].count("Hardware Error Status") == 1
        assert data["states"].count("Present Input Voltage") == 1


def test_driver_shaped_indirect_read_length_tracks_the_live_profiles():
    expected_bytes = sum(CONTROL_ITEM_BYTES[item] for item in CYCLIC_STATE_ITEMS)
    assert expected_bytes == 14

    probe = PROBE.read_text(encoding="utf-8")
    assert f"DRIVER_INDIRECT_LENGTH = {expected_bytes}" in probe


def test_driver_health_is_fail_closed_when_state_monitoring_is_incomplete():
    source = DRIVER.read_text(encoding="utf-8")

    assert 'it.name == "Hardware Error Status"' in source
    assert "dxl_hw_err_[id] = 0x00;" in source
    assert '== "Hardware Error Status"' in source
    assert "HardwareErrorMonitoringComplete(dxl_comm_id_id_, dxl_hw_err_)" in source
    assert "hardware_error_monitoring_complete && hardware_error_mask == 0" in source
    assert '"hardware_error_monitoring_complete"' in source
    assert '"input_voltage_monitoring_complete"' in source
    assert '"input_voltage_min_v"' in source
    assert '"input_voltage_min_id"' in source
