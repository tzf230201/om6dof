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

"""Offline contracts for the fail-closed torque-off diagnostic stack."""

import importlib.util
from pathlib import Path

import pytest
from launch import LaunchContext


PACKAGE = Path(__file__).resolve().parents[1]
LAUNCH_FILE = PACKAGE / "launch" / "torque_off_diagnostic.launch.py"
DRIVER_PACKAGE = PACKAGE.parent / "dynamixel_hardware_interface"
DRIVER_SOURCE = DRIVER_PACKAGE / "src" / "dynamixel_hardware_interface.cpp"
DRIVER_HEADER = (
    DRIVER_PACKAGE
    / "include"
    / "dynamixel_hardware_interface"
    / "dynamixel_hardware_interface.hpp"
)

SPEC = importlib.util.spec_from_file_location("torque_off_diagnostic", LAUNCH_FILE)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def _context(arm_supported):
    context = LaunchContext()
    context.launch_configurations["arm_supported"] = arm_supported
    context.launch_configurations["port_name"] = "/tmp/never-opened-by-test"
    context.launch_configurations["baud_rate"] = "1000000"
    context.launch_configurations["diagnostic_update_rate_hz"] = "100"
    return context


def test_preflight_requires_explicit_physical_support(monkeypatch):
    monkeypatch.setattr(
        MODULE,
        "_query_service_state",
        lambda: ("loaded", "inactive", "ok"),
    )
    monkeypatch.setattr(
        MODULE,
        "_render_diagnostic_description",
        lambda context: "<robot/>",
    )

    with pytest.raises(RuntimeError, match="mechanically support"):
        MODULE._preflight(_context("false"))

    assert MODULE._preflight(_context("true"))


@pytest.mark.parametrize("rate", ["10", "20", "50", "100"])
def test_diagnostic_update_rate_accepts_only_supported_sweep_values(rate):
    context = _context("true")
    context.launch_configurations["diagnostic_update_rate_hz"] = rate
    assert MODULE._diagnostic_update_rate(context) == int(rate)


@pytest.mark.parametrize(
    "rate", ["", "0", "5", "25", "101", "10.0", "010", "fast"]
)
def test_diagnostic_update_rate_fails_closed_before_port_open(rate):
    context = _context("true")
    context.launch_configurations["diagnostic_update_rate_hz"] = rate
    with pytest.raises(RuntimeError, match="must be one of"):
        MODULE._diagnostic_update_rate(context)


@pytest.mark.parametrize(
    "load_state,active_state",
    [
        ("loaded", "active"),
        ("loaded", "activating"),
        ("loaded", "deactivating"),
        ("loaded", "failed"),
        ("not-found", "inactive"),
        ("unknown", "unknown"),
    ],
)
def test_preflight_fails_closed_unless_service_is_loaded_and_inactive(
    monkeypatch, load_state, active_state
):
    monkeypatch.setattr(
        MODULE,
        "_query_service_state",
        lambda: (load_state, active_state, "test"),
    )

    with pytest.raises(RuntimeError, match=r"loaded\+inactive"):
        MODULE._preflight(_context("true"))


def test_launch_hard_codes_real_torque_off_profile_and_no_motion_controller():
    source = LAUNCH_FILE.read_text(encoding="utf-8")

    assert '"torque_off_diagnostic_mode:=true"' in source
    assert '"use_fake_hardware:=false"' in source
    assert '"current_control:=false"' in source
    assert 'DeclareLaunchArgument("torque_off_diagnostic_mode"' not in source
    assert '"arm_supported",' in source
    assert 'default_value="false"' in source
    assert '"diagnostic_update_rate_hz"' in source
    assert "SUPPORTED_UPDATE_RATES_HZ = (10, 20, 50, 100)" in source
    assert '"update_rate": ParameterValue(' in source
    assert '"joint_state_broadcaster"' in source
    assert '"arm_controller"' not in source
    assert '"gripper_controller"' not in source
    assert '"forward_position_controller"' not in source
    assert '"forward_effort_controller"' not in source


def test_rendered_description_is_validated_before_it_is_forwarded(monkeypatch):
    rendered = MODULE._render_diagnostic_description(_context("true"))
    assert "<robot" in rendered
    assert "torque_off_diagnostic_mode" in rendered

    good = """
    <robot><ros2_control><hardware>
      <plugin>dynamixel_hardware_interface/DynamixelHardware</plugin>
      <param name="torque_off_diagnostic_mode">True</param>
      <param name="disable_torque_at_init">true</param>
      <param name="read_transport_mode">sequential_single_sync</param>
    </hardware></ros2_control></robot>
    """
    MODULE._validate_rendered_description(good)

    unsafe = good.replace(">True<", ">False<")
    with pytest.raises(RuntimeError, match="does not enable"):
        MODULE._validate_rendered_description(unsafe)

    unsafe_transport = good.replace(
        ">sequential_single_sync<", ">multi_sync<"
    )
    with pytest.raises(RuntimeError, match="read transport"):
        MODULE._validate_rendered_description(unsafe_transport)

    monkeypatch.setattr(
        MODULE,
        "_query_service_state",
        lambda: ("loaded", "inactive", "ok"),
    )
    monkeypatch.setattr(
        MODULE,
        "_render_diagnostic_description",
        lambda context: good,
    )
    actions = MODULE._preflight(_context("true"))
    assert actions[0].__class__.__name__ == "SetLaunchConfiguration"


def test_xacro_profiles_forward_a_default_off_diagnostic_flag():
    wrapper = (PACKAGE / "urdf" / "om6dof.urdf.xacro").read_text(
        encoding="utf-8"
    )
    assert (
        '<xacro:arg name="torque_off_diagnostic_mode" default="false"/>'
        in wrapper
    )
    assert (
        'torque_off_diagnostic_mode="$(arg torque_off_diagnostic_mode)"'
        in wrapper
    )

    for filename in (
        "om6dof.ros2_control.xacro",
        "om6dof.ros2_control.current.xacro",
    ):
        profile = (PACKAGE / "urdf" / filename).read_text(encoding="utf-8")
        assert "torque_off_diagnostic_mode:=false" in profile
        assert (
            '<param name="torque_off_diagnostic_mode">'
            "${torque_off_diagnostic_mode}</param>"
        ) in profile


def test_driver_enforces_torque_off_beyond_the_launch_layer():
    source = DRIVER_SOURCE.read_text(encoding="utf-8")
    header = DRIVER_HEADER.read_text(encoding="utf-8")

    assert "bool torque_off_diagnostic_mode_{false};" in header
    assert "ParseBooleanHardwareParameter(diagnostic_value" in source
    assert "std::tolower(character)" in source
    assert "auto_enable_torque_on_start_ = false;" in source
    assert "restrict_critical_write_service_ = true;" in source
    assert "disable_torque_at_init = true;" in source
    assert "DynamixelDisable(dxl_comm_id_id_)" in source
    assert 'EnforceTorqueOffDiagnosticMode("hardware activation")' in source
    assert 'EnforceTorqueOffDiagnosticMode("cyclic read")' in source
    assert "Rejected all register writes in torque-off diagnostic mode" in source
    assert "Rejected Dynamixel reboot in torque-off diagnostic mode" in source
    assert "Torque enable is inhibited by torque-off diagnostic mode." in source


def test_health_uses_the_expected_torque_state_for_each_mode():
    source = DRIVER_SOURCE.read_text(encoding="utf-8")

    assert (
        "const bool torque_state_expected = torque_off_diagnostic_mode_ ?\n"
        "    torque_all_disabled : torque_all_enabled;"
    ) in source
    assert "bus_health_.CommunicationHealthy() && torque_state_expected" in source
    assert 'key = "torque_all_disabled";' in source
    assert 'key = "torque_enable_inhibited";' in source
    assert (
        "status.values[kHealthTorqueAllDisabled].value = torque_all_disabled ? "
        '"true" : "false";'
    ) in source
    assert (
        "status.values[kHealthTorqueEnableInhibited].value =\n"
        '    torque_off_diagnostic_mode_ ? "true" : "false";'
    ) in source
    assert '"OK (torque-off diagnostic mode)"' in source
