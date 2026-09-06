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

import re
from pathlib import Path


PACKAGE = Path(__file__).resolve().parents[1]
SOURCE = PACKAGE / "src" / "dynamixel_hardware_interface.cpp"
BRINGUP = PACKAGE.parent / "om6dof_bringup"


def test_runtime_read_uses_fixed_bounded_timeout_without_changing_failure_accounting():
    source = SOURCE.read_text(encoding="utf-8")

    assert "ReadMultiDxlData(read_packet_timeout_ms_)" in source
    assert "std::max(period_ms, read_packet_timeout_ms_)" not in source

    # Packet waiting and safety accounting intentionally use different values.
    # The deadline stays bounded even after a scheduler stall, while failure
    # duration/counters continue to track the real control period.
    assert "read_error_duration_ = read_error_duration_ + period;" in source
    assert "bus_health_.RecordReadFailure(" in source
    assert re.search(
        r"consecutive_read_failures_\s*>=\s*"
        r"consecutive_failure_shutdown_threshold_",
        source,
    )


def test_startup_read_also_uses_the_validated_timeout():
    source = SOURCE.read_text(encoding="utf-8")

    assert "ReadMultiDxlData(read_packet_timeout_ms_)" in source
    assert "ParseReadPacketTimeoutMs(" in source


def test_om6dof_profiles_forward_the_central_30_ms_deadline():
    safety = (BRINGUP / "config" / "hardware_safety.yaml").read_text(
        encoding="utf-8"
    )
    assert "read_packet_timeout_ms: 30.0" in safety

    for filename in (
        "om6dof.ros2_control.xacro",
        "om6dof.ros2_control.current.xacro",
    ):
        profile = (BRINGUP / "urdf" / filename).read_text(encoding="utf-8")
        assert "read_packet_timeout_ms:=30.0" in profile
        assert (
            '<param name="read_packet_timeout_ms">'
            "${read_packet_timeout_ms}</param>"
        ) in profile

    description = (BRINGUP / "urdf" / "om6dof.urdf.xacro").read_text(
        encoding="utf-8"
    )
    assert (
        'read_packet_timeout_ms="'
        "${hardware_safety['read_packet_timeout_ms']}"
        '"'
    ) in description
