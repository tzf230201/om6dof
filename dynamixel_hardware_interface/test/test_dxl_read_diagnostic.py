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

"""Offline safety and source-contract tests for dxl_read_diagnostic."""

import importlib.machinery
import importlib.util
from pathlib import Path

import pytest


SCRIPT = Path(__file__).resolve().parents[1] / "scripts" / "dxl_read_diagnostic"
LOADER = importlib.machinery.SourceFileLoader("dxl_read_diagnostic", str(SCRIPT))
SPEC = importlib.util.spec_from_loader(LOADER.name, LOADER)
DIAGNOSTIC = importlib.util.module_from_spec(SPEC)
LOADER.exec_module(DIAGNOSTIC)


class FakePacketHandler:
    """Provide descriptions needed by the statistics helpers."""

    def getTxRxResult(self, code):
        return f"comm-{code}"

    def getRxPacketError(self, code):
        return f"device-{code}"


class FakePort:
    """Record packet deadlines without opening a serial device."""

    def __init__(self):
        self.timeouts = []
        self.ser = None

    def setPacketTimeoutMillis(self, timeout):
        self.timeouts.append(timeout)

    def getBytesAvailable(self):
        return 0


class FakeGroup:
    """Model a successful 14-byte GroupSyncRead response."""

    def __init__(self, port, packet, address, length):
        del port, packet
        self.address = address
        self.length = length
        self.ids = []

    def addParam(self, dxl_id):
        self.ids.append(dxl_id)
        return True

    def txPacket(self):
        return DIAGNOSTIC.COMM_SUCCESS

    def rxPacket(self):
        return DIAGNOSTIC.COMM_SUCCESS

    def isAvailable(self, dxl_id, address, length):
        return (
            dxl_id in self.ids
            and address == self.address
            and length == self.length
        )

    def getData(self, dxl_id, address, length):
        del dxl_id, address, length
        raise AssertionError("14-byte GroupSyncRead must not use SDK getData")


def test_defaults_match_the_exact_om6dof_bus():
    args = DIAGNOSTIC.build_parser().parse_args([])

    assert args.port == DIAGNOSTIC.DEFAULT_PORT
    assert args.ids == (31, 32, 33, 24, 35, 26, 37)
    assert DIAGNOSTIC.SAFETY_IDS == (31, 32, 33, 24, 35, 26, 37)
    assert DIAGNOSTIC.PROTOCOL_VERSION == 2.0
    assert DIAGNOSTIC.BAUD_RATE == 1_000_000
    assert args.timeout_ms == 30.0
    assert args.retries == 0
    assert not args.drain_after_tx
    assert args.interval_wait == "sleep"
    assert args.phase == DIAGNOSTIC.PHASE_ALL


@pytest.mark.parametrize("state", [
    "active", "activating", "deactivating", "failed", "unknown", "unavailable",
])
def test_service_guard_fails_closed_for_every_non_inactive_state(state):
    assert DIAGNOSTIC._unsafe_service_state(state)


def test_service_guard_only_accepts_conclusive_inactive_state():
    assert not DIAGNOSTIC._unsafe_service_state("inactive")


def test_missing_systemd_unit_cannot_masquerade_as_inactive(monkeypatch):
    class Result:
        returncode = 0
        stdout = "LoadState=not-found\nActiveState=inactive\n"
        stderr = ""

    monkeypatch.setattr(
        DIAGNOSTIC.subprocess, "run", lambda *args, **kwargs: Result()
    )

    state, detail = DIAGNOSTIC.service_state("bogus.service")

    assert state == "unavailable:inactive"
    assert detail == "LoadState=not-found"
    assert DIAGNOSTIC._unsafe_service_state(state)


def test_torque_preflight_marks_nonzero_and_uncertain_ids_unsafe(monkeypatch):
    results = iter([
        (0, DIAGNOSTIC.COMM_SUCCESS, 0),
        (1, DIAGNOSTIC.COMM_SUCCESS, 0),
        (None, -3002, 0),
    ])
    monkeypatch.setattr(DIAGNOSTIC, "read_register", lambda *args: next(results))

    stats = DIAGNOSTIC.verify_torque_disabled(
        object(), FakePacketHandler(), (31, 32, 33), 30.0
    )

    assert not stats["all_disabled"]
    assert stats["unsafe_ids"] == [32, 33]
    assert stats["success"] == 2
    assert sum(stats["communication_errors"].values()) == 1


def test_health_snapshot_reports_raw_and_decoded_values(monkeypatch):
    results = iter([
        (0, DIAGNOSTIC.COMM_SUCCESS, 0),
        (121, DIAGNOSTIC.COMM_SUCCESS, 0),
        (36, DIAGNOSTIC.COMM_SUCCESS, 0),
    ])
    monkeypatch.setattr(DIAGNOSTIC, "read_register", lambda *args: next(results))

    snapshot = DIAGNOSTIC.read_health_snapshot(
        object(), FakePacketHandler(), (31,), 30.0
    )

    servo = snapshot["per_id"]["31"]
    assert servo["hardware_error_status"]["raw"] == 0
    assert servo["hardware_error_status"]["decoded"]["flags"] == []
    assert servo["present_input_voltage"]["raw"] == 121
    assert servo["present_input_voltage"]["decoded"] == {"volts": 12.1}
    assert servo["present_temperature"]["raw"] == 36
    assert servo["present_temperature"]["decoded"] == {"celsius": 36}
    assert snapshot["healthy"]


def test_health_snapshot_nonzero_status_or_read_error_is_non_pass(monkeypatch):
    results = iter([
        (0x22, DIAGNOSTIC.COMM_SUCCESS, 0),
        (None, -3002, 0),
        (41, DIAGNOSTIC.COMM_SUCCESS, 0x04),
    ])
    monkeypatch.setattr(DIAGNOSTIC, "read_register", lambda *args: next(results))

    snapshot = DIAGNOSTIC.read_health_snapshot(
        object(), FakePacketHandler(), (37,), 30.0
    )

    hardware = snapshot["per_id"]["37"]["hardware_error_status"]
    assert hardware["decoded"]["flags"] == ["overload"]
    assert hardware["decoded"]["unknown_bits"] == "0x02"
    assert snapshot["nonzero_hardware_error_ids"] == [37]
    assert sum(snapshot["communication_errors"].values()) == 1
    assert sum(snapshot["device_errors"].values()) == 1
    assert not snapshot["all_reads_ok"]
    assert not snapshot["all_hardware_errors_clear"]
    assert not snapshot["healthy"]


def test_driver_shaped_group_phase_uses_634_by_14_without_decoding():
    port = FakePort()

    stats = DIAGNOSTIC.run_sync_trials(
        port, FakePacketHandler(), FakeGroup, (31, 32), 3, 30.0, 0.0,
        DIAGNOSTIC.DRIVER_INDIRECT_ADDRESS,
        DIAGNOSTIC.DRIVER_INDIRECT_LENGTH,
    )

    assert stats["attempted"] == 3
    assert stats["success"] == 3
    assert stats["communication_errors"] == {}
    assert port.timeouts == [30.0, 30.0, 30.0]


def test_diagnostic_group_records_expected_id_not_claimed_culprit(monkeypatch):
    class Packet(FakePacketHandler):
        def getProtocolVersion(self):
            return 2.0

        def readRx(self, port, dxl_id, length):
            del port, length
            if dxl_id == 32:
                return [], -3002, 0
            return [0] * 14, DIAGNOSTIC.COMM_SUCCESS, 0

    times = iter((1.000, 1.003))
    monkeypatch.setattr(DIAGNOSTIC.time, "monotonic", lambda: next(times))
    group = DIAGNOSTIC.DiagnosticGroupSyncRead(
        FakePort(), Packet(), DIAGNOSTIC.DRIVER_INDIRECT_ADDRESS,
        DIAGNOSTIC.DRIVER_INDIRECT_LENGTH,
    )
    for dxl_id in (31, 32, 33):
        assert group.addParam(dxl_id)

    assert group.rxPacket() == -3002
    assert group.last_rx_failure == {
        "expected_id": 32,
        "expected_index": 1,
        "responses_completed": 1,
        "communication_code": -3002,
        "device_error": 0,
        "rx_elapsed_ms": 3.0,
        "queued_bytes_after_failure": 0,
    }


def test_sync_stats_aggregate_expected_id_failure_context():
    class FailingGroup(FakeGroup):
        def __init__(self, *args):
            super().__init__(*args)
            self.last_rx_failure = None

        def rxPacket(self):
            self.last_rx_failure = {
                "expected_id": 32,
                "expected_index": 1,
                "responses_completed": 1,
                "communication_code": -3002,
                "device_error": 0,
                "rx_elapsed_ms": 2.5,
                "queued_bytes_after_failure": 25,
            }
            return -3002

    stats = DIAGNOSTIC.run_sync_trials(
        FakePort(), FakePacketHandler(), FailingGroup, (31, 32), 2,
        30.0, 0.0, DIAGNOSTIC.DRIVER_INDIRECT_ADDRESS,
        DIAGNOSTIC.DRIVER_INDIRECT_LENGTH,
    )

    assert stats["failure_by_expected_id"] == {"32": 2}
    assert stats["failure_by_expected_index"] == {"1": 2}
    assert [event["trial_index"] for event in stats["failure_events"]] == [0, 1]
    assert stats["failure_events_truncated"] == 0


def test_sync_retry_is_bounded_recorded_and_does_not_hide_transport_error():
    class FailOnceGroup(FakeGroup):
        def __init__(self, *args):
            super().__init__(*args)
            self.rx_calls = 0
            self.last_rx_failure = None

        def rxPacket(self):
            self.rx_calls += 1
            if self.rx_calls == 1:
                self.last_rx_failure = {
                    "expected_id": 31,
                    "expected_index": 0,
                    "responses_completed": 0,
                    "communication_code": -3002,
                    "device_error": 0,
                    "rx_elapsed_ms": 30.0,
                    "queued_bytes_after_failure": 0,
                }
                return -3002
            self.last_rx_failure = None
            return DIAGNOSTIC.COMM_SUCCESS

    stats = DIAGNOSTIC.run_sync_trials(
        FakePort(), FakePacketHandler(), FailOnceGroup, (31,), 1,
        30.0, 0.0, DIAGNOSTIC.DRIVER_INDIRECT_ADDRESS,
        DIAGNOSTIC.DRIVER_INDIRECT_LENGTH, retries=1,
    )

    assert stats["attempted"] == 1
    assert stats["success"] == 1
    assert stats["physical_attempts"] == 2
    assert stats["retry_attempts"] == 1
    assert stats["recovered_after_retry"] == 1
    assert sum(stats["communication_errors"].values()) == 1
    assert stats["failure_events"][0]["retry_index"] == 0


def test_sync_optional_tty_drain_precedes_receive_deadline():
    events = []

    class Serial:
        def flush(self):
            events.append("drain")

    class Port(FakePort):
        def __init__(self):
            super().__init__()
            self.ser = Serial()

        def setPacketTimeoutMillis(self, timeout):
            events.append(("timeout", timeout))
            super().setPacketTimeoutMillis(timeout)

    class Group(FakeGroup):
        def rxPacket(self):
            events.append("receive")
            return DIAGNOSTIC.COMM_SUCCESS

    stats = DIAGNOSTIC.run_sync_trials(
        Port(), FakePacketHandler(), Group, (31,), 1, 30.0, 0.0,
        DIAGNOSTIC.DRIVER_INDIRECT_ADDRESS,
        DIAGNOSTIC.DRIVER_INDIRECT_LENGTH, drain_after_tx=True,
    )

    assert stats["success"] == 1
    assert events == ["drain", ("timeout", 30.0), "receive"]


def test_busy_interval_wait_does_not_call_sleep(monkeypatch):
    times = iter((1.0, 1.0, 1.001, 1.002))
    monkeypatch.setattr(DIAGNOSTIC.time, "monotonic", lambda: next(times))
    monkeypatch.setattr(
        DIAGNOSTIC.time, "sleep",
        lambda *_: pytest.fail("busy interval unexpectedly slept"),
    )

    DIAGNOSTIC.wait_interval(2.0, "busy")


def test_group_only_phase_is_explicit_and_read_only():
    args = DIAGNOSTIC.build_parser().parse_args([
        "--phase", "driver-group-only", "--ids", "37,31", "--trials", "5000",
    ])

    assert args.phase == DIAGNOSTIC.PHASE_DRIVER_GROUP_ONLY
    assert args.ids == (37, 31)
    assert args.trials == 5000


def test_direct_driver_phase_is_explicit_and_read_only():
    args = DIAGNOSTIC.build_parser().parse_args([
        "--phase", "driver-individual-only", "--ids", "31",
        "--trials", "5000",
    ])

    assert args.phase == DIAGNOSTIC.PHASE_DRIVER_INDIVIDUAL_ONLY
    assert args.ids == (31,)
    assert args.trials == 5000


def test_group_only_run_skips_original_three_phases_but_keeps_full_gate(
        monkeypatch):
    class Serial:
        def fileno(self):
            return 123

    class Port(FakePort):
        def __init__(self, path):
            super().__init__()
            self.path = path
            self.ser = Serial()

        def setBaudRate(self, baud_rate):
            return baud_rate == DIAGNOSTIC.BAUD_RATE

        def clearPort(self):
            pass

        def closePort(self):
            self.ser = None

    sync_calls = []
    torque_ids = []

    monkeypatch.setattr(DIAGNOSTIC, "PortHandler", Port)
    monkeypatch.setattr(DIAGNOSTIC, "PacketHandler", lambda protocol: object())
    monkeypatch.setattr(
        DIAGNOSTIC, "check_external_ownership",
        lambda *args, **kwargs: {"unsafe": False},
    )
    monkeypatch.setattr(DIAGNOSTIC.fcntl, "flock", lambda *args: None)
    monkeypatch.setattr(DIAGNOSTIC.fcntl, "ioctl", lambda *args: None)

    def torque_check(port, packet, ids, timeout):
        del port, packet, timeout
        torque_ids.append(ids)
        return {"all_disabled": True}

    monkeypatch.setattr(DIAGNOSTIC, "verify_torque_disabled", torque_check)
    monkeypatch.setattr(
        DIAGNOSTIC, "read_health_snapshot",
        lambda *args: {"healthy": True},
    )
    monkeypatch.setattr(
        DIAGNOSTIC, "run_individual_trials",
        lambda *args: pytest.fail("group-only invoked an individual read phase"),
    )

    def sync_trials(*args, **kwargs):
        sync_calls.append(args)
        assert kwargs == {
            "retries": 0,
            "drain_after_tx": False,
            "interval_wait": "sleep",
        }
        return {"attempted": 20, "success": 20}

    monkeypatch.setattr(DIAGNOSTIC, "run_sync_trials", sync_trials)
    args = DIAGNOSTIC.build_parser().parse_args([
        "--port", "/dev/null", "--phase", "driver-group-only",
        "--ids", "37,31", "--trials", "20",
    ])

    summary, exit_code = DIAGNOSTIC.run(args)

    assert exit_code == DIAGNOSTIC.EXIT_OK
    assert summary["result"] == "pass"
    assert "individual_present_position" not in summary
    assert "group_sync_read_present_position" not in summary
    assert len(sync_calls) == 1
    assert sync_calls[0][3] == (37, 31)
    assert sync_calls[0][-2:] == (
        DIAGNOSTIC.DRIVER_INDIRECT_ADDRESS,
        DIAGNOSTIC.DRIVER_INDIRECT_LENGTH,
    )
    assert torque_ids == [DIAGNOSTIC.SAFETY_IDS, DIAGNOSTIC.SAFETY_IDS]


def test_source_contains_only_read_side_dynamixel_sdk_operations():
    source = SCRIPT.read_text(encoding="utf-8")
    forbidden_api_tokens = (
        "write1ByteTxRx(",
        "write2ByteTxRx(",
        "write4ByteTxRx(",
        "writeTxRx(",
        "regWriteTxRx(",
        "GroupSyncWrite(",
        "GroupBulkWrite(",
        "reboot(",
        "factoryReset(",
    )

    assert "TORQUE_ENABLE_ADDRESS = 64" in source
    assert "PRESENT_POSITION_ADDRESS = 132" in source
    assert "HARDWARE_ERROR_STATUS_ADDRESS = 70" in source
    assert "PRESENT_INPUT_VOLTAGE_ADDRESS = 144" in source
    assert "PRESENT_TEMPERATURE_ADDRESS = 146" in source
    assert "DRIVER_INDIRECT_ADDRESS = 634" in source
    assert "DRIVER_INDIRECT_LENGTH = 14" in source
    assert source.count(
        "port_handler, packet_handler, SAFETY_IDS, args.timeout_ms"
    ) == 4
    assert "TIOCEXCL" in source
    assert "LOCK_EX | fcntl.LOCK_NB" in source
    assert not any(token in source for token in forbidden_api_tokens)
