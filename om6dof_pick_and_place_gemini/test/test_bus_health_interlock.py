"""Node integration tests for the Dynamixel bus-health execution interlock."""

import threading
import time
from types import SimpleNamespace

import pytest

rclpy = pytest.importorskip("rclpy")

from om6dof_pick_and_place_gemini.bus_health import BusHealthTracker  # noqa: E402
from om6dof_pick_and_place_gemini.gemini_pick_node import GeminiPickNode  # noqa: E402


def diagnostic(read_count=0):
    fields = {
        "schema_version": "1",
        "driver_instance_id": "driver-a",
        "read_failure_count": str(read_count),
        "write_failure_count": "0",
        "consecutive_read_failures": "0",
        "consecutive_write_failures": "0",
        "current_read_error": "0",
        "current_write_error": "0",
        "last_comm_error": "-7" if read_count else "0",
        "last_failure_stamp_ns": "1" if read_count else "0",
        "fail_safe_triggered": "false",
        "torque_all_enabled": "true",
        "hardware_error_mask": "0",
    }
    status = SimpleNamespace(
        name="dynamixel_hardware_interface/BusHealth", level=0, message="OK",
        values=[SimpleNamespace(key=key, value=value)
                for key, value in fields.items()])
    return SimpleNamespace(status=[status])


def test_counter_increment_during_motion_trips_the_existing_cancel_path():
    node = object.__new__(GeminiPickNode)
    node._bus_health = BusHealthTracker()
    node._worker_lock = threading.Lock()
    node._motion_sequence_active = True
    node._stop_requested = False
    values = {
        "execute_motion": True,
        "dynamixel_health_timeout_s": 0.3,
        "dynamixel_health_clean_window_s": 1.0,
    }
    node._param = values.__getitem__
    trips = []
    node._trip_execution_interlock = trips.append

    now = time.monotonic()
    node._bus_health.update(diagnostic(0), received_at=now - 2.0)
    node._bus_health.update(diagnostic(0), received_at=now)
    assert node._bus_health_assessment().ready

    GeminiPickNode._on_dynamixel_health(node, diagnostic(1))

    assert len(trips) == 1
    assert "failure counter advanced" in trips[0]


def test_periodic_monitor_trips_when_health_topic_becomes_stale():
    node = object.__new__(GeminiPickNode)
    node._worker_lock = threading.Lock()
    node._motion_sequence_active = True
    node._stop_requested = False
    node.moveit = SimpleNamespace(
        physical_action_in_flight=False, motion_faulted=False)
    node._param = lambda name: {"execute_motion": True}[name]
    node._execution_interlocks_ready = lambda: False
    trips = []
    node._trip_execution_interlock = trips.append

    GeminiPickNode._monitor_execution_interlocks(node)

    assert trips == ["controller state became unavailable or unsafe"]


def test_safe_controller_state_still_refuses_an_unhealthy_bus():
    node = object.__new__(GeminiPickNode)
    node._lock = threading.Lock()
    node._operation_mode = "AUTONOMOUS"
    node._remote_enabled = False
    node._param = lambda name: {
        "operation_mode_state_topic": "/mode",
        "remote_enabled_state_topic": "/remote",
    }[name]
    node.count_publishers = lambda _topic: 1
    node._dynamixel_bus_ready = lambda: False
    node.get_logger = lambda: SimpleNamespace(error=lambda _message: None)

    assert not GeminiPickNode._execution_interlocks_ready(node)


def test_multiple_health_publishers_fail_closed():
    errors = []
    node = object.__new__(GeminiPickNode)
    node._param = lambda name: {
        "dynamixel_health_topic": "/dynamixel_hardware_interface/health",
    }[name]
    node.count_publishers = lambda _topic: 2
    node._last_bus_health_log_reason = ""
    node._last_bus_health_log_stamp = 0.0
    node.get_logger = lambda: SimpleNamespace(error=errors.append)

    assert not GeminiPickNode._dynamixel_bus_ready(node)
    assert errors and "exactly one publisher" in errors[0]
