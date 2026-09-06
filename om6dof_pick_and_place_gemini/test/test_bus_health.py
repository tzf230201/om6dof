"""Tests for the motion-facing persistent Dynamixel bus-health gate."""

from types import SimpleNamespace

from om6dof_pick_and_place_gemini.bus_health import BusHealthTracker


def diagnostic(*, instance="driver-a", read=0, write=0,
               consecutive_read=0, consecutive_write=0,
               current_read=0, current_write=0, last_error=0,
               last_stamp=0, fail_safe=False, torque=True,
               hardware_error=0, level=0, omit=()):
    values = {
        "schema_version": "1",
        "driver_instance_id": instance,
        "read_failure_count": str(read),
        "write_failure_count": str(write),
        "consecutive_read_failures": str(consecutive_read),
        "consecutive_write_failures": str(consecutive_write),
        "current_read_error": str(current_read),
        "current_write_error": str(current_write),
        "last_comm_error": str(last_error),
        "last_failure_stamp_ns": str(last_stamp),
        "fail_safe_triggered": str(fail_safe).lower(),
        "torque_all_enabled": str(torque).lower(),
        "hardware_error_mask": str(hardware_error),
    }
    for key in omit:
        values.pop(key)
    status = SimpleNamespace(
        name="dynamixel_hardware_interface/BusHealth",
        level=level,
        message="OK" if level == 0 else "fault",
        values=[SimpleNamespace(key=key, value=value)
                for key, value in values.items()],
    )
    return SimpleNamespace(status=[status])


def assess(tracker, now, window=5.0, timeout=0.3):
    return tracker.assess(
        timeout_s=timeout, clean_window_s=window, now=now)


def test_requires_a_fresh_sample_and_the_complete_clean_window():
    tracker = BusHealthTracker()
    assert not assess(tracker, 0.0).ready

    tracker.update(diagnostic(), received_at=10.0)
    waiting = assess(tracker, 12.0, window=5.0, timeout=3.0)
    assert not waiting.ready
    assert "incomplete" in waiting.reason

    tracker.update(diagnostic(), received_at=15.0)
    ready = assess(tracker, 15.0)
    assert ready.ready
    assert ready.read_failure_count == 0
    assert ready.write_failure_count == 0


def test_humble_byte_diagnostic_level_is_normalized_without_weakening_gate():
    tracker = BusHealthTracker()
    tracker.update(diagnostic(level=b"\x00"), received_at=0.0)
    assert assess(tracker, 0.0, window=0.0).ready

    tracker.update(diagnostic(level=b"\x02"), received_at=1.0)
    result = assess(tracker, 1.0, window=0.0)
    assert not result.ready
    assert "not OK" in result.reason


def test_healthy_sample_with_advanced_counter_restarts_the_window():
    tracker = BusHealthTracker()
    tracker.update(diagnostic(read=4, write=2), received_at=0.0)
    tracker.update(diagnostic(read=4, write=2), received_at=5.0)
    assert assess(tracker, 5.0).ready

    # This is the important dropped-error-frame case: the diagnostic itself is
    # already OK again, but its persistent read counter exposes the fault.
    tracker.update(diagnostic(read=5, write=2, last_error=-7,
                              last_stamp=6_000_000_000), received_at=6.0)
    result = assess(tracker, 6.0)
    assert not result.ready
    assert result.clean_duration_s == 0.0
    assert result.read_failure_count == 5

    tracker.update(diagnostic(read=5, write=2, last_error=-7,
                              last_stamp=6_000_000_000), received_at=11.0)
    assert assess(tracker, 11.0).ready


def test_live_error_and_recovery_require_a_new_clean_window():
    tracker = BusHealthTracker()
    tracker.update(diagnostic(), received_at=0.0)
    tracker.update(diagnostic(), received_at=5.0)
    assert assess(tracker, 5.0).ready

    tracker.update(diagnostic(
        read=1, consecutive_read=1, current_read=-7, last_error=-7,
        last_stamp=6_000_000_000, level=2), received_at=6.0)
    failed = assess(tracker, 6.0)
    assert not failed.ready
    assert "read communication" in failed.reason

    tracker.update(diagnostic(
        read=1, last_error=-7, last_stamp=6_000_000_000),
        received_at=7.0)
    assert not assess(tracker, 11.9, timeout=5.0).ready
    assert assess(tracker, 12.0, timeout=5.1).ready


def test_driver_restart_or_counter_rollback_never_inherits_clean_time():
    tracker = BusHealthTracker()
    tracker.update(diagnostic(read=8), received_at=0.0)
    tracker.update(diagnostic(read=8), received_at=5.0)
    assert assess(tracker, 5.0).ready

    tracker.update(diagnostic(instance="driver-b", read=0), received_at=6.0)
    assert not assess(tracker, 6.0).ready
    tracker.update(diagnostic(instance="driver-b", read=0), received_at=11.0)
    assert assess(tracker, 11.0).ready

    tracker.update(diagnostic(instance="driver-b", read=9), received_at=12.0)
    tracker.update(diagnostic(instance="driver-b", read=2), received_at=13.0)
    assert not assess(tracker, 13.0).ready


def test_stale_malformed_torque_and_hardware_faults_fail_closed():
    tracker = BusHealthTracker()
    tracker.update(diagnostic(), received_at=0.0)
    assert not assess(tracker, 1.0, window=0.0).ready
    assert "stale" in assess(tracker, 1.0, window=0.0).reason

    tracker.update(diagnostic(omit=("write_failure_count",)), received_at=2.0)
    malformed = assess(tracker, 2.0, window=0.0)
    assert not malformed.ready
    assert "missing fields" in malformed.reason

    tracker.update(diagnostic(torque=False, level=2), received_at=3.0)
    torque = assess(tracker, 3.0, window=0.0)
    assert not torque.ready
    assert "torque" in torque.reason

    tracker.update(diagnostic(hardware_error=4, level=2), received_at=4.0)
    hardware = assess(tracker, 4.0, window=0.0)
    assert not hardware.ready
    assert "0x4" in hardware.reason


def test_fail_safe_latch_and_invalid_configuration_fail_closed():
    tracker = BusHealthTracker()
    tracker.update(diagnostic(fail_safe=True, level=2), received_at=1.0)
    assert "fail-safe" in assess(tracker, 1.0, window=0.0).reason

    tracker.update(diagnostic(), received_at=2.0)
    assert not tracker.assess(
        timeout_s=0.0, clean_window_s=0.0, now=2.0).ready
    assert not tracker.assess(
        timeout_s=1.0, clean_window_s=-1.0, now=2.0).ready
