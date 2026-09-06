"""Fail-closed consumer for the persistent Dynamixel bus-health diagnostic."""

from __future__ import annotations

import math
import threading
import time
from dataclasses import asdict, dataclass
from typing import Dict, Optional


DEFAULT_STATUS_NAME = "dynamixel_hardware_interface/BusHealth"
SCHEMA_VERSION = 1
_REQUIRED_FIELDS = {
    "schema_version",
    "driver_instance_id",
    "read_failure_count",
    "write_failure_count",
    "consecutive_read_failures",
    "consecutive_write_failures",
    "current_read_error",
    "current_write_error",
    "last_comm_error",
    "last_failure_stamp_ns",
    "fail_safe_triggered",
    "torque_all_enabled",
    "hardware_error_mask",
}


@dataclass(frozen=True)
class BusHealthAssessment:
    """One immutable answer used by preflight, status, and the live monitor."""

    ready: bool
    reason: str
    sample_age_s: Optional[float]
    clean_duration_s: float
    clean_window_s: float
    driver_instance_id: str
    read_failure_count: Optional[int]
    write_failure_count: Optional[int]
    last_comm_error: Optional[int]
    last_failure_stamp_ns: Optional[int]

    def as_dict(self) -> Dict[str, object]:
        return asdict(self)


class BusHealthTracker:
    """Track persistent counters and require an uninterrupted clean window.

    Counters, rather than a single error-valued sample, are the safety signal.
    If an error DiagnosticArray is dropped, the next healthy message still
    carries the increment and restarts the clean window.
    """

    def __init__(self, status_name: str = DEFAULT_STATUS_NAME) -> None:
        self.status_name = str(status_name)
        self._lock = threading.Lock()
        self._valid = False
        self._reported_healthy = False
        self._last_receive_at: Optional[float] = None
        self._clean_since: Optional[float] = None
        self._reason = "no Dynamixel bus-health sample received"
        self._driver_instance_id = ""
        self._read_failure_count: Optional[int] = None
        self._write_failure_count: Optional[int] = None
        self._last_comm_error: Optional[int] = None
        self._last_failure_stamp_ns: Optional[int] = None

    @staticmethod
    def _parse_bool(value: str, name: str) -> bool:
        normalized = str(value).strip().lower()
        if normalized in ("true", "1"):
            return True
        if normalized in ("false", "0"):
            return False
        raise ValueError(f"{name} is not a boolean")

    @staticmethod
    def _parse_int(fields: Dict[str, str], name: str,
                   *, nonnegative: bool = False) -> int:
        try:
            value = int(fields[name])
        except (KeyError, TypeError, ValueError) as exc:
            raise ValueError(f"{name} is not an integer") from exc
        if nonnegative and value < 0:
            raise ValueError(f"{name} must be non-negative")
        return value

    @staticmethod
    def _parse_level(value) -> int:
        """Normalize DiagnosticStatus.level across rclpy representations.

        Humble may expose the IDL ``byte`` field as a one-byte ``bytes``
        object (for example ``b'\\x00'``) instead of a Python integer.
        """
        if isinstance(value, (bytes, bytearray, memoryview)):
            raw = bytes(value)
            if len(raw) != 1:
                raise ValueError("diagnostic level is not one byte")
            level = raw[0]
        elif isinstance(value, str) and len(value) == 1 \
                and not value.isdigit():
            level = ord(value)
        else:
            level = int(value)
        if level not in (0, 1, 2, 3):
            raise ValueError(f"diagnostic level {level} is invalid")
        return level

    def _invalidate(self, reason: str, received_at: float) -> None:
        with self._lock:
            self._valid = False
            self._reported_healthy = False
            self._last_receive_at = received_at
            self._clean_since = None
            self._reason = reason

    def update(self, message, received_at: Optional[float] = None) -> None:
        """Consume a ``diagnostic_msgs/DiagnosticArray``-shaped object."""
        now = time.monotonic() if received_at is None else float(received_at)
        if not math.isfinite(now):
            return
        matches = [status for status in getattr(message, "status", [])
                   if str(getattr(status, "name", "")) == self.status_name]
        if len(matches) != 1:
            self._invalidate(
                f"expected one diagnostic named {self.status_name!r}, "
                f"received {len(matches)}", now)
            return

        status = matches[0]
        pairs = list(getattr(status, "values", []))
        fields = {str(getattr(item, "key", "")):
                  str(getattr(item, "value", "")) for item in pairs}
        if len(fields) != len(pairs):
            self._invalidate("Dynamixel health contains duplicate keys", now)
            return
        missing = sorted(_REQUIRED_FIELDS.difference(fields))
        if missing:
            self._invalidate(
                "Dynamixel health is missing fields: " + ", ".join(missing),
                now)
            return

        try:
            schema = self._parse_int(fields, "schema_version",
                                     nonnegative=True)
            if schema != SCHEMA_VERSION:
                raise ValueError(
                    f"unsupported Dynamixel health schema {schema}")
            instance = fields["driver_instance_id"].strip()
            if not instance:
                raise ValueError("driver_instance_id is empty")
            read_count = self._parse_int(
                fields, "read_failure_count", nonnegative=True)
            write_count = self._parse_int(
                fields, "write_failure_count", nonnegative=True)
            consecutive_read = self._parse_int(
                fields, "consecutive_read_failures", nonnegative=True)
            consecutive_write = self._parse_int(
                fields, "consecutive_write_failures", nonnegative=True)
            current_read = self._parse_int(fields, "current_read_error")
            current_write = self._parse_int(fields, "current_write_error")
            last_error = self._parse_int(fields, "last_comm_error")
            last_stamp = self._parse_int(
                fields, "last_failure_stamp_ns", nonnegative=True)
            fail_safe = self._parse_bool(
                fields["fail_safe_triggered"], "fail_safe_triggered")
            torque_enabled = self._parse_bool(
                fields["torque_all_enabled"], "torque_all_enabled")
            hardware_error = self._parse_int(
                fields, "hardware_error_mask", nonnegative=True)
            level = self._parse_level(getattr(status, "level", -1))
        except (TypeError, ValueError) as exc:
            self._invalidate(f"invalid Dynamixel health: {exc}", now)
            return

        reported_healthy = (
            level == 0 and not fail_safe and torque_enabled
            and hardware_error == 0 and consecutive_read == 0
            and consecutive_write == 0 and current_read == 0
            and current_write == 0)
        if fail_safe:
            unhealthy_reason = "Dynamixel driver fail-safe is latched"
        elif hardware_error:
            unhealthy_reason = (
                f"Dynamixel hardware error mask is 0x{hardware_error:x}")
        elif not torque_enabled:
            unhealthy_reason = "Dynamixel torque is not enabled on every actuator"
        elif consecutive_read or current_read:
            unhealthy_reason = (
                "Dynamixel read communication is failing "
                f"(consecutive={consecutive_read}, error={current_read})")
        elif consecutive_write or current_write:
            unhealthy_reason = (
                "Dynamixel write communication is failing "
                f"(consecutive={consecutive_write}, error={current_write})")
        elif level != 0:
            unhealthy_reason = (
                "Dynamixel diagnostic is not OK: "
                f"{getattr(status, 'message', '')}")
        else:
            unhealthy_reason = "Dynamixel health fields are inconsistent"

        with self._lock:
            previous_receive = self._last_receive_at
            previous_instance = self._driver_instance_id
            previous_read = self._read_failure_count
            previous_write = self._write_failure_count
            previous_healthy = self._reported_healthy

            reset_reason = ""
            if previous_receive is not None and now < previous_receive:
                reported_healthy = False
                unhealthy_reason = "monotonic receive time moved backwards"
            elif previous_instance and instance != previous_instance:
                reset_reason = "Dynamixel driver instance changed"
            elif (previous_read is not None and previous_write is not None
                  and (read_count < previous_read
                       or write_count < previous_write)):
                reset_reason = "Dynamixel failure counter reset"
            elif (previous_read is not None and previous_write is not None
                  and (read_count > previous_read
                       or write_count > previous_write)):
                reset_reason = (
                    "Dynamixel failure counter advanced "
                    f"(read {previous_read}->{read_count}, "
                    f"write {previous_write}->{write_count})")

            self._valid = True
            self._reported_healthy = reported_healthy
            self._last_receive_at = now
            self._driver_instance_id = instance
            self._read_failure_count = read_count
            self._write_failure_count = write_count
            self._last_comm_error = last_error
            self._last_failure_stamp_ns = last_stamp

            if not reported_healthy:
                self._clean_since = None
                self._reason = unhealthy_reason
            elif (reset_reason or not previous_healthy
                  or self._clean_since is None):
                self._clean_since = now
                self._reason = reset_reason or "collecting clean bus-health window"
            else:
                self._reason = "Dynamixel bus-health window is clean"

    def assess(self, *, timeout_s: float, clean_window_s: float,
               now: Optional[float] = None) -> BusHealthAssessment:
        """Return whether an execution command may use the Dynamixel bus."""
        current = time.monotonic() if now is None else float(now)
        timeout = float(timeout_s)
        clean_window = float(clean_window_s)
        with self._lock:
            receive_at = self._last_receive_at
            clean_since = self._clean_since
            valid = self._valid
            reported_healthy = self._reported_healthy
            reason = self._reason
            instance = self._driver_instance_id
            read_count = self._read_failure_count
            write_count = self._write_failure_count
            last_error = self._last_comm_error
            last_stamp = self._last_failure_stamp_ns

        age = None if receive_at is None else current - receive_at
        clean_duration = (0.0 if clean_since is None
                          else max(0.0, current - clean_since))
        if not math.isfinite(timeout) or timeout <= 0.0:
            ready = False
            reason = "dynamixel_health_timeout_s must be positive"
        elif not math.isfinite(clean_window) or clean_window < 0.0:
            ready = False
            reason = "dynamixel_health_clean_window_s must be non-negative"
        elif not math.isfinite(current):
            ready = False
            reason = "bus-health assessment time is invalid"
        elif not valid or receive_at is None:
            ready = False
        elif age is None or age < 0.0 or age > timeout:
            ready = False
            reason = (
                "Dynamixel bus-health sample is stale "
                f"(age={age:.3f}s, limit={timeout:.3f}s)")
        elif not reported_healthy or clean_since is None:
            ready = False
        elif clean_duration < clean_window:
            ready = False
            detail = (
                "Dynamixel bus clean window is incomplete "
                f"({clean_duration:.1f}/{clean_window:.1f}s)")
            if reason not in ("collecting clean bus-health window",
                              "Dynamixel bus-health window is clean"):
                reason = f"{reason}; {detail}"
            else:
                reason = detail
        else:
            ready = True
            reason = (
                "Dynamixel bus healthy "
                f"({clean_duration:.1f}s clean, read_failures={read_count}, "
                f"write_failures={write_count})")

        return BusHealthAssessment(
            ready=ready, reason=reason, sample_age_s=age,
            clean_duration_s=clean_duration, clean_window_s=clean_window,
            driver_instance_id=instance,
            read_failure_count=read_count,
            write_failure_count=write_count,
            last_comm_error=last_error,
            last_failure_stamp_ns=last_stamp)
