#!/usr/bin/env python3
"""Fail-closed semantic scene-readiness gate for the four-object experiment.

The checker is read-only: it subscribes to a ``std_msgs/msg/String`` topic and
never creates a publisher, service client, action client, or motion command.
The current topology node publishes a JSON array whose entries contain at
least ``class`` and the stable DD-GNG ``node_id``.  A scene is ready only when
all required classes have enough *unique* supporting nodes in qualifying
messages spanning the requested continuous interval.

ROS imports are deliberately lazy so the parser and stability logic can be
unit-tested on machines without ROS 2 installed.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import sys
import time
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Callable, Sequence


SCHEMA_VERSION = 1
DEFAULT_TOPIC = "/om6dof_topo_gng_v2/labels"
DEFAULT_CLASSES = ("bottle", "cup", "banana", "remote")


class LabelPayloadError(ValueError):
    """The label payload cannot be trusted as a semantic observation."""

    def __init__(self, issues: Sequence[str]):
        self.issues = tuple(issues)
        super().__init__("; ".join(self.issues))


@dataclass(frozen=True)
class LabelSnapshot:
    """Validated class support reconstructed from one labels message."""

    counts: dict[str, int]
    total_entries: int
    ignored_unknown_entries: int
    duplicate_entries: int


def _is_node_id(value: Any) -> bool:
    # bool is a subclass of int in Python and must not be accepted as an ID.
    return isinstance(value, int) and not isinstance(value, bool) and value >= 0


def parse_label_payload(payload: str, required_classes: Sequence[str]) -> LabelSnapshot:
    """Parse the live ``String.data`` JSON schema and count unique node support.

    Only fields used by the readiness decision are required.  Extra fields
    (currently ``index``, ``confidence``, and ``x/y/z``) are accepted so a
    harmless publisher extension cannot silently break the gate.  Any entry
    without a valid class or non-negative integer ``node_id`` invalidates the
    complete message; partially corrupt messages are never used as evidence.
    """

    if not isinstance(payload, str):
        raise LabelPayloadError(("payload is not a string",))
    try:
        decoded = json.loads(payload)
    except (json.JSONDecodeError, TypeError) as exc:
        raise LabelPayloadError((f"invalid JSON: {exc}",)) from exc

    if not isinstance(decoded, list):
        raise LabelPayloadError(("top-level JSON value must be an array",))

    required = set(required_classes)
    support: dict[str, set[int]] = {name: set() for name in required_classes}
    node_owner: dict[int, str] = {}
    issues: list[str] = []
    ignored_unknown = 0
    duplicates = 0

    for offset, entry in enumerate(decoded):
        if not isinstance(entry, dict):
            issues.append(f"entry[{offset}] is not an object")
            continue

        class_name = entry.get("class")
        node_id = entry.get("node_id")
        if not isinstance(class_name, str) or not class_name.strip():
            issues.append(f"entry[{offset}].class must be a non-empty string")
            continue
        class_name = class_name.strip().lower()
        if not _is_node_id(node_id):
            issues.append(f"entry[{offset}].node_id must be a non-negative integer")
            continue

        previous_owner = node_owner.get(node_id)
        if previous_owner is not None and previous_owner != class_name:
            issues.append(
                f"node_id {node_id} has conflicting classes "
                f"{previous_owner!r} and {class_name!r}"
            )
            continue
        node_owner[node_id] = class_name

        if class_name not in required:
            ignored_unknown += 1
            continue
        if node_id in support[class_name]:
            duplicates += 1
            continue
        support[class_name].add(node_id)

    if issues:
        raise LabelPayloadError(issues)

    return LabelSnapshot(
        counts={name: len(support[name]) for name in required_classes},
        total_entries=len(decoded),
        ignored_unknown_entries=ignored_unknown,
        duplicate_entries=duplicates,
    )


class StabilityTracker:
    """Track message-backed, simultaneous semantic stability using monotonic time."""

    def __init__(
        self,
        required_classes: Sequence[str],
        min_supporting_nodes: int,
        stable_duration_sec: float,
        max_message_gap_sec: float,
    ) -> None:
        self.required_classes = tuple(required_classes)
        self.min_supporting_nodes = min_supporting_nodes
        self.stable_duration_ns = round(stable_duration_sec * 1_000_000_000)
        self.max_message_gap_ns = round(max_message_gap_sec * 1_000_000_000)

        self.messages_total = 0
        self.messages_valid = 0
        self.messages_malformed = 0
        self.entries_total = 0
        self.entries_unknown = 0
        self.entries_duplicate = 0
        self.gap_resets = 0
        self.malformed_resets = 0
        self.threshold_resets = 0
        self.last_schema_error = ""

        self.first_message_ns: int | None = None
        self.last_message_ns: int | None = None
        self.last_valid_message_ns: int | None = None
        self.current_counts = {name: 0 for name in self.required_classes}
        self.maximum_counts = {name: 0 for name in self.required_classes}
        self.qualifying_samples = {name: 0 for name in self.required_classes}
        self.class_ready_since_ns: dict[str, int | None] = {
            name: None for name in self.required_classes
        }
        self.class_last_qualifying_ns: dict[str, int | None] = {
            name: None for name in self.required_classes
        }
        self.class_longest_ns = {name: 0 for name in self.required_classes}
        self.all_ready_since_ns: int | None = None
        self.all_last_qualifying_ns: int | None = None
        self.all_longest_ns = 0
        self.all_qualifying_messages = 0
        self.ready = False
        self._gap_is_open = False

    def _close_intervals(self, timestamp_ns: int) -> None:
        for name, started_ns in self.class_ready_since_ns.items():
            if started_ns is not None:
                supported_until_ns = self.class_last_qualifying_ns[name]
                end_ns = supported_until_ns if supported_until_ns is not None else timestamp_ns
                self.class_longest_ns[name] = max(
                    self.class_longest_ns[name], max(0, end_ns - started_ns)
                )
                self.class_ready_since_ns[name] = None
                self.class_last_qualifying_ns[name] = None
        if self.all_ready_since_ns is not None:
            end_ns = (
                self.all_last_qualifying_ns
                if self.all_last_qualifying_ns is not None
                else timestamp_ns
            )
            self.all_longest_ns = max(
                self.all_longest_ns, max(0, end_ns - self.all_ready_since_ns)
            )
            self.all_ready_since_ns = None
            self.all_last_qualifying_ns = None

    def _break_continuity(self, timestamp_ns: int, reason: str) -> None:
        self._close_intervals(timestamp_ns)
        self.current_counts = {name: 0 for name in self.required_classes}
        if reason == "gap":
            self.gap_resets += 1
        elif reason == "malformed":
            self.malformed_resets += 1
        elif reason == "threshold":
            self.threshold_resets += 1

    def observe(self, payload: str, timestamp_ns: int) -> bool:
        """Consume one message. Return True only after repeated stable evidence."""

        if self.last_message_ns is not None:
            gap_ns = timestamp_ns - self.last_message_ns
            if gap_ns > self.max_message_gap_ns:
                # Evidence ends at the preceding observation, not at the end of
                # an unobserved interval.  This is intentionally conservative.
                self._break_continuity(self.last_message_ns, "gap")

        self._gap_is_open = False
        self.messages_total += 1
        if self.first_message_ns is None:
            self.first_message_ns = timestamp_ns
        self.last_message_ns = timestamp_ns

        try:
            snapshot = parse_label_payload(payload, self.required_classes)
        except LabelPayloadError as exc:
            self.messages_malformed += 1
            self.last_schema_error = str(exc)
            self._break_continuity(timestamp_ns, "malformed")
            return False

        self.messages_valid += 1
        self.last_valid_message_ns = timestamp_ns
        self.entries_total += snapshot.total_entries
        self.entries_unknown += snapshot.ignored_unknown_entries
        self.entries_duplicate += snapshot.duplicate_entries
        self.current_counts = dict(snapshot.counts)

        for name, count in snapshot.counts.items():
            self.maximum_counts[name] = max(self.maximum_counts[name], count)
            if count >= self.min_supporting_nodes:
                self.qualifying_samples[name] += 1
                started_ns = self.class_ready_since_ns[name]
                if started_ns is None:
                    started_ns = timestamp_ns
                    self.class_ready_since_ns[name] = started_ns
                self.class_last_qualifying_ns[name] = timestamp_ns
                self.class_longest_ns[name] = max(
                    self.class_longest_ns[name], timestamp_ns - started_ns
                )
            else:
                started_ns = self.class_ready_since_ns[name]
                if started_ns is not None:
                    supported_until_ns = self.class_last_qualifying_ns[name]
                    end_ns = (
                        supported_until_ns
                        if supported_until_ns is not None
                        else timestamp_ns
                    )
                    self.class_longest_ns[name] = max(
                        self.class_longest_ns[name], max(0, end_ns - started_ns)
                    )
                    self.class_ready_since_ns[name] = None
                    self.class_last_qualifying_ns[name] = None

        all_above_threshold = all(
            snapshot.counts[name] >= self.min_supporting_nodes
            for name in self.required_classes
        )
        if all_above_threshold:
            self.all_qualifying_messages += 1
            if self.all_ready_since_ns is None:
                self.all_ready_since_ns = timestamp_ns
            self.all_last_qualifying_ns = timestamp_ns
            self.all_longest_ns = max(
                self.all_longest_ns, timestamp_ns - self.all_ready_since_ns
            )
            # Success is checked only on receipt of another qualifying message,
            # never by extrapolating a lone observation through wall-clock time.
            self.ready = (
                timestamp_ns - self.all_ready_since_ns >= self.stable_duration_ns
            )
        else:
            if self.all_ready_since_ns is not None:
                end_ns = (
                    self.all_last_qualifying_ns
                    if self.all_last_qualifying_ns is not None
                    else timestamp_ns
                )
                self.all_longest_ns = max(
                    self.all_longest_ns, max(0, end_ns - self.all_ready_since_ns)
                )
                self.all_ready_since_ns = None
                self.all_last_qualifying_ns = None
                self.threshold_resets += 1
            self.ready = False
        return self.ready

    def poll_for_gap(self, timestamp_ns: int) -> None:
        """Invalidate an open interval if the publisher goes silent."""

        if self.last_message_ns is None or self._gap_is_open:
            return
        if timestamp_ns - self.last_message_ns > self.max_message_gap_ns:
            self._break_continuity(self.last_message_ns, "gap")
            self._gap_is_open = True
            self.ready = False

    @staticmethod
    def _seconds(value_ns: int) -> float:
        return round(value_ns / 1_000_000_000.0, 6)

    def report_fields(self) -> dict[str, Any]:
        return {
            "message_statistics": {
                "total": self.messages_total,
                "valid": self.messages_valid,
                "malformed": self.messages_malformed,
                "last_schema_error": self.last_schema_error or None,
                "gap_resets": self.gap_resets,
                "malformed_resets": self.malformed_resets,
                "threshold_resets": self.threshold_resets,
                "all_classes_qualifying_messages": self.all_qualifying_messages,
            },
            "entry_statistics": {
                "total": self.entries_total,
                "ignored_unknown_class": self.entries_unknown,
                "deduplicated": self.entries_duplicate,
            },
            "class_support": {
                name: {
                    "current_unique_nodes": self.current_counts[name],
                    "maximum_unique_nodes": self.maximum_counts[name],
                    "qualifying_messages": self.qualifying_samples[name],
                    "longest_continuous_sec": self._seconds(self.class_longest_ns[name]),
                    "meets_threshold_now": (
                        self.current_counts[name] >= self.min_supporting_nodes
                    ),
                }
                for name in self.required_classes
            },
            "longest_all_classes_continuous_sec": self._seconds(self.all_longest_ns),
        }


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat(timespec="milliseconds")


def parse_classes(csv_value: str) -> tuple[str, ...]:
    values = tuple(item.strip().lower() for item in csv_value.split(",") if item.strip())
    if not values:
        raise ValueError("required classes cannot be empty")
    if len(set(values)) != len(values):
        raise ValueError("required classes must be unique")
    return values


def classify_timeout(tracker: StabilityTracker) -> tuple[str, list[str], list[str]]:
    if tracker.messages_total == 0:
        return "timeout_no_messages", list(tracker.required_classes), list(tracker.required_classes)
    if tracker.messages_valid == 0:
        return "timeout_no_valid_messages", list(tracker.required_classes), list(tracker.required_classes)

    never_supported = [
        name
        for name in tracker.required_classes
        if tracker.maximum_counts[name] < tracker.min_supporting_nodes
    ]
    individually_unstable = [
        name
        for name in tracker.required_classes
        if tracker.class_longest_ns[name] < tracker.stable_duration_ns
    ]
    if never_supported:
        return "timeout_missing_classes", never_supported, individually_unstable
    if individually_unstable:
        return "timeout_unstable_classes", [], individually_unstable
    return "timeout_never_simultaneously_stable", [], []


def base_report(args: argparse.Namespace, required_classes: Sequence[str]) -> dict[str, Any]:
    return {
        "schema_version": SCHEMA_VERSION,
        "gate": "four_object_semantic_scene_readiness",
        "mode": args.mode,
        "topic": args.topic,
        "semantic_presence_required": args.mode == "experiment",
        "configuration": {
            "required_classes": list(required_classes),
            "min_supporting_nodes_per_class": args.min_supporting_nodes,
            "stable_duration_sec": args.stable_duration_sec,
            "max_message_gap_sec": args.max_message_gap_sec,
            "timeout_sec": args.timeout_sec,
        },
        "started_utc": utc_now(),
    }


def emit_report(report: dict[str, Any], report_path: str | None) -> None:
    report_text = json.dumps(report, sort_keys=True, separators=(",", ":"), allow_nan=False)
    if report_path:
        output = Path(report_path).expanduser()
        output.parent.mkdir(parents=True, exist_ok=True)
        temporary = output.with_name(output.name + ".tmp")
        temporary.write_text(report_text + "\n", encoding="utf-8")
        temporary.replace(output)
    print(report_text, flush=True)


def numeric_env(name: str, default: str) -> str:
    return os.environ.get(name, default)


def build_argument_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Verify stable DD-GNG semantic support for bottle,cup,banana,remote. "
            "This checker is read-only and never commands the robot."
        )
    )
    parser.add_argument(
        "--mode",
        choices=("experiment", "instrumentation"),
        default=os.environ.get("SCENE_READINESS_MODE", "experiment"),
        help="experiment enforces semantic presence; instrumentation explicitly skips it",
    )
    parser.add_argument(
        "--topic", default=os.environ.get("SCENE_LABELS_TOPIC", DEFAULT_TOPIC)
    )
    parser.add_argument(
        "--required-classes",
        default=os.environ.get("SCENE_REQUIRED_CLASSES", ",".join(DEFAULT_CLASSES)),
        help="comma-separated COCO class names (default: frozen four-object set)",
    )
    parser.add_argument(
        "--min-supporting-nodes",
        type=int,
        default=int(numeric_env("SCENE_MIN_SUPPORTING_NODES", "3")),
    )
    parser.add_argument(
        "--stable-duration-sec",
        type=float,
        default=float(numeric_env("SCENE_STABLE_DURATION_SEC", "1.0")),
    )
    parser.add_argument(
        "--max-message-gap-sec",
        type=float,
        default=float(numeric_env("SCENE_MAX_MESSAGE_GAP_SEC", "0.5")),
    )
    parser.add_argument(
        "--timeout-sec",
        type=float,
        default=float(numeric_env("SCENE_READINESS_TIMEOUT_SEC", "15.0")),
    )
    parser.add_argument(
        "--poll-period-sec",
        type=float,
        default=float(numeric_env("SCENE_READINESS_POLL_SEC", "0.05")),
        help=argparse.SUPPRESS,
    )
    parser.add_argument(
        "--report-json",
        default=os.environ.get("SCENE_READINESS_REPORT_JSON"),
        help="optional file receiving the same single-object JSON report as stdout",
    )
    parser.add_argument("--node-name", default="four_object_scene_readiness")
    return parser


def validate_arguments(args: argparse.Namespace) -> tuple[tuple[str, ...], list[str]]:
    errors: list[str] = []
    try:
        required_classes = parse_classes(args.required_classes)
    except ValueError as exc:
        required_classes = DEFAULT_CLASSES
        errors.append(str(exc))
    if args.min_supporting_nodes < 1:
        errors.append("min-supporting-nodes must be at least 1")
    for field in (
        "stable_duration_sec",
        "max_message_gap_sec",
        "timeout_sec",
        "poll_period_sec",
    ):
        value = getattr(args, field)
        if not math.isfinite(value) or value <= 0:
            errors.append(field.replace("_", "-") + " must be finite and greater than 0")
    if (
        math.isfinite(args.timeout_sec)
        and math.isfinite(args.stable_duration_sec)
        and args.timeout_sec < args.stable_duration_sec
    ):
        errors.append("timeout-sec must be at least stable-duration-sec")
    if not args.topic.startswith("/"):
        errors.append("topic must be an absolute ROS name beginning with '/'")
    return required_classes, errors


def run_ros_gate(
    args: argparse.Namespace,
    required_classes: Sequence[str],
    monotonic_ns: Callable[[], int] = time.monotonic_ns,
) -> tuple[int, dict[str, Any]]:
    report = base_report(args, required_classes)
    started_ns = monotonic_ns()
    tracker = StabilityTracker(
        required_classes,
        args.min_supporting_nodes,
        args.stable_duration_sec,
        args.max_message_gap_sec,
    )

    try:
        import rclpy
        from rclpy.node import Node
        from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
        from std_msgs.msg import String
    except ImportError as exc:
        report.update(
            {
                "outcome": "ros_import_error",
                "gate_passed": False,
                "semantic_ready": False,
                "error": str(exc),
                "elapsed_sec": 0.0,
                "finished_utc": utc_now(),
                "exit_code": 2,
            }
        )
        report.update(tracker.report_fields())
        return 2, report

    # argparse has already consumed this program's options; do not ask rclpy
    # to reinterpret them as ROS arguments.
    rclpy.init(args=[])
    node: Node | None = None
    try:
        node = Node(args.node_name)
        qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )

        def on_labels(message: String) -> None:
            tracker.observe(message.data, monotonic_ns())

        # Keep the returned subscription alive for the lifetime of the node.
        subscription = node.create_subscription(String, args.topic, on_labels, qos)
        del subscription  # rclpy Node owns the subscription after creation.

        deadline_ns = started_ns + round(args.timeout_sec * 1_000_000_000)
        poll_sec = min(args.poll_period_sec, args.max_message_gap_sec / 2.0)
        while rclpy.ok() and monotonic_ns() < deadline_ns and not tracker.ready:
            remaining_sec = max(0.0, (deadline_ns - monotonic_ns()) / 1_000_000_000.0)
            rclpy.spin_once(node, timeout_sec=min(poll_sec, remaining_sec))
            tracker.poll_for_gap(monotonic_ns())

        finished_ns = monotonic_ns()
        if tracker.ready:
            outcome = "semantic_ready"
            missing_classes: list[str] = []
            unstable_classes: list[str] = []
            exit_code = 0
        else:
            outcome, missing_classes, unstable_classes = classify_timeout(tracker)
            exit_code = 1
        report.update(
            {
                "outcome": outcome,
                "gate_passed": exit_code == 0,
                "semantic_ready": tracker.ready,
                "missing_classes": missing_classes,
                "unstable_classes": unstable_classes,
                "elapsed_sec": round((finished_ns - started_ns) / 1_000_000_000.0, 6),
                "finished_utc": utc_now(),
                "exit_code": exit_code,
            }
        )
        report.update(tracker.report_fields())
        return exit_code, report
    except KeyboardInterrupt:
        finished_ns = monotonic_ns()
        report.update(
            {
                "outcome": "interrupted",
                "gate_passed": False,
                "semantic_ready": False,
                "elapsed_sec": round((finished_ns - started_ns) / 1_000_000_000.0, 6),
                "finished_utc": utc_now(),
                "exit_code": 130,
            }
        )
        report.update(tracker.report_fields())
        return 130, report
    except Exception as exc:  # Report middleware/runtime failures as data, not a traceback-only result.
        finished_ns = monotonic_ns()
        report.update(
            {
                "outcome": "runtime_error",
                "gate_passed": False,
                "semantic_ready": False,
                "error": f"{type(exc).__name__}: {exc}",
                "elapsed_sec": round((finished_ns - started_ns) / 1_000_000_000.0, 6),
                "finished_utc": utc_now(),
                "exit_code": 2,
            }
        )
        report.update(tracker.report_fields())
        return 2, report
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def main(argv: Sequence[str] | None = None) -> int:
    parser = build_argument_parser()
    args = parser.parse_args(argv)
    required_classes, errors = validate_arguments(args)
    if errors:
        report = base_report(args, required_classes)
        report.update(
            {
                "outcome": "invalid_arguments",
                "gate_passed": False,
                "semantic_ready": False,
                "errors": errors,
                "finished_utc": utc_now(),
                "elapsed_sec": 0.0,
                "exit_code": 2,
            }
        )
        emit_report(report, args.report_json)
        return 2

    if args.mode == "instrumentation":
        report = base_report(args, required_classes)
        report.update(
            {
                "outcome": "skipped_instrumentation_only",
                "gate_passed": True,
                "semantic_ready": None,
                "skip_reason": "semantic presence is not an instrumentation-smoke requirement",
                "finished_utc": utc_now(),
                "elapsed_sec": 0.0,
                "exit_code": 0,
            }
        )
        emit_report(report, args.report_json)
        return 0

    exit_code, report = run_ros_gate(args, required_classes)
    emit_report(report, args.report_json)
    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
