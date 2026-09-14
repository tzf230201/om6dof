#!/usr/bin/env python3
"""Append a timestamped human-observed event without rewriting prior data."""

from __future__ import annotations

import argparse
import csv
import fcntl
import time
from datetime import datetime, timezone
from pathlib import Path


EVENTS = {
    "recording_started",
    "recording_stopped",
    "scene_ready",
    "target_selected",
    "detected",
    "semantic_stable",
    "instance_selected",
    "intersection_found",
    "plan_exact_valid",
    "approach_reached",
    "grasp_closed",
    "lift_success",
    "place_success",
    "target_complete",
    "abort",
}
GLOBAL_EVENTS = {"recording_started", "recording_stopped", "scene_ready"}
FIELDS = [
    "utc_iso",
    "unix_ns",
    "monotonic_ns",
    "run_id",
    "opportunity_id",
    "reset_index",
    "target_order",
    "event",
    "object",
    "object_position",
    "result",
    "note",
]


def load_opportunity(run_directory: Path, opportunity_id: str) -> dict[str, str]:
    manifest = run_directory / "trial_manifest.csv"
    if not manifest.is_file():
        raise SystemExit(f"trial manifest missing: {manifest}")
    with manifest.open(newline="", encoding="utf-8") as handle:
        matches = [
            row for row in csv.DictReader(handle)
            if row.get("opportunity_id") == opportunity_id
        ]
    if len(matches) != 1:
        raise SystemExit(
            f"opportunity_id must match exactly one manifest row; got {len(matches)}: "
            f"{opportunity_id}"
        )
    return matches[0]


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("run_directory", type=Path)
    parser.add_argument("event", choices=sorted(EVENTS))
    parser.add_argument("--opportunity-id", default="")
    parser.add_argument("--result", choices=["na", "success", "failure", "not_run"], default="na")
    parser.add_argument("--note", default="")
    args = parser.parse_args()

    if not args.run_directory.is_dir():
        raise SystemExit(f"run directory does not exist: {args.run_directory}")
    if args.event not in GLOBAL_EVENTS and not args.opportunity_id:
        raise SystemExit(f"--opportunity-id is required for event {args.event}")
    opportunity = (
        load_opportunity(args.run_directory, args.opportunity_id)
        if args.opportunity_id
        else {}
    )
    metadata_run_id = args.run_directory.name
    if opportunity and opportunity.get("run_id") != metadata_run_id:
        raise SystemExit(
            f"opportunity {args.opportunity_id} belongs to {opportunity.get('run_id')}, "
            f"not {metadata_run_id}"
        )
    output = args.run_directory / "operator_events.csv"
    now = datetime.now(timezone.utc)
    row = {
        "utc_iso": now.isoformat(),
        "unix_ns": time.time_ns(),
        "monotonic_ns": time.monotonic_ns(),
        "run_id": metadata_run_id,
        "opportunity_id": args.opportunity_id,
        "reset_index": opportunity.get("reset_index", ""),
        "target_order": opportunity.get("target_order", ""),
        "event": args.event,
        "object": opportunity.get("target_object", ""),
        "object_position": opportunity.get("object_position", ""),
        "result": args.result,
        "note": args.note,
    }
    with output.open("a+", newline="", encoding="utf-8") as handle:
        fcntl.flock(handle.fileno(), fcntl.LOCK_EX)
        handle.seek(0, 2)
        new_file = handle.tell() == 0
        writer = csv.DictWriter(handle, fieldnames=FIELDS)
        if new_file:
            writer.writeheader()
        writer.writerow(row)
        handle.flush()
        fcntl.flock(handle.fileno(), fcntl.LOCK_UN)
    suffix = f" for {args.opportunity_id}" if args.opportunity_id else ""
    print(f"recorded {args.event}{suffix} at {row['utc_iso']}")


if __name__ == "__main__":
    main()
