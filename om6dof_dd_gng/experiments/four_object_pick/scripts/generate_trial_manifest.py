#!/usr/bin/env python3
"""Generate the fixed 12-reset, 48-target four-object trial manifest."""

from __future__ import annotations

import csv
from pathlib import Path


OBJECTS = ["bottle", "cup", "banana", "remote"]
OBJECT_IDS = {"bottle": "O1", "cup": "O2", "banana": "O3", "remote": "O4"}
COCO_CLASS_IDS = {"bottle": 39, "cup": 41, "banana": 46, "remote": 65}
ORDERS = [
    ["bottle", "cup", "banana", "remote"],
    ["cup", "bottle", "remote", "banana"],
    ["banana", "remote", "bottle", "cup"],
    ["remote", "banana", "cup", "bottle"],
]
POSITIONS = ["P1", "P2", "P3", "P4"]


def rows():
    for reset_index in range(12):
        run_id = f"run_{reset_index + 1:03d}"
        order = ORDERS[reset_index % len(ORDERS)]
        position_shift = reset_index % len(POSITIONS)
        object_positions = {
            name: POSITIONS[(index + position_shift) % len(POSITIONS)]
            for index, name in enumerate(OBJECTS)
        }
        for target_order, target in enumerate(order, start=1):
            yield {
                "protocol_schema_version": 1,
                "opportunity_id": f"{run_id}_t{target_order:02d}",
                "run_id": run_id,
                "reset_index": reset_index + 1,
                "scene_id": f"scene_{reset_index + 1:03d}",
                "target_order": target_order,
                "target_object": target,
                "target_instance_id": OBJECT_IDS[target],
                "coco_class": target,
                "coco_class_id": COCO_CLASS_IDS[target],
                "object_position": object_positions[target],
                "method": "gng",
                "roadmap_seed": 0,
                "graph_node_budget": 500,
                "attempt_index": 1,
                "query_id": "",
                "target_environment_node_id": "",
                "scene_snapshot_sha256": "",
                "development_or_confirmatory": "pilot",
                "execution_expected": "preview_only_until_gate_C",
            }


def main() -> None:
    output = Path(__file__).resolve().parents[1] / "trial_manifest.csv"
    manifest = list(rows())
    with output.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(manifest[0]))
        writer.writeheader()
        writer.writerows(manifest)
    print(f"wrote {len(manifest)} target opportunities to {output}")


if __name__ == "__main__":
    main()
