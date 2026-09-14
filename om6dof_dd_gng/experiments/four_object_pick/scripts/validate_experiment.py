#!/usr/bin/env python3
"""Validate the frozen four-object pilot manifest and key configuration."""

from __future__ import annotations

import csv
import sys
from collections import Counter, defaultdict
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
EXPECTED_OBJECTS = {"bottle": 39, "cup": 41, "banana": 46, "remote": 65}
EXPECTED_POSITIONS = {"P1", "P2", "P3", "P4"}


def require(condition: bool, message: str, failures: list[str]) -> None:
    if not condition:
        failures.append(message)


def main() -> None:
    failures: list[str] = []
    manifest_path = ROOT / "trial_manifest.csv"
    with manifest_path.open(newline="", encoding="utf-8") as handle:
        rows = list(csv.DictReader(handle))

    require(len(rows) == 48, f"expected 48 opportunities, got {len(rows)}", failures)
    require(len({row["opportunity_id"] for row in rows}) == len(rows),
            "opportunity_id values are not unique", failures)
    require(len({row["run_id"] for row in rows}) == 12, "expected 12 run_id values", failures)
    require(Counter(row["target_object"] for row in rows) == Counter({name: 12 for name in EXPECTED_OBJECTS}),
            "objects are not balanced at 12 opportunities each", failures)

    by_run: dict[str, list[dict[str, str]]] = defaultdict(list)
    by_object_position: dict[str, Counter] = defaultdict(Counter)
    by_object_order: dict[str, Counter] = defaultdict(Counter)
    for row in rows:
        by_run[row["run_id"]].append(row)
        name = row["target_object"]
        require(name in EXPECTED_OBJECTS, f"unexpected object: {name}", failures)
        if name in EXPECTED_OBJECTS:
            require(int(row["coco_class_id"]) == EXPECTED_OBJECTS[name],
                    f"wrong COCO id for {name}", failures)
        require(row["object_position"] in EXPECTED_POSITIONS,
                f"invalid position: {row['object_position']}", failures)
        require(row["method"] == "gng", "pilot method must remain gng", failures)
        require(row["development_or_confirmatory"] == "pilot",
                "manifest must not claim confirmatory data", failures)
        by_object_position[name][row["object_position"]] += 1
        by_object_order[name][row["target_order"]] += 1

    for run_id, run_rows in by_run.items():
        require(len(run_rows) == 4, f"{run_id} does not contain four targets", failures)
        require({row["target_object"] for row in run_rows} == set(EXPECTED_OBJECTS),
                f"{run_id} does not contain each object exactly once", failures)
    for name in EXPECTED_OBJECTS:
        require(by_object_position[name] == Counter({position: 3 for position in EXPECTED_POSITIONS}),
                f"{name} is not balanced across positions", failures)
        require(by_object_order[name] == Counter({str(order): 3 for order in range(1, 5)}),
                f"{name} is not balanced across target order", failures)

    config = (ROOT / "config" / "four_object_topo_gng_v2.yaml").read_text(encoding="utf-8")
    require('target_classes: "bottle,cup,banana,remote"' in config,
            "target_classes is not frozen", failures)
    require("yolo_period_sec: 0.5" in config, "YOLO period is not frozen to 0.5 s", failures)
    require("query_mode: false" in config, "pilot must remain preview/live mode", failures)

    if failures:
        for failure in failures:
            print(f"FAIL  {failure}", file=sys.stderr)
        raise SystemExit(1)
    print("PASS  manifest: 48 unique opportunities; objects, positions, and order balanced")
    print("PASS  config: GNG preview pilot; four classes frozen; YOLO 2 Hz")


if __name__ == "__main__":
    main()
