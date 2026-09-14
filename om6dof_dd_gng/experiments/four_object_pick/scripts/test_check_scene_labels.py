#!/usr/bin/env python3
"""Focused unit tests for the read-only semantic scene-readiness gate."""

from __future__ import annotations

import json
import sys
import unittest
from pathlib import Path


sys.path.insert(0, str(Path(__file__).resolve().parent))

from check_scene_labels import (  # noqa: E402
    DEFAULT_CLASSES,
    LabelPayloadError,
    StabilityTracker,
    classify_timeout,
    parse_label_payload,
)


def payload(counts: dict[str, int]) -> str:
    rows: list[dict[str, object]] = []
    node_id = 1
    for class_name, count in counts.items():
        for _ in range(count):
            rows.append(
                {
                    "index": node_id - 1,
                    "node_id": node_id,
                    "class": class_name,
                    "confidence": 0.9,
                    "x": 0.1,
                    "y": 0.2,
                    "z": 0.3,
                }
            )
            node_id += 1
    return json.dumps(rows)


class LabelParserTest(unittest.TestCase):
    def test_counts_unique_nodes_and_ignores_unknown_classes(self) -> None:
        rows = json.loads(payload({"bottle": 3, "cup": 2, "person": 1}))
        rows.append(dict(rows[0]))  # duplicate support must not inflate bottle
        snapshot = parse_label_payload(json.dumps(rows), DEFAULT_CLASSES)
        self.assertEqual(snapshot.counts["bottle"], 3)
        self.assertEqual(snapshot.counts["cup"], 2)
        self.assertEqual(snapshot.counts["banana"], 0)
        self.assertEqual(snapshot.ignored_unknown_entries, 1)
        self.assertEqual(snapshot.duplicate_entries, 1)

    def test_rejects_wrong_root_and_partial_corruption(self) -> None:
        with self.assertRaises(LabelPayloadError):
            parse_label_payload('{"class":"bottle","node_id":1}', DEFAULT_CLASSES)
        with self.assertRaises(LabelPayloadError):
            parse_label_payload('[{"class":"bottle"}]', DEFAULT_CLASSES)
        with self.assertRaises(LabelPayloadError):
            parse_label_payload("not JSON", DEFAULT_CLASSES)

    def test_rejects_one_node_assigned_to_two_classes(self) -> None:
        conflicting = [
            {"class": "bottle", "node_id": 7},
            {"class": "cup", "node_id": 7},
        ]
        with self.assertRaises(LabelPayloadError):
            parse_label_payload(json.dumps(conflicting), DEFAULT_CLASSES)


class StabilityTrackerTest(unittest.TestCase):
    ALL_THREE = {name: 3 for name in DEFAULT_CLASSES}

    def tracker(self) -> StabilityTracker:
        return StabilityTracker(DEFAULT_CLASSES, 3, 1.0, 0.5)

    def test_requires_repeated_simultaneous_support_across_duration(self) -> None:
        tracker = self.tracker()
        self.assertFalse(tracker.observe(payload(self.ALL_THREE), 0))
        self.assertFalse(tracker.observe(payload(self.ALL_THREE), 400_000_000))
        self.assertFalse(tracker.observe(payload(self.ALL_THREE), 800_000_000))
        self.assertTrue(tracker.observe(payload(self.ALL_THREE), 1_000_000_000))
        self.assertEqual(tracker.all_qualifying_messages, 4)

    def test_threshold_loss_resets_continuous_interval(self) -> None:
        tracker = self.tracker()
        tracker.observe(payload(self.ALL_THREE), 0)
        tracker.observe(payload(self.ALL_THREE), 400_000_000)
        missing_remote = dict(self.ALL_THREE)
        missing_remote["remote"] = 2
        self.assertFalse(tracker.observe(payload(missing_remote), 600_000_000))
        self.assertFalse(tracker.observe(payload(self.ALL_THREE), 1_000_000_000))
        self.assertFalse(tracker.observe(payload(self.ALL_THREE), 1_400_000_000))
        self.assertFalse(tracker.observe(payload(self.ALL_THREE), 1_800_000_000))
        self.assertTrue(tracker.observe(payload(self.ALL_THREE), 2_000_000_000))

    def test_message_gap_and_malformed_payload_reset_interval(self) -> None:
        tracker = self.tracker()
        tracker.observe(payload(self.ALL_THREE), 0)
        tracker.observe(payload(self.ALL_THREE), 400_000_000)
        self.assertFalse(tracker.observe(payload(self.ALL_THREE), 1_100_000_000))
        self.assertEqual(tracker.gap_resets, 1)
        self.assertFalse(tracker.observe("[] broken", 1_200_000_000))
        self.assertEqual(tracker.messages_malformed, 1)
        self.assertEqual(tracker.malformed_resets, 1)

    def test_poll_closes_silent_interval_without_extrapolating_success(self) -> None:
        tracker = self.tracker()
        tracker.observe(payload(self.ALL_THREE), 0)
        tracker.observe(payload(self.ALL_THREE), 400_000_000)
        tracker.poll_for_gap(1_000_000_000)
        self.assertFalse(tracker.ready)
        self.assertEqual(tracker.gap_resets, 1)
        self.assertAlmostEqual(tracker.all_longest_ns / 1e9, 0.4)

    def test_timeout_classification_distinguishes_missing_and_unstable(self) -> None:
        missing = self.tracker()
        missing.observe(payload({"bottle": 3, "cup": 3}), 0)
        outcome, absent, unstable = classify_timeout(missing)
        self.assertEqual(outcome, "timeout_missing_classes")
        self.assertEqual(absent, ["banana", "remote"])
        self.assertIn("banana", unstable)

        unstable_tracker = self.tracker()
        unstable_tracker.observe(payload(self.ALL_THREE), 0)
        unstable_tracker.observe(payload(self.ALL_THREE), 400_000_000)
        outcome, absent, unstable = classify_timeout(unstable_tracker)
        self.assertEqual(outcome, "timeout_unstable_classes")
        self.assertEqual(absent, [])
        self.assertEqual(unstable, list(DEFAULT_CLASSES))


if __name__ == "__main__":
    unittest.main()
