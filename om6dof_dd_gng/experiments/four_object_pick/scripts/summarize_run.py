#!/usr/bin/env python3
"""Create a compact JSON summary from one recorded run using stdlib only."""

from __future__ import annotations

import argparse
import csv
import json
import statistics
from collections import Counter, defaultdict
from pathlib import Path


def percentile(values: list[float], fraction: float) -> float | None:
    if not values:
        return None
    ordered = sorted(values)
    index = (len(ordered) - 1) * fraction
    low = int(index)
    high = min(low + 1, len(ordered) - 1)
    return ordered[low] + (ordered[high] - ordered[low]) * (index - low)


def describe(values: list[float]) -> dict:
    q1 = percentile(values, 0.25)
    q3 = percentile(values, 0.75)
    return {
        "n": len(values),
        "mean": statistics.fmean(values) if values else None,
        "median": statistics.median(values) if values else None,
        "q1": q1,
        "q3": q3,
        "iqr": q3 - q1 if q1 is not None and q3 is not None else None,
        "p90": percentile(values, 0.90),
        "p95": percentile(values, 0.95),
        "min": min(values) if values else None,
        "max": max(values) if values else None,
    }


def describe_timed(samples: list[tuple[int, float, float | None]]) -> dict:
    result = describe([value for _, value, _ in samples])
    ordered = sorted(samples)
    if not ordered:
        result["time_weighted_mean"] = None
        result["weighting_basis"] = None
        return result
    if len(ordered) == 1:
        result["time_weighted_mean"] = ordered[0][1]
        result["weighting_basis"] = "single_sample"
        return result
    explicit_weights = [sample[2] for sample in ordered]
    if all(weight is not None and weight > 0 for weight in explicit_weights):
        result["time_weighted_mean"] = sum(
            value * float(weight)
            for (_, value, weight) in ordered
        ) / sum(float(weight) for weight in explicit_weights)
        result["weighting_basis"] = "preceding_measurement_interval"
        return result
    deltas = [
        max(1, ordered[index + 1][0] - ordered[index][0])
        for index in range(len(ordered) - 1)
    ]
    final_delta = int(statistics.median(deltas))
    weights = deltas + [max(1, final_delta)]
    result["time_weighted_mean"] = sum(
        value * weight for (_, value, _), weight in zip(ordered, weights)
    ) / sum(weights)
    result["weighting_basis"] = "next_sample_interval_with_median_tail"
    return result


def read_csv(path: Path) -> list[dict[str, str]]:
    if not path.exists():
        return []
    with path.open(newline="", encoding="utf-8") as handle:
        return list(csv.DictReader(handle))


def parse_pids(value: str) -> set[str]:
    return {item for item in value.split(";") if item}


def read_key_values(path: Path) -> dict[str, str]:
    values: dict[str, str] = {}
    if not path.is_file():
        return values
    for line in path.read_text(encoding="utf-8", errors="replace").splitlines():
        if "=" in line:
            key, value = line.split("=", 1)
            values.setdefault(key, value)
    return values


def read_json_object(path: Path) -> dict:
    if not path.is_file():
        return {}
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError):
        return {}
    return value if isinstance(value, dict) else {}


def count_nonempty_lines(path: Path) -> int:
    if not path.is_file():
        return 0
    with path.open(encoding="utf-8", errors="replace") as handle:
        return sum(bool(line.strip()) for line in handle)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("run_directory", type=Path)
    parser.add_argument(
        "--output", type=Path,
        help="optional output path; use this when re-analysing an immutable raw run",
    )
    args = parser.parse_args()

    events = read_csv(args.run_directory / "operator_events.csv")
    start_candidates = [
        int(row["monotonic_ns"]) for row in events
        if row.get("event") == "recording_started" and row.get("monotonic_ns")
    ]
    stop_candidates = [
        int(row["monotonic_ns"]) for row in events
        if row.get("event") == "recording_stopped" and row.get("monotonic_ns")
    ]
    recording_start_ns = min(start_candidates) if start_candidates else None
    recording_stop_ns = max(stop_candidates) if stop_candidates else None

    def in_recording_window(row: dict[str, str], field: str = "monotonic_ns") -> bool:
        value = row.get(field, "")
        if not value or recording_start_ns is None or recording_stop_ns is None:
            return True
        sample_ns = int(value)
        return recording_start_ns <= sample_ns <= recording_stop_ns

    resources = read_csv(args.run_directory / "module_resources.csv")
    owners: dict[tuple[str, str], set[str]] = defaultdict(set)
    for row in resources:
        for pid in parse_pids(row.get("pids", "")):
            owners[(row.get("monotonic_ns", ""), pid)].add(row.get("module", ""))
    overlap_keys = {
        key for key, modules in owners.items()
        if len(modules) > 1 and in_recording_window({"monotonic_ns": key[0]})
    }
    overlap_signatures = Counter(
        ";".join(sorted(owners[key])) for key in overlap_keys
    )
    recorded_match_conflicts = [
        row for row in read_csv(args.run_directory / "process_match_conflicts.csv")
        if in_recording_window(row)
    ]

    by_module: dict[str, dict[str, list[tuple[int, float, float | None]]]] = defaultdict(
        lambda: defaultdict(list)
    )
    resource_quality: dict[str, Counter] = defaultdict(Counter)
    previous_pids: dict[str, set[str]] = defaultdict(set)
    previous_sample_ns: dict[str, int] = defaultdict(lambda: -1)
    rate_fields = {"cpu_percent", "read_mb_s", "write_mb_s"}
    for row in resources:
        module = row.get("module", "")
        quality = resource_quality[module]
        pids = parse_pids(row.get("pids", ""))
        sample_ns = int(row.get("monotonic_ns", "0") or 0)
        if not in_recording_window(row):
            quality["rows_outside_recording_window"] += 1
            previous_pids[module] = pids
            previous_sample_ns[module] = sample_ns
            continue
        quality["rows_total"] += 1
        if not pids:
            quality["rows_without_pid"] += 1
            previous_pids[module] = set()
            previous_sample_ns[module] = sample_ns
            continue
        quality["rows_with_pid"] += 1
        ambiguous = any((row.get("monotonic_ns", ""), pid) in overlap_keys for pid in pids)
        if ambiguous:
            quality["rows_excluded_process_match_conflict"] += 1
            previous_pids[module] = pids
            previous_sample_ns[module] = sample_ns
            continue

        explicit_interval = row.get("sample_interval_sec", "")
        interval_start_ns = (
            sample_ns - int(float(explicit_interval) * 1e9)
            if explicit_interval else previous_sample_ns[module]
        )
        rate_fully_in_window = (
            recording_start_ns is None or recording_stop_ns is None
            or interval_start_ns >= recording_start_ns
        )
        legacy_rate_complete = (
            bool(previous_pids[module])
            and pids.issubset(previous_pids[module])
            and rate_fully_in_window
        )
        cpu_complete = (
            row.get("cpu_sample_complete") == "1"
            if row.get("cpu_sample_complete", "") != ""
            else legacy_rate_complete
        ) and rate_fully_in_window
        io_complete = (
            row.get("io_sample_complete") == "1"
            if row.get("io_sample_complete", "") != ""
            else legacy_rate_complete
        ) and rate_fully_in_window
        pss_value = row.get("pss_mb", "")
        pss_complete = (
            row.get("pss_sample_complete") == "1"
            if row.get("pss_sample_complete", "") != ""
            else bool(pss_value) and not (
                float(pss_value) == 0.0 and float(row.get("rss_mb", "0") or 0.0) > 0.0
            )
        )
        quality["cpu_rows_valid" if cpu_complete else "cpu_rows_bootstrap_or_partial"] += 1
        quality["io_rows_valid" if io_complete else "io_rows_bootstrap_or_unavailable"] += 1
        quality["pss_rows_valid" if pss_complete else "pss_rows_unavailable_or_partial"] += 1
        rate_weight = float(explicit_interval) if explicit_interval else None
        for field in ("cpu_percent", "rss_mb", "pss_mb", "threads", "read_mb_s", "write_mb_s"):
            if field == "cpu_percent" and not cpu_complete:
                continue
            if field in {"read_mb_s", "write_mb_s"} and not io_complete:
                continue
            if field == "pss_mb" and not pss_complete:
                continue
            value = row.get(field, "")
            if value == "":
                continue
            by_module[module][field].append(
                (sample_ns, float(value), rate_weight if field in rate_fields else None)
            )
        previous_pids[module] = pids
        previous_sample_ns[module] = sample_ns

    systems = read_csv(args.run_directory / "system_resources.csv")
    system_metrics: dict[str, list[float]] = defaultdict(list)
    system_cpu_excluded = 0
    previous_system_ns = -1
    for index, row in enumerate(systems):
        sample_ns = int(row.get("monotonic_ns", "0") or 0)
        explicit_interval = row.get("sample_interval_sec", "")
        interval_start_ns = (
            sample_ns - int(float(explicit_interval) * 1e9)
            if explicit_interval else previous_system_ns
        )
        rate_fully_in_window = (
            recording_start_ns is None or recording_stop_ns is None
            or interval_start_ns >= recording_start_ns
        )
        cpu_complete = (
            row.get("cpu_sample_complete") == "1"
            if row.get("cpu_sample_complete", "") != ""
            else index > 0
        ) and rate_fully_in_window
        if not in_recording_window(row):
            previous_system_ns = sample_ns
            continue
        for field in (
            "cpu_percent", "load1", "load5", "load15", "mem_available_mb",
            "mem_used_percent", "swap_used_mb",
        ):
            if field == "cpu_percent" and not cpu_complete:
                system_cpu_excluded += 1
                continue
            system_metrics[field].append(float(row[field]))
        previous_system_ns = sample_ns

    plans_all = read_csv(args.run_directory / "reachability_plan.csv")
    plans = [row for row in plans_all if in_recording_window(row, "received_monotonic_ns")]
    plan_metrics: dict[str, list[float]] = defaultdict(list)
    plan_metrics_by_reason: dict[str, dict[str, list[float]]] = defaultdict(
        lambda: defaultdict(list)
    )
    for row in plans:
        reason = row.get("reason", "") or "unspecified"
        for field in ("planning_time_ms", "exact_time_ms", "exact_checks", "exact_replans"):
            value = float(row[field])
            plan_metrics[field].append(value)
            plan_metrics_by_reason[reason][field].append(value)

    environments_all = read_csv(args.run_directory / "environment_graph.csv")
    environments = [
        row for row in environments_all if in_recording_window(row, "received_monotonic_ns")
    ]
    environment_metrics: dict[str, list[float]] = defaultdict(list)
    environment_interval_bootstrap_excluded = 0
    for row in environments:
        for field in ("nodes", "edges", "labelled_nodes", "message_interval_ms"):
            value = float(row[field])
            if field == "message_interval_ms" and value <= 0:
                environment_interval_bootstrap_excluded += 1
                continue
            environment_metrics[field].append(value)

    perception_rows_all = read_csv(args.run_directory / "perception_metrics.csv")
    perception_rows = [
        row for row in perception_rows_all
        if in_recording_window(row, "received_monotonic_ns")
    ]
    perception_metrics: dict[str, list[float]] = defaultdict(list)
    accepted_perception = [row for row in perception_rows if row.get("accepted") == "1"]
    for row in perception_rows:
        value = row.get("capture_wait_ms", "")
        if value and float(value) >= 0:
            perception_metrics["capture_wait_ms"].append(float(value))
    accepted_stage_fields = (
        "alignment_ms", "camera_tf_lookup_ms", "body_tf_lookup_ms",
        "robot_graph_publication_ms", "deprojection_world_tf_self_mask_ms",
        "dd_gng_update_ms", "graph_snapshot_ms", "yolo_submit_ms", "yolo_snapshot_ms",
        "semantic_fusion_ms", "core_publication_ms", "debug_image_ms",
        "processing_total_ms", "host_receipt_to_pipeline_complete_ms",
    )
    for row in accepted_perception:
        for field in accepted_stage_fields:
            value = row.get(field, "")
            if value and float(value) >= 0:
                perception_metrics[field].append(float(value))
    seen_yolo_results: set[str] = set()
    for row in perception_rows:
        sequence = row.get("yolo_result_sequence", "")
        value = row.get("yolo_inference_ms", "")
        if sequence and sequence != "0" and sequence not in seen_yolo_results and value:
            seen_yolo_results.add(sequence)
            if float(value) >= 0:
                perception_metrics["yolo_inference_ms_unique_results"].append(float(value))
    perception_drops = Counter(
        row.get("drop_reason", "") or "accepted" for row in perception_rows
    )

    events_by_object: dict[str, Counter] = defaultdict(Counter)
    events_by_opportunity: dict[str, list[dict[str, str]]] = defaultdict(list)
    for row in events:
        if row.get("object"):
            events_by_object[row["object"]][row.get("event", "unknown")] += 1
        if row.get("opportunity_id"):
            events_by_opportunity[row["opportunity_id"]].append({
                "event": row.get("event", ""),
                "result": row.get("result", ""),
                "monotonic_ns": row.get("monotonic_ns", ""),
            })

    health_rows = read_csv(args.run_directory / "topic_health.csv")
    health_accumulators: dict[str, dict] = defaultdict(lambda: {
        "messages_in_windows": 0,
        "observed_duration_sec": 0.0,
        "rates": [],
        "window_kinds": Counter(),
        "last_monotonic_ns": -1,
        "last_messages_total": 0,
    })
    for row in health_rows:
        topic = row.get("topic", "")
        accumulator = health_accumulators[topic]
        accumulator["messages_in_windows"] += int(row.get("messages_in_interval", "0") or 0)
        accumulator["observed_duration_sec"] += float(row.get("interval_sec", "0") or 0.0)
        accumulator["rates"].append(float(row.get("rate_hz", "0") or 0.0))
        accumulator["window_kinds"][row.get("window_kind", "legacy_unspecified") or "legacy_unspecified"] += 1
        row_ns = int(row.get("monotonic_ns", "0") or 0)
        if row_ns >= accumulator["last_monotonic_ns"]:
            accumulator["last_monotonic_ns"] = row_ns
            accumulator["last_messages_total"] = int(row.get("messages_total", "0") or 0)
    topic_health = {}
    for topic, accumulator in sorted(health_accumulators.items()):
        duration = accumulator["observed_duration_sec"]
        topic_health[topic] = {
            "last_reported_messages_total": accumulator["last_messages_total"],
            "messages_in_windows": accumulator["messages_in_windows"],
            "observed_duration_sec": duration,
            "overall_rate_hz": (
                accumulator["messages_in_windows"] / duration if duration > 0 else None
            ),
            "window_rate_hz": describe(accumulator["rates"]),
            "window_kinds": dict(sorted(accumulator["window_kinds"].items())),
        }
    recorded_topic_counts = {
        "reachability_plan": len(plans_all),
        "reachability_graph": len(read_csv(args.run_directory / "reachability_graph.csv")),
        "environment_graph": len(environments_all),
        "labels": count_nonempty_lines(args.run_directory / "semantic_labels.jsonl"),
        "status": count_nonempty_lines(args.run_directory / "topo_status.jsonl"),
        "perception_metrics": count_nonempty_lines(args.run_directory / "perception_metrics.jsonl"),
    }
    health_count_discrepancies = {
        topic: {
            "recorded_messages": count,
            "last_health_total": topic_health.get(topic, {}).get("last_reported_messages_total"),
        }
        for topic, count in recorded_topic_counts.items()
        if topic_health.get(topic, {}).get("last_reported_messages_total") != count
    }

    metadata = read_key_values(args.run_directory / "run_metadata.txt")
    run_id = metadata.get("run_id", args.run_directory.name)
    manifest_rows = read_csv(args.run_directory / "trial_manifest.csv")
    expected_opportunity_ids = {
        row.get("opportunity_id", "") for row in manifest_rows
        if row.get("run_id") == run_id and row.get("opportunity_id")
    }
    observed_opportunity_ids = set(events_by_opportunity) & expected_opportunity_ids
    terminal_events = {"target_complete", "abort"}
    terminal_opportunity_ids = {
        opportunity_id for opportunity_id, timeline in events_by_opportunity.items()
        if opportunity_id in expected_opportunity_ids
        and any(item["event"] in terminal_events for item in timeline)
    }
    scene_ready_seen = any(row.get("event") == "scene_ready" for row in events)
    recording_started_seen = any(row.get("event") == "recording_started" for row in events)
    recording_stopped_seen = any(row.get("event") == "recording_stopped" for row in events)
    required_files = [
        "experiment.yaml", "trial_manifest.csv", "four_object_topo_gng_v2.yaml",
        "module_resources.csv", "system_resources.csv", "reachability_plan.csv",
        "reachability_graph.csv", "environment_graph.csv", "perception_metrics.csv",
        "operator_events.csv", "scene_readiness.json",
        "rosbag/metadata.yaml",
    ]
    required_files_present = {
        name: (args.run_directory / name).is_file() for name in required_files
    }
    required_files_nonempty = {
        name: (args.run_directory / name).is_file()
        and (args.run_directory / name).stat().st_size > 0
        for name in required_files
    }
    post_cleanup_failure = (
        (args.run_directory / "run_incomplete.flag").is_file()
        or metadata.get("recorder_failure_detected") == "1"
    )
    recording_integrity_ok = (
        all(required_files_nonempty.values())
        and recording_started_seen
        and recording_stopped_seen
        and (args.run_directory / "run_complete.flag").is_file()
        and not post_cleanup_failure
    )
    stream_coverage = {
        "module_resource_rows": sum(in_recording_window(row) for row in resources),
        "system_resource_rows": sum(in_recording_window(row) for row in systems),
        "reachability_graph_rows": recorded_topic_counts["reachability_graph"],
        "reachability_plan_rows": len(plans),
        "environment_graph_rows": len(environments),
        "perception_metric_rows_parsed": len(perception_rows),
        "perception_metric_messages_raw": recorded_topic_counts["perception_metrics"],
    }
    stream_coverage_ok = all(value > 0 for value in stream_coverage.values())
    opportunities_complete = (
        bool(expected_opportunity_ids)
        and terminal_opportunity_ids == expected_opportunity_ids
    )
    resource_match_unambiguous = not overlap_keys and not recorded_match_conflicts
    topic_health_totals_match = not health_count_discrepancies
    scene_readiness = read_json_object(args.run_directory / "scene_readiness.json")
    semantic_scene_ready = (
        scene_readiness.get("mode") == "experiment"
        and scene_readiness.get("gate_passed") is True
        and scene_readiness.get("semantic_ready") is True
    )
    unexpected_module_activity = {
        module: int(resource_quality[module].get("rows_with_pid", 0))
        for module in ("yolo_preview", "ordinary_perception", "pick")
        if resource_quality[module].get("rows_with_pid", 0) > 0
    }
    analysis_blockers = []
    if not recording_integrity_ok:
        analysis_blockers.append("recording_integrity_failed")
    if not scene_ready_seen:
        analysis_blockers.append("scene_ready_event_missing")
    if not opportunities_complete:
        analysis_blockers.append("not_all_manifest_opportunities_have_target_complete_or_abort")
    if not stream_coverage_ok:
        analysis_blockers.append("one_or_more_required_metric_streams_have_no_rows")
    if not resource_match_unambiguous:
        analysis_blockers.append("resource_process_matches_are_ambiguous")
    if not topic_health_totals_match:
        analysis_blockers.append("topic_health_final_totals_do_not_match_recorded_messages")
    if not semantic_scene_ready:
        analysis_blockers.append("semantic_scene_readiness_gate_not_passed")
    if unexpected_module_activity:
        analysis_blockers.append("unexpected_legacy_or_execution_process_activity")
    valid_plan_messages = sum(row.get("valid") == "1" for row in plans)
    exact_valid_plan_messages = sum(row.get("exact_valid") == "1" for row in plans)
    result = {
        "summary_schema": "om6dof.four_object_run_summary.v2",
        "run_directory": str(args.run_directory.resolve()),
        "run_id": run_id,
        "recording_window": {
            "start_monotonic_ns": recording_start_ns,
            "stop_monotonic_ns": recording_stop_ns,
            "duration_sec": (
                (recording_stop_ns - recording_start_ns) / 1e9
                if recording_start_ns is not None and recording_stop_ns is not None
                else None
            ),
        },
        "measurement_scope": {
            "plan_rows": "planner publication messages, not independent target opportunities",
            "run_complete_flag": "recorder/bag shutdown integrity only, not experiment completeness",
            "resource_cpu_percent": "Linux process CPU; 100 percent equals one logical CPU core",
            "metric_statistics_window": "recording_started through recording_stopped monotonic timestamps",
            "topic_health_window": "metrics-recorder lifetime; startup and shutdown partial windows are labeled",
        },
        "module_resources": {
            module: {field: describe_timed(values) for field, values in fields.items()}
            for module, fields in sorted(by_module.items())
        },
        "resource_measurement_quality": {
            "monitor_format": (
                "validity_annotated_exclusive_v2"
                if resources and "cpu_sample_complete" in resources[0]
                else "legacy_inferred"
            ),
            "module_rows": {
                module: dict(sorted(counts.items()))
                for module, counts in sorted(resource_quality.items())
            },
            "overlapping_pid_samples_detected_in_module_csv": len(overlap_keys),
            "overlapping_unique_pids": sorted({pid for _, pid in overlap_keys}),
            "overlap_module_signatures": dict(sorted(overlap_signatures.items())),
            "exclusive_monitor_conflict_rows": len(recorded_match_conflicts),
            "exclusive_monitor_conflict_signatures": dict(sorted(Counter(
                row.get("matching_modules", "") for row in recorded_match_conflicts
            ).items())),
            "unexpected_module_activity": unexpected_module_activity,
            "unexpected_modules_checked": [
                "yolo_preview", "ordinary_perception", "pick"
            ],
            "ambiguous_rows_are_excluded_from_resource_statistics": True,
            "bootstrap_or_partial_rate_rows_are_excluded": True,
            "system_cpu_bootstrap_rows_excluded": system_cpu_excluded,
        },
        "system_resources": {field: describe(values) for field, values in system_metrics.items()},
        "reachability_plan_messages": {
            "received": len(plans),
            "valid": valid_plan_messages,
            "exact_collision_valid": exact_valid_plan_messages,
            "live_preview_query_id_zero": sum(
                row.get("query_id", "") in {"", "0"} for row in plans
            ),
            "distinct_nonzero_query_ids": len({
                row.get("query_id", "") for row in plans
                if row.get("query_id", "") not in {"", "0"}
            }),
            "distinct_graph_revisions": sorted({
                row.get("graph_revision", "") for row in plans if row.get("graph_revision", "")
            }),
            "reasons": dict(sorted(Counter(row.get("reason", "") for row in plans).items())),
            "timings_all_messages": {
                field: describe(values) for field, values in plan_metrics.items()
            },
            "timings_by_reason": {
                reason: {field: describe(values) for field, values in metrics.items()}
                for reason, metrics in sorted(plan_metrics_by_reason.items())
            },
        },
        "environment_graph": {
            **{field: describe(values) for field, values in environment_metrics.items()},
            "message_interval_bootstrap_rows_excluded": environment_interval_bootstrap_excluded,
        },
        "perception": {
            "samples": len(perception_rows),
            "accepted_frames": len(accepted_perception),
            "acceptance_rate": (
                len(accepted_perception) / len(perception_rows) if perception_rows else None
            ),
            "drop_reasons": dict(sorted(perception_drops.items())),
            "timings_ms": {
                field: describe(values) for field, values in sorted(perception_metrics.items())
            },
            "last_cumulative_counters": ({
                key: perception_rows[-1].get(key, "") for key in (
                    "capture_attempts_total", "received_frames_total", "published_frames_total",
                    "dropped_received_frames_total", "capture_failures_total",
                    "camera_timeouts_total", "camera_errors_total", "invalid_frames_total",
                    "stale_before_tf_total", "camera_tf_unavailable_total",
                    "body_tf_unavailable_total", "stale_before_deprojection_total",
                    "stale_before_publish_total", "processing_errors_total",
                    "device_frame_gap_total", "device_frame_number_resets_total",
                )
            } if perception_rows else {}),
        },
        "operator_event_count": len(events),
        "operator_events_by_object": {
            name: dict(sorted(counts.items())) for name, counts in sorted(events_by_object.items())
        },
        "opportunity_timelines": dict(sorted(events_by_opportunity.items())),
        "topic_health": topic_health,
        "topic_health_count_crosscheck": {
            "recorded_topic_counts": recorded_topic_counts,
            "discrepancies": health_count_discrepancies,
            "all_final_totals_match": topic_health_totals_match,
        },
        "visual_and_robot_state_delivery": {
            "debug_image_messages_reported": topic_health.get("debug_image", {}).get(
                "last_reported_messages_total"
            ),
            "joint_state_messages_reported": topic_health.get("joint_states", {}).get(
                "last_reported_messages_total"
            ),
        },
        "scene_readiness": scene_readiness or {
            "outcome": "report_missing",
            "gate_passed": False,
            "semantic_ready": False,
        },
        "completeness": {
            "recording_integrity": {
                "ok": recording_integrity_ok,
                "required_files_present": required_files_present,
                "required_files_nonempty": required_files_nonempty,
                "run_complete_flag_present": (args.run_directory / "run_complete.flag").is_file(),
                "run_incomplete_flag_present": (args.run_directory / "run_incomplete.flag").is_file(),
                "recorder_failure_detected_metadata": metadata.get(
                    "recorder_failure_detected", "pending_at_initial_summary"
                ),
                "recording_started_event": recording_started_seen,
                "recording_stopped_event": recording_stopped_seen,
            },
            "trial_endpoint_coverage": {
                "scene_ready_event": scene_ready_seen,
                "manifest_opportunities_expected": len(expected_opportunity_ids),
                "manifest_opportunity_ids_expected": sorted(expected_opportunity_ids),
                "opportunities_with_any_event": len(observed_opportunity_ids),
                "opportunity_ids_with_any_event": sorted(observed_opportunity_ids),
                "opportunities_with_target_complete_or_abort": len(terminal_opportunity_ids),
                "terminal_opportunity_ids": sorted(terminal_opportunity_ids),
                "all_manifest_opportunities_terminal": opportunities_complete,
            },
            "metric_stream_rows": stream_coverage,
            "metric_stream_coverage_ok": stream_coverage_ok,
            "semantic_scene_ready": semantic_scene_ready,
            "analysis_ready": not analysis_blockers,
            "analysis_blockers": analysis_blockers,
        },
    }
    output = args.output or (args.run_directory / "run_summary.json")
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(result, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(output)


if __name__ == "__main__":
    main()
