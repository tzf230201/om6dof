#!/usr/bin/env python3
"""Render an instrumentation-only dashboard from one recorded run directory.

The plot intentionally uses only recorded CSV values. Missing files, columns,
or samples are shown as unavailable rather than imputed.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import textwrap
from collections import defaultdict
from dataclasses import dataclass
from itertools import cycle
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
from matplotlib.ticker import MaxNLocator  # noqa: E402


TITLE = "INSTRUMENTATION SMOKE — NOT PAPER RESULT"
PALETTE = (
    "#0072B2",  # blue
    "#D55E00",  # vermillion
    "#009E73",  # green
    "#CC79A7",  # purple
    "#E69F00",  # orange
    "#56B4E9",  # sky blue
    "#000000",
)
DEFAULT_MODULES = ("topo_gng", "reachability", "ordinary_perception", "move_group")
MODULE_LABELS = {
    "topo_gng": "topo_gng (DD-GNG + YOLO, combined)",
    "reachability": "reachability planner",
    "ordinary_perception": "ordinary perception",
    "move_group": "MoveIt move_group",
}
STAGES = (
    ("processing_total_ms", "frame processing", "#111111", 2.2),
    ("camera_tf_lookup_ms", "camera TF lookup", "#0072B2", 1.4),
    ("alignment_ms", "RGB-depth alignment", "#56B4E9", 1.2),
    ("deprojection_world_tf_self_mask_ms", "deprojection + world TF + mask", "#009E73", 1.2),
    ("dd_gng_update_ms", "DD-GNG update", "#D55E00", 1.4),
    ("semantic_fusion_ms", "semantic fusion", "#CC79A7", 1.2),
    ("core_publication_ms", "core publication", "#E69F00", 1.2),
)
TOPIC_LABELS = {
    "environment_graph": "environment graph",
    "labels": "semantic labels",
    "perception_metrics": "perception metrics",
    "reachability_graph": "reachability graph",
    "reachability_plan": "reachability plan",
    "status": "topology status",
}


@dataclass(frozen=True)
class RecordingWindow:
    start_ns: int
    stop_ns: int
    start_utc: str
    stop_utc: str

    @property
    def duration_sec(self) -> float:
        return (self.stop_ns - self.start_ns) / 1e9


def read_csv(path: Path, warnings: list[str]) -> list[dict[str, str]]:
    if not path.is_file():
        warnings.append(f"missing {path.name}")
        return []
    try:
        with path.open(newline="", encoding="utf-8") as handle:
            return list(csv.DictReader(handle))
    except (OSError, csv.Error) as error:
        warnings.append(f"could not read {path.name}: {error}")
        return []


def finite_value(row: dict[str, str], field: str) -> float | None:
    raw = row.get(field, "")
    if raw is None or str(raw).strip() == "":
        return None
    try:
        value = float(raw)
    except (TypeError, ValueError):
        return None
    return value if math.isfinite(value) else None


def integer_value(row: dict[str, str], field: str) -> int | None:
    raw = row.get(field, "")
    try:
        return int(raw)
    except (TypeError, ValueError):
        return None


def operator_recording_window(
    rows: list[dict[str, str]], warnings: list[str]
) -> RecordingWindow | None:
    starts = [row for row in rows if row.get("event") == "recording_started"]
    stops = [row for row in rows if row.get("event") == "recording_stopped"]
    if len(starts) != 1 or len(stops) != 1:
        warnings.append(
            "operator event bounds require exactly one recording_started and one "
            "recording_stopped; using recorder-lifetime fallback"
        )
        return None
    start_ns = integer_value(starts[0], "monotonic_ns")
    stop_ns = integer_value(stops[0], "monotonic_ns")
    if start_ns is None or stop_ns is None or stop_ns <= start_ns:
        warnings.append(
            "operator event monotonic bounds are missing or invalid; using "
            "recorder-lifetime fallback"
        )
        return None
    return RecordingWindow(
        start_ns=start_ns,
        stop_ns=stop_ns,
        start_utc=starts[0].get("utc_iso", "not recorded"),
        stop_utc=stops[0].get("utc_iso", "not recorded"),
    )


def filter_point_samples(
    rows: list[dict[str, str]], timestamp_field: str, window: RecordingWindow,
    source_name: str, warnings: list[str],
) -> list[dict[str, str]]:
    filtered: list[dict[str, str]] = []
    missing_timestamps = 0
    for row in rows:
        timestamp = integer_value(row, timestamp_field)
        if timestamp is None:
            missing_timestamps += 1
        elif window.start_ns <= timestamp <= window.stop_ns:
            filtered.append(row)
    if missing_timestamps:
        warnings.append(
            f"excluded {missing_timestamps} {source_name} rows without {timestamp_field}"
        )
    return filtered


def filter_health_intervals(
    rows: list[dict[str, str]], window: RecordingWindow, warnings: list[str]
) -> tuple[list[dict[str, str]], int]:
    """Keep only complete periodic rate intervals inside the operator window."""
    filtered: list[dict[str, str]] = []
    interval_keys: set[tuple[int, int]] = set()
    malformed = 0
    partial_overlap = 0
    excluded_kinds: defaultdict[str, int] = defaultdict(int)
    for row in rows:
        kind = row.get("window_kind", "").strip()
        if kind and kind != "periodic":
            excluded_kinds[kind] += 1
            continue
        interval_end = integer_value(row, "monotonic_ns")
        interval_sec = finite_value(row, "interval_sec")
        if interval_end is None or interval_sec is None or interval_sec <= 0.0:
            malformed += 1
            continue
        interval_start = interval_end - int(round(interval_sec * 1e9))
        overlaps = interval_end >= window.start_ns and interval_start <= window.stop_ns
        fully_contained = (
            interval_start >= window.start_ns and interval_end <= window.stop_ns
        )
        if fully_contained:
            output = dict(row)
            output["_plot_monotonic_ns"] = str((interval_start + interval_end) // 2)
            filtered.append(output)
            interval_keys.add((interval_start, interval_end))
        elif overlaps:
            partial_overlap += 1
    if malformed:
        warnings.append(f"excluded {malformed} malformed topic-health rows")
    if partial_overlap:
        warnings.append(
            f"excluded {partial_overlap} topic-health rows whose intervals only "
            "partially overlap the operator window"
        )
    if excluded_kinds:
        details = ", ".join(
            f"{kind or 'untyped'}={count}" for kind, count in sorted(excluded_kinds.items())
        )
        warnings.append(f"excluded non-periodic topic-health rows ({details})")
    return filtered, len(interval_keys)


def recorded_origin(datasets: list[tuple[list[dict[str, str]], str]]) -> int:
    timestamps: list[int] = []
    for rows, field in datasets:
        for row in rows:
            value = integer_value(row, field)
            if value is not None:
                timestamps.append(value)
    return min(timestamps) if timestamps else 0


def relative_seconds(row: dict[str, str], field: str, origin_ns: int) -> float | None:
    value = integer_value(row, field)
    if value is None or origin_ns == 0:
        return None
    return (value - origin_ns) / 1e9


def series(
    rows: list[dict[str, str]], time_field: str, value_field: str, origin_ns: int
) -> tuple[np.ndarray, np.ndarray]:
    points: list[tuple[float, float]] = []
    for row in rows:
        elapsed = relative_seconds(row, time_field, origin_ns)
        value = finite_value(row, value_field)
        if elapsed is not None and value is not None:
            points.append((elapsed, value))
    points.sort(key=lambda item: item[0])
    if not points:
        return np.array([]), np.array([])
    return np.asarray([item[0] for item in points]), np.asarray([item[1] for item in points])


def style_axis(axis, *, x_grid: bool = True) -> None:
    axis.spines["top"].set_visible(False)
    axis.spines["right"].set_visible(False)
    axis.tick_params(labelsize=8.5, colors="#333333")
    axis.grid(axis="y", color="#D8DCE2", linewidth=0.65, alpha=0.8)
    if x_grid:
        axis.grid(axis="x", color="#ECEEF1", linewidth=0.5, alpha=0.7)
    axis.set_axisbelow(True)


def mark_unavailable(axis, detail: str) -> None:
    axis.set_facecolor("#F5F6F8")
    axis.text(
        0.5, 0.52, "NOT RECORDED",
        ha="center", va="center", transform=axis.transAxes,
        fontsize=13, fontweight="bold", color="#777777",
    )
    axis.text(
        0.5, 0.39, detail,
        ha="center", va="center", transform=axis.transAxes,
        fontsize=8.5, color="#666666", wrap=True,
    )
    axis.set_xticks([])
    axis.set_yticks([])
    for spine in axis.spines.values():
        spine.set_color("#B8BDC6")


def percentile_label(values: np.ndarray) -> str:
    return f"p50 {np.median(values):.2g} / p95 {np.percentile(values, 95):.2g} ms"


def plot_resources(
    figure, grid_slot, rows: list[dict[str, str]], origin_ns: int,
    requested_modules: tuple[str, ...], warnings: list[str], xmax: float,
    time_axis_label: str,
) -> tuple[object, object]:
    subgrid = grid_slot.subgridspec(2, 1, height_ratios=(1.05, 1.0), hspace=0.08)
    cpu_axis = figure.add_subplot(subgrid[0])
    pss_axis = figure.add_subplot(subgrid[1], sharex=cpu_axis)
    cpu_axis.set_title(
        "(a) Synchronized process resources",
        loc="left", fontsize=11.5, fontweight="bold", pad=8,
    )

    active_by_module: dict[str, list[dict[str, str]]] = defaultdict(list)
    for row in rows:
        pid_count = integer_value(row, "pid_count")
        if pid_count is not None and pid_count > 0:
            active_by_module[row.get("module", "")].append(row)
    modules = [name for name in requested_modules if active_by_module.get(name)]
    if not modules and active_by_module:
        modules = sorted(active_by_module)[:4]
        warnings.append("requested resource modules absent; plotted first active modules")
    if not modules:
        mark_unavailable(cpu_axis, "No rows with pid_count > 0 in module_resources.csv")
        mark_unavailable(pss_axis, "No active process samples")
        return cpu_axis, pss_axis

    colors = {name: color for name, color in zip(modules, cycle(PALETTE))}
    for module in modules:
        module_rows = active_by_module[module]
        x_cpu, cpu = series(module_rows, "monotonic_ns", "cpu_percent", origin_ns)
        # The monitor's first CPU sample has no preceding /proc tick baseline.
        if len(cpu) > 1 and cpu[0] == 0.0 and np.any(cpu[1:] > 0.0):
            x_cpu, cpu = x_cpu[1:], cpu[1:]
        x_pss, pss = series(module_rows, "monotonic_ns", "pss_mb", origin_ns)
        label = MODULE_LABELS.get(module, module.replace("_", " "))
        if len(cpu):
            cpu_axis.plot(x_cpu, cpu, color=colors[module], linewidth=1.7, label=label)
        if len(pss):
            pss_axis.plot(x_pss, pss, color=colors[module], linewidth=1.7)

    cpu_axis.set_ylabel("CPU (%)\n100 = one core", fontsize=8.5)
    pss_axis.set_ylabel("PSS (MiB)", fontsize=8.5)
    pss_axis.set_xlabel(time_axis_label, fontsize=9)
    cpu_axis.tick_params(labelbottom=False)
    cpu_axis.set_ylim(bottom=0)
    pss_axis.set_ylim(bottom=0)
    cpu_axis.set_xlim(0, xmax)
    style_axis(cpu_axis)
    style_axis(pss_axis)
    cpu_axis.legend(
        loc="upper left", fontsize=7.3, frameon=True, framealpha=0.92,
        edgecolor="#D0D3D8", ncol=1,
    )
    cpu_axis.text(
        0.99, 0.96,
        "topo_gng is one process:\nRealSense + DD-GNG + async YOLO",
        transform=cpu_axis.transAxes, ha="right", va="top", fontsize=7.4,
        color="#7A3E00", bbox={"facecolor": "#FFF4E5", "edgecolor": "#E69F00",
                               "boxstyle": "round,pad=0.3", "alpha": 0.95},
    )
    return cpu_axis, pss_axis


def plot_perception(
    axis, rows: list[dict[str, str]], origin_ns: int, warnings: list[str], xmax: float,
    time_axis_label: str,
) -> None:
    axis.set_title(
        "(b) Perception-stage latency",
        loc="left", fontsize=11.5, fontweight="bold", pad=8,
    )
    plotted = 0
    all_positive: list[float] = []
    for field, label, color, width in STAGES:
        x, values = series(rows, "received_monotonic_ns", field, origin_ns)
        mask = values > 0.0
        x, values = x[mask], values[mask]
        if not len(values):
            continue
        all_positive.extend(values.tolist())
        axis.plot(
            x, values, color=color, linewidth=width,
            alpha=0.92 if field == "processing_total_ms" else 0.78,
            label=f"{label}: {percentile_label(values)}",
        )
        plotted += 1

    # Inference runs asynchronously. Plot one point per newly observed result
    # sequence so a repeated stale value is not mistaken for extra samples.
    inference_points: list[tuple[float, float]] = []
    seen_result_sequences: set[int] = set()
    for row in rows:
        result_sequence = integer_value(row, "yolo_result_sequence")
        elapsed = relative_seconds(row, "received_monotonic_ns", origin_ns)
        inference = finite_value(row, "yolo_inference_ms")
        if (
            result_sequence is not None and result_sequence > 0
            and result_sequence not in seen_result_sequences
            and elapsed is not None and inference is not None and inference > 0.0
        ):
            seen_result_sequences.add(result_sequence)
            inference_points.append((elapsed, inference))
    if inference_points:
        x = np.asarray([point[0] for point in inference_points])
        values = np.asarray([point[1] for point in inference_points])
        all_positive.extend(values.tolist())
        axis.scatter(
            x, values, s=19, marker="D", color="#7B3294", edgecolors="white",
            linewidths=0.35, zorder=5,
            label=f"YOLO inference (async): {percentile_label(values)}",
        )
        plotted += 1

    if not plotted:
        mark_unavailable(axis, "No finite, positive perception latency samples")
        warnings.append("perception latency samples unavailable")
        return
    if max(all_positive) / max(min(all_positive), 1e-9) >= 100.0:
        axis.set_yscale("log")
        axis.set_ylabel("Latency (ms, log scale)", fontsize=9)
    else:
        axis.set_ylim(bottom=0)
        axis.set_ylabel("Latency (ms)", fontsize=9)
    axis.set_xlabel(time_axis_label, fontsize=9)
    axis.set_xlim(0, xmax)
    style_axis(axis)
    axis.legend(
        loc="upper left", fontsize=6.8, frameon=True, framealpha=0.93,
        edgecolor="#D0D3D8", ncol=1,
    )
    axis.text(
        0.99, 0.02, f"{len(rows)} recorded frame samples",
        transform=axis.transAxes, ha="right", va="bottom", fontsize=7.5,
        color="#555555",
    )


def readable_reason(reason: str) -> str:
    known = {
        "waiting_for_labeled_environment_target": "waiting for labeled environment target",
        "no_reachable_target": "no reachable target",
        "exact_collision_failed": "exact collision validation failed",
    }
    return known.get(reason, reason.replace("_", " ") if reason else "reason not recorded")


def plot_planning(
    axis, rows: list[dict[str, str]], origin_ns: int, warnings: list[str], xmax: float,
    time_axis_label: str,
) -> None:
    axis.set_title(
        "(c) Reachability planning latency and outcome",
        loc="left", fontsize=11.5, fontweight="bold", pad=8,
    )
    groups: dict[str, list[tuple[float, float, bool]]] = defaultdict(list)
    for row in rows:
        elapsed = relative_seconds(row, "received_monotonic_ns", origin_ns)
        latency = finite_value(row, "planning_time_ms")
        if elapsed is None or latency is None:
            continue
        valid = str(row.get("valid", "")).strip().lower() in {"1", "true", "yes"}
        groups[row.get("reason", "")].append((elapsed, latency, valid))
    if not groups:
        mark_unavailable(axis, "No reachability_plan rows with planning_time_ms")
        warnings.append("reachability planning samples unavailable")
        return

    total = sum(len(points) for points in groups.values())
    valid_total = sum(valid for points in groups.values() for _, _, valid in points)
    for (reason, points), color in zip(sorted(groups.items()), cycle(PALETTE)):
        x = np.asarray([point[0] for point in points])
        values = np.asarray([point[1] for point in points])
        validity = np.asarray([point[2] for point in points], dtype=bool)
        short_reason = textwrap.fill(readable_reason(reason), width=27)
        label = f"{short_reason} (n={len(points)})"
        if np.any(validity):
            axis.scatter(
                x[validity], values[validity], marker="o", s=38,
                color=color, edgecolors="white", linewidths=0.5, label=label, zorder=4,
            )
            label = None
        if np.any(~validity):
            axis.scatter(
                x[~validity], values[~validity], marker="x", s=42,
                color=color, linewidths=1.5, label=label, zorder=4,
            )
        order = np.argsort(x)
        axis.plot(x[order], values[order], color=color, linewidth=0.8, alpha=0.38)

    axis.set_xlim(0, xmax)
    axis.set_ylim(bottom=0)
    axis.set_xlabel(time_axis_label, fontsize=9)
    axis.set_ylabel("Planning publication latency (ms)", fontsize=9)
    style_axis(axis)
    axis.legend(
        loc="best", fontsize=7.3, frameon=True, framealpha=0.93,
        edgecolor="#D0D3D8", title="Recorded reason", title_fontsize=7.6,
    )
    axis.text(
        0.99, 0.05, f"valid plans: {valid_total}/{total}\n○ valid   × invalid",
        transform=axis.transAxes, ha="right", va="bottom", fontsize=8,
        color="#333333", bbox={"facecolor": "white", "edgecolor": "#D0D3D8",
                                "boxstyle": "round,pad=0.3", "alpha": 0.92},
    )


def plot_rates_and_detections(
    axis, health_rows: list[dict[str, str]], perception_rows: list[dict[str, str]],
    origin_ns: int, warnings: list[str], xmax: float, time_axis_label: str,
) -> None:
    axis.set_title(
        "(d) Message rates and semantic output",
        loc="left", fontsize=11.5, fontweight="bold", pad=8,
    )
    topic_rows: dict[str, list[dict[str, str]]] = defaultdict(list)
    for row in health_rows:
        topic_rows[row.get("topic", "")].append(row)
    plotted_rates = 0
    for (topic, rows), color in zip(sorted(topic_rows.items()), cycle(PALETTE)):
        x, values = series(rows, "_plot_monotonic_ns", "rate_hz", origin_ns)
        if not len(values):
            continue
        axis.plot(
            x, values, marker="o", markersize=3.0, linewidth=1.3,
            color=color, label=TOPIC_LABELS.get(topic, topic.replace("_", " ")),
        )
        plotted_rates += 1
    axis.set_xlim(0, xmax)
    axis.set_ylim(bottom=0)
    axis.set_xlabel(time_axis_label, fontsize=9)
    axis.set_ylabel("Received rate (Hz)", fontsize=9)
    style_axis(axis)

    semantic_axis = axis.twinx()
    semantic_axis.spines["top"].set_visible(False)
    semantic_axis.tick_params(labelsize=8.5, colors="#333333")
    semantic_axis.set_ylabel("Count per perception frame", fontsize=9)
    semantic_axis.yaxis.set_major_locator(MaxNLocator(integer=True))
    x_det, detections = series(
        perception_rows, "received_monotonic_ns", "detections", origin_ns
    )
    x_lab, labels = series(
        perception_rows, "received_monotonic_ns", "labeled_nodes", origin_ns
    )
    semantic_handles = []
    if len(detections):
        handle = semantic_axis.scatter(
            x_det, detections, s=15, marker="s", color="#D55E00", alpha=0.7,
            label="YOLO detections / frame", zorder=5,
        )
        semantic_handles.append(handle)
    if len(labels):
        handle = semantic_axis.scatter(
            x_lab, labels, s=15, marker="^", color="#CC79A7", alpha=0.65,
            label="labeled GNG nodes / frame", zorder=5,
        )
        semantic_handles.append(handle)
    if semantic_handles:
        semantic_max = max(
            [float(np.max(values)) for values in (detections, labels) if len(values)] + [0.0]
        )
        semantic_axis.set_ylim(0, max(1.0, semantic_max * 1.12))
        if semantic_max == 0.0:
            axis.text(
                0.99, 0.04, "Recorded result: 0 detections, 0 labeled nodes",
                transform=axis.transAxes, ha="right", va="bottom", fontsize=8,
                color="#9B2C2C", bbox={"facecolor": "#FFF3F3", "edgecolor": "#D55E00",
                                         "boxstyle": "round,pad=0.3", "alpha": 0.94},
            )
    else:
        semantic_axis.set_yticks([])
        semantic_axis.set_ylabel("Semantic counts not recorded", fontsize=8, color="#777777")
        warnings.append("detection/labeled-node samples unavailable")

    rate_handles, rate_labels = axis.get_legend_handles_labels()
    semantic_labels = [handle.get_label() for handle in semantic_handles]
    if rate_handles or semantic_handles:
        axis.legend(
            rate_handles + semantic_handles, rate_labels + semantic_labels,
            loc="upper left", fontsize=7.0, frameon=True, framealpha=0.93,
            edgecolor="#D0D3D8", ncol=2,
        )
    if not plotted_rates and not semantic_handles:
        mark_unavailable(axis, "No message-rate or semantic-count samples")


def utc_extent(*datasets: list[dict[str, str]]) -> tuple[str, str]:
    values: list[str] = []
    for rows in datasets:
        for row in rows:
            value = row.get("utc_iso") or row.get("received_utc")
            if value:
                values.append(value)
    return (min(values), max(values)) if values else ("not recorded", "not recorded")


def tegrastats_annotation(path: Path, warnings: list[str]) -> str:
    if not path.is_file():
        warnings.append("tegrastats_summary.json absent; GPU/power values not shown")
        return "Jetson GPU/power summary not present; no power values shown"
    try:
        summary = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as error:
        warnings.append(f"could not read tegrastats_summary.json: {error}")
        return "Jetson GPU/power summary unreadable; no power values shown"

    statistics = summary.get("metric_statistics")
    if not isinstance(statistics, list):
        warnings.append("tegrastats summary has no metric_statistics list")
        return "Jetson GPU/power statistics not available"

    def valid_number(value) -> float | None:
        try:
            result = float(value)
        except (TypeError, ValueError):
            return None
        return result if math.isfinite(result) else None

    gr3d = next(
        (entry for entry in statistics if entry.get("metric") == "gr3d_load"), None
    )
    gr3d_text = "GR3D load not reported"
    if gr3d:
        mean = valid_number(gr3d.get("mean"))
        maximum = valid_number(gr3d.get("max"))
        if mean is not None and maximum is not None:
            gr3d_text = f"GR3D load mean/max {mean:.1f}/{maximum:.1f}%"

    rails: list[str] = []
    for entry in statistics:
        if entry.get("metric") != "power_instantaneous" or entry.get("unit") != "mW":
            continue
        mean = valid_number(entry.get("mean"))
        maximum = valid_number(entry.get("max"))
        component = str(entry.get("component", "")).strip()
        if component and mean is not None and maximum is not None:
            rails.append(f"{component} {mean / 1000.0:.2f}/{maximum / 1000.0:.2f} W")
    rails.sort()
    sample_count = integer_value(
        {"value": str(summary.get("parsed_sample_count", ""))}, "value"
    )
    sample_text = f"n={sample_count}" if sample_count is not None else "n not recorded"
    if rails:
        return (
            f"Jetson whole-recorder telemetry ({sample_text}; not event-filtered): "
            f"{gr3d_text}; instantaneous rail mean/max: "
            + ", ".join(rails)
            + ". Rails are reported individually and are not summed"
        )
    warnings.append("tegrastats summary contains no instantaneous power rails")
    return (
        f"Jetson whole-recorder telemetry ({sample_text}; not event-filtered): "
        f"{gr3d_text}; no power rail values reported"
    )


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Generate a non-claim instrumentation dashboard from a recorded run."
    )
    parser.add_argument("run_directory", type=Path)
    parser.add_argument(
        "--output-prefix", type=Path,
        help="path without extension; defaults inside RUN_DIRECTORY/figures",
    )
    parser.add_argument(
        "--modules", default=",".join(DEFAULT_MODULES),
        help="comma-separated resource modules to display",
    )
    parser.add_argument("--dpi", type=int, default=300)
    parser.add_argument("--force", action="store_true")
    args = parser.parse_args()

    run_directory = args.run_directory.expanduser().resolve()
    if not run_directory.is_dir():
        raise SystemExit(f"run directory does not exist: {run_directory}")
    if args.dpi < 100:
        raise SystemExit("--dpi must be at least 100")
    requested_modules = tuple(
        item.strip() for item in args.modules.split(",") if item.strip()
    )

    output_prefix = (
        args.output_prefix.expanduser().resolve()
        if args.output_prefix
        else run_directory / "figures" / "instrumentation_smoke_dashboard"
    )
    png_path = output_prefix.with_suffix(".png")
    pdf_path = output_prefix.with_suffix(".pdf")
    for output in (png_path, pdf_path):
        if output.exists() and not args.force:
            raise SystemExit(f"refusing to overwrite {output}; pass --force")
    output_prefix.parent.mkdir(parents=True, exist_ok=True)

    warnings: list[str] = []
    all_resource_rows = read_csv(run_directory / "module_resources.csv", warnings)
    all_perception_rows = read_csv(run_directory / "perception_metrics.csv", warnings)
    all_planning_rows = read_csv(run_directory / "reachability_plan.csv", warnings)
    all_health_rows = read_csv(run_directory / "topic_health.csv", warnings)
    operator_rows = read_csv(run_directory / "operator_events.csv", warnings)
    event_window = operator_recording_window(operator_rows, warnings)

    if event_window is not None:
        resource_rows = filter_point_samples(
            all_resource_rows, "monotonic_ns", event_window,
            "module-resource", warnings,
        )
        perception_rows = filter_point_samples(
            all_perception_rows, "received_monotonic_ns", event_window,
            "perception", warnings,
        )
        planning_rows = filter_point_samples(
            all_planning_rows, "received_monotonic_ns", event_window,
            "reachability-plan", warnings,
        )
        health_rows, health_window_count = filter_health_intervals(
            all_health_rows, event_window, warnings
        )
        origin_ns = event_window.start_ns
        xmax = event_window.duration_sec
        utc_start, utc_end = event_window.start_utc, event_window.stop_utc
        window_duration_sec = event_window.duration_sec
        window_description = "operator event window"
        time_axis_label = "Time from recording_started event (s)"
    else:
        resource_rows = all_resource_rows
        perception_rows = all_perception_rows
        planning_rows = all_planning_rows
        health_rows = []
        for row in all_health_rows:
            output = dict(row)
            if row.get("monotonic_ns"):
                output["_plot_monotonic_ns"] = row["monotonic_ns"]
            health_rows.append(output)
        health_window_count = len({
            row.get("monotonic_ns") for row in health_rows if row.get("monotonic_ns")
        })
        origin_ns = recorded_origin([
            (resource_rows, "monotonic_ns"),
            (perception_rows, "received_monotonic_ns"),
            (planning_rows, "received_monotonic_ns"),
            (health_rows, "_plot_monotonic_ns"),
        ])
        relative_times = [
            relative_seconds(row, field, origin_ns)
            for rows, field in (
                (resource_rows, "monotonic_ns"),
                (perception_rows, "received_monotonic_ns"),
                (planning_rows, "received_monotonic_ns"),
                (health_rows, "_plot_monotonic_ns"),
            )
            for row in rows
        ]
        finite_times = [value for value in relative_times if value is not None]
        raw_duration = max(finite_times) if finite_times else 1.0
        window_duration_sec = max(0.0, raw_duration)
        xmax = max(1.0, raw_duration * 1.02)
        utc_start, utc_end = utc_extent(
            resource_rows, perception_rows, planning_rows, health_rows
        )
        window_description = "recorder lifetime fallback"
        time_axis_label = "Time from first recorded sample (s)"
    jetson_note = tegrastats_annotation(run_directory / "tegrastats_summary.json", warnings)

    plt.rcParams.update({
        "font.family": "DejaVu Sans",
        "font.size": 9,
        "axes.titlecolor": "#1E2732",
        "axes.labelcolor": "#333333",
        "figure.facecolor": "white",
        "axes.facecolor": "white",
        "savefig.facecolor": "white",
        "pdf.fonttype": 42,
        "ps.fonttype": 42,
    })
    figure = plt.figure(figsize=(15.8, 10.2))
    grid = figure.add_gridspec(2, 2, left=0.07, right=0.95, bottom=0.145, top=0.825,
                              wspace=0.24, hspace=0.31)
    cpu_axis, pss_axis = plot_resources(
        figure, grid[0, 0], resource_rows, origin_ns, requested_modules, warnings, xmax,
        time_axis_label,
    )
    perception_axis = figure.add_subplot(grid[0, 1])
    planning_axis = figure.add_subplot(grid[1, 0])
    rate_axis = figure.add_subplot(grid[1, 1])
    plot_perception(
        perception_axis, perception_rows, origin_ns, warnings, xmax, time_axis_label
    )
    plot_planning(
        planning_axis, planning_rows, origin_ns, warnings, xmax, time_axis_label
    )
    plot_rates_and_detections(
        rate_axis, health_rows, perception_rows, origin_ns, warnings, xmax,
        time_axis_label,
    )

    figure.suptitle(TITLE, x=0.5, y=0.975, fontsize=19, fontweight="bold", color="#A61B1B")
    figure.text(
        0.5, 0.940,
        f"Run: {run_directory.name}  |  {window_description}: "
        f"{window_duration_sec:.3f} s  |  UTC: {utc_start} to {utc_end}",
        ha="center", va="center", fontsize=9.0, color="#3D4651",
    )
    figure.text(
        0.5, 0.914,
        f"Included samples: resource rows n={len(resource_rows)}, perception frames "
        f"n={len(perception_rows)}, planner publications n={len(planning_rows)}, "
        f"topic-health full intervals n={health_window_count} ({len(health_rows)} rows)",
        ha="center", va="center", fontsize=8.6, color="#3D4651",
    )
    figure.text(
        0.5, 0.887, jetson_note,
        ha="center", va="center", fontsize=7.8, color="#4A4F57",
    )
    figure.text(
        0.5, 0.50, "INSTRUMENTATION SMOKE\nNOT PAPER RESULT",
        ha="center", va="center", rotation=27, fontsize=47, fontweight="bold",
        color="#B00020", alpha=0.045, zorder=0,
    )
    footer = (
        "Recorded samples only; no interpolation or missing-data imputation. "
        "topo_gng CPU/PSS combines RealSense capture, DD-GNG, and async YOLO in one process. "
        "A zero-valued first process-CPU baseline is excluded when present."
    )
    if event_window is not None:
        footer += (
            " Point samples are bounded inclusively by recording_started/stopped. "
            "Message rates use only complete periodic intervals wholly inside that window, "
            "plotted at interval midpoints; startup, shutdown_partial, and boundary-overlap "
            "intervals are excluded."
        )
    if warnings:
        footer += "  Data notes: " + "; ".join(dict.fromkeys(warnings)) + "."
    figure.text(0.07, 0.018, textwrap.fill(footer, width=185), ha="left", va="bottom",
                fontsize=7.7, color="#4A4F57")

    png_metadata = {
        "Title": TITLE,
        "Description": "Recorded instrumentation smoke dashboard; not a paper result.",
        "Author": "OM6DOF experiment tooling",
    }
    pdf_metadata = {
        "Title": TITLE,
        "Subject": "Recorded instrumentation smoke dashboard; not a paper result.",
        "Creator": "plot_run_dashboard.py",
        "Keywords": "instrumentation, smoke test, DD-GNG, YOLO, reachability",
    }
    figure.savefig(png_path, dpi=args.dpi, metadata=png_metadata)
    figure.savefig(pdf_path, metadata=pdf_metadata)
    plt.close(figure)
    print(f"PNG {png_path}")
    print(f"PDF {pdf_path}")
    print(f"samples resources={len(resource_rows)} perception={len(perception_rows)} "
          f"planning={len(planning_rows)} health={len(health_rows)}")
    print(
        f"window={window_description!r} duration_sec={window_duration_sec:.9f} "
        f"origin_monotonic_ns={origin_ns} health_full_intervals={health_window_count}"
    )
    if warnings:
        print("data notes: " + "; ".join(dict.fromkeys(warnings)))


if __name__ == "__main__":
    main()
