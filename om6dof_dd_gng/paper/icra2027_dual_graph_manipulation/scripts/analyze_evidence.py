#!/usr/bin/env python3
"""Audit saved workspace data for the new paper; never runs ROS, IK or hardware.

Only the selected CSV, plots and descriptive summaries are generated. The source
experiment and its historical summary are read-only. Numerical IK residuals are
not physical localization errors or manipulation success measurements.
"""
from __future__ import annotations

import argparse
import collections
import csv
import hashlib
import json
import math
from pathlib import Path
import statistics

PAPER = Path(__file__).resolve().parents[1]
REPO = Path(__file__).resolve().parents[4]
DEFAULT = REPO / "experiments/cartesian_workspace/results/comparison_v1_v2_25mm_20260909/v2"
STATUS_LABELS = {
    "pose_found": "Full pose, conditioning threshold satisfied (green)",
    "pose_near_singular": "Full pose, near singular (blue)",
    "orientation_unresolved": "Position only; requested orientation unresolved",
    "collision_candidates_only": "Only colliding position candidates found",
    "position_unresolved": "Position unresolved within search budget",
}


def sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def read_csv(path):
    with Path(path).open(newline="") as stream:
        reader = csv.DictReader(stream)
        return reader.fieldnames, list(reader)


def quantile(values, fraction):
    v = sorted(values)
    index = (len(v) - 1) * fraction
    lo = math.floor(index)
    hi = math.ceil(index)
    return v[lo] + (v[hi] - v[lo]) * (index - lo)


def descriptive(values):
    assert values and all(math.isfinite(value) for value in values)
    return {"n": len(values), "min": min(values), "median": statistics.median(values),
            "mean": statistics.mean(values), "p95": quantile(values, .95), "max": max(values)}


def provenance(path):
    path = Path(path).resolve()
    try:
        name = str(path.relative_to(REPO))
    except ValueError:
        name = str(path)
    return {"repository_relative_path": name, "sha256": sha256(path), "bytes": path.stat().st_size}


def audit_smoke_runs():
    base = REPO / "om6dof_dd_gng/experiments/four_object_pick/data"
    records = []
    for directory in sorted(base.glob("smoke_*")):
        if not directory.is_dir():
            continue
        _, plans = read_csv(directory / "reachability_plan.csv")
        _, events = read_csv(directory / "operator_events.csv")
        summary_path = directory / "run_summary.json"
        summary = json.loads(summary_path.read_text())
        records.append({
            "directory": str(directory.relative_to(REPO)),
            "excluded_from_paper_outcomes": True,
            "reason": "Instrumentation smoke run, explicitly excluded by the dataset README",
            "raw_planner_publication_rows": len(plans),
            "raw_valid_planner_publication_rows": sum(row["valid"] == "1" for row in plans),
            "raw_plan_reasons": dict(collections.Counter(row["reason"] for row in plans)),
            "operator_events": dict(collections.Counter(row["event"] for row in events)),
            "analysis_ready": summary.get("completeness", {}).get("analysis_ready"),
            "provenance": [provenance(summary_path), provenance(directory / "reachability_plan.csv"),
                           provenance(directory / "operator_events.csv")],
        })
    return records


def plot_workspace(green, output):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    plt.rcParams.update({"font.family": "DejaVu Sans", "font.size": 8,
                         "axes.labelsize": 8, "axes.titlesize": 8,
                         "xtick.labelsize": 7, "ytick.labelsize": 7,
                         "pdf.fonttype": 42, "ps.fonttype": 42})
    fig, axes = plt.subplots(1, 2, figsize=(3.5, 2.2), constrained_layout=True)
    x = [float(row["x_mm"]) / 1000 for row in green]
    for ax, ordinate, title in zip(axes, ["y", "z"], ["(a) XY projection", "(b) XZ projection"]):
        values = [float(row[f"{ordinate}_mm"]) / 1000 for row in green]
        ax.scatter(x, values, s=2.6, color="#15803d", alpha=.7, linewidths=0, rasterized=True)
        ax.axhline(0, color=".65", lw=.4, zorder=0)
        ax.axvline(0, color=".65", lw=.4, zorder=0)
        ax.set(xlabel="X (m)", ylabel=f"{ordinate.upper()} (m)", title=title,
               xlim=(-.1, .43), ylim=(-.4, .4))
        ax.set_xticks([0, .2, .4])
        ax.set_yticks([-.4, -.2, 0, .2, .4])
        ax.set_aspect("equal", adjustable="box")
        ax.grid(alpha=.18, linewidth=.4)
        for spine in ax.spines.values():
            spine.set_linewidth(.5)
    output.mkdir(parents=True, exist_ok=True)
    fig.savefig(output / "workspace_audit.pdf", metadata={"CreationDate": None, "ModDate": None})
    fig.savefig(output / "workspace_audit.png", dpi=350)
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dataset", type=Path, default=DEFAULT)
    parser.add_argument("--output", type=Path, default=PAPER)
    parser.add_argument("--no-plot", action="store_true")
    args = parser.parse_args()
    dataset = args.dataset.resolve()
    data_out = args.output / "data"
    data_out.mkdir(parents=True, exist_ok=True)
    fields, rows = read_csv(dataset / "points.csv")
    source_summary = json.loads((dataset / "summary.json").read_text())
    counts = collections.Counter(row["status"] for row in rows)
    assert dict(counts) == source_summary["counts"], "CSV status counts disagree with saved summary"
    assert len(rows) == source_summary["points"]
    assert len({tuple(float(row[c]) for c in ["x_mm", "y_mm", "z_mm"]) for row in rows}) == len(rows)
    green = [row for row in rows if row["status"] == "pose_found"
             and row["position_found"] == "1" and row["pose_found"] == "1"]
    position_count = sum(row["position_found"] == "1" for row in rows)
    pose_count = sum(row["pose_found"] == "1" for row in rows)
    assert position_count == source_summary["position_found"]
    assert pose_count == source_summary["pose_found"]
    scanner_args = source_summary["arguments"]
    for row in green:
        assert all(math.isfinite(float(row[f"q{i}_rad"])) for i in range(1, 7))
        assert float(row["position_error_mm"]) <= scanner_args["position_tolerance_mm"]
        assert float(row["orientation_error_deg"]) <= scanner_args["orientation_tolerance_deg"]
        assert float(row["rcond"]) >= scanner_args["singular_threshold"]
    with (data_out / "green_witnesses.csv").open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fields, lineterminator="\n")
        writer.writeheader()
        writer.writerows(green)
    table = [{"status": status, "description": label, "count": counts[status],
              "percent_of_grid": 100 * counts[status] / len(rows)}
             for status, label in STATUS_LABELS.items()]
    with (data_out / "workspace_status_counts.csv").open("w", newline="") as stream:
        writer = csv.DictWriter(stream, table[0].keys(), lineterminator="\n")
        writer.writeheader()
        writer.writerows(table)
    # Historical gripper_rotation applies a fixed tip/gripper rotation. For the
    # recorded zero roll/pitch, local tip +X points down regardless of radial yaw.
    zero_roll_pitch = all(abs(float(row["target_roll_deg"])) < 1e-12
                          and abs(float(row["target_pitch_deg"])) < 1e-12 for row in green)
    result = {
        "scope": "Offline analysis of stored V2 IK scan; no new motion, IK or collision tests",
        "grid_rows": len(rows), "grid_spacing_mm": scanner_args["spacing_mm"],
        "sphere_radius_mm": source_summary["radius_mm"],
        "position_witnesses": position_count, "full_pose_witnesses_including_near_singular": pose_count,
        "selected_green_witnesses": len(green), "selected_percent_of_grid": 100 * len(green) / len(rows),
        "selected_percent_of_position_witnesses": 100 * len(green) / position_count,
        "status_table": table, "scanner_settings": scanner_args,
        "historical_scan_elapsed_seconds_not_remeasured": source_summary["elapsed_seconds"],
        "green_numerical_residual_statistics": {
            key: descriptive([float(row[key]) for row in green])
            for key in ["position_error_mm", "orientation_error_deg", "rcond", "joint_margin_deg"]},
        "green_grid_coordinate_extent_mm": {
            key: [min(float(row[key]) for row in green), max(float(row[key]) for row in green)]
            for key in ["x_mm", "y_mm", "z_mm"]},
        "orientation_audit": {
            "all_selected_target_roll_pitch_zero": zero_roll_pitch,
            "target_yaw_deg": descriptive([float(row["target_yaw_deg"]) for row in green]),
            "tip_from_gripper_in_scan_cpp": [[0, 0, -1], [0, 1, 0], [1, 0, 0]],
            "target_tip_rotation_formula": "Rz(yaw) Ry(pitch) Rx(roll) transpose(tip_from_gripper)",
            "requested_tip_local_positive_x_world_axis": [0, 0, -1] if zero_roll_pitch else None,
            "requested_tip_local_positive_z_world_axis_formula": "[cos(yaw), sin(yaw), 0]" if zero_roll_pitch else None,
            "orientation_fact": "R_tip e_x = -e_z and R_tip e_z = [cos(yaw), sin(yaw), 0] for every selected target",
            "achieved_axis_deviation_bound_deg_from_recorded_rotation_residual": max(float(row["orientation_error_deg"]) for row in green),
            "scope": "Analytic target-orientation audit, not new FK; achieved angular residual bounds apply",
            "implication": "This bank is orientation-specific; it cannot establish arbitrary front/side approach coverage",
        },
        "limitations": source_summary["limitations"] + [
            "Numerical residuals are relative to the saved model, not measured robot or camera accuracy",
            "Selected rows contain one numerical joint witness each, not one executed hardware trial",
            "Joint margin is measured from already shrunken limits, so zero is compatible with the configured 0.02 rad hard-limit margin",
            "No current cluster-centre reach, gripper closure, lift or collision-avoidance success rate is established",
        ],
        "excluded_smoke_runs": audit_smoke_runs(),
    }
    (data_out / "workspace_summary.json").write_text(json.dumps(result, indent=2, sort_keys=True) + "\n")
    inputs = [dataset / name for name in ["points.csv", "summary.json", "model.urdf", "seed_bank.csv", "command.sh", "SHA256SUMS"]]
    inputs += [REPO / "experiments/cartesian_workspace/cpp/scan.cpp",
               REPO / "om6dof_dd_gng/launch/ddgng_experiment_reachability.launch.py",
               REPO / "om6dof_dd_gng/experiments/four_object_pick/data/README.md"]
    manifest = {"inputs": [provenance(path) for path in inputs],
                "derived_green_csv_sha256": sha256(data_out / "green_witnesses.csv"),
                "analysis_script_sha256": sha256(__file__)}
    original_hashes = {}
    for line in (dataset / "SHA256SUMS").read_text().splitlines():
        digest, path = line.split(maxsplit=1)
        original_hashes[Path(path).name] = digest
    manifest["historical_hash_checks"] = {
        "scanner_source_matches_saved_record": sha256(REPO / "experiments/cartesian_workspace/cpp/scan.cpp") == original_hashes["scan.cpp"],
        "model_snapshot_matches_saved_record": sha256(dataset / "model.urdf") == original_hashes["model.urdf"],
    }
    assert all(manifest["historical_hash_checks"].values()), "Historical scanner/model provenance mismatch"
    (data_out / "source_provenance.json").write_text(json.dumps(manifest, indent=2, sort_keys=True) + "\n")
    if not args.no_plot:
        plot_workspace(green, args.output / "figures")
    print(json.dumps({"grid": len(rows), "green": len(green), "status_counts": dict(counts),
                      "csv_summary_consistent": True, "orientation_specific_downward_tip_x": zero_roll_pitch}, indent=2))


if __name__ == "__main__":
    main()
