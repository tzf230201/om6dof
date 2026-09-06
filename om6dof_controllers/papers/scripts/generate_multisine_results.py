#!/usr/bin/env python3
"""Generate reproducible multisine figures and chronological held-out metrics.

The source log is the physical ``teaching_all`` run.  Raw DYNAMIXEL Present
Current is modeled in raw units; it is never relabeled as a directly measured
joint torque.  ``Tg`` is evaluated from the current whole-URDF potential-energy
model imported from the controller fitting script.
"""

import argparse
import csv
import importlib.util
import json
import math
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np


ARM_JOINTS = ("joint1", "joint2", "joint3", "joint4", "joint5", "joint6")
PLOT_JOINTS = ("joint2", "joint3", "joint5")
SUPPORTED_GRAVITY_JOINTS = set(PLOT_JOINTS)
DEADZONE_RAD_S = 0.02
VELOCITY_MIN_RAD_S = 0.005


def load_fit_module(path):
    spec = importlib.util.spec_from_file_location("fit_gravity_friction", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def load_log(path):
    records = []
    with path.open(newline="", encoding="utf-8") as handle:
        for row in csv.DictReader(handle):
            time_s = float(row["elapsed_s"])
            positions = {
                joint: float(row[f"{joint}_position_rad"])
                for joint in ARM_JOINTS
            }
            velocities = {
                joint: float(row[f"{joint}_velocity_rad_s"])
                for joint in ARM_JOINTS
            }
            currents = {
                joint: float(row[f"{joint}_effort_raw"])
                for joint in ARM_JOINTS
            }
            if all(math.isfinite(value) for values in (positions, velocities, currents)
                   for value in values.values()):
                records.append({
                    "time_s": time_s,
                    "positions": positions,
                    "velocities": velocities,
                    "currents": currents,
                })
    if not records:
        raise RuntimeError(f"No valid samples in {path}")
    return records


def gravity_records(records, model):
    for record in records:
        record["gravity"] = model.gravity(record["positions"])
    return records


def feature_matrix(records, joint, include_gravity):
    columns = []
    if include_gravity:
        columns.append([record["gravity"][joint] for record in records])
    columns.extend([
        [math.tanh(record["velocities"][joint] / DEADZONE_RAD_S) for record in records],
        [record["velocities"][joint] for record in records],
        [1.0 for _ in records],
    ])
    return np.asarray(columns, dtype=float).T


def metrics(measured, predicted):
    residual = measured - predicted
    rmse = float(math.sqrt(np.mean(residual ** 2)))
    mae = float(np.mean(np.abs(residual)))
    centered = measured - np.mean(measured)
    denominator = float(np.dot(centered, centered))
    r_squared = float(1.0 - np.dot(residual, residual) / denominator) if denominator > 0.0 else float("nan")
    return {"rmse_raw": rmse, "mae_raw": mae, "r_squared": r_squared}


def fit_joint(train_records, test_records, joint):
    train = [record for record in train_records if abs(record["velocities"][joint]) >= VELOCITY_MIN_RAD_S]
    test = [record for record in test_records if abs(record["velocities"][joint]) >= VELOCITY_MIN_RAD_S]
    if len(train) < 20 or len(test) < 20:
        raise RuntimeError(f"Insufficient nonzero-velocity samples for {joint}")
    gravity_span = np.ptp([record["gravity"][joint] for record in train])
    include_gravity = bool(gravity_span >= 1.0e-6)
    design_train = feature_matrix(train, joint, include_gravity)
    y_train = np.asarray([record["currents"][joint] for record in train], dtype=float)
    parameters, _, rank, singular = np.linalg.lstsq(design_train, y_train, rcond=None)
    design_test = feature_matrix(test, joint, include_gravity)
    y_test = np.asarray([record["currents"][joint] for record in test], dtype=float)
    predicted_test = design_test @ parameters

    if include_gravity:
        gravity_scale, coulomb, viscous, bias = (float(value) for value in parameters)
    else:
        gravity_scale = None
        coulomb, viscous, bias = (float(value) for value in parameters)

    return {
        "joint": joint,
        "gravity_identifiable": include_gravity,
        "gravity_scale_raw_per_nm": gravity_scale,
        "coulomb_raw": coulomb,
        "viscous_raw_per_rad_s": viscous,
        "bias_raw": bias,
        "train_samples": len(train),
        "test_samples": len(test),
        "rank": int(rank),
        "condition_number": float(singular[0] / singular[-1]) if singular[-1] > 0.0 else float("inf"),
        "test": metrics(y_test, predicted_test),
        "test_time_s": [record["time_s"] for record in test],
        "test_current_raw": y_test.tolist(),
        "test_prediction_raw": predicted_test.tolist(),
    }


def whole_run_ranges(records):
    output = {}
    for joint in ARM_JOINTS:
        positions = [record["positions"][joint] for record in records]
        velocities = [record["velocities"][joint] for record in records]
        gravity = [record["gravity"][joint] for record in records]
        currents = [record["currents"][joint] for record in records]
        output[joint] = {
            "position_min_rad": float(min(positions)),
            "position_max_rad": float(max(positions)),
            "velocity_min_rad_s": float(min(velocities)),
            "velocity_max_rad_s": float(max(velocities)),
            "gravity_min_nm": float(min(gravity)),
            "gravity_max_nm": float(max(gravity)),
            "current_min_raw": float(min(currents)),
            "current_max_raw": float(max(currents)),
        }
    return output


def configure_plot_style():
    plt.rcParams.update({
        "font.size": 8,
        "axes.labelsize": 8,
        "axes.titlesize": 8,
        "legend.fontsize": 7,
        "xtick.labelsize": 7,
        "ytick.labelsize": 7,
        "pdf.fonttype": 42,
        "ps.fonttype": 42,
        "axes.spines.top": False,
        "axes.spines.right": False,
    })


def save_motion_figure(records, output):
    configure_plot_style()
    one_cycle = [record for record in records if record["time_s"] <= 63.0]
    colors = {"joint2": "#0072B2", "joint3": "#D55E00", "joint5": "#009E73"}
    rows = [
        ("positions", "q [rad]", "(a) measured position"),
        ("velocities", "q̇ [rad/s]", "(b) measured velocity"),
        ("gravity", "Tg [N·m]", "(c) URDF gravity torque"),
    ]
    figure, axes = plt.subplots(3, 1, figsize=(7.16, 4.45), sharex=True, constrained_layout=True)
    for axis, (field, ylabel, title) in zip(axes, rows):
        for joint in PLOT_JOINTS:
            axis.plot(
                [record["time_s"] for record in one_cycle],
                [record[field][joint] for record in one_cycle],
                label=joint.upper(), color=colors[joint], linewidth=1.1,
            )
        axis.set_ylabel(ylabel)
        axis.set_title(title, loc="left")
        axis.grid(True, color="0.88", linewidth=0.6)
    axes[0].legend(ncol=3, loc="best", frameon=False)
    axes[-1].set_xlabel("elapsed time [s]")
    figure.savefig(output, bbox_inches="tight")
    plt.close(figure)


def save_heldout_figure(fits, split_s, output):
    configure_plot_style()
    colors = {"joint2": "#0072B2", "joint3": "#D55E00", "joint5": "#009E73"}
    figure, axes = plt.subplots(1, 3, figsize=(7.16, 2.25), sharex=True, constrained_layout=True)
    for axis, joint in zip(axes, PLOT_JOINTS):
        fit = fits[joint]
        time_s = np.asarray(fit["test_time_s"])
        first_window = time_s <= min(time_s[0] + 25.0, time_s[-1])
        axis.plot(time_s[first_window] - split_s,
                  np.asarray(fit["test_current_raw"])[first_window],
                  color="0.30", linewidth=0.9, label="measured")
        axis.plot(time_s[first_window] - split_s,
                  np.asarray(fit["test_prediction_raw"])[first_window],
                  color=colors[joint], linewidth=1.1, label="predicted")
        axis.set_title(f"{joint.upper()}: $R^2$={fit['test']['r_squared']:.3f}")
        axis.set_xlabel("held-out time [s]")
        axis.grid(True, color="0.88", linewidth=0.6)
    axes[0].set_ylabel("Present Current [raw]")
    axes[0].legend(frameon=False, loc="best")
    figure.savefig(output, bbox_inches="tight")
    plt.close(figure)


def write_metrics_csv(path, fits, ranges):
    fields = [
        "joint", "position_min_rad", "position_max_rad", "velocity_min_rad_s",
        "velocity_max_rad_s", "gravity_min_nm", "gravity_max_nm",
        "current_min_raw", "current_max_raw", "gravity_scale_raw_per_nm",
        "test_rmse_raw", "test_mae_raw", "test_r_squared", "gravity_used",
    ]
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        for joint in ARM_JOINTS:
            fit = fits[joint]
            row = dict(ranges[joint])
            row.update({
                "joint": joint,
                "gravity_scale_raw_per_nm": "" if fit["gravity_scale_raw_per_nm"] is None else fit["gravity_scale_raw_per_nm"],
                "test_rmse_raw": fit["test"]["rmse_raw"],
                "test_mae_raw": fit["test"]["mae_raw"],
                "test_r_squared": fit["test"]["r_squared"],
                "gravity_used": joint in SUPPORTED_GRAVITY_JOINTS,
            })
            writer.writerow(row)


def public_summary(fits, ranges, source, stride, split_s, duration):
    report = {
        "source_csv": str(source),
        "raw_samples": None,
        "analysis_stride": stride,
        "effective_sample_rate_hz": None,
        "chronological_train_interval_s": [0.0, split_s],
        "chronological_test_interval_s": [split_s, duration],
        "velocity_min_rad_s": VELOCITY_MIN_RAD_S,
        "friction_deadzone_rad_s": DEADZONE_RAD_S,
        "gravity_support_joints": sorted(SUPPORTED_GRAVITY_JOINTS),
        "ranges": ranges,
        "fits": {},
    }
    for joint, fit in fits.items():
        report["fits"][joint] = {
            key: value for key, value in fit.items()
            if key not in ("test_time_s", "test_current_raw", "test_prediction_raw")
        }
    return report


def main():
    package_root = Path(__file__).resolve().parents[2]
    repository_root = package_root.parent
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--csv",
        type=Path,
        default=package_root / (
            "papers/experiments/multisine_identification/data/"
            "multisine_teaching_all.csv"))
    parser.add_argument("--xacro", type=Path,
                        default=repository_root /
                        "om6dof_description/urdf/om6dof.urdf.xacro")
    parser.add_argument("--fit-script", type=Path,
                        default=package_root / "scripts/fit_gravity_friction.py")
    parser.add_argument("--output-dir", type=Path,
                        default=package_root / "papers")
    parser.add_argument("--stride", type=int, default=10,
                        help="Evaluate the URDF gravity model every N raw rows (default: 10).")
    parser.add_argument("--start-s", type=float, default=2.0,
                        help="Discard startup data before this elapsed time (default: 2).")
    parser.add_argument("--split-s", type=float, default=120.0,
                        help="End of the chronological training interval in seconds (default: 120).")
    parser.add_argument("--guard-s", type=float, default=2.0,
                        help="Unscored temporal gap after training in seconds (default: 2).")
    parser.add_argument("--end-margin-s", type=float, default=2.0,
                        help="Discard settling data this long before log end (default: 2).")
    args = parser.parse_args()
    if (args.stride <= 0 or args.start_s < 0.0 or args.split_s <= args.start_s or
            args.guard_s < 0.0 or args.end_margin_s < 0.0):
        parser.error("invalid stride or temporal split arguments")

    raw_records = load_log(args.csv)
    duration = raw_records[-1]["time_s"]
    test_start_s = args.split_s + args.guard_s
    test_end_s = duration - args.end_margin_s
    if test_start_s >= test_end_s:
        parser.error("the requested train/guard/test intervals do not fit inside the log")
    records = raw_records[::args.stride]
    fit_module = load_fit_module(args.fit_script)
    model = fit_module.UrdfGravity(fit_module.expand_xacro(str(args.xacro)))
    gravity_records(records, model)
    train_records = [record for record in records
                     if args.start_s <= record["time_s"] < args.split_s]
    test_records = [record for record in records
                    if test_start_s <= record["time_s"] < test_end_s]
    fits = {joint: fit_joint(train_records, test_records, joint) for joint in ARM_JOINTS}
    ranges = whole_run_ranges(records)

    figures = args.output_dir / "figures"
    data_dir = args.output_dir / "data"
    figures.mkdir(parents=True, exist_ok=True)
    data_dir.mkdir(parents=True, exist_ok=True)
    save_motion_figure(records, figures / "multisine_motion_measurements.pdf")
    save_heldout_figure(fits, test_start_s, figures / "multisine_heldout_current_fit.pdf")
    write_metrics_csv(data_dir / "multisine_teaching_all_heldout_metrics.csv", fits, ranges)
    summary = public_summary(fits, ranges, args.csv, args.stride, args.split_s, duration)
    summary["chronological_train_interval_s"] = [args.start_s, args.split_s]
    summary["guard_interval_s"] = [args.split_s, test_start_s]
    summary["chronological_test_interval_s"] = [test_start_s, test_end_s]
    summary["raw_samples"] = len(raw_records)
    summary["effective_sample_rate_hz"] = len(records) / duration
    with (data_dir / "multisine_teaching_all_heldout_summary.json").open("w", encoding="utf-8") as handle:
        json.dump(summary, handle, indent=2)
        handle.write("\n")

    print(f"raw samples: {len(raw_records)}")
    print(f"analysis samples: {len(records)} (stride {args.stride})")
    print(f"chronological split: train [{args.start_s:.1f}, {args.split_s:.1f}) s, "
          f"guard [{args.split_s:.1f}, {test_start_s:.1f}) s, "
          f"test [{test_start_s:.1f}, {test_end_s:.1f}) s")
    for joint in ARM_JOINTS:
        fit = fits[joint]
        scale = fit["gravity_scale_raw_per_nm"]
        scale_text = "--" if scale is None else f"{scale:.3f} raw/Nm"
        print(f"{joint}: scale={scale_text}, test RMSE={fit['test']['rmse_raw']:.3f} raw, "
              f"test R2={fit['test']['r_squared']:.3f}")


if __name__ == "__main__":
    main()
