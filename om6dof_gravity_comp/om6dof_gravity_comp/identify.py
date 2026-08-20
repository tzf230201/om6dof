"""Fit a current-space gravity and friction model from logged data.

The model, per joint i
----------------------

    I_i  =  a_i * tau_g_nominal_i(q)      gravity, shape from the URDF
          + b_i * sign(qd_i)               Coulomb friction
          + c_i * qd_i                     viscous friction
          + d_i                            bias

This is Approach A: the URDF supplies the *shape* of the gravity term through
KDL, and ``a_i`` absorbs everything that shape gets wrong by a scale -- the
current-to-torque constant, the gearing, and any uniform error in the link
masses. It cannot fix a wrong mass *distribution*, and an ``a_i`` far from
what the nominal model implies is itself the evidence that the distribution
is wrong. That is the useful diagnostic, not a failure.

Working in raw current rather than newton-metres is deliberate: the effective
torque constant of these servos is not calibrated, so converting first would
inject an unknown scale into the data. Here it lands in ``a_i`` instead,
where it is identified along with everything else.

Coulomb friction near zero velocity
-----------------------------------
``sign(qd)`` is discontinuous at zero, and around zero the measured velocity
is mostly quantisation noise, so its sign is close to random. Two ways out,
both available:

  smooth (default)  replace sign(qd) with tanh(qd / eps). Keeps every sample,
                    including the slow ones where gravity dominates and the
                    fit needs them most, and the Coulomb term simply fades to
                    zero rather than chattering.

  exclude           drop samples with |qd| < eps. Truer to the idea that
                    Coulomb friction is undefined at rest, but it throws away
                    the low-speed data.

The fraction removed (or smoothed through) is reported either way.
"""

from __future__ import annotations

import argparse
import csv
import math
import os
import sys
from datetime import datetime
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np

from om6dof_gravity_comp.units import CURRENT_TICK_MA, CURRENT_UNIT_RAW, JOINT_NAMES

MODEL_VERSION = "1.0"
FEATURE_NAMES = ("gravity_nominal", "coulomb", "viscous", "bias")
DEFAULT_DEADZONE = 0.02   # rad/s
# A joint's parameters are only identifiable from stretches where that joint
# actually moves. A dataset built by exciting one joint at a time is mostly
# stationary for any given joint, and including those stretches both drowns
# the useful data and -- because the split is by time -- can put a joint's
# entire excitation on one side of the train/validation line. Selecting the
# active stretch per joint fixes both.
DEFAULT_ACTIVE_VELOCITY = 0.01   # rad/s


# --------------------------------------------------------------------------- #
#  data                                                                        #
# --------------------------------------------------------------------------- #
def load_csv(path: str) -> Tuple[Dict[str, np.ndarray], Dict[str, str]]:
    """Read a logger CSV into arrays, keeping the '#' metadata."""
    metadata: Dict[str, str] = {}
    rows: List[dict] = []
    with open(path, "r", newline="") as handle:
        lines = []
        for line in handle:
            if line.startswith("#"):
                body = line[1:].strip()
                if ":" in body:
                    key, _, value = body.partition(":")
                    metadata[key.strip()] = value.strip()
                continue
            lines.append(line)
        reader = csv.DictReader(lines)
        for row in reader:
            rows.append(row)
    if not rows:
        raise ValueError(f"{path} contains no samples")

    columns: Dict[str, np.ndarray] = {}
    for key in rows[0]:
        try:
            columns[key] = np.array([float(r[key]) for r in rows])
        except (TypeError, ValueError):
            columns[key] = np.array([r[key] for r in rows], dtype=object)
    return columns, metadata


def stack_joint_arrays(columns: Dict[str, np.ndarray], prefix: str) -> np.ndarray:
    """Columns named <prefix><joint> gathered in canonical joint order."""
    missing = [n for n in JOINT_NAMES if f"{prefix}{n}" not in columns]
    if missing:
        raise ValueError(f"dataset is missing columns for {missing}")
    return np.column_stack([columns[f"{prefix}{n}"] for n in JOINT_NAMES])


# --------------------------------------------------------------------------- #
#  regressor                                                                   #
# --------------------------------------------------------------------------- #
# Speed at which the Stribeck excess has decayed to 1/e. Measured on this
# arm the friction falls across roughly 0.02-0.5 rad/s, so a scale in the
# middle of that captures the shape without needing a nonlinear fit.
DEFAULT_STRIBECK_SCALE = 0.15


def stribeck_feature(velocity, scale: float = DEFAULT_STRIBECK_SCALE):
    """The extra friction present just after breakaway, fading with speed.

    Measured here, friction is highest at the lowest speeds and falls as the
    joint gets going -- 48.8 down to 40.5 raw on joint 3. A model of
    b*sign(qd) + c*qd cannot represent a falling curve with c positive, so it
    fits c negative instead, which is physically impossible and was mistaken
    twice for an error in the gravity model. This column carries the falling
    part so the viscous term is free to be what it should be.
    """
    velocity = np.asarray(velocity, dtype=float)
    return np.sign(velocity) * np.exp(-np.abs(velocity) / max(scale, 1e-9))


def friction_features(
    velocity: np.ndarray, deadzone: float, mode: str = "smooth"
) -> Tuple[np.ndarray, np.ndarray]:
    """Coulomb and viscous columns for one joint.

    Returns (coulomb, viscous). In smooth mode the Coulomb column is
    tanh(qd/deadzone), which is +/-1 well away from zero and passes through
    zero continuously.
    """
    velocity = np.asarray(velocity, dtype=float)
    if mode == "smooth":
        scale = max(float(deadzone), 1e-9)
        coulomb = np.tanh(velocity / scale)
    elif mode == "exclude":
        coulomb = np.sign(velocity)
    else:
        raise ValueError(f"unknown deadzone mode {mode!r}")
    return coulomb, velocity


def build_regressor(
    gravity_nominal: np.ndarray,
    velocity: np.ndarray,
    deadzone: float,
    mode: str,
    stribeck: bool = False,
    stribeck_scale: float = DEFAULT_STRIBECK_SCALE,
) -> np.ndarray:
    """Y for one joint, in FEATURE_NAMES order (plus Stribeck when asked)."""
    coulomb, viscous = friction_features(velocity, deadzone, mode)
    columns = [
        np.asarray(gravity_nominal, dtype=float),
        coulomb,
        viscous,
        np.ones_like(viscous),
    ]
    if stribeck:
        columns.append(stribeck_feature(velocity, stribeck_scale))
    return np.column_stack(columns)


def feature_names(stribeck: bool = False):
    return FEATURE_NAMES + (("stribeck",) if stribeck else ())


# --------------------------------------------------------------------------- #
#  fitting                                                                     #
# --------------------------------------------------------------------------- #
def fit_least_squares(
    regressor: np.ndarray, target: np.ndarray, ridge: float = 0.0
) -> np.ndarray:
    """Ordinary least squares, or ridge when a positive penalty is given.

    The bias column is left unpenalised: shrinking it toward zero would bias
    every other coefficient to make up the offset.
    """
    regressor = np.asarray(regressor, dtype=float)
    target = np.asarray(target, dtype=float)
    if ridge <= 0.0:
        solution, *_ = np.linalg.lstsq(regressor, target, rcond=None)
        return solution
    penalty = np.eye(regressor.shape[1]) * float(ridge)
    penalty[-1, -1] = 0.0
    gram = regressor.T @ regressor + penalty
    return np.linalg.solve(gram, regressor.T @ target)


def metrics(measured: np.ndarray, predicted: np.ndarray) -> Dict[str, float]:
    residual = np.asarray(measured, float) - np.asarray(predicted, float)
    total = np.sum((measured - np.mean(measured)) ** 2)
    return {
        "rmse": float(np.sqrt(np.mean(residual ** 2))),
        "mae": float(np.mean(np.abs(residual))),
        "r2": float(1.0 - np.sum(residual ** 2) / total) if total > 0 else float("nan"),
        "max_abs_error": float(np.max(np.abs(residual))) if residual.size else float("nan"),
        "samples": int(residual.size),
    }


def time_split(count: int, validation_fraction: float = 0.3) -> Tuple[np.ndarray, np.ndarray]:
    """Split by time, not at random.

    Neighbouring samples at 100 Hz are nearly identical, so a random split
    puts near-copies of the training data into validation and reports an
    error far below what the model would show on a fresh run.
    """
    cut = int(count * (1.0 - validation_fraction))
    cut = max(1, min(count - 1, cut))
    return np.arange(0, cut), np.arange(cut, count)


# --------------------------------------------------------------------------- #
#  pipeline                                                                    #
# --------------------------------------------------------------------------- #
def nominal_gravity_columns(positions: np.ndarray) -> np.ndarray:
    """tau_g from the URDF via KDL, one column per joint.

    Imported lazily so the fitting code stays usable (and testable) on a
    machine without PyKDL or a robot description.
    """
    import subprocess
    from ament_index_python.packages import get_package_share_directory
    from om6dof_gravity_comp.gravity_model import GravityModel

    share = get_package_share_directory("om6dof_description")
    path = os.path.join(share, "urdf", "om6dof.urdf.xacro")
    result = subprocess.run(["xacro", path], capture_output=True, text=True,
                            timeout=60)
    if result.returncode != 0:
        raise RuntimeError(f"xacro failed: {result.stderr.strip()}")
    model = GravityModel(result.stdout)
    return np.array([model.torques(row) for row in positions])


def fit_dataset(
    columns: Dict[str, np.ndarray],
    gravity: np.ndarray,
    deadzone: float = DEFAULT_DEADZONE,
    mode: str = "smooth",
    ridge: float = 0.0,
    validation_fraction: float = 0.3,
    active_velocity: float = DEFAULT_ACTIVE_VELOCITY,
    stribeck: bool = False,
    stribeck_scale: float = DEFAULT_STRIBECK_SCALE,
) -> Dict[str, dict]:
    """Fit every joint and score it on data the fit never saw."""
    velocity = stack_joint_arrays(columns, "qd_")
    current = stack_joint_arrays(columns, "i_raw_")

    results: Dict[str, dict] = {}
    for index, joint in enumerate(JOINT_NAMES):
        qd = velocity[:, index]
        target = current[:, index]
        nominal = gravity[:, index]

        # Only the stretches where this joint was actually driven. Without
        # this, a one-joint-at-a-time dataset trains each joint almost
        # entirely on samples where it was standing still.
        active = np.abs(qd) >= active_velocity
        active_percent = float(100.0 * active.mean())
        if active.sum() < 100:
            results[joint] = {
                "error": "joint barely moved in this dataset; excite it first",
                "active_percent": active_percent,
                "samples_total": int(qd.size),
            }
            continue
        qd, target, nominal = qd[active], target[active], nominal[active]

        keep = np.ones_like(qd, dtype=bool)
        if mode == "exclude":
            keep = np.abs(qd) >= deadzone
        removed = float(100.0 * (1.0 - keep.mean()))

        regressor = build_regressor(nominal[keep], qd[keep], deadzone, mode,
                                    stribeck, stribeck_scale)
        target_kept = target[keep]
        if regressor.shape[0] <= regressor.shape[1]:
            results[joint] = {"error": "not enough samples after the deadzone"}
            continue

        # How much of the current actually tracks gravity. Without this a
        # gravity coefficient fitted to a joint with no gravity signal reads
        # like a measurement, and its sign reads like a finding -- which is
        # exactly how joint5's negative coefficient was first misread. A
        # strong overall R2 does not rescue it: friction and bias alone
        # explain the current well while gravity contributes nothing.
        gravity_span = float(np.ptp(nominal))
        if gravity_span > 1e-9 and np.std(target_kept) > 1e-9:
            correlation = float(np.corrcoef(nominal[keep], target_kept)[0, 1])
        else:
            correlation = 0.0

        train, validate = time_split(regressor.shape[0], validation_fraction)
        coefficients = fit_least_squares(
            regressor[train], target_kept[train], ridge)
        results[joint] = {
            "coefficients": dict(zip(feature_names(stribeck),
                                     coefficients.tolist())),
            "train": metrics(target_kept[train], regressor[train] @ coefficients),
            "validation": metrics(
                target_kept[validate], regressor[validate] @ coefficients),
            "gravity_correlation": correlation,
            "gravity_span_nm": gravity_span,
            "gravity_identifiable": bool(abs(correlation) >= 0.6),
            "deadzone_removed_percent": removed,
            "active_percent": active_percent,
            "samples_total": int(velocity.shape[0]),
            "samples_used": int(keep.sum()),
            "velocity_span": [float(qd.min()), float(qd.max())],
            "current_span": [float(target.min()), float(target.max())],
        }
    return results


def save_yaml(path: str, results: Dict[str, dict], meta: Dict[str, str]) -> None:
    import yaml

    payload = {
        "om6dof_current_model": {
            "model_version": MODEL_VERSION,
            "fitted_at_iso": datetime.now().astimezone().isoformat(),
            "dataset": meta.get("dataset", "unknown"),
            "dataset_written_at": meta.get("written_at_iso", "unknown"),
            "current_unit": CURRENT_UNIT_RAW,
            "current_tick_ma": CURRENT_TICK_MA,
            "joint_order": list(JOINT_NAMES),
            "features": list(FEATURE_NAMES),
            "deadzone_mode": meta.get("deadzone_mode", "smooth"),
            "velocity_deadzone": float(meta.get("velocity_deadzone", DEFAULT_DEADZONE)),
            "ridge": float(meta.get("ridge", 0.0)),
            "joints": results,
        }
    }
    os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
    with open(path, "w") as handle:
        yaml.safe_dump(payload, handle, sort_keys=False)


def load_yaml(path: str) -> dict:
    import yaml
    with open(path, "r") as handle:
        return yaml.safe_load(handle)["om6dof_current_model"]


def make_plots(
    columns: Dict[str, np.ndarray],
    gravity: np.ndarray,
    results: Dict[str, dict],
    deadzone: float,
    mode: str,
    directory: str,
) -> List[str]:
    """One figure per view, no combined panels, no hidden constants."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    os.makedirs(directory, exist_ok=True)
    velocity = stack_joint_arrays(columns, "qd_")
    position = stack_joint_arrays(columns, "q_")
    current = stack_joint_arrays(columns, "i_raw_")
    written: List[str] = []

    for index, joint in enumerate(JOINT_NAMES):
        entry = results.get(joint, {})
        if "coefficients" not in entry:
            continue
        coefficients = np.array([entry["coefficients"][k] for k in FEATURE_NAMES])
        regressor = build_regressor(
            gravity[:, index], velocity[:, index], deadzone, mode)
        predicted = regressor @ coefficients
        residual = current[:, index] - predicted

        for name, plot in (
            ("measured_vs_predicted", lambda ax: (
                ax.plot(current[:, index], label="measured", linewidth=0.8),
                ax.plot(predicted, label="predicted", linewidth=0.8),
                ax.set_xlabel("sample"), ax.set_ylabel("current (raw ticks)"),
                ax.legend())),
            ("residual", lambda ax: (
                ax.plot(residual, linewidth=0.8),
                ax.set_xlabel("sample"),
                ax.set_ylabel("residual current (raw ticks)"))),
            ("residual_histogram", lambda ax: (
                ax.hist(residual, bins=60),
                ax.set_xlabel("residual current (raw ticks)"),
                ax.set_ylabel("count"))),
            ("residual_vs_velocity", lambda ax: (
                ax.scatter(velocity[:, index], residual, s=2),
                ax.set_xlabel("joint velocity (rad/s)"),
                ax.set_ylabel("residual current (raw ticks)"))),
            ("residual_vs_position", lambda ax: (
                ax.scatter(position[:, index], residual, s=2),
                ax.set_xlabel("joint position (rad)"),
                ax.set_ylabel("residual current (raw ticks)"))),
        ):
            figure, axis = plt.subplots(figsize=(8, 4))
            plot(axis)
            axis.set_title(f"{joint} — {name.replace('_', ' ')}")
            axis.grid(True, alpha=0.3)
            figure.tight_layout()
            target = os.path.join(directory, f"{joint}_{name}.png")
            figure.savefig(target, dpi=110)
            plt.close(figure)
            written.append(target)
    return written


def report(results: Dict[str, dict]) -> str:
    lines = [
        f"{'joint':8} {'a(grav)':>10} {'b(coul)':>9} {'c(visc)':>9} "
        f"{'d(bias)':>9} {'val R2':>8} {'g.corr':>8} {'g.span':>8} {'n':>7}"
    ]
    for joint in JOINT_NAMES:
        entry = results.get(joint, {})
        if "coefficients" not in entry:
            lines.append(f"{joint:8} {entry.get('error', 'not fitted')}")
            continue
        c = entry["coefficients"]
        v = entry["validation"]
        extra = (f" {c['stribeck']:9.2f}" if "stribeck" in c else "")
        lines.append(
            f"{joint:8} {c['gravity_nominal']:10.3f} {c['coulomb']:9.2f} "
            f"{c['viscous']:9.2f} {c['bias']:9.2f} "
            f"{v['r2']:8.3f} {entry['gravity_correlation']:8.3f} "
            f"{entry['gravity_span_nm']:8.3f} {v['samples']:7d}" + extra
            + ("" if entry["gravity_identifiable"]
               else "   <- gravity not identifiable")
        )
    lines.append("")
    lines.append("g.corr  correlation between nominal gravity and measured "
                 "current; below 0.6 the")
    lines.append("        gravity coefficient is fitting noise, whatever its "
                 "sign or the overall R2.")
    lines.append("g.span  how much the nominal gravity torque varies over the "
                 "data, in Nm.")
    return "\n".join(lines)


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("dataset", help="CSV from the logger")
    parser.add_argument("--output", default="config/identified_gravity_friction.yaml")
    parser.add_argument("--plots", default=None,
                        help="directory for figures; omit to skip plotting")
    parser.add_argument("--deadzone", type=float, default=DEFAULT_DEADZONE,
                        help="rad/s below which Coulomb friction is ill-defined")
    parser.add_argument("--deadzone-mode", choices=("smooth", "exclude"),
                        default="smooth")
    parser.add_argument("--ridge", type=float, default=0.0,
                        help="ridge penalty; 0 means ordinary least squares")
    parser.add_argument("--validation-fraction", type=float, default=0.3)
    parser.add_argument("--stribeck", action="store_true",
                        help="add a falling-friction column; measured "
                             "friction on this arm drops with speed, which a "
                             "plain viscous term can only fit with an "
                             "impossible negative coefficient")
    parser.add_argument("--stribeck-scale", type=float,
                        default=DEFAULT_STRIBECK_SCALE)
    parser.add_argument("--active-velocity", type=float,
                        default=DEFAULT_ACTIVE_VELOCITY,
                        help="rad/s above which a joint counts as being "
                             "driven; only those samples are fitted")
    parser.add_argument("--compare-ridge", action="store_true",
                        help="also fit with a ridge penalty and print both")
    args = parser.parse_args(argv)

    columns, meta = load_csv(args.dataset)
    print(f"{len(next(iter(columns.values())))} samples from {args.dataset}")
    if meta.get("current_unit") and meta["current_unit"] != CURRENT_UNIT_RAW:
        print(f"WARNING: dataset says current_unit={meta['current_unit']}, "
              f"this code assumes {CURRENT_UNIT_RAW}", file=sys.stderr)

    positions = stack_joint_arrays(columns, "q_")
    print("computing nominal gravity from the URDF...")
    gravity = nominal_gravity_columns(positions)

    runs = [("ordinary least squares", args.ridge)]
    if args.compare_ridge:
        runs.append(("ridge (1.0)", 1.0 if args.ridge <= 0 else args.ridge * 10))

    final = None
    for label, ridge in runs:
        results = fit_dataset(columns, gravity, args.deadzone,
                              args.deadzone_mode, ridge,
                              args.validation_fraction, args.active_velocity,
                              args.stribeck, args.stribeck_scale)
        print(f"\n=== {label} ===")
        print(report(results))
        if final is None:
            final = (results, ridge)

    results, ridge = final
    meta_out = dict(meta)
    meta_out.update({
        "dataset": os.path.abspath(args.dataset),
        "deadzone_mode": args.deadzone_mode,
        "velocity_deadzone": args.deadzone,
        "ridge": ridge,
    })
    save_yaml(args.output, results, meta_out)
    print(f"\nparameters written to {args.output}")

    if args.plots:
        written = make_plots(columns, gravity, results, args.deadzone,
                             args.deadzone_mode, args.plots)
        print(f"{len(written)} figures written to {args.plots}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
