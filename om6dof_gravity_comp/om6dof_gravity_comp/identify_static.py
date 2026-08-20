"""Identify the gravity current from a static sweep, where the arm stands still.

Why the moving fit is not enough
--------------------------------
The moving model carries a velocity-dependent friction term, and at zero
velocity that term contributes nothing while the arm is still being held up.
Fitted only on moving data it has never seen the standing case, and measured
here it scores as low as R2 = -0.62 there. A leader arm is often nearly
still, so this is the half that matters.

How standing still is separable at all
--------------------------------------
It is not, from one visit. Holding a pose, the motor supplies gravity minus
whatever the gearbox is holding by friction, and the two cannot be told
apart. Arriving at the same pose from opposite directions separates them,
because friction opposes the last motion and gravity does not:

    gravity  = (I_from_above + I_from_below) / 2
    friction = (I_from_above - I_from_below) / 2

So this looks for dwells -- stretches where every joint is still -- groups
the ones at the same pose, and uses the pair.

What it fits
------------
Per joint, on the gravity half only:

    I_gravity = a * tau_nominal(q) + d

No velocity terms: there is no velocity. The friction half is reported
separately as the holding friction, which is the number that says how much
of the arm's weight the gearbox carries on its own.
"""

from __future__ import annotations

import argparse
import os
import sys
from datetime import datetime
from typing import Dict, List, Tuple

import numpy as np

from om6dof_gravity_comp.identify import (
    fit_least_squares,
    load_csv,
    metrics,
    nominal_gravity_columns,
    stack_joint_arrays,
    time_split,
)
from om6dof_gravity_comp.units import CURRENT_TICK_MA, CURRENT_UNIT_RAW, JOINT_NAMES

STILL_VELOCITY = 0.005      # rad/s; below this every joint counts as stopped
MIN_DWELL_SAMPLES = 50      # at 100 Hz, half a second of settled data
SETTLE_FRACTION = 0.4       # drop the first part of each dwell while it rings
POSE_TOLERANCE_RAD = 0.02   # two dwells within this are the same pose


def find_dwells(
    velocity: np.ndarray,
    still_velocity: float = STILL_VELOCITY,
    min_samples: int = MIN_DWELL_SAMPLES,
) -> List[Tuple[int, int]]:
    """Contiguous stretches where every joint is stopped."""
    still = np.all(np.abs(velocity) < still_velocity, axis=1)
    dwells = []
    start = None
    for index, value in enumerate(still):
        if value and start is None:
            start = index
        elif not value and start is not None:
            if index - start >= min_samples:
                dwells.append((start, index))
            start = None
    if start is not None and len(still) - start >= min_samples:
        dwells.append((start, len(still)))
    return dwells


def approach_directions(
    velocity: np.ndarray,
    dwells: List[Tuple[int, int]],
    lookback: int = 30,
) -> np.ndarray:
    """Which way each joint was travelling just before it stopped.

    The whole separation depends on this. Pairing two dwells that were both
    approached from the same side leaves the friction in place instead of
    cancelling it, and nothing downstream can tell -- the numbers simply come
    out wrong, which is what happened when poses were grouped by position
    alone.
    """
    signs = []
    for start, _ in dwells:
        window = velocity[max(0, start - lookback):start]
        if window.size == 0:
            signs.append(np.zeros(velocity.shape[1]))
            continue
        # The largest excursion, not the mean: the tail of the move is
        # already decelerating toward zero.
        peak = window[np.argmax(np.abs(window), axis=0),
                      np.arange(velocity.shape[1])]
        signs.append(np.sign(peak))
    return np.array(signs)


def summarise_dwells(
    positions: np.ndarray,
    current: np.ndarray,
    dwells: List[Tuple[int, int]],
    settle_fraction: float = SETTLE_FRACTION,
) -> Tuple[np.ndarray, np.ndarray]:
    """Mean pose and mean current for each dwell, after it has settled.

    The first part of every dwell is dropped: the arm is still ringing down
    from the move, and that transient current has nothing to do with holding
    the pose.
    """
    poses, currents = [], []
    for start, end in dwells:
        skip = start + int((end - start) * settle_fraction)
        if end - skip < 5:
            continue
        poses.append(positions[skip:end].mean(axis=0))
        currents.append(current[skip:end].mean(axis=0))
    return np.array(poses), np.array(currents)


def group_poses(
    poses: np.ndarray, tolerance: float = POSE_TOLERANCE_RAD
) -> List[List[int]]:
    """Dwells at the same pose, however many times it was visited."""
    groups: List[List[int]] = []
    for index, pose in enumerate(poses):
        for group in groups:
            if np.max(np.abs(poses[group[0]] - pose)) <= tolerance:
                group.append(index)
                break
        else:
            groups.append([index])
    return groups


def separate(
    poses: np.ndarray,
    currents: np.ndarray,
    groups: List[List[int]],
    directions: np.ndarray,
    joint: int,
) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Gravity and holding friction, per joint, from opposed approaches.

    Done per joint because the approach direction is per joint: one pose can
    be reached with joint 2 coming down and joint 3 coming up. A pose whose
    visits all share a direction for this joint is dropped -- keeping it
    would fold that pose's friction into the gravity estimate, which is
    exactly the error that made the first version of this fit worthless.
    """
    kept_poses, gravity, friction = [], [], []
    for group in groups:
        rising = [i for i in group if directions[i, joint] > 0]
        falling = [i for i in group if directions[i, joint] < 0]
        if not rising or not falling:
            continue
        high = currents[rising, joint].mean()
        low = currents[falling, joint].mean()
        kept_poses.append(poses[group].mean(axis=0))
        gravity.append(0.5 * (high + low))
        friction.append(0.5 * abs(high - low))
    return np.array(kept_poses), np.array(gravity), np.array(friction)


def leave_one_out(design: np.ndarray, target: np.ndarray) -> dict:
    """Cross-validate by holding out one pose at a time.

    A static sweep yields tens of poses, not thousands of samples, and they
    arrive in grid order. Splitting the tail off as validation then hands
    over one corner of the workspace -- three points, from a region the fit
    never saw -- and the resulting R2 says more about the split than the
    model. Leaving out one pose at a time uses every point for both jobs and
    is the honest measure at this sample size.
    """
    predictions = np.zeros_like(target)
    for index in range(len(target)):
        keep = np.arange(len(target)) != index
        coefficients = fit_least_squares(design[keep], target[keep])
        predictions[index] = design[index] @ coefficients
    return metrics(target, predictions)


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("dataset")
    parser.add_argument("--output",
                        default="config/identified_static.yaml")
    parser.add_argument("--still-velocity", type=float, default=STILL_VELOCITY)
    parser.add_argument("--validation-fraction", type=float, default=0.3)
    args = parser.parse_args(argv)

    columns, meta = load_csv(args.dataset)
    if meta.get("current_unit") not in (None, CURRENT_UNIT_RAW):
        print(f"dataset is in {meta['current_unit']}", file=sys.stderr)
        return 1

    positions = stack_joint_arrays(columns, "q_")
    velocity = stack_joint_arrays(columns, "qd_")
    current = stack_joint_arrays(columns, "i_raw_")

    dwells = find_dwells(velocity, args.still_velocity)
    print(f"{len(dwells)} dwells found in {positions.shape[0]} samples")
    if len(dwells) < 8:
        print("too few dwells; run static_sweep first", file=sys.stderr)
        return 1

    dwell_poses, dwell_currents = summarise_dwells(positions, current, dwells)
    directions = approach_directions(velocity, dwells)
    groups = group_poses(dwell_poses)
    print(f"{len(groups)} distinct poses among the dwells")

    results: Dict[str, dict] = {}
    print(f"\n{'joint':8} {'a(grav)':>10} {'d(bias)':>9} {'LOO R2':>8} "
          f"{'g.corr':>8} {'hold fric':>10} {'poses':>7}")
    for index, joint in enumerate(JOINT_NAMES):
        poses, gravity_current, friction_current = separate(
            dwell_poses, dwell_currents, groups, directions, index)
        if len(poses) < 6:
            print(f"{joint:8} only {len(poses)} poses approached from both "
                  "sides for this joint")
            results[joint] = {"error": "too few opposed approaches",
                              "poses": int(len(poses))}
            continue
        nominal = nominal_gravity_columns(poses)
        target = gravity_current
        column = nominal[:, index]
        span = float(np.ptp(column))
        correlation = (float(np.corrcoef(column, target)[0, 1])
                       if span > 1e-9 and np.std(target) > 1e-9 else 0.0)
        design = np.column_stack([column, np.ones_like(column)])
        coefficients = fit_least_squares(design, target)
        score = leave_one_out(design, target)
        holding = float(np.mean(friction_current))
        identifiable = abs(correlation) >= 0.6
        results[joint] = {
            "gravity": float(coefficients[0]),
            "bias": float(coefficients[1]),
            "gravity_correlation": correlation,
            "gravity_identifiable": identifiable,
            "holding_friction_raw": holding,
            "validation": score,
        }
        results[joint]["poses"] = int(len(poses))
        print(f"{joint:8} {coefficients[0]:10.2f} {coefficients[1]:9.2f} "
              f"{score['r2']:8.3f} {correlation:8.3f} {holding:10.1f} "
              f"{len(poses):7d}"
              + ("" if identifiable else "   <- not identifiable"))

    print(f"\nholding friction is the current the gearbox carries on its own, "
          f"in raw ticks ({CURRENT_TICK_MA} mA each)")

    import yaml
    payload = {
        "om6dof_static_model": {
            "model_version": "static-1.0",
            "fitted_at_iso": datetime.now().astimezone().isoformat(),
            "dataset": os.path.abspath(args.dataset),
            "current_unit": CURRENT_UNIT_RAW,
            "joint_order": list(JOINT_NAMES),
            "joints": results,
        }
    }
    os.makedirs(os.path.dirname(args.output) or ".", exist_ok=True)
    with open(args.output, "w") as handle:
        yaml.safe_dump(payload, handle, sort_keys=False)
    print(f"written to {args.output}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
