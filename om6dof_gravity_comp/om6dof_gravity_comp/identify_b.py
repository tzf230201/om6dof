"""Approach B: identify the mass parameters themselves, not a scale on the URDF's.

Approach A fits one number per joint that scales the URDF's gravity shape. It
cannot fix a wrong *distribution* of mass -- only a uniform error -- which is
why its viscous friction came out negative on this arm: the fit had nowhere
else to put the shape error.

Here the shape is identified too. Gravity torque is linear in the per-link
mass parameters (see gravity_regressor), so the whole model stays linear:

    I = [ Y_gravity(q) | sign(qd) | qd | 1 ] . [ phi ; b ; c ; d ]

phi absorbs the current-to-torque scale along with the masses, so no
calibration constant is needed here either.

Rank deficiency is expected, not a fault: mass on a joint whose axis lies
along gravity never shows up, and neighbouring links trade off. The fit uses
a least-norm solution and reports the rank so the result is not mistaken for
a set of physically separated masses.
"""

from __future__ import annotations

import argparse
import os
import subprocess
import sys
from datetime import datetime
from typing import Dict

import numpy as np

from om6dof_gravity_comp.identify import (
    DEFAULT_ACTIVE_VELOCITY,
    DEFAULT_DEADZONE,
    friction_features,
    load_csv,
    metrics,
    stack_joint_arrays,
    time_split,
)
from om6dof_gravity_comp.units import CURRENT_UNIT_RAW, JOINT_NAMES


def _load_regressor():
    from ament_index_python.packages import get_package_share_directory
    from om6dof_gravity_comp.gravity_regressor import GravityRegressor
    share = get_package_share_directory("om6dof_description")
    path = os.path.join(share, "urdf", "om6dof.urdf.xacro")
    result = subprocess.run(["xacro", path], capture_output=True, text=True,
                            timeout=60)
    if result.returncode != 0:
        raise RuntimeError(f"xacro failed: {result.stderr.strip()}")
    return GravityRegressor(result.stdout)


def build_design(
    regressor,
    positions: np.ndarray,
    velocity: np.ndarray,
    deadzone: float,
    mode: str,
) -> np.ndarray:
    """Stack every joint's equation into one system.

    All six joints share phi -- the same masses load all of them -- so they
    are fitted together rather than one at a time. Friction is per joint, so
    each gets its own three columns.
    """
    samples = positions.shape[0]
    joints = len(JOINT_NAMES)
    parameters = regressor.parameter_count
    columns = parameters + 3 * joints
    design = np.zeros((samples * joints, columns))

    for index in range(samples):
        gravity_block = regressor.regressor(positions[index])
        for joint in range(joints):
            row = index * joints + joint
            design[row, :parameters] = gravity_block[joint]
            coulomb, viscous = friction_features(
                np.array([velocity[index, joint]]), deadzone, mode)
            base = parameters + 3 * joint
            design[row, base] = coulomb[0]
            design[row, base + 1] = viscous[0]
            design[row, base + 2] = 1.0
    return design


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("dataset")
    parser.add_argument("--output",
                        default="config/identified_gravity_friction_b.yaml")
    parser.add_argument("--deadzone", type=float, default=DEFAULT_DEADZONE)
    parser.add_argument("--deadzone-mode", choices=("smooth", "exclude"),
                        default="smooth")
    parser.add_argument("--active-velocity", type=float,
                        default=DEFAULT_ACTIVE_VELOCITY)
    parser.add_argument("--validation-fraction", type=float, default=0.3)
    parser.add_argument("--max-samples", type=int, default=4000,
                        help="cap on rows used; the regressor is rebuilt per "
                             "sample so the whole log is needlessly slow")
    args = parser.parse_args(argv)

    columns, meta = load_csv(args.dataset)
    if meta.get("current_unit") not in (None, CURRENT_UNIT_RAW):
        print(f"dataset is in {meta['current_unit']}, expected "
              f"{CURRENT_UNIT_RAW}", file=sys.stderr)
        return 1

    positions = stack_joint_arrays(columns, "q_")
    velocity = stack_joint_arrays(columns, "qd_")
    current = stack_joint_arrays(columns, "i_raw_")

    # Only stretches where something is moving; a stationary arm says nothing
    # about friction and repeats one gravity configuration.
    moving = np.any(np.abs(velocity) >= args.active_velocity, axis=1)
    if moving.sum() < 200:
        print("too few moving samples; excite the arm first", file=sys.stderr)
        return 1
    index = np.where(moving)[0]
    if index.size > args.max_samples:
        index = index[np.linspace(0, index.size - 1, args.max_samples).astype(int)]
    positions, velocity, current = (positions[index], velocity[index],
                                    current[index])
    print(f"{index.size} moving samples selected")

    print("building the gravity regressor...")
    regressor = _load_regressor()
    design = build_design(regressor, positions, velocity, args.deadzone,
                          args.deadzone_mode)
    target = current.reshape(-1)

    rank = int(np.linalg.matrix_rank(design, tol=1e-8))
    print(f"design matrix {design.shape}, rank {rank}")
    if rank < design.shape[1]:
        print(f"  {design.shape[1] - rank} parameter combinations are not "
              "separable from this data; using the least-norm solution")

    # Split by whole samples, not by rows, so a sample's six joint equations
    # never straddle the train/validation line.
    train_samples, validate_samples = time_split(
        positions.shape[0], args.validation_fraction)
    joints = len(JOINT_NAMES)
    train_rows = np.concatenate(
        [np.arange(s * joints, (s + 1) * joints) for s in train_samples])
    validate_rows = np.concatenate(
        [np.arange(s * joints, (s + 1) * joints) for s in validate_samples])

    solution, *_ = np.linalg.lstsq(design[train_rows], target[train_rows],
                                   rcond=None)
    predicted = design @ solution

    print(f"\n{'joint':8} {'RMSE':>9} {'MAE':>9} {'R2':>8} {'maxerr':>9}")
    for joint_index, joint in enumerate(JOINT_NAMES):
        rows = validate_rows[validate_rows % joints == joint_index]
        score = metrics(target[rows], predicted[rows])
        print(f"{joint:8} {score['rmse']:9.2f} {score['mae']:9.2f} "
              f"{score['r2']:8.3f} {score['max_abs_error']:9.1f}")

    overall = metrics(target[validate_rows], predicted[validate_rows])
    print(f"\noverall validation RMSE {overall['rmse']:.2f} raw, "
          f"R2 {overall['r2']:.3f}")

    import yaml
    payload = {
        "om6dof_current_model_b": {
            "model_version": "B-1.0",
            "fitted_at_iso": datetime.now().astimezone().isoformat(),
            "dataset": os.path.abspath(args.dataset),
            "current_unit": CURRENT_UNIT_RAW,
            "joint_order": list(JOINT_NAMES),
            "gravity_parameters": dict(zip(
                regressor.parameter_names,
                [float(v) for v in solution[:regressor.parameter_count]])),
            "friction": {
                joint: {
                    "coulomb": float(solution[regressor.parameter_count + 3 * i]),
                    "viscous": float(solution[regressor.parameter_count + 3 * i + 1]),
                    "bias": float(solution[regressor.parameter_count + 3 * i + 2]),
                }
                for i, joint in enumerate(JOINT_NAMES)
            },
            "design_rank": rank,
            "design_columns": int(design.shape[1]),
            "samples": int(positions.shape[0]),
        }
    }
    os.makedirs(os.path.dirname(args.output) or ".", exist_ok=True)
    with open(args.output, "w") as handle:
        yaml.safe_dump(payload, handle, sort_keys=False)
    print(f"written to {args.output}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
