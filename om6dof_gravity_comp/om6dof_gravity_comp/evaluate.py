"""Score an already-fitted model against a dataset it was never fitted on.

The validation split inside ``identify`` is a held-out *tail of the same run*,
which shares that run's poses and speeds. A second, separately recorded
dataset is the harder and more honest test, and this is what runs it.
"""

from __future__ import annotations

import argparse
import sys
from typing import Dict

import numpy as np

from om6dof_gravity_comp.identify import (
    DEFAULT_STRIBECK_SCALE,
    build_regressor,
    feature_names,
    load_csv,
    load_yaml,
    metrics,
    nominal_gravity_columns,
    stack_joint_arrays,
)
from om6dof_gravity_comp.units import CURRENT_UNIT_RAW, JOINT_NAMES


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("model", help="YAML written by identify")
    parser.add_argument("dataset", help="a CSV the model was not fitted on")
    args = parser.parse_args(argv)

    model = load_yaml(args.model)
    if model.get("current_unit") not in (None, CURRENT_UNIT_RAW):
        print(f"model is in {model['current_unit']}, not {CURRENT_UNIT_RAW}",
              file=sys.stderr)
        return 1

    columns, meta = load_csv(args.dataset)
    positions = stack_joint_arrays(columns, "q_")
    velocity = stack_joint_arrays(columns, "qd_")
    current = stack_joint_arrays(columns, "i_raw_")
    gravity = nominal_gravity_columns(positions)

    deadzone = float(model.get("velocity_deadzone", 0.02))
    mode = str(model.get("deadzone_mode", "smooth"))

    print(f"{'joint':8} {'RMSE':>9} {'MAE':>9} {'R2':>8} {'maxerr':>9} {'n':>7}")
    worst = 0.0
    for index, joint in enumerate(JOINT_NAMES):
        entry = model["joints"].get(joint, {})
        if "coefficients" not in entry:
            print(f"{joint:8} not fitted")
            continue
        # Follow whatever columns the model was fitted with. Scoring a
        # Stribeck model with the plain regressor would silently drop a term
        # and report the error as the model's.
        stribeck = "stribeck" in entry["coefficients"]
        coefficients = np.array(
            [entry["coefficients"][k] for k in feature_names(stribeck)])
        regressor = build_regressor(
            gravity[:, index], velocity[:, index], deadzone, mode,
            stribeck, float(model.get("stribeck_scale", DEFAULT_STRIBECK_SCALE)))
        score = metrics(current[:, index], regressor @ coefficients)
        worst = max(worst, score["rmse"])
        print(f"{joint:8} {score['rmse']:9.2f} {score['mae']:9.2f} "
              f"{score['r2']:8.3f} {score['max_abs_error']:9.1f} "
              f"{score['samples']:7d}")

    print(f"\nworst joint RMSE: {worst:.2f} raw ticks "
          f"({worst * float(model.get('current_tick_ma', 2.69)):.0f} mA)")
    print("Residual is raw current, not Nm. Converting needs a calibrated "
          "torque constant this arm does not have yet.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
