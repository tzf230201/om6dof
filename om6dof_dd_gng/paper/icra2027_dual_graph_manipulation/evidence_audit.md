# Evidence audit for the new ICRA 2027 draft

This audit separates the implemented dual-graph method from results actually
available in the repository. It does not treat screenshots, preview messages,
unit-test counts, or recorder completion as successful physical pick trials.
No robot, camera, ROS node, new IK scan, or exact collision checker was run for
this audit. The original experiments and previous papers were not modified.

## Reproduce

From `ros2_ws/src`:

```bash
python3 om6dof/om6dof_dd_gng/paper/icra2027_dual_graph_manipulation/scripts/analyze_evidence.py
```

Python 3 and Matplotlib suffice. `--no-plot` produces all tabular results with
only the Python standard library. The script checks row uniqueness, recomputes
status totals, compares them with the saved scan summary, validates every green
witness's finite joint values and recorded residual/conditioning thresholds,
and verifies the original scanner-source and model-snapshot SHA256 records.
It does not remeasure the saved solver residuals.

Generated files:

- `data/workspace_summary.json`: exact counts, descriptive statistics, scanner
  settings, orientation audit, and excluded smoke-record inventory.
- `data/workspace_status_counts.csv`: compact status table.
- `data/green_witnesses.csv`: selected witness rows; preserves original numeric
  text and column ordering. This is derived offline numerical data.
- `data/source_provenance.json`: source and derived SHA256 provenance.
- `scripts/check_witness_frames.py` and `data/frame_verification.json`: independent
  NumPy/XML forward kinematics of all 3,198 stored joint witnesses, using the
  original frozen URDF rather than the current robot model.
- `figures/workspace_audit.pdf` and `.png`: XY and XZ projections of the 3,198
  selected green samples, suitable for a 3.5-inch manuscript column. Several
  3-D samples can project to the same pixel; these are not new 2-D experiments.

## What the available workspace file establishes

Source: `experiments/cartesian_workspace/results/comparison_v1_v2_25mm_20260909/v2/points.csv`
(paths in this document are relative to the `om6dof` repository).

CSV SHA256:
`ded51632edcd1b041cd3e3d07c9d9aa04dace55ef1624fdf80f6fb1ad636fa1b`.

The recorded experiment is an offline C++17 Eigen/KDL bounded multi-start
Levenberg–Marquardt scan of a saved V2 URDF. It contains 33,401 grid positions,
25 mm spacing inside a 500 mm radius sphere. Settings are: a 2,000-sample random
seed bank, four nearest bank seeds plus ready/zero seeds, up to 100 solver
iterations, RNG seed 42, 0.02 rad joint-limit shrinkage, 25 mm chain-capsule
radius, 1 mm position tolerance, 0.5 degree orientation tolerance, Jacobian
translation scaling by 0.3 m, and reciprocal condition threshold 0.01. Each grid
point tests a single requested orientation with radial yaw, not all of SO(3).

| Recorded category | Count | Percentage of all grid points |
| --- | ---: | ---: |
| Full pose, conditioning threshold satisfied (`pose_found`, green) | 3,198 | 9.5746% |
| Full pose near singular (`pose_near_singular`, blue) | 334 | 1.0000% |
| Position witness, requested orientation unresolved | 11,643 | 34.8582% |
| Only colliding position candidates found | 358 | 1.0718% |
| Position unresolved within search budget | 17,868 | 53.4954% |
| Total | 33,401 | 100% |

There are 15,175 position witnesses and 3,532 full-pose witnesses when the blue
category is included. The experiment launch selects exactly 3,198 green rows:
9.57% of the scanned grid and 21.07% of the position witnesses. Numeric
`pose_found=1` alone would incorrectly include 334 near-singular blue rows.
An unresolved sample does not prove mechanical unreachability.

| Quantity over the 3,198 selected witnesses | Minimum | Median | Maximum |
| --- | ---: | ---: | ---: |
| Recorded position residual (mm) | 0.0000483111 | 0.000340806 | 0.999672 |
| Recorded orientation residual (degrees) | 0.000000954262 | 0.0000689125 | 0.439607 |
| Scaled Jacobian reciprocal condition | 0.0100197 | 0.0451951 | 0.120240 |
| Margin to shrunken joint limits (degrees) | 0 | 25.6589 | 70.8541 |

These small residuals measure numerical agreement with the saved URDF, **not**
camera accuracy or measured robot endpoint error. Zero margin in the last row
is a boundary of already shrunken limits; it is compatible with the configured
0.02 rad margin to hard limits. The saved scan time, 36.0456 s, is one historical
run without a new timing measurement and must not be presented as a benchmark
of the current planner.

## Critical orientation finding: this green bank requests downward tool +X

The scanner source matches the hash recorded with the historical scan:
`c9eaf86ab68fbf77b85abb2e9ddb2ee99b2d38498a848ad85fa9da40f16d4d06`.
The exact relevant source is `experiments/cartesian_workspace/cpp/scan.cpp`,
lines 115–119:

```cpp
M3 gripper_rotation(const V3 &rpy) {
  M3 tip_from_gripper;
  tip_from_gripper << 0,0,-1, 0,1,0, 1,0,0;
  return (Eigen::AngleAxisd(rpy[2], V3::UnitZ()) * Eigen::AngleAxisd(rpy[1], V3::UnitY()) *
          Eigen::AngleAxisd(rpy[0], V3::UnitX())).toRotationMatrix() * tip_from_gripper.transpose();
}
```

The saved summary names `end_effector_link` as the tip. Every selected green
row has requested roll and pitch equal to zero. At lines 246–248, radial mode
sets yaw to `atan2(y,x)` before calling the function above. Define

```text
A = [[0, 0, -1],
     [0, 1,  0],
     [1, 0,  0]]
R_tip(yaw) = R_z(yaw) A^T
R_tip e_x = [0, 0, -1]^T
R_tip e_z = [cos(yaw), sin(yaw), 0]^T.
```

Thus a radial scan label describes the horizontal direction of local **+Z**,
while local **+X points down** in every requested full-pose target. The maximum
recorded whole-rotation residual, 0.439607 degrees, also bounds each achieved
tool-axis angular deviation from its requested axis. This derivation concerns
the stored orientation targets; the independent joint-level check below also
verifies the actual numerical witness orientations.

### Independent forward-kinematics confirmation

Reproduce the separate verification with:

```bash
python3 om6dof/om6dof_dd_gng/paper/icra2027_dual_graph_manipulation/scripts/check_witness_frames.py
```

This program uses NumPy and XML parsing only. It reconstructs the URDF chain
from `world` through the six revolute joints to `end_effector_link`, applies
URDF origin transforms followed by joint-axis rotations, and evaluates all
3,198 stored six-joint configurations. It first checks the original CSV and
frozen URDF SHA256 values. It does not import ROS/KDL or query any robot.

| Independent FK quantity | Maximum over 3,198 witnesses |
| --- | ---: |
| Local +X angle from world downward (degrees) | 0.439607047709 |
| Local +Z tilt from the horizontal plane (degrees) | 0.439607047709 |
| Absolute vertical component of the unit local +Z axis | 0.007672515119 |
| Difference from recorded KDL position residual (mm) | 1.21158e-13 |
| Difference from recorded KDL orientation residual (degrees) | 2.35247e-14 |

Every computed rotation is orthonormal to a Frobenius error below 1e-12. The
independent FK residuals match the archived KDL residuals for every row. All
selected witnesses have local +X within 0.5 degrees of downward; none in this
bank has a horizontal local-+X approach. These statements concern the frozen
numerical model, not measured physical gripper orientations. The full statistics,
checks, chain, script hash and input hashes are recorded in
`data/frame_verification.json`.

This matters because the current pregrasp filter uses local `[1,0,0]`. These
green poses can satisfy a downward approach without providing a horizontal
front approach. Changing a drawn arrow or choosing the cluster centre cannot
create missing orientations in a fixed witness bank. A subsequent study must
generate and validate witnesses under the actual tool-frame convention and
include front/side orientations, or perform separately validated online pose
refinement. The present results do not establish that a front-facing grasp is
reachable, nor that this is the only cause of any live planning failure.

## What collision evidence means

The offline scan checks nonadjacent chain capsules at candidate endpoints.
It does not inspect complete robot meshes, a target, a table, obstacles,
interpolated paths, payload dynamics, or physical clearances. The full sampled
sphere includes negative Z. Green is therefore an admissible numerical seed
category, not a guarantee of a collision-free scene-specific motion.

The current planner's exact mesh checks and target protection are implementation
properties. They need independent per-query and executed-motion results to
support performance or reliability claims. A projected graph edge is not by
itself a proof of collision-free swept motion. A sampled collision checker also
does not provide an unqualified continuous-time collision guarantee.

## Excluded evidence

`om6dof_dd_gng/experiments/four_object_pick/data/README.md` explicitly excludes
`smoke_*` folders from paper trials. Both discovered recordings have only
`recording_started` and `recording_stopped` operator events:

- `smoke_instrumentation_clean_20260910_run_001`: `analysis_ready=false`;
  16 raw planner publications, zero valid publications, all waiting for a
  labelled target. Its summary has 10 plan rows inside the official recording
  window; raw rows include recorder startup/shutdown intervals. These repeated
  publications are not independent trials.
- `smoke_live_20260910_run_001`: nine raw planner publications, zero valid
  publications, all waiting for a labelled target. This is a superseded
  instrumentation capture.

The two smoke folders establish recorder/instrumentation behavior only. They
cannot contribute manipulation success, target selection accuracy, grasp
success, latency with objects, or collision avoidance rates. The local
`semantic_ddgng_v1_20260910` folder contains historical source/test validation,
not executed pickup trials of the present cluster-centre method. Earlier paper
datasets describe earlier systems; they must be cited as legacy evidence only,
never relabelled as a benchmark of this implementation.

## Claims the new draft can and cannot make now

Supported: implemented architecture; equations traceable to source; exact
saved-data category counts; witness residual and conditioning distributions;
source/model provenance; the orientation-convention finding; a reproducible
diagnostic demonstration of why positional reachability is insufficient.

Not supported yet: physical pick-and-lift success percentage; superiority to
MoveIt/OMPL or any baseline; statistically measured collision avoidance;
camera-calibration accuracy from the solver residuals; frontend/Boxer speed
claims from screenshots; exhaustive reachability; or universal side-grasp
coverage. A paper targeting those claims needs new, explicitly logged trials
with shared scenes, task outcomes, baselines, and failure categories.
