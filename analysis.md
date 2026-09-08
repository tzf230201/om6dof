# OM6DOF Cartesian workspace analysis

This offline experiment contains **3911 discrete points**. The report shows every constant-X, constant-Y, and constant-Z plane at multiples of **50 mm**, from negative to positive coordinates within the sample range. On each plane, the other two coordinates vary; the report is not limited to the X=Y=Z=0 slices.

Solver parameters, target orientation, tolerances, model radius, seeds, and computation time are recorded in [summary.json](summary.json). Joint values and the orientation actually tested at each point are in [points.csv](points.csv). Open the [interactive 3D viewer](viewer3d.html), [all constant-coordinate planes](index.html), or [per-plane statistics](slices.csv).

## Main results

- Position found without requiring a specific orientation: **1928/3911 (49.3%)**.
- Both position and the tested orientation found: **380/3911 (9.7%)**.
- Among the poses found, 12 points are classified as near singularity. This describes the best candidate found; it does not prove that every configuration at that point is singular.
- 114 selected position candidates have less than 1 degree of margin to an effective joint limit. This indicates sensitivity of those candidates, not that every solution lies near a limit.

| Category | Points | Percentage of all points |
|---|---:|---:|
| Pose found | 368 | 9.41% |
| Pose near singularity | 12 | 0.31% |
| Position found; orientation unresolved | 1548 | 39.58% |
| Colliding candidates only | 9 | 0.23% |
| Position unresolved | 1974 | 50.47% |

These percentages count only tested grid points; they do not measure continuous workspace volume or the physical robot's probability of success.

## Interpretation for mechanical design

1. **Green** provides a witness joint configuration satisfying the position, tested orientation, joint limits, and model self-collision check.
2. **Yellow** means a pose was found, but the best candidate's Jacobian is poorly conditioned. Small Cartesian motions may require large joint changes. The rcond metric depends on length normalization; compare only experiments with identical settings.
3. **Blue** separates orientation constraints from position reachability. It helps evaluate gripper direction, wrist range, and tasks allowing a free orientation. It does not prove the requested orientation is impossible.
4. **Purple** means the search found only colliding position candidates under the approximate collision model; it does not prove that no collision-free configuration exists.
5. **Red** means the position search did not succeed. Possible causes include geometry, joint limits, too few seeds, or convergence to a local minimum. Do not treat it as proof of a mechanical dead zone.

Investigate suspected problem regions with a finer local grid, more seeds/iterations, several orientations, and mesh collision checks. Comparing zero joint margin with the operational margin can separate software constraints from the URDF range, but does not change or establish the physical hardware limits.

## Selected-candidate statistics

Margins are measured against the effective joint limits used by the scanner. Position-only results use the candidate with the smallest position error; full-pose results use the candidate with the best rcond.

| Metric | Samples | Minimum | Median | Maximum |
|---|---:|---:|---:|---:|
| Pose rcond (dimensionless) | 380 | 0.00167 | 0.05846 | 0.12593 |
| Position-candidate joint margin (deg) | 1928 | 0.00000 | 14.21682 | 70.85408 |
| Position error of found poses (mm) | 380 | 0.00005 | 0.00033 | 0.76553 |
| Orientation error of found poses (deg) | 380 | 0.00000 | 0.00005 | 0.17307 |

Example candidates with the smallest joint margins (all joint values are available in points.csv):

| X (mm) | Y (mm) | Z (mm) | Margin (deg) | Status |
|---:|---:|---:|---:|---|
| -350 | 100 | 50 | 0.0000 | Position found; orientation unresolved |
| -300 | -200 | 50 | 0.0000 | Position found; orientation unresolved |
| -300 | -150 | 200 | 0.0000 | Position found; orientation unresolved |
| -300 | -100 | 200 | 0.0000 | Position found; orientation unresolved |
| -300 | 150 | 100 | 0.0000 | Position found; orientation unresolved |
| -300 | 200 | 50 | 0.0000 | Position found; orientation unresolved |
| -300 | 200 | 150 | 0.0000 | Position found; orientation unresolved |
| -250 | -250 | 200 | 0.0000 | Position found; orientation unresolved |

## All constant-coordinate planes

Each point appears once in each axis group when the grid aligns with the slice increment. Do not add the X+Y+Z group counts and interpret them as unique points. Planes with no samples are marked n/a, not counted as failures.

### Constant X

![All constant-X planes](slices_X.svg)

| X (mm) | Tested | Position found | Pose found | Near singularity | Orientation unresolved | Colliding only | Position unresolved |
|---:|---:|---:|---:|---:|---:|---:|---:|
| [-450](slices/X_minus_450mm.svg) | 45 | 0 (0.0%) | 0 (0.0%) | 0 | 0 | 0 | 45 |
| [-400](slices/X_minus_400mm.svg) | 101 | 0 (0.0%) | 0 (0.0%) | 0 | 0 | 0 | 101 |
| [-350](slices/X_minus_350mm.svg) | 145 | 28 (19.3%) | 0 (0.0%) | 0 | 28 | 0 | 117 |
| [-300](slices/X_minus_300mm.svg) | 185 | 69 (37.3%) | 0 (0.0%) | 0 | 69 | 0 | 116 |
| [-250](slices/X_minus_250mm.svg) | 221 | 103 (46.6%) | 0 (0.0%) | 0 | 103 | 0 | 118 |
| [-200](slices/X_minus_200mm.svg) | 249 | 135 (54.2%) | 0 (0.0%) | 0 | 135 | 0 | 114 |
| [-150](slices/X_minus_150mm.svg) | 277 | 156 (56.3%) | 0 (0.0%) | 0 | 156 | 0 | 121 |
| [-100](slices/X_minus_100mm.svg) | 293 | 170 (58.0%) | 0 (0.0%) | 0 | 170 | 0 | 123 |
| [-50](slices/X_minus_50mm.svg) | 293 | 177 (60.4%) | 0 (0.0%) | 0 | 177 | 1 | 115 |
| [0](slices/X_plus_0mm.svg) | 293 | 171 (58.4%) | 17 (5.8%) | 0 | 154 | 6 | 116 |
| [50](slices/X_plus_50mm.svg) | 293 | 176 (60.1%) | 52 (17.7%) | 0 | 124 | 2 | 115 |
| [100](slices/X_plus_100mm.svg) | 293 | 174 (59.4%) | 51 (17.4%) | 2 | 123 | 0 | 119 |
| [150](slices/X_plus_150mm.svg) | 277 | 168 (60.6%) | 52 (18.8%) | 2 | 116 | 0 | 109 |
| [200](slices/X_plus_200mm.svg) | 249 | 143 (57.4%) | 50 (20.1%) | 2 | 93 | 0 | 106 |
| [250](slices/X_plus_250mm.svg) | 221 | 118 (53.4%) | 57 (25.8%) | 0 | 61 | 0 | 103 |
| [300](slices/X_plus_300mm.svg) | 185 | 89 (48.1%) | 65 (35.1%) | 2 | 24 | 0 | 96 |
| [350](slices/X_plus_350mm.svg) | 145 | 46 (31.7%) | 33 (22.8%) | 2 | 13 | 0 | 99 |
| [400](slices/X_plus_400mm.svg) | 101 | 5 (5.0%) | 3 (3.0%) | 2 | 2 | 0 | 96 |
| [450](slices/X_plus_450mm.svg) | 45 | 0 (0.0%) | 0 (0.0%) | 0 | 0 | 0 | 45 |

### Constant Y

![All constant-Y planes](slices_Y.svg)

| Y (mm) | Tested | Position found | Pose found | Near singularity | Orientation unresolved | Colliding only | Position unresolved |
|---:|---:|---:|---:|---:|---:|---:|---:|
| [-450](slices/Y_minus_450mm.svg) | 45 | 0 (0.0%) | 0 (0.0%) | 0 | 0 | 0 | 45 |
| [-400](slices/Y_minus_400mm.svg) | 101 | 0 (0.0%) | 0 (0.0%) | 0 | 0 | 0 | 101 |
| [-350](slices/Y_minus_350mm.svg) | 145 | 40 (27.6%) | 13 (9.0%) | 0 | 27 | 0 | 105 |
| [-300](slices/Y_minus_300mm.svg) | 185 | 76 (41.1%) | 29 (15.7%) | 0 | 47 | 0 | 109 |
| [-250](slices/Y_minus_250mm.svg) | 221 | 111 (50.2%) | 28 (12.7%) | 2 | 83 | 0 | 110 |
| [-200](slices/Y_minus_200mm.svg) | 249 | 139 (55.8%) | 22 (8.8%) | 0 | 117 | 0 | 110 |
| [-150](slices/Y_minus_150mm.svg) | 277 | 159 (57.4%) | 26 (9.4%) | 1 | 133 | 0 | 118 |
| [-100](slices/Y_minus_100mm.svg) | 293 | 173 (59.0%) | 28 (9.6%) | 2 | 145 | 0 | 120 |
| [-50](slices/Y_minus_50mm.svg) | 293 | 178 (60.8%) | 30 (10.2%) | 1 | 148 | 1 | 114 |
| [0](slices/Y_plus_0mm.svg) | 293 | 177 (60.4%) | 27 (9.2%) | 0 | 150 | 6 | 110 |
| [50](slices/Y_plus_50mm.svg) | 293 | 177 (60.4%) | 31 (10.6%) | 1 | 146 | 2 | 114 |
| [100](slices/Y_plus_100mm.svg) | 293 | 173 (59.0%) | 28 (9.6%) | 2 | 145 | 0 | 120 |
| [150](slices/Y_plus_150mm.svg) | 277 | 159 (57.4%) | 26 (9.4%) | 1 | 133 | 0 | 118 |
| [200](slices/Y_plus_200mm.svg) | 249 | 139 (55.8%) | 22 (8.8%) | 0 | 117 | 0 | 110 |
| [250](slices/Y_plus_250mm.svg) | 221 | 111 (50.2%) | 28 (12.7%) | 2 | 83 | 0 | 110 |
| [300](slices/Y_plus_300mm.svg) | 185 | 76 (41.1%) | 29 (15.7%) | 0 | 47 | 0 | 109 |
| [350](slices/Y_plus_350mm.svg) | 145 | 40 (27.6%) | 13 (9.0%) | 0 | 27 | 0 | 105 |
| [400](slices/Y_plus_400mm.svg) | 101 | 0 (0.0%) | 0 (0.0%) | 0 | 0 | 0 | 101 |
| [450](slices/Y_plus_450mm.svg) | 45 | 0 (0.0%) | 0 (0.0%) | 0 | 0 | 0 | 45 |

### Constant Z

![All constant-Z planes](slices_Z.svg)

| Z (mm) | Tested | Position found | Pose found | Near singularity | Orientation unresolved | Colliding only | Position unresolved |
|---:|---:|---:|---:|---:|---:|---:|---:|
| [-450](slices/Z_minus_450mm.svg) | 45 | 0 (0.0%) | 0 (0.0%) | 0 | 0 | 0 | 45 |
| [-400](slices/Z_minus_400mm.svg) | 101 | 0 (0.0%) | 0 (0.0%) | 0 | 0 | 0 | 101 |
| [-350](slices/Z_minus_350mm.svg) | 145 | 0 (0.0%) | 0 (0.0%) | 0 | 0 | 0 | 145 |
| [-300](slices/Z_minus_300mm.svg) | 185 | 0 (0.0%) | 0 (0.0%) | 0 | 0 | 0 | 185 |
| [-250](slices/Z_minus_250mm.svg) | 221 | 0 (0.0%) | 0 (0.0%) | 0 | 0 | 0 | 221 |
| [-200](slices/Z_minus_200mm.svg) | 249 | 65 (26.1%) | 0 (0.0%) | 0 | 65 | 0 | 184 |
| [-150](slices/Z_minus_150mm.svg) | 277 | 110 (39.7%) | 0 (0.0%) | 0 | 110 | 0 | 167 |
| [-100](slices/Z_minus_100mm.svg) | 293 | 142 (48.5%) | 0 (0.0%) | 0 | 142 | 0 | 151 |
| [-50](slices/Z_minus_50mm.svg) | 293 | 166 (56.7%) | 22 (7.5%) | 0 | 144 | 1 | 126 |
| [0](slices/Z_plus_0mm.svg) | 293 | 177 (60.4%) | 34 (11.6%) | 2 | 143 | 4 | 112 |
| [50](slices/Z_plus_50mm.svg) | 293 | 186 (63.5%) | 46 (15.7%) | 0 | 140 | 2 | 105 |
| [100](slices/Z_plus_100mm.svg) | 293 | 189 (64.5%) | 47 (16.0%) | 2 | 142 | 2 | 102 |
| [150](slices/Z_plus_150mm.svg) | 277 | 188 (67.9%) | 53 (19.1%) | 2 | 135 | 0 | 89 |
| [200](slices/Z_plus_200mm.svg) | 249 | 181 (72.7%) | 68 (27.3%) | 2 | 113 | 0 | 68 |
| [250](slices/Z_plus_250mm.svg) | 221 | 162 (73.3%) | 68 (30.8%) | 2 | 94 | 0 | 59 |
| [300](slices/Z_plus_300mm.svg) | 185 | 141 (76.2%) | 42 (22.7%) | 2 | 99 | 0 | 44 |
| [350](slices/Z_plus_350mm.svg) | 145 | 110 (75.9%) | 0 (0.0%) | 0 | 110 | 0 | 35 |
| [400](slices/Z_plus_400mm.svg) | 101 | 76 (75.2%) | 0 (0.0%) | 0 | 76 | 0 | 25 |
| [450](slices/Z_plus_450mm.svg) | 45 | 35 (77.8%) | 0 (0.0%) | 0 | 35 | 0 | 10 |

## Limitations

- No ROS commands, servo connections, or robot motion. All results describe model kinematics, not measured physical accuracy.
- Each point tests one orientation defined by the experiment settings, not every rotation in SO(3). In radial mode, yaw follows the point's azimuth.
- The bounding sphere is a conservative model sampling domain, not a guarantee that every position inside its radius is reachable. Regions below the base are also sampled if inside the sphere; a table/floor is not modeled.
- Collision checks use approximate model links at the endpoint, not precise mechanical meshes, environmental obstacles, or a verified path from a starting configuration.
- Payload, flexibility, backlash, calibration, motor torque, gravity, velocity, and acceleration are not evaluated.
- Colored markers mark sample centers; they do not fill or classify the area/volume between samples. Unsampled positions are not assessed. Green circles, yellow squares, blue triangles, purple diamonds, and red X markers distinguish the categories.
- A confirmed mechanical dead zone requires additional evidence; numerical IK failure alone is insufficient.

## Experiment parameters (summary.json snapshot)

Check `orientation`, `rpy_deg`, reference frame, position/orientation tolerances, and joint margin before comparing maps. The reporter does not change scanner parameters or data.

```json
{
"engine":"C++17 Eigen/KDL, bounded multi-start LM",
"points":3911,
"radius_mm":491.30865204668481,
"conservative_urdf_bound_mm":491.30865204668481,
"elapsed_seconds":3.6804227200000001,
"position_found":1928,
"pose_found":380,
"arguments":{"spacing_mm":50,"samples":2000,"seeds":4,"iterations":100,"seed":42,"orientation":"radial","base_link":"world","tip_link":"end_effector_link","joint_margin_rad":0.02,"collision_radius_mm":25,"position_tolerance_mm":1,"orientation_tolerance_deg":0.5,"length_scale_mm":300,"singular_threshold":0.01,"rpy_deg":[0,0,0]},
"hard_lower_rad":[-1.5707963267948966,-2.0420352248333655,-1.5707963267948966,-1.2566370614359172,-1.5707963267948966,-1.5707963267948966],
"hard_upper_rad":[1.5707963267948966,2.1048670779051615,2.1362830044410597,1.2566370614359172,2.1048670779051615,1.5707963267948966],
"effective_lower_rad":[-1.5507963267948965,-2.0220352248333655,-1.5507963267948965,-1.2366370614359172,-1.5507963267948965,-1.5507963267948965],
"effective_upper_rad":[1.5507963267948965,2.0848670779051615,2.1162830044410597,1.2366370614359172,2.0848670779051615,1.5507963267948965],
"counts":{"collision_candidates_only":9,"orientation_unresolved":1548,"pose_found":368,"pose_near_singular":12,"position_unresolved":1974},
"limitations":["Finite grid and seeds; unresolved is not proven unreachable","One orientation per point; not exhaustive SO(3)","Endpoint capsule collision only, no trajectory/obstacles/dynamics","Full sphere includes negative Z; no table","Singularity refers to best tested pose witness, not all configurations"]}

```
