# OM6DOF Cartesian workspace experiment (offline)

Purpose: visualize gaps and difficult regions in the workspace described by the
URDF and joint limits, without starting ROS nodes, publishing commands, opening
the serial port, or moving the robot. The experiment is separate from the
teleop/controller runtime.

## New: v1 / v2 workspace comparison

The current source models have been rescanned on a **matched 25 mm grid inside
a 500 mm sphere: 33,401 points per model**, with 41 planes per axis. Read the
[v1 / v2 analysis](ANALYSIS_V1_V2_25MM_20260909.md). Both scans use the same
orientation, random seed, search budget, tolerances and collision approximation.
The original public demo below is unchanged and uses a smaller bounding sphere.

[Open the published comparison](https://tzf230201.github.io/om6dof/comparison/comparison.html).
V2 is the selected direction for the next mechanical revision because this
matched test finds 3,532 complete poses versus 2,935 for V1 (+20.34% relative).
Position coverage and conditioning have trade-offs, detailed in the analysis;
this does not automatically change the runtime robot-description defaults.

The dedicated `comparison.html` includes synchronized v1/v2/overlap 3D panels,
constant-X/Y/Z slices, position-versus-pose comparison, paired point inspection,
category transitions, and links to each model's full 3D viewer and 2D report.
Data is embedded: it works offline without a server or any robot connection.
The GUI and reports are in English. Numerical scanning and comparison statistics
are generated in C++; JavaScript only displays the saved data.

To reproduce from `~/ros2_ws/src` (choose unused output directories):

```bash
bash om6dof/experiments/cartesian_workspace/run_cpp.sh /tmp/om6dof-compare-v1 \
  --model v1 --spacing-mm 25 --radius-mm 500
bash om6dof/experiments/cartesian_workspace/run_cpp.sh /tmp/om6dof-compare-v2 \
  --model v2 --spacing-mm 25 --radius-mm 500
/tmp/om6dof-workspace-cpp-build/workspace_report_cpp \
  --input /tmp/om6dof-compare-v1 --compare /tmp/om6dof-compare-v2 \
  --output /tmp/om6dof-comparison
xdg-open /tmp/om6dof-comparison/comparison.html
```

`--model v1` and `--model v2` render the **current source checkout**, including
the matching payload YAML, without needing a package rebuild. Omitting `--model`
retains the old installed-overlay behaviour. Each scan saves `model.urdf`,
`summary.json`, `seed_bank.csv`, `command.sh`, and `SHA256SUMS` for provenance.
The comparison refuses mismatched settings/grids/orientations, duplicate XYZ,
inconsistent summary counts, and existing output directories. Its output copies
the individual presentation/data files into `v1/` and `v2/`; source scans remain
untouched. `comparison.json` records aggregate overlap and the transition matrix;
`comparison_slices.csv` records per-plane counts and percentage-point deltas.

For this checkout's completed run, open:

```bash
xdg-open ~/ros2_ws/src/om6dof/experiments/cartesian_workspace/results/comparison_v1_v2_25mm_20260909/report/comparison.html
```

Generated `results/` are local and gitignored on `main`; the generator and analysis
are versioned there. A validated snapshot of the self-contained `report/` directory
is published under `comparison/` on the separate `gh-pages` branch, without
replacing the old demo. Generating another local report does not automatically
update that published snapshot.

**Scope:** this is a kinematic workspace comparison, with chain-capsule collision
checks. It does **not** evaluate the new link3 mesh or D435/bracket collision
geometry, masses, CoM, payload dynamics, physical accuracy, or safe trajectories.
Unresolved samples are not proof of unreachable mechanical dead zones.

Comparison regression checks:

```bash
PYTHONDONTWRITEBYTECODE=1 python3 -m pytest -q -p no:cacheprovider \
  om6dof/experiments/cartesian_workspace/test_run_cpp.py \
  om6dof/experiments/cartesian_workspace/cpp/test_comparison.py
node om6dof/experiments/cartesian_workspace/cpp/test_comparison_view.js \
  /tmp/om6dof-comparison/comparison.html
```

## Public browser demo

Open the [interactive 3D workspace viewer](https://tzf230201.github.io/om6dof/viewer3d.html)
or the [2D slice report](https://tzf230201.github.io/om6dof/).
Visitors only need a browser: no download, ROS installation, or local server is
required. The demo is a saved 25 mm scan, not a live IK solver or robot-control
interface. Rotating the view and changing filters do not run new experiments.

GitHub Pages serves the static files from the separate `gh-pages` branch at its
root, with `.nojekyll`. This publication does not change the `main` branch or
the robot runtime. The published bundle includes the two HTML pages, analysis,
CSV/JSON data, and all SVG slice maps. The model snapshot and numerical seed
bank are not part of the website. Regenerating a local scan does not
automatically update this public snapshot.

### Higher-resolution snapshot (25 mm)

The denser demo contains **31,799 points**, up from 3,911 at 50 mm (8.13 times
as many). It uses the same model snapshot, 491.3 mm outer bound, seed bank,
orientation mode, joint margins, and tolerances; only the grid spacing changes.
The report includes **39 planes per axis**, from -475 to +475 mm in 25 mm
increments (117 planes total). A finer grid adds samples, not interpolation or
a guarantee that unresolved points are unreachable.

To generate a new 25 mm scan from the currently installed model:

```bash
bash om6dof/experiments/cartesian_workspace/run_cpp.sh /tmp/om6dof-uji-25mm-new \
  --spacing-mm 25
```

The wrapper matches the 2D slice interval to `--spacing-mm`. For an existing
25 mm result, use `workspace_report_cpp --input RESULT_DIR --slice-step-mm 25`.
The older 50 mm commands and comparative analysis below remain useful as the
baseline; omit `--spacing-mm` to retain that coarser scan.

## C++ version — 50 mm grid and all constant-coordinate slices

Results and comparisons of four configurations tested on September 9, 2026 are
available in the [50 mm grid analysis](ANALYSIS_50MM_20260909.md).

Both the numerical scan and report generation use **C++17** (Eigen/KDL and STL).
Python is not used for the FK/IK loops or report generation; ROS `xacro` is still
used to prepare the URDF. The robot and teleop do not need to be running.

```bash
cd ~/ros2_ws/src
bash om6dof/experiments/cartesian_workspace/run_cpp.sh /tmp/om6dof-uji-50mm
xdg-open /tmp/om6dof-uji-50mm/index.html
```

The script prepares the ROS environment, builds in Release mode under
`/tmp/om6dof-workspace-cpp-build`, renders the model, scans the workspace, and
generates the report. Existing output directories are not overwritten: use a
new output name for another scan. Build dependencies are CMake, a C++17 compiler,
Eigen3, Boost headers, KDL, `kdl_parser`, and `urdf` from ROS Humble. The first build takes longer
than subsequent scans.

For the tested model, the scan evaluates all 3,911 grid centers spaced 50 mm
apart within the conservative bounding sphere of radius 491.3 mm. The report
shows **19 planes per axis**: constant X, constant Y, and constant Z from
-450, -400, ..., 0, ..., +450 mm. On each plane, the other two coordinates vary
in 50 mm increments. This is not limited to X/Y/Z=0. Each unique point is
calculated once and reused in the three slice groups; 57 slices do not mean
57 robot scans.

C++ outputs:

- `index.html`: report entry point, with links to the slices and 3D viewer.
- `viewer3d.html`: interactive, self-contained 3D point-cloud viewer.
- `slices_X.svg`, `slices_Y.svg`, `slices_Z.svg`: sheets showing all planes per axis.
- `slices/*.svg`: larger individual planes; point tooltips show errors/margins.
- `analysis.md`: category, conditioning, joint-margin, and per-plane analysis.
- `points.csv`, `slices.csv`, `summary.json`: raw data, slice statistics, and parameters.
- `model.urdf`, `seed_bank.csv`: model snapshot and numerical seeds.

### Interactive 3D viewer

Open **3D viewer** from `index.html`, or open `viewer3d.html` directly. Rotate,
pan, and zoom the point cloud, filter categories, select constant-X/Y/Z slices,
and select points to inspect their recorded results. Slice controls filter
existing samples; they do not run another IK scan. The viewer uses plain
JavaScript with a Canvas-based 3D projection, not WebGL, and has no CDN or
external network dependency. The point data is embedded in the HTML, so a web
server is optional. All workspace calculations remain in C++.

Both the 2D maps and 3D viewer encode categories with color **and shape**:
green circles = pose found; blue triangles = near-singular pose;
yellow squares = position found but orientation unresolved;
red X marks = position unresolved; purple diamonds = only colliding
candidates found. Legends use the same shapes as the plotted samples.

If a Flatpak browser cannot open local files because of a document-portal
error, serve the existing output on localhost:

```bash
/usr/bin/python3 -m http.server 8765 \
  --bind 127.0.0.1 \
  --directory /tmp/om6dof-uji-50mm
```

Leave that terminal running, then open or refresh
`http://127.0.0.1:8765/viewer3d.html` in the browser on the same computer. Python
only serves the result files here; it does not perform the scan. Stop the server
with Ctrl+C when finished. If the server is already running for this directory,
just refresh the page after the report has been regenerated.

To update an existing result directory with the latest English report and 3D
viewer, rebuild the reporter and regenerate the report without rescanning:

```bash
cd ~/ros2_ws/src
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
cmake -S om6dof/experiments/cartesian_workspace/cpp \
  -B /tmp/om6dof-workspace-cpp-build -DCMAKE_BUILD_TYPE=Release
cmake --build /tmp/om6dof-workspace-cpp-build -j2
/tmp/om6dof-workspace-cpp-build/workspace_report_cpp \
  --input /tmp/om6dof-uji-50mm --slice-step-mm 50
```

This replaces generated presentation files using the existing scan data. It
does not rerun the numerical experiment or modify robot services.

Validate the generated interactive viewer without a browser or network:

```bash
node om6dof/experiments/cartesian_workspace/cpp/test_viewer3d.js \
  /tmp/om6dof-uji-50mm/viewer3d.html
```

These tests exercise projection, category/slice controls, point selection,
recorded values, camera views, rotation, pan, zoom and reset. The viewer has
also been rendered successfully in isolated headless Chromium on the AGX.

### Scan options and validation

Example with a fixed orientation (the same RPY at every point):

```bash
bash om6dof/experiments/cartesian_workspace/run_cpp.sh /tmp/om6dof-uji-fixed \
  --orientation fixed --rpy-deg 0 0 0
```

A larger search budget for checking unresolved points:

```bash
bash om6dof/experiments/cartesian_workspace/run_cpp.sh /tmp/om6dof-uji-deep \
  --samples 10000 --seeds 12 --iterations 200
```

Other arguments match the Python version below (`--spacing-mm`, radius, margin,
and tolerances). The C++ `--urdf` option accepts rendered URDF XML, not xacro.
By default the wrapper uses the description from the active ROS overlay; select
`--model v1` / `--model v2` for the current source, or use the executable directly
to test another rendered snapshot. The wrapper report matches the requested
grid spacing (50 mm by default). When calling the reporter directly with a
different slice interval, planes without samples are marked n/a, not failed.

Validation: C++ FK/KDL results are compared against Python; each fixture
candidate is independently checked for FK, joint limits, collision, and
orientation error. C++ uses monotonic backtracking for position and pose IK and
an `mt19937` seed generator. Results therefore need not exactly match the older
Python scanner, which uses PCG64 and a different position-IK step rule. Do not
compare percentages between runs without checking their grids, seeds, and
search budgets.

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
/tmp/om6dof-workspace-cpp-build/workspace_scan_cpp \
  --urdf /tmp/om6dof-workspace-cpp-model.urdf --self-test
PYTHONDONTWRITEBYTECODE=1 ROS_LOG_DIR=/tmp/om6dof-workspace-tests \
  /usr/bin/python3 -m pytest -q -p no:cacheprovider \
  om6dof/experiments/cartesian_workspace/test_cpp_scan.py \
  om6dof/experiments/cartesian_workspace/cpp/test_report.py
```

For the self-test command above, prepare the XML with `xacro ... -o
/tmp/om6dof-workspace-cpp-model.urdf` or use a scan's `model.urdf` path. Native
tests use that default path; override it with `OM6DOF_CPP_SCAN_URDF` and
`OM6DOF_CPP_SCAN_BINARY`. Python is only a reference in these tests, not the C++
scanning engine.

## Earlier Python version

From `~/ros2_ws/src`:

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
export MPLCONFIGDIR=/tmp/om6dof-workspace-mpl

# Initial coarse scan. Every grid center inside the sphere is tested.
/usr/bin/python3 om6dof/experiments/cartesian_workspace/workspace_scan.py \
  --spacing-mm 120 --samples 1000 --seeds 2 --iterations 60 \
  --output /tmp/om6dof-workspace-preview

# Finer resolution takes longer. Use a new output directory for each run.
/usr/bin/python3 om6dof/experiments/cartesian_workspace/workspace_scan.py \
  --spacing-mm 40 --samples 5000 --seeds 6 --iterations 160 \
  --output /tmp/om6dof-workspace-40mm
```

Requires system Python with NumPy, SciPy, Matplotlib, PyKDL, and the ROS Humble
environment for xacro rendering. No running robot, package build, or service
restart is required. By default, it uses the description package from the
sourced overlay: inspect the resulting `model.urdf` to confirm the intended
design was tested.

The default radius is the sum of translation lengths along the URDF chain: a
conservative upper bound, **not a guaranteed reachable radius**. The sphere is
centered on frame `world`, with tip `end_effector_link`, matching the controller
defaults. Override these with `--radius-mm`, `--base-link`, `--tip-link`, and
`--urdf-pkg`. Samples include negative Z; no table or floor is modeled. Halving
the spacing increases the point count by approximately eight times. This is
discrete sampling, not a check of every point in continuous space.

## Orientation and classification

Each point is tested first with unconstrained orientation, then with one
specified gripper orientation. The default `--orientation radial` means the
gripper is horizontal and points outward from the Z axis:
roll/pitch=0, yaw=atan2(y,x). Yaw defaults to 0 on the Z axis. Angles use the
controller's target gripper frame (X along the gripper), not raw URDF tip RPY.

For the same orientation throughout the workspace:

```bash
/usr/bin/python3 om6dof/experiments/cartesian_workspace/workspace_scan.py \
  --orientation fixed --rpy-deg 0 0 0 --spacing-mm 60 \
  --output /tmp/om6dof-workspace-fixed
```

The table below describes the **earlier Python-only report**, which retains its
original colors. The current C++ report and public viewer instead use **blue
triangles** for near-singular poses and **yellow squares** for positions whose
orientation is unresolved. Compare the category labels, not colors, when using
older reports.

| Legacy Python color | Meaning |
|---|---|
| Green | Position + orientation solution found, not near-singular under the test threshold |
| Yellow | Pose solution found, but the best tested configuration is near-singular |
| Blue | Position reachable; requested orientation solution not found |
| Purple | Only position candidates colliding under the capsule model were found |
| Red | Position solution not found within the search budget |

IK uses multiple seeds from a deterministic random FK bank, READY, and zero;
solutions are verified against FK and effective joint limits. The default limits
are reduced by a 0.02 rad margin, as in the controller. To study the URDF's
mechanical range without that margin, use `--joint-margin-rad 0` and compare
results. These are theoretical design limits, not permission to drive the
physical robot to its hard stops.

Default tolerances: 1 mm position and 0.5 degrees orientation. Collision checking
uses a 25 mm-radius capsule skeleton, as in the controller, not full collision
meshes. Paths from the robot's current pose and external obstacles are not
checked. **Finding an endpoint solution does not mean moving there is safe.**

Near-singularity is calculated from `sigma_min/sigma_max` of the 6D Jacobian
after dividing its linear rows by a 300 mm length scale. The default threshold
of 0.01 is configurable with `--singular-threshold`; it is a numerical indicator,
not a measure of physical positioning error. Singularity depends on joint
configuration; the same Cartesian point may have a better configuration. The
CSV retains the pose candidate with the best ratio found by the search.

## Python experiment outputs and interpretation

- `workspace_3d.png`: unconstrained-position map and tested-orientation map.
- `workspace_slices.png`: XY, XZ, and YZ slices through the sphere center.
- `points.csv`: every grid center, category, errors, conditioning, distance to
  joint limits, target angles, and witness joint values for solutions found.
- `summary.json`: parameters, category counts, limits, URDF hash, duration, and
  experiment limitations.
- `model.urdf` and `seed_bank.npz`: model and seeds for reproducing results.

Red/purple points **do not prove mechanical dead zones**: numerical IK search
can fail. Recheck suspicious regions with more seeds/iterations and a finer
grid, and compare radial vs fixed orientation and margins of 0 vs 0.02 rad.
Do not conclude that a design is defective from a single scan. Motor strength,
payload, backlash, calibration, and structural deflection are not tested here.

An interrupted run leaves a partial CSV. `summary.json` and both PNGs are only
created after every point has been processed. Existing result directories are
not overwritten.

Experiment function tests:

```bash
PYTHONDONTWRITEBYTECODE=1 /usr/bin/python3 -m pytest -q -p no:cacheprovider \
  om6dof/experiments/cartesian_workspace/test_workspace_scan.py
```
