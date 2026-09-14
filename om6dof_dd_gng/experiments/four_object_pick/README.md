# Four-object semantic-topology experiment

This directory prepares a reproducible four-object experiment for the
OM6DOF DD-GNG stack.  It deliberately separates the experiment that the
current code can run safely (perception + reachability-plan preview) from
physical pick execution, which is not yet connected to the reachability
planner.

## Selected objects

All four names are native COCO/YOLOX classes.  Use one instance with a matte,
high-contrast surface and keep it within the nominal gripper opening verified
with a physical gauge before any motion test.

| ID | COCO class | Recommended specimen | Nominal grasp |
|---|---|---|---|
| O1 | `bottle` | opaque 250--500 mL plastic bottle, 45--55 mm body | side pinch at mid-body |
| O2 | `cup` | handleless matte paper cup, 50--60 mm body | side pinch below rim |
| O3 | `banana` | firm single banana, 30--40 mm mid-body | top/oblique pinch at middle |
| O4 | `remote` | high-contrast remote, 18--25 mm thick | top pinch; place on a 25 mm matte pedestal |

Avoid transparent bottles/cups, reflective metal, black-on-black objects,
full containers, loose cables, and objects wider than the verified gripper
opening.  The remote pedestal is part of the fixture and is not a grasp target.

## Experimental stages and readiness gates

### A. Single-object recognition calibration (ready)

1. Place one object at each marked table position for 30 s.
2. Verify the expected COCO class, at least three supporting DD-GNG nodes,
   stable world-frame node positions, and no robot-body labels.
3. Repeat for all objects and positions.  Freeze confidence thresholds after
   this development-only calibration.

### B. Four-object mapping and planning preview (ready after preflight)

1. Stop the prior preview before changing a layout. Put all four objects in
   P1--P4, at least 120 mm apart, according to the current manifest row.
2. Start a fresh frozen perception/planning process for every scene reset:

   First stop the user web-stream service, because it otherwise respawns the
   legacy `dd_gng_yolo.py` every three seconds and competes for the RealSense:

   ```bash
   systemctl --user stop om6dof-dd-gng.service
   ```

   ```bash
   ros2 launch om6dof_dd_gng topo_gng_node.launch.py \
     model_version:=v2 publish_robot_state:=true launch_reachability:=true \
     params_file:=$HOME/ros2_ws/src/om6dof/om6dof_dd_gng/experiments/four_object_pick/config/four_object_topo_gng_v2.yaml
   ```

   This keeps YOLO at 2 Hz and restricts detections to the four frozen COCO
   classes.
   The equivalent guarded command is `./scripts/launch_preview.sh`; it refuses
   to start if the web-stream service or another known process may already own
   the RealSense. Restore the dashboard stream only after the experiment with
   `systemctl --user start om6dof-dd-gng.service`.
3. Use the same fixed warm-up interval for every reset (recommended pilot
   value: 10 s), then run `preflight.sh` and start the recorder. The default
   preflight is fail-closed: all four classes must each support at least three
   unique DD-GNG nodes simultaneously for 1 s, with no labels-topic gap above
   0.5 s. Do not move an object during warm-up or recording.
4. Record semantic/environment topology, reachability graph, path preview,
   joint states, TF, resource telemetry, and all visual streams.
5. Stop both recorder and preview before rotating object positions for the
   next `run_id` in `trial_manifest.csv`.

The fresh-process boundary is mandatory: DD-GNG and semantic labels retain
online history. Rearranging objects under a running topology node would mix
the previous and current scene and invalidate a between-reset comparison.

The current environment message labels individual nodes, not object
instances.  Therefore this stage measures mapping and plan-preview behavior;
it must not be described as autonomous four-object selection.

### C. Physical pick (blocked by an implementation gate)

Do not command motion from this experiment directory.  The current
reachability node is preview-only and has no controller/action client.  A
physical run requires, at minimum:

- instance-level object aggregation and explicit target selection;
- calibrated grasp, pre-grasp, retreat, and drop poses including orientation;
- a reviewed executor for the arm and gripper;
- collision recheck immediately before execution, timeout/abort handling,
  joint-limit checks, and a tested emergency stop;
- one-object low-speed dry runs before a four-object scene.

## Trial design

`generate_trial_manifest.py` produces 12 scene resets and four target rows per
reset (48 target opportunities).  A balanced four-sequence order is repeated
three times, while object-to-position assignment rotates across P1--P4.  Keep
lighting, background, camera settings, table pose, object instances, roadmap
parameters, and robot start state fixed.  Do not tune after inspecting the
recorded outcomes.

Primary integration endpoint:

`lift_success`: the selected object is raised 50 mm and held for 3 s without
contacting another object.  Until Stage C is enabled, report only the earlier
stage endpoints and mark lift/execution fields `not_run`.

Stage endpoints, in order:

1. `detected`: correct YOLO class appears above the frozen threshold;
2. `semantic_stable`: the same class has sufficient DD-GNG support for 1 s;
3. `instance_selected`: requested class/instance is unambiguous;
4. `intersection_found`: a reachable configuration intersects the target;
5. `plan_exact_valid`: lazy MoveIt/FCL validation accepts the preview;
6. `approach_reached`, `grasp_closed`, `lift_success`, `place_success`.

Report both end-to-end success and the conditional transition rate at each
stage.  A failed earlier stage is not silently removed from later summaries.

## Recording

Run the robot/perception launch separately, then:

```bash
cd ~/ros2_ws/src/om6dof/om6dof_dd_gng/experiments/four_object_pick
./scripts/preflight.sh
./scripts/record_trial.sh run_001
```

For a recorder/instrumentation smoke without physical objects, opt in
explicitly so the semantic check is recorded as skipped rather than passed:

```bash
SCENE_READINESS_MODE=instrumentation RECORD_DURATION_SEC=5 \
  ./scripts/record_trial.sh run_001
```

Do not use that mode for an experimental run. A normal run writes the
machine-readable semantic decision to `scene_readiness.json` before recording.

For a separate UVC outside camera, identify its device with
`v4l2-ctl --list-devices`, then start the recorder from the Jetson graphical
session (so `$DISPLAY` is valid):

```bash
OUTSIDE_CAMERA_DEVICE=/dev/video6 CAPTURE_DESKTOP=1 \
  ./scripts/record_trial.sh run_001
```

Do not select a `/dev/video*` endpoint belonging to the RealSense used by the
topology node. If no separate UVC camera is available, record the outside view
on a phone/camera and create a visible synchronization event (LED flash or
hand clap) in both views at the start and end.

Press Ctrl-C once to stop recording cleanly.  The recorder never starts a
controller or publishes a trajectory.  Use `mark_trial_event.py` from another
terminal to record human-observed events:

For a fixed-duration calibration capture, set an integer number of seconds,
for example `RECORD_DURATION_SEC=30 ./scripts/record_trial.sh run_001`.

```bash
./scripts/mark_trial_event.py data/run_001 scene_ready --note "all four visible; sync flash"
./scripts/mark_trial_event.py data/run_001 target_selected \
  --opportunity-id run_001_t01
./scripts/mark_trial_event.py data/run_001 plan_exact_valid \
  --opportunity-id run_001_t01 --result success
```

The event tool accepts only the frozen stage vocabulary, looks up object and
position from the manifest, and appends under a file lock. This prevents a
typing error from silently creating a fifth object or breaking the join from
an event to its target opportunity.

Each run contains:

- ROS 2 bag data for TF, joint state, semantic topology, reachability, and plan;
- `reachability_*.csv`, `environment_graph.csv`, label/status JSONL;
- per-process CPU, RSS, PSS, I/O, and thread counts;
- system RAM, swap, CPU, and load;
- Jetson `tegrastats` output when available;
- optional outside-camera and desktop/RViz video;
- Git/config/hardware provenance and timestamped operator events.

Metric definitions and claim boundaries are fixed in `DATA_DICTIONARY.md`.

Video capture uses `ffmpeg` when present and automatically falls back to the
GStreamer stack installed on the AGX.

This 48-opportunity manifest is an integration/perception pilot, not the
paired GNG-versus-PRM speed benchmark. A speed claim must replay frozen scene
snapshots as atomic queries with the same start joints, target node, roadmap
stream, node budget and timeout for both methods; live publications are not
independent trials.

## Visual documentation

Use the six-panel composition in `figures/visual_capture_layout.png`:

1. outside camera showing the complete robot, table, and safety operator;
2. wrist-camera POV;
3. YOLO boxes/classes/confidence;
4. DD-GNG semantic environment topology;
5. reachability graph, selected goal, and planned path;
6. synchronized module latency/resource traces and stage result.

Print or display the matching page from `figures/scene_cards.pdf` before each
reset. The 12 validated cards show the P1--P4 assignment, target order,
opportunity IDs, robot/camera side, spacing constraint, and synchronization
slate. The plotted +/-80 mm offsets are suggested marks only and must be
calibrated to the real table frame before data collection.

`visual_shot_list.csv` is the capture checklist. Record an overhead still and
sync marker for every reset; these make scene reconstruction and independent
video alignment possible even when the outside camera has its own clock.

The native C++ topology node publishes the timestamped, rate-limited
`/om6dof_topo_gng_v2/debug_image/compressed` stream at 2 Hz. It combines the
wrist-camera POV, YOLO box/class/confidence overlay, projected DD-GNG nodes,
and a compact frame/node/inference HUD without opening the RealSense twice.
The same stream is included in every ROS bag. Capture a standalone JPEG for
the shot log with:

```bash
./scripts/capture_debug_image.py data/debug_preview/run_001_sync.jpg \
  --timeout-sec 10
```

This is an annotated POV, not a separate raw-RGB stream. For the six-panel
figure, use the full frame for the robot POV and a detection crop for the YOLO
panel, retaining the same timestamp. Do not open the same RealSense from the
legacy Python preview while the native node owns it.

After a run, generate the synchronized instrumentation figure with:

```bash
python3 figures/plot_run_dashboard.py data/run_001
```

The dashboard uses measured data only and marks absent streams instead of
imputing values. Smoke dashboards must retain their non-paper watermark.
The current clean recorder qualification is
`figures/instrumentation_smoke_dashboard_smoke_instrumentation_clean_20260910_run_001.png`;
it is documentation of instrumentation health, not an experiment result.

## Analysis rules

- Preserve every attempted target, including failures and timeouts.
- Use monotonic timestamps for elapsed time and UTC only for synchronization.
- Summarize latency with median, IQR, p90, and p95.
- For GNG versus PRM, pair the same scene, target, start state, and trial order.
- Report time-to-outcome on all paired attempts; success-conditional planning
  time is secondary.
- Resource summaries use per-module peak and time-weighted mean RSS/PSS/CPU.
- Store raw data read-only after the run and analyze into a different folder.
