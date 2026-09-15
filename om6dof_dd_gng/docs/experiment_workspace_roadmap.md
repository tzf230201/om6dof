# Experiment workspace roadmap (separate V2 launch)

Run from an AGX desktop terminal, after stopping any existing planning/camera launch:

```bash
source /opt/ros/humble/setup.bash
source /home/kublab/ros2_ws/install/setup.bash
ros2 launch om6dof_dd_gng ddgng_experiment_reachability.launch.py \
  sample_selection:=green \
  camera_calibration_file:=/home/kublab/.config/om6dof/d435_hand_eye.yaml \
  execution_enabled:=false
```

The default `sample_selection:=green` uses only green experiment poses (`status=pose_found`),
excluding blue near-singular and yellow orientation-unresolved points. Green is a
category from the original scan; all current-model collision checks still apply.

This opens YOLO + DD-GNG environment processing, the experiment-derived robot
roadmap, RViz, the target GUI, and the existing C++ YOLO 2D window from
`yolox_ddgng_analysis.launch.py`. The viewer subscribes to
`/om6dof_topo_gng_v2/rgb/image/compressed` and runs its own RGB-only YOLO detector;
it never opens another RealSense device. Use `launch_yolo_2d:=false` to hide it.
For an already running session, open only this existing viewer in an AGX desktop terminal:

```bash
RMW_IMPLEMENTATION=rmw_fastrtps_cpp ros2 launch om6dof_dd_gng yolox_viewer.launch.py display_window:=true
```

The launch does not start hardware controllers, enable
torque, publish synthetic joints, or automatically move the robot. Actual
`/joint_states` and the existing controller stack are needed. The camera has one
owner. This launch replaces the previous perception/planning launch for the session.

Choose the object, Set planning target, then Preview EoE path in RViz. The default
coordinator task is move_to_target: a measured-start connector followed by robot
graph edges, ending near a target environment node. It does not operate the gripper.
This separate launch sets `target_node_selection=component_center_neighborhood`.
For each same-class edge-connected component, it retains the 12 observed surface
nodes nearest the component's 3D bounding-box center. This avoids favoring the
bottle's top just because a robot node lies closest to it, while giving exact
collision validation several middle-height approach alternatives. The candidates
remain actual surface nodes, never an invented point inside the object; separate
components are not averaged together. If none has a validated intersection,
planning fails rather than silently returning to the top. Override the number with
`component_center_candidates:=N`. The existing intersection tolerance still applies
to the robot endpoint. All object geometry remains in collision checks.

The separate launch uses `exact_replan_budget:=80` by default. This is a search
budget only: every rejected edge remains rejected, and collision radii, mesh checks
and target protection are unchanged. It can be reduced for faster diagnosis.

At the verified folded ready pose, `link2` and `link6` make mechanical contact.
FCL reports coplanar/zero-distance mesh contact as a collision, so this launch
allows only the explicit pair `link2:link6`. It does not restore the SRDF's broad
`Never` exemptions: every other non-adjacent pair remains collision-checked.
This changes the approach destination; it does not validate grasp orientation,
finger clearance or automatically close the gripper. Explicit atomic query targets
and the other launches retain their existing selection behavior.
Execution remains opt-in with `execution_enabled:=true` plus the explicit GUI button
and existing validation gates. Launching with true alone does not send motion.

## Data and model

Default source:
`~/ros2_ws/src/om6dof/experiments/cartesian_workspace/results/comparison_v1_v2_25mm_20260909/v2/points.csv`.
Override with `workspace_samples_file:=/absolute/path/points.csv`.

The launch filters the CSV into its temporary session directory and records the
filtered SHA256 in the generated reachability configuration. A JSON manifest records
the source path/hash, selection, count, and snapshot hash. Original experiment files are
never modified. Environment parameters are copied unchanged from `topo_gng_v2.yaml`;
only the reachability block is overridden. Existing launch defaults are unchanged.
Session files are removed on launch shutdown.

- By default, read only rows with status=pose_found, position_found=1 and pose_found=1.
  The V2 dataset contains 3,198 such green witnesses. The numeric pose_found flag
  alone is insufficient because blue near-singular rows also set it to 1.
- Optional `sample_selection:=all` restores all position_found=1 witnesses,
  including blue and orientation-unresolved rows. This is not the default.
- Each row must supply finite XYZ in mm and q1_rad through q6_rad in robot order.
- Recompute each node's full EoE pose using current URDF FK; CSV XYZ are checks,
  not the runtime node coordinates. Reject the dataset if FK differs by >5 mm
  from a requested grid point (the original scanner tolerance was 1 mm).
- Recheck current joint limits and strict self-collision. Remove duplicate joint
  configurations; coincident XYZ with different joint configurations remain distinct.
- Use every accepted recorded configuration. No GNG quantization or new local
  samples are added in this backend; current EoE connects via a validated start edge.

## Edges and collision

Cartesian spatial buckets accelerate neighbor discovery. Sparse edges are ranked
by normalized joint-space distance, within the existing Cartesian/joint connection
limits. These edges are **candidates**, not an assertion of collision-free travel.
RViz uses the `workspace_candidate_edges` namespace for them.

During planning, collision-invalid intersection goals are excluded. Dijkstra finds
the nearest reachable target intersection, then the start connector and selected
edges undergo body-capsule and full robot-mesh FCL checks against environment
nodes (spheres) and edges (cylinders), including target geometry. Failed edges are
blocked and the search retries within the existing replan budget. Only a completely
validated path is published as valid. The imported backend avoids preallocating
large swept-body caches for all candidate edges.

The dataset stores selected IK witnesses, not every IK branch. Sparse environmental
observations, collision-model accuracy and finite edge-check spacing remain relevant;
this is not a physical pickup certification or a guarantee of connectivity everywhere.

## Green-only validation on the saved bottle scene

With the earlier single-node `component_center` policy, the 170-node observed
bottle component selected existing node 50943 at Z=0.176913 m (component Z extent
0.133037–0.211438 m). An isolated diagnostic replay with `exact_max_replans=200` returned
`target_intersection_exact_blocked_or_disconnected` after 159 rejected candidates
and 9,676 exact state checks (approximately 776 ms). This is not a validated
trajectory to the middle. The higher budget was a replay-only override; launch
search limits were not changed. No fallback to the top is performed. The following
older results used all semantic target nodes before center selection was added.

All 3,198 green witnesses passed current-model node validation. The roadmap had
18,554 candidate edges in one connected component; every node's joint configuration
was verified against a green source row. Maximum FK discrepancy was 1 mm.

The saved bottle scene returned `exact_collision_replan_exhausted` after 21 replans
and 1,256 exact checks (final planning cycle approximately 224 ms). No validated
trajectory was produced. A connected candidate graph does not guarantee a path
after full-body collision checks, and this exhausted search does not prove that
no path exists. Collision settings and search budgets were not loosened.
Replay ran on isolated domain 71 without physical robot or camera operation.

## Earlier all-category validation on the saved bottle scene

15,175 position-found input rows; 15,064 accepted current-model nodes; 111 rejected
by current limits/self-collision/duplicate filtering. Maximum FK discrepancy was
1 mm. The graph had 93,355 candidate edges and 13 connected components.

A snapshot replay found an 8-edge path ending 35.92 mm from the target, with 124
exact state checks in an approximately 83 ms planning cycle. Coordinator replay
accepted 10 trajectory points (measured start plus 9 recorded graph nodes).
An injected obstacle at the initial EoE correctly rejected the path. All replay
traffic used isolated ROS domain 71; no arm/gripper action was called.
