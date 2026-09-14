# Experiment workspace roadmap (separate V2 launch)

Run from an AGX desktop terminal, after stopping any existing planning/camera launch:

```bash
source /opt/ros/humble/setup.bash
source /home/kublab/ros2_ws/install/setup.bash
ros2 launch om6dof_dd_gng ddgng_experiment_reachability.launch.py \
  camera_calibration_file:=/home/kublab/.config/om6dof/d435_hand_eye.yaml \
  execution_enabled:=false
```

This opens YOLO + DD-GNG environment processing, the experiment-derived robot
roadmap, RViz, and the target GUI. It does not start hardware controllers, enable
torque, publish synthetic joints, or automatically move the robot. Actual
`/joint_states` and the existing controller stack are needed. The camera has one
owner. This launch replaces the previous perception/planning launch for the session.

Choose the object, Set planning target, then Preview EoE path in RViz. The default
coordinator task is move_to_target: a measured-start connector followed by robot
graph edges, ending near a target environment node. It does not operate the gripper.
Execution remains opt-in with `execution_enabled:=true` plus the explicit GUI button
and existing validation gates. Launching with true alone does not send motion.

## Data and model

Default source:
`~/ros2_ws/src/om6dof/experiments/cartesian_workspace/results/comparison_v1_v2_25mm_20260909/v2/points.csv`.
Override with `workspace_samples_file:=/absolute/path/points.csv`.

The launch copies the CSV into its temporary session directory and records its
SHA256 in the generated reachability configuration. Original experiment files are
never modified. Environment parameters are copied unchanged from `topo_gng_v2.yaml`;
only the reachability block is overridden. Existing launch defaults are unchanged.
Session files are removed on launch shutdown.

- Read rows with position_found=1, including orientation-unresolved witnesses.
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

## Checked on the saved bottle scene

15,175 position-found input rows; 15,064 accepted current-model nodes; 111 rejected
by current limits/self-collision/duplicate filtering. Maximum FK discrepancy was
1 mm. The graph had 93,355 candidate edges and 13 connected components.

A snapshot replay found an 8-edge path ending 35.92 mm from the target, with 124
exact state checks in an approximately 83 ms planning cycle. Coordinator replay
accepted 10 trajectory points (measured start plus 9 recorded graph nodes).
An injected obstacle at the initial EoE correctly rejected the path. All replay
traffic used isolated ROS domain 71; no arm/gripper action was called.
