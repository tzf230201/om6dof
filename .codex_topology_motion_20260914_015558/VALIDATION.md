# Two-topology trajectory planning — 2026-09-14

## Requested behavior

YOLO + semantic DD-GNG environment is retained. The robot roadmap represents
end-effector poses with associated joint configurations; its edges represent
validated joint-space transitions. A trajectory starts at measured joints,
connects to the robot roadmap through a validated connector, follows roadmap
edges and ends at the nearest reachable collision-valid target intersection.
A positional move is separate from grasping.

## Findings

Captured live 500-node graph had a path 1 -> 352 -> 405 -> 0 with goal distance
0.015123857 m. The coordinator additionally required grasp alignment >=0.30,
which is not the task constraint for a positional move. Legacy scene generation
also omitted target geometry and nearby geometry for pickup. When the full
scene was checked, the base roadmap's nearby intersection candidates could not
produce a collision-valid route for this captured scene.

## Changes

- graph_pick task_mode defaults to move_to_target. It consumes the exact-validated
  robot roadmap trajectory, retains calibration/age/current-joints/target/bus/
  execution guards, and sends only the frozen arm trajectory after explicit Execute.
  It does not open/close the gripper or retreat. pickup remains a separate configured
  task with the original alignment and gripper requirements. Unknown task modes fail.
- V2 reachability enables include_target_in_collision: FCL scene construction uses
  all finite environment nodes and valid edges, including target and its neighborhood.
  Capsule obstacle checks remain a cheap prefilter; full scene checks use robot meshes
  and environment spheres/cylinders. Collision-invalid goal nodes are removed before
  Dijkstra, avoiding repeated attempts via other edges into the same colliding goal.
- V2 enables bounded target_refinement_enabled: only after base graph planning fails,
  deterministic local Halton perturbations around up to 16 nearby roadmap seeds add
  valid configurations and up to 3 connections per accepted node. Each node and edge
  passes joint/self/capsule/full scene checks. Maximum 128 added nodes; each refinement
  cycle checks up to 256 samples and stops starting new attempts after 150 ms (an
  in-flight candidate check can exceed that duration). Samples use +/-0.45 rad within
  actual joint bounds; this is a sampling radius, not an execution/safety limit.
  Nodes persist until graph rebuild; graph revision increments only on growth.
  Published graph_method is gng+local_prm when local nodes have been added.
- Neither the environment learner nor its semantic settings were edited. No frozen
  experiments or paper results changed. The legacy default C++ planner has both new
  flags disabled, preserving its original search/scene behavior for old fixtures.
- GUI reports move-to-target without gripper actuation and explains collision blockers.
- move_to_target requires the planner's target-protected validation reason, preventing
  an old running planner from silently certifying the stronger scene requirement.

## Validation

- 114 Python/launch tests passed: pick-and-place package, combined preview-node tests,
  and V2 launch/provenance tests. Four added cases cover task alignment separation and
  arm-only execution success/failure/interlock rejection without any gripper call.
- 13 existing C++ reachability tests passed.
- colcon symlink build of om6dof_dd_gng + om6dof_pick_and_place passed.
- Snapshot replay, ROS_DOMAIN_ID=71 and no robot connections: protected base roadmap
  rejected the scene; bounded local refinement found a valid path [1,508], graph size
  510. Goal distance 0.047968402 m; planning cycle 202.678 ms; 485 exact state checks.
  Every consecutive path pair was verified against published graph edges.
- Offline coordinator fixture built from that geometry, with synthetic track metadata:
  accepted 3 trajectory points (measured start + 2 graph nodes). Every graph waypoint
  matched the published node's joint configuration. Remaining blocker was deliberately
  execution_disabled_at_launch. This fixture is not a live semantic-track validation.
- Negative replay with an obstacle at initial EoE: valid=false,
  current_full_body_intersects_environment, empty trajectory path.
- No controller restart, torque operation, arm goal or gripper goal was performed.

## Test limitations and operational state

This validates saved geometry and modeled collision, not a physical pickup or arbitrary
future scenes. FCL checks mesh collision at discretized joint steps (0.05 rad), not a
continuous swept-volume proof. Sparse DD-GNG cannot represent unseen obstacles. Existing
fixed-base collision exemptions and modeled gripper geometry remain; teleop offsets
alter the physical path and are not certified by this nominal roadmap result.

Initial replay attempts remapped legacy topic names while V2 YAML used explicit V2 topic
names; no replay data reached the intended subscribers. The first such attempt briefly
published preview graph data on the live V2 topic names (no actions); it was stopped.
Subsequent tests used domain71 and explicit topic parameters, with no connection to robot
nodes. The failed remap attempts are not counted as successful validation.

One build was inadvertently started in src and interrupted; its newly created build/install
artifacts were moved into aborted_build_wrong_cwd. Existing src/log history was preserved.

Running planning processes were not restarted and still need a planning-launch restart.
Do not start a second camera owner: Ctrl-C only the existing planning launch, then relaunch.
Hardware controller service remains running. Launch defaults execution=false, so preview
can be reviewed without enabling motion; true still requires an explicit Execute click.

## Files and evidence

before.json, after.json, change.diff, snapshot.json, success_result.json,
replay_result.json (negative case), replay.py, check_coordinator.py, gtest.log.
Environment source topo_gng_node.cpp unchanged in this task; current SHA256:
838114c45feffe65383b7a9b7008585ddd48021e34779fc10986ae8d800c474a.
