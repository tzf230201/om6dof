# Graph-pick additive controller compatibility — 2026-09-14

## Cause and change

The running graph_pick required operation_mode=AUTONOMOUS and remote_enabled=false.
The updated offset-only controller publishes remote_enabled=true and uses coordinate
modes such as CARTESIAN; those fields no longer describe exclusive command ownership.
Removed these subscriptions and gates in graph_pick. The coordinator now polls
/controller_manager/list_controllers (read-only, 0.5 s) and requires fresh (1.5 s from
request start) active controllers with distinct, exact claims for the configured arm joints:

- arm_controller: joint_trajectory_controller/JointTrajectoryController, position.
- forward_offset_controller: forward_command_controller/ForwardCommandController, position_offset.

Wrong controller types, absent/inactive controllers, missing/duplicate/wrong claims,
conflicting active claims and missing/stale service replies block execution.
Only one query may remain pending; timeout removes/cancels it and late replies cannot
refresh the snapshot. UI translates the new reasons. Added controller_manager_msgs dependency.

Calibration, explicit execution_enabled=false default, preview TTL, live target/joint
revalidation, approach alignment, bus freshness/torque/errors and failure counters remain.
No changes to hardware composition, rebase, motion limits, collision algorithms or reachability.
Topology checks do not certify collision safety of the trajectory after a teleop offset.

## DDS finding

The shell used rmw_cyclonedds_cpp; the actual hardware systemd unit specifies
rmw_fastrtps_cpp. Bounded ListControllers requests using CycloneDDS timed out (12 s,
then 8 s). Hardware journal at corresponding times reported RTPS_READER_HISTORY
payload 24 bytes > history 11 bytes. The same read-only request using FastDDS succeeded
and returned both required controllers active with the exact separate claims above.
Planning launch now defaults to FastDDS for all its children; standalone graph_pick
also defaults to FastDDS. Both expose rmw_implementation for deliberate reconfiguration.
No hardware service environment or process was changed.

## Validation

- 95 tests passed: pick-and-place package tests plus controller combined-preview-node tests.
  Includes 30 graph-pick tests covering additive acceptance, failed topology, query timeout,
  delayed reply freshness and continuing bus protection.
- 15 existing V2 launch/provenance tests passed.
- Installed planning launch --show-args succeeds with rmw_fastrtps_cpp and execution=false defaults.
- colcon build --symlink-install for om6dof_dd_gng and om6dof_pick_and_place passed.
- Six-second installed GraphPickNode probe, unique name and isolated output/service endpoints,
  execution_enabled=false: received controller topology and hardware health; 780 callback-loop
  observations; final interlock blockers empty. These are callback observations, not 780
  independent controller snapshots (poll remains 2 Hz). No planning or action call was sent.
- FastDDS probe emitted an SHM port-lock warning but service and topic delivery succeeded.
- Hardware journal independently contained intermittent ID031 SYNC_READ_FAIL followed by
  recovery. Snapshot topology success is not a bus reliability certification.
- No camera opened by tests/probes; no robot/gripper goal, torque command, or hardware restart.

## Deployment state

Build/install updated; original GUI/planning launch PID 29358 and graph_pick PID29368
were deliberately left running with their original execution_enabled=false configuration.
Relaunch only the planning terminal to load new Python processes and consistent DDS.
Do not run another camera launch while the original planning launch owns the camera.
After relaunch make a new preview; execution still requires its explicit button and all guards.

## Protected hashes unchanged

- reachability_graph_node.cpp: a3605e37110eac1d2e00e7a6ea9271edbc6721d295173a8ee689f378d3e98431
- reachability_graph.hpp: d3bdc9fab36695217dcd4dde7fa2b67b181c180f3346eab5d0c159b3d4bcbce8

Before copies and hashes: before.json and launch_before.json in this directory.
After hashes and change diff: after.json and change.diff. Existing user changes preserved.
