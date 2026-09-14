# Graph-pick fault diagnostics — 2026-09-13

The 17:24 run loaded a verified D435i hand-eye artifact (serial
243222076197, 9 samples, translation RMSE 0.0121 m, rotation RMSE 0.0562
rad). Planning reached an executable five-point preview. Execution entered
`opening gripper` at 1789287899.3848 and never reached `following_graph_path`;
the coordinator was fault-latched by 1789287905.2434.

The first logged Dynamixel communication failure occurred later, at
1789287928.3027, and recovered after one read failure, so it did not cause the
initial fault. The old coordinator retained only the boolean latch; a later
preview overwrote the original message. The historical evidence therefore
locates the fault at opening-gripper/its live interlocks but cannot identify
the exact branch retrospectively.

Changes:

- retain the first motion fault reason until coordinator restart;
- log that reason once at ERROR level;
- include `motion_fault_reason` in graph-pick status;
- show the reason in the target GUI instead of only `Coordinator motion sedang fault`.

No interlock, timeout, controller, torque, motion, reachability, or calibration
behavior was changed.

Validation:

- both ROS packages built successfully;
- `om6dof_pick_and_place`: 75 tests passed;
- `om6dof_dd_gng`: 144 tests passed;
- direct GUI summary check displayed the retained gripper timeout reason.
