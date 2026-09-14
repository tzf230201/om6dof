# Concurrent automatic reference and joystick trim

Status: **isolated preview, not a replacement for the physical controller**.
The nodes have no motor publisher, action client/server, controller-switch
client or hardware-enable parameter. Legacy controller and DD-GNG pickup
execution retain their existing ownership model. Do not route preview output
to a motor controller.

The requested default physical combined controller is still pending. It needs
live whole-body/environment collision validation of the combined path, timing
under joint velocity/acceleration limits, action result semantics and measured
tracking/interlock checks. The existing reachability validation service only
works in separate query mode, not as a streaming validator on the active
reachability instance. Nominal-path approval cannot certify a modified path.
No calibration, pose, TTL or execution guards were relaxed.

## Implemented behavior

Two independent inputs operate concurrently:

- JointTrajectory: `/om6dof_topo_gng_v2/graph_pick/trajectory_preview` by default.
  Volatile subscription: no automatic replay of retained previews at startup.
- World-frame TwistStamped: `/om6dof/combined_preview/teleop_twist`.
  Linear velocities are m/s; angular velocities are rad/s, both in world.

```
p_combined(t) = FK(q_nominal(t)).position + offset_world(t)
R_combined(t) = R_offset(t) * FK(q_nominal(t)).rotation
```

The nominal reference uses piecewise linear joint-position interpolation for
visualization, not a hardware execution profile. Offset translation integrates
Twist linear velocity; rotation increments premultiply the stored world offset.
Neutral or expired input retains offset while the nominal clock continues.
Finishing the nominal preview retains offset and permits more joystick trim.
New previews retain offset but require neutral input before accepting trim.
Restart the preview process to clear the offset.

Prototype bounds: linear norm 0.05 m/s, angular norm 0.5 rad/s, offset
translation norm 0.10 m, offset rotation angle 0.5 rad, timeout 0.3 seconds.
These bound references and are not validated physical robot settings.
Stale, future, duplicate, out-of-order, nonfinite or overspeed packets stop
integration. Reconnect/input loss requires neutral before accepting input.

Corrected joint references require finite IK results, effective V2 URDF joint
limits (0.02 rad margin), <=1 mm position and 0.5 degree orientation residual,
and sampled approximate self-collision checking. Invalid results suppress
joint-reference publication. Environment collision, actuator dynamics and
tracking are NOT validated. `preview_only=true`, `executable=false` always.
The magenta curve is a requested pose; it can be invalid. Inspect status rather
than interpreting its color as collision clearance.

## Run on the AGX desktop

```
source /opt/ros/humble/setup.bash
source /home/kublab/ros2_ws/install/setup.bash
ros2 launch om6dof_teleop combined_preview.launch.py
```

This starts the compositor and RViz, without camera, hardware, gamepad, fake
joint state or TF publishers. Existing planning supplies real `/joint_states`
and new trajectory previews. Click **Preview EoE path in RViz**, not Execute.

For Logitech input add `gamepad:=true`. First stop the legacy teleop adapter
that reads the same gamepad, because that separate adapter can still move the
real robot. This launch does not stop existing processes. No AUTO/MANUAL button
is needed in the preview adapter. Start, Back, mode and gripper buttons issue
no robot operations.

Use `rviz:=false` for an existing RViz. Add MarkerArray topic
`/om6dof/combined_preview/markers`, fixed frame `world`. Green is the nominal
automatic reference; magenta is the combined reference.

```
ros2 topic echo --full-length /om6dof/combined_preview/status
```

Development tests use synthetic messages and mock publishers. No real robot
execution, controller restart or live joystick movement was performed.
