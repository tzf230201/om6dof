# om6dof_controller

> **Control-profile compatibility:** this package is the normal position-command
> path for MoveIt/jog/teleop and is not used by the isolated Mode 0 leader arm.
> Do not start it as a second U2D2 owner or send position commands while the
> leader stack is active. See the
> [leader-arm gravity-compensation record](../docs/leader_arm_gravity_compensation.md).

High-level command converter between generic OM6DOF jog commands and the
existing ros2_control `forward_position_controller`. This package does not
open U2D2 and is not a ros2_control plugin; `om6dof_bringup` remains the single
hardware owner. It is the sole publisher of final six-joint targets on
`/forward_position_controller/commands`.

## Command contract

The two canonical arm-command topics are:

| Topic | Type | Meaning |
|---|---|---|
| `/om6dof/operation_mode` | `std_msgs/msg/String` | Select command interpretation |
| `/om6dof/control_cmd` | `std_msgs/msg/Float64MultiArray` | Exactly six finite velocity values |

`operation_mode` accepts:

- `JOINT`: `[q1_dot .. q6_dot]` in rad/s. From autonomous ownership this first
  switches controllers and ramps the arm to READY.
- `CARTESIAN`: `[vx, vy, vz, roll_dot, pitch_dot, yaw_dot]`; translation is in
  the world/base frame, rotation is in the tool frame.
- `CYLINDRICAL`: `[radius_dot, theta_dot, z_dot, roll_dot, pitch_dot, yaw_dot]`.
- `AUTONOMOUS`: restore `arm_controller` for MoveIt.
- `READY` or `STARTUP`: transient joint-space pose request; effective mode
  returns to JOINT after reaching the pose.
- `REST`: move to the captured startup/rest pose, then restore
  `arm_controller` ownership for MoveIt (`AUTONOMOUS`).

The command stream expires after 0.3 seconds. On timeout the target is reseeded
from measured joint feedback, so an old velocity never continues moving.

Joint limits for teleop are read at controller startup from the rendered
`om6dof_description` URDF, then reduced by `joint_limit_margin` (default
0.02 rad). There is no duplicate controller YAML limit table.

Linear velocities and `radius_dot` use m/s. Joint and angular velocities,
including `theta_dot`, use rad/s. `operation_mode` is a discrete request;
`control_cmd` is a velocity stream and should normally be published at 20--50
Hz. The gripper is intentionally outside this six-axis command contract and
continues to use `/gripper_controller/gripper_cmd`.

Confirmed read-only state is published on:

- `/om6dof/operation_mode/state`
- `/om6dof/remote_enabled/state`

Both state topics use reliable, transient-local QoS. `remote_enabled` becomes
true only after `forward_position_controller` owns the arm interfaces; the
operation-mode state reports the mode actually accepted by the controller.

## Absolute-target pipeline

Web inputs (mm/degrees) are converted to SI once, then published on
`/om6dof/target_cmd`. The controller validates ownership, fresh feedback,
effective URDF limits and the requested duration. Cartesian/cylindrical IK
tries several initial configurations before accepting an approximation;
each candidate is checked with FK **after** limit clamping. When enabled,
self-collision checking samples the complete joint-space path, not just its
endpoint. This approximate link-capsule check does not detect external
obstacles or guarantee continuous collision freedom.

An accepted target follows a software triangular velocity profile through
`/forward_position_controller/commands` and the existing ros2_control hardware
interface. Acceleration takes half the requested duration, deceleration the
other half. Duration must be 0.5–5 seconds; a request exceeding the configured
joint velocity limit is rejected with its required minimum time. Cartesian
targets interpolate **joint positions**, not a straight Cartesian path.

`/om6dof/target_status` reports measured feedback and errors:

- `running`: the time profile is in progress.
- `holding`: the ramp has ended; the final goal remains commanded.
- `reached`: measured joint error is at most 0.5 degrees and, for pose targets,
  Cartesian position/orientation errors are at most 1 mm / 0.5 degrees.
- `approximate`: the approximate joint goal is reached, but the requested
  Cartesian pose still differs; the message includes its residual error.
- `stopped`, `blocked`, `rejected`, `timeout`: cancellation or failure details.

`active` denotes an executing ramp, not the lifetime of the retained goal.
Accuracy reported here is based on encoder feedback and URDF FK; it is not
an independent measurement of the physical gripper position.

### Cartesian absolute-target orientation

After an absolute-target time profile finishes, its final joint goal remains
commanded even when the servos are still catching up. Neutral teleop packets
do not replace that goal. Stop target, a nonzero jog, an explicit mode change,
or a new pose request takes over; jogging starts from measured feedback.
Loss of feedback cancels the retained goal so it cannot replay on recovery.
Profile completion is time-based and is not a measurement of target accuracy.
This retention applies to the existing software profile; it does not enable
per-target Dynamixel register programming.

For `/om6dof/target_cmd` with `mode: CARTESIAN`, the six values are
`[x, y, z, roll, pitch, yaw]` in metres/radians (web inputs use mm/degrees).
Orientation describes a frame at the URDF tip whose X points along the
gripper: X = URDF tip +Z, Y = tip +Y, Z = tip -X. Zero RPY means a horizontal
gripper facing base +X, with its lateral axis along base +Y. Yaw is the
heading viewed from above; roll twists around the gripper; positive pitch
tilts toward base -Z. Euler heading remains undefined for a vertical gripper.
Both Cartesian target IK and `target_status.current.cartesian` apply this
fixed frame transform in opposite directions. Positions, cylindrical targets,
URDF geometry and velocity-jog conventions are unchanged.

This changes the meaning of previously saved Cartesian target angles. Reload
the monitor and use current position to populate new targets after updating
both the monitor and controller.

## Starting

Start `om6dof_bringup` first, then:

```bash
ros2 launch om6dof_controller controller.launch.py
```

The complete hardware + controller + Go2W adapter can instead be started with:

```bash
ros2 launch om6dof_teleop full_stack.launch.py
```

## Examples

Enable remote ownership and enter READY:

```bash
ros2 topic pub --once /om6dof/operation_mode \
  std_msgs/msg/String "{data: JOINT}"
```

Joint jog:

```bash
ros2 topic pub --rate 20 /om6dof/control_cmd \
  std_msgs/msg/Float64MultiArray \
  "{data: [0.2, 0, 0, 0, 0, 0]}"
```

Cartesian +X:

```bash
ros2 topic pub --once /om6dof/operation_mode \
  std_msgs/msg/String "{data: CARTESIAN}"
ros2 topic pub --rate 20 /om6dof/control_cmd \
  std_msgs/msg/Float64MultiArray \
  "{data: [0.02, 0, 0, 0, 0, 0]}"
```

Return ownership to MoveIt:

```bash
ros2 topic pub --once /om6dof/operation_mode \
  std_msgs/msg/String "{data: AUTONOMOUS}"
```

Multiple publishers on `/om6dof/control_cmd` are last-writer-wins; they are not
summed. A future teleop/autonomy overlay must use a deliberate command mux.
