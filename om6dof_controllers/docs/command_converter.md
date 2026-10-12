# High-level command converter

> **Control-profile compatibility:** this Python node is the normal position-command
> path for MoveIt/jog/teleop and is not used by the isolated Mode 0 leader arm.
> Do not start it as a second U2D2 owner or send position commands while the
> leader stack is active. See the
> [leader-arm gravity-compensation record](../../docs/leader_arm_gravity_compensation.md).

This Python node now lives in `om6dof_controllers`, alongside the C++ plugins.
It keeps the ROS node name `/om6dof_controller`. It does not open U2D2;
`om6dof_bringup` remains the hardware owner. Jog input produces additive joint
offsets on `/forward_offset_controller/commands`. Absolute poses and web targets
go to `/arm_controller/follow_joint_trajectory`; that controller owns the time
profile and absolute-position channel. The package merge changes neither path.

## Command contract

The two canonical arm-command topics are:

| Topic | Type | Meaning |
|---|---|---|
| `/om6dof/operation_mode` | `std_msgs/msg/String` | Select command interpretation |
| `/om6dof/control_cmd` | `std_msgs/msg/Float64MultiArray` | Exactly six finite velocity values |

`operation_mode` accepts:

- `JOINT`: `[q1_dot .. q6_dot]` in rad/s on the operator offset channel.
- `CARTESIAN`: `[vx, vy, vz, roll_dot, pitch_dot, yaw_dot]`; translation is in
  the world/base frame, rotation is in the tool frame.
- `CYLINDRICAL`: `[radius_dot, theta_dot, z_dot, roll_dot, pitch_dot, yaw_dot]`.
- `AUTONOMOUS`: reset the operator offset; `arm_controller` remains active.
- `READY` or `STARTUP`: request an absolute joint-space pose through `arm_controller`.
- `REST`: request the captured startup/rest pose through `arm_controller`.

The jog command stream expires after 0.3 seconds; expired velocity commands
are not integrated further. This watchdog is separate from an absolute
trajectory accepted by `arm_controller`.

The current V2 READY configuration is `[0, -60, 15, 0, 60, 0]` degrees; read
`config/controller.yaml` or the live parameters for the active values. The YAML parameters
`ready_pose_degrees` and `transition_pose_degrees` are expressed in degrees;
the controller converts them to radians internally. Guarded READY/STARTUP/rest
moves first pass through the transition pose, whose default is all zero except
joint 3 at -90 degrees, before continuing to the requested target.
The controller's FK, IK, and joint-limit loading use
`urdf/om6dof_v2.urdf.xacro`; therefore target feedback and commands share the
same v2 mounting geometry.

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

Both state topics use reliable, transient-local QoS. `remote_enabled` reports
the live operator channel, not an ownership switch or hardware safety check.
The operation-mode state reports the mode accepted by the node.

## Absolute-target pipeline

Web inputs (mm/degrees) are converted to SI once, then published on
`/om6dof/target_cmd`. The controller checks fresh feedback,
effective URDF limits and the requested duration. Cartesian/cylindrical IK
tries several initial configurations before accepting an approximation;
each candidate is checked with FK **after** limit clamping. When enabled,
self-collision checking samples the complete joint-space path, not just its
endpoint. This approximate link-capsule check does not detect external
obstacles or guarantee continuous collision freedom.

An accepted target is sent as a `FollowJointTrajectory` goal to `arm_controller`.
The trajectory controller owns interpolation and timing. Cartesian targets
are converted to **joint positions**, not guaranteed straight Cartesian paths.

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
ros2 launch om6dof_controllers controller.launch.py
```

The complete hardware + controller + Go2W adapter can instead be started with:

```bash
ros2 launch om6dof_teleop full_stack.launch.py
```

## Examples

Select joint jog interpretation (this is not a READY motion request):

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

Reset the operator offset for autonomous motion:

```bash
ros2 topic pub --once /om6dof/operation_mode \
  std_msgs/msg/String "{data: AUTONOMOUS}"
```

Multiple publishers on `/om6dof/control_cmd` are last-writer-wins; they are not
summed. A future teleop/autonomy overlay must use a deliberate command mux.
