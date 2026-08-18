# om6dof_phosphobot_bridge

Exposes the OM6DOF arm to [phosphobot](https://phospho.ai) over HTTP, without
giving it the servo bus.

## Why

phosphobot's manipulator drivers open the serial port and speak Dynamixel
themselves. For the OM6DOF that would mean fighting `om6dof-hardware.service`
for `/dev/ttyUSB0` — only one process may hold it — and losing everything
`om6dof_controller` provides: IK, velocity ceilings, pose profiles, and the
JOINT/CARTESIAN/CYLINDRICAL modes.

Here phosphobot talks HTTP to this node, and this node talks ROS. The
controller keeps sole ownership of the hardware.

## Run

    ros2 run om6dof_phosphobot_bridge bridge_node

Listens on `127.0.0.1:8021` (localhost only — it commands a real arm and has
no authentication).

## API

| Method | Path | Body | Effect |
|---|---|---|---|
| GET | `/state` | — | joint positions (rad), gripper, mode, remote_enabled, staleness |
| POST | `/positions` | `{"joints": [6 floats]}` | absolute joint target in radians |
| POST | `/stop` | — | clear the target, command zero velocity |
| POST | `/mode` | `{"mode": "JOINT"}` | JOINT, CARTESIAN, CYLINDRICAL, READY, STARTUP, AUTONOMOUS, TOGGLE_REST_READY |
| POST | `/gripper` | `{"command": "open"}` | open or close |

## The position loop

phosphobot writes *absolute joint positions*; `om6dof_controller` accepts only
*velocities* on `/om6dof/control_cmd`. (`/om6dof/target_cmd` is published by the
web monitor but nothing subscribes to it.)

Rather than bypass the controller by publishing to
`/forward_position_controller/commands` — which would skip the safety this
bridge exists to preserve — the node closes the loop itself: a proportional
controller drives measured position toward the target and emits bounded
velocities.

Targets expire after 1 s. A client that stops asking leaves the arm stopped,
not running.

Tuning constants are at the top of `bridge_node.py`: `POSITION_GAIN`,
`MAX_JOINT_VELOCITY` (0.25 rad/s, deliberately below the controller's own
ceiling), and `POSITION_TOLERANCE`.
