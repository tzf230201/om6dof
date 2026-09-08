# om6dof_teleop

Human-input adapter for the OM6DOF normal position-control profile. It reads
one selected source and publishes only the canonical ROS interfaces below; it
does not open U2D2, call Dynamixel SDK, solve kinematics, or switch controllers.

```text
input → om6dof_teleop → /om6dof/operation_mode + /om6dof/control_cmd
                       → om6dof_controller → ros2_control → hardware
```

Gripper buttons publish `/om6dof/gripper_cmd` (`open` or `close`).
`om6dof_controller` owns the subsequent gripper action call.

## Run

```bash
source ~/ros2_ws/install/setup.bash
ros2 launch om6dof_teleop teleop.launch.py input_source:=go2w
```

Valid sources are `go2w`, `keyboard`, `gamepad` (Logicool/Logitech F710 in
XInput mode), `airbus` (Thrustmaster TCA), and `web`. The `web` source accepts
raw browser joystick input on `/om6dof/teleop/web_input`; it still maps that
input to arm commands only inside this package. Change a running source safely:

```bash
ros2 param set /om6dof_teleop input_source gamepad
```

The node clears prior input, sends zero velocity, and waits for neutral input
from the replacement device before permitting motion.

On the F710, `Start` takes/releases manual ownership, `Logo` cycles motion
mode, and `Back` performs `REST`: return to the captured startup/rest pose,
then hand ownership back to autonomous control.

## Keyboard

`g` toggles manual ownership, `m` cycles JOINT/CARTESIAN/CYLINDRICAL, `r`
requests READY/STARTUP, `[` and `]` operate the gripper, and `Esc` exits.
Use `+`/`-` to increase/decrease arm speed by 10%; the default is 100% and
the maximum is 500%.
On the F710, holding D-pad up/down repeats the adjustment every 0.2 seconds.

- JOINT: `1/q` through `6/y`
- Cartesian/cylindrical: `w/s`, `a/d`, `r/f`, `u/o`, `i/k`, `j/l`

## Canonical command contract

| Topic | Type | Description |
|---|---|---|
| `/om6dof/operation_mode` | `std_msgs/msg/String` | JOINT, CARTESIAN, CYLINDRICAL, AUTONOMOUS, READY, STARTUP, REST |
| `/om6dof/control_cmd` | `std_msgs/msg/Float64MultiArray` | Six velocity values for the selected mode |
| `/om6dof/teleop/input_state` | `std_msgs/msg/String` (JSON) | Selected input source, device connection, axes, buttons, and last event for dashboard visualization |

`control_cmd` is a 50 Hz velocity stream, not an absolute pose. Only one
publisher should control it at a time.

## Full hardware stack

The deployment launch remains available because
`om6dof-hardware.service` uses it:

```bash
ros2 launch om6dof_teleop full_stack.launch.py input_source:=go2w
```

Never run this position-control stack while `om6dof_leader_controller` owns
the U2D2 bus.
