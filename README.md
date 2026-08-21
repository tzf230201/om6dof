# OM6DOF ROS 2 System

ROS 2 Humble stack for the OM6DOF six-axis manipulator, Dynamixel hardware,
ros2_control, MoveIt, RealSense perception, pick-and-place, and DD-GNG.

## Deployment

| Host | Responsibilities |
|---|---|
| Jetson AGX (`kublab`) | OM6DOF, U2D2, Dynamixel, ros2_control, MoveIt, RealSense, perception, perception-pick, DD-GNG, and the web monitor |
| Jetson NX | Unitree Go2W services only |

The AGX dashboard runs with `go2w_enabled:=false` and remains usable when the
NX is disconnected. The normal `om6dof_bringup` stack and the isolated
`om6dof_leader_controller` stack are the only intended U2D2 owners. They are
mutually exclusive; never run two hardware owners.

## Leader-arm research profile

![OM6DOF control profiles](docs/assets/leader-control-architecture.svg)

The commissioned leader-arm path uses six Dynamixel actuators in Operating
Mode 0 (Current Control), a URDF/KDL gravity model, signed N·m-to-mA conversion,
explicit arming, a GUIDE heartbeat, current clamps, and slew limits. Staged
testing eliminated the activation snap and made the arm nearly balanced at
`gravity_scale: 1.0` on the tested robot.

This remains research commissioning: current conversion is provisional, no
Coulomb/Stribeck friction model is active, and HOLD impedance is disabled after
an oscillatory first trial. See the complete paper-oriented record:

- [Leader-arm gravity compensation: architecture, equations, experiments, safety, and limitations](docs/leader_arm_gravity_compensation.md)
- [Documentation and visual-asset index](docs/README.md)

## Packages

| Package | Purpose |
|---|---|
| `om6dof_description` | URDF/Xacro and meshes |
| `om6dof_bringup` | U2D2 owner and ros2_control configuration |
| `om6dof_controller` | Operation modes, IK, limits, watchdogs, and controller switching |
| `om6dof_teleop` | Optional Go2W adapter; disabled on the AGX |
| `om6dof_moveit_config` | MoveIt configuration |
| `om6dof_perception` | RealSense RGB-D and YOLOX perception |
| `om6dof_pick_and_place` | MoveIt and perception pickup |
| `om6dof_dd_gng` | DD-GNG semantic camera stream |
| `om6dof_leader_controller` (companion package beside this repo) | isolated Mode 0 gravity compensation and hand guiding |

## Install dependencies

```bash
sudo apt update
sudo apt install \
  ros-humble-ros2-control \
  ros-humble-ros2-controllers \
  ros-humble-moveit \
  ros-humble-librealsense2 \
  ros-humble-controller-manager-msgs \
  freeglut3-dev

python3 -m pip install --user pyrealsense2==2.58.2.10647
python3 -c 'import pyrealsense2; print(pyrealsense2.__file__)'
```

`ros-humble-librealsense2` provides the native library but not the Python
binding, so `pyrealsense2` must be installed separately.

## Download the YOLOX model

```bash
mkdir -p ~/.cache/om6dof_perception
curl -L --fail \
  -o ~/.cache/om6dof_perception/yolox_s.onnx \
  https://github.com/opencv/opencv_zoo/raw/main/models/object_detection_yolox/object_detection_yolox_2022nov.onnx

echo "c5c2d13e59ae883e6af3b45daea64af4833a4951c92d116ec270d9ddbe998063  $HOME/.cache/om6dof_perception/yolox_s.onnx" \
  | sha256sum -c -
```

## Build

Do not source the Unitree environment on the standalone AGX. Clear any stale
CycloneDDS interface override first:

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
unset CYCLONEDDS_URI

colcon build --symlink-install --packages-up-to \
  om6dof_teleop om6dof_moveit_config om6dof_perception \
  om6dof_pick_and_place om6dof_dd_gng application_web_monitor

source install/setup.bash
```

Build the companion leader package separately when working on leader mode:

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select om6dof_leader_controller
source install/setup.bash
```

Build the DD-GNG native core:

```bash
cd ~/ros2_ws/src/om6dof/om6dof_dd_gng/realsense_ddgng
cmake -S . -B build_om6dof
cmake --build build_om6dof -j
test -f build_om6dof/libddgng.so
```

## Manual hardware run

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
unset CYCLONEDDS_URI

ros2 launch om6dof_teleop full_stack.launch.py \
  start_go2w_teleop:=false
```

The stack starts in `AUTONOMOUS`. `arm_controller` is used by MoveIt;
`forward_position_controller` is used by streamed JOINT, CARTESIAN, and
CYLINDRICAL commands. The two arm controllers are mutually exclusive.

### Isolated leader-arm run

Do not add the leader controller to a running normal stack. Support the arm,
stop the normal U2D2 owner, then use the AGX desktop icon **OM6DOF Leader Arm**
or run:

```bash
/home/kublab/.local/bin/om6dof_leader_launcher.sh
```

The launcher starts the `/leader` namespace with torque OFF, requires explicit
ARM confirmation, verifies six torque states and gravity current, and requests
disarm plus torque OFF on exit. Full manual terminal commands and the distinction
between Mode 0 and the legacy Mode 5 profile are in the
[leader-arm research document](docs/leader_arm_gravity_compensation.md).

## Install systemd services

Install the hardware system service:

```bash
sudo install -o root -g root -m 0644 \
  ~/ros2_ws/install/om6dof_teleop/share/om6dof_teleop/systemd/om6dof-hardware.service \
  /etc/systemd/system/om6dof-hardware.service
sudo systemctl daemon-reload
sudo systemctl enable --now om6dof-hardware.service
```

Install the AGX user services:

```bash
mkdir -p ~/.config/systemd/user

install -m 0644 \
  ~/ros2_ws/install/om6dof_perception/share/om6dof_perception/systemd/om6dof-perception-user.service \
  ~/.config/systemd/user/om6dof-perception.service
install -m 0644 \
  ~/ros2_ws/install/om6dof_pick_and_place/share/om6dof_pick_and_place/systemd/om6dof-perception-pick-user.service \
  ~/.config/systemd/user/om6dof-perception-pick.service
install -m 0644 \
  ~/ros2_ws/install/om6dof_dd_gng/share/om6dof_dd_gng/systemd/om6dof-dd-gng.service \
  ~/.config/systemd/user/om6dof-dd-gng.service
install -m 0644 \
  ~/ros2_ws/install/application_web_monitor/share/application_web_monitor/systemd/om6dof-web-monitor-user.service \
  ~/.config/systemd/user/om6dof-web-monitor.service

systemctl --user daemon-reload
systemctl --user enable --now om6dof-web-monitor.service
```

Install the dashboard restart permission:

```bash
sudo install -o root -g root -m 0440 \
  ~/ros2_ws/install/application_web_monitor/share/application_web_monitor/sudoers/om6dof-web-monitor \
  /etc/sudoers.d/om6dof-web-monitor
sudo visudo -cf /etc/sudoers.d/om6dof-web-monitor
```

Open `http://<agx-ip>:8080`.

## Camera modes

Perception and DD-GNG both own the same RealSense and cannot run together.

Standard perception:

```bash
systemctl --user stop om6dof-dd-gng.service
systemctl --user start om6dof-perception.service om6dof-perception-pick.service
```

DD-GNG:

```bash
systemctl --user stop om6dof-perception.service om6dof-perception-pick.service
systemctl --user start om6dof-dd-gng.service
```

| Mode | JPEG topic |
|---|---|
| Perception | `/application_web_monitor/perception/image/compressed` |
| DD-GNG | `/application_web_monitor/ddgng/image/compressed` |

Verify a stream:

```bash
ros2 topic info -v /application_web_monitor/perception/image/compressed
ros2 topic echo --once \
  /application_web_monitor/perception/image/compressed --field format
```

Publisher count must be at least one and the expected format is `jpeg`.

## Operation modes

Publish mode requests to `/om6dof/operation_mode`:

| Mode | Behavior |
|---|---|
| `AUTONOMOUS` | Give arm ownership to MoveIt |
| `JOINT` | Acquire streamed joint control and move through READY |
| `CARTESIAN` | Stream end-effector linear/angular velocity |
| `CYLINDRICAL` | Stream radial, angular, Z, and tool angular velocity |
| `READY` | Move to the configured ready pose |
| `STARTUP` | Move to the initial feedback pose |

```bash
ros2 topic pub --once /om6dof/operation_mode \
  std_msgs/msg/String "{data: JOINT}"
```

`/om6dof/control_cmd` requires exactly six finite values at 20-50 Hz. Do not
run multiple publishers on this topic.

## Verification

```bash
systemctl status om6dof-hardware.service --no-pager
systemctl --user status om6dof-web-monitor.service --no-pager
ros2 control list_controllers
ros2 node list
```

Expected initial controller states:

- `joint_state_broadcaster`: active
- `arm_controller`: active
- `forward_position_controller`: inactive
- `gripper_controller`: active

## Troubleshooting

Camera missing:

```bash
systemctl --user status om6dof-perception.service --no-pager
journalctl _SYSTEMD_USER_UNIT=om6dof-perception.service -n 100 --no-pager
python3 -c 'import pyrealsense2; print(pyrealsense2.__file__)'
lsusb | grep -i realsense
ros2 topic info -v /application_web_monitor/perception/image/compressed
```

DD-GNG exits immediately:

```bash
journalctl _SYSTEMD_USER_UNIT=om6dof-dd-gng.service -n 100 --no-pager
test -f ~/ros2_ws/src/om6dof/om6dof_dd_gng/realsense_ddgng/build_om6dof/libddgng.so
test -f ~/.cache/om6dof_perception/yolox_s.onnx
```

`list_controllers` timeout is a hardware issue, not a camera issue:

```bash
systemctl status om6dof-hardware.service --no-pager
journalctl -u om6dof-hardware.service -n 100 --no-pager
```

If ROS fails when the NX or Ethernet cable is disconnected, run:

```bash
unset CYCLONEDDS_URI
```

## Safety

- Keep the arm workspace clear before startup or restart.
- Never run two U2D2 hardware owners.
- Support the arm and verify six Torque Enable states OFF before changing
  between normal and leader hardware profiles.
- In the Mode 0 leader profile, ROS `effort` carries current in mA, not torque
  in N·m. Never write raw KDL torque directly to that interface.
- Never run perception and DD-GNG simultaneously.
- Return to `AUTONOMOUS` before executing MoveIt trajectories.
- Expose dashboard port 8080 only on a trusted LAN or behind a firewall/VPN.

See each package README for package-specific details.
