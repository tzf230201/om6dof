# om6dof_bringup

Canonical hardware and ros2_control bringup for normal OM6DOF position control.
It is the only U2D2 owner for the MoveIt/jog/teleop stack. The separate
`om6dof_leader_controller` package is an alternative leader-only hardware owner;
the two stacks must never run simultaneously.

```text
MoveIt ------> arm_controller ---------------------------+
                                                          |
command sources --> om6dof_controller                    |
                     | conversion / IK / switching        |
                     +--> forward_position_controller ----+
                                                          v
                                             controller_manager
                                                   |
                                      dynamixel_hardware_interface
                                                   |
                                                  U2D2
```

No teleop, pick-and-place, or MoveIt node should open the serial port directly.

## Controllers

`config/controllers.yaml` defines:

| Controller | Plugin | Boot state |
|---|---|---|
| `joint_state_broadcaster` | `JointStateBroadcaster` | active |
| `arm_controller` | `JointTrajectoryController` | active |
| `forward_position_controller` | `ForwardCommandController` | inactive |
| `gripper_controller` | `GripperActionController` | active |

The two arm controllers claim the same six `position` command interfaces and
are mutually exclusive:

- `arm_controller` is the autonomous/MoveIt trajectory path.
- `forward_position_controller` is the final position-command path for remote
  JOINT, CARTESIAN, and CYLINDRICAL modes. Conversion and coordinate IK live in
  `om6dof_controller`; the hardware interface still sees only joint positions.
- `gripper_controller` remains independent and active in either state.

`om6dof_controller` asks `/controller_manager/switch_controller` to exchange
the two arm controllers atomically and is the sole publisher to
`/forward_position_controller/commands`. Teleop and other command sources use
`/om6dof/operation_mode` plus `/om6dof/control_cmd`; they never create another
hardware interface.

## Hardware profiles

| Launch/profile | DXL mode | Semantics | Status |
|---|---:|---|---|
| `hardware.launch.py` | 3, Position Control | six arm position commands | normal MoveIt/jog/teleop path |
| `hardware.launch.py current_control:=true` | 5, Current-based Position Control | position plus current ceiling | legacy research profile; not recommended for the commissioned leader arm |
| `om6dof_leader_controller leader.launch.py` | 0, Current Control | signed current in mA | commissioned gravity/GUIDE research path |

Mode 5 still contains an actuator position loop. On the physical OM6DOF it
became stiff and moved toward a position when the legacy leader controller was
activated. The new leader stack is intentionally separate, uses no arm Goal
Position, starts with torque OFF, and converts KDL torque to signed current.

See [the complete leader-arm gravity-compensation record](../docs/leader_arm_gravity_compensation.md).

## Real hardware

```bash
source ~/unitree_ros2/setup.sh
source ~/ros2_ws/install/setup.bash
ros2 launch om6dof_bringup hardware.launch.py
```

Options:

```bash
ros2 launch om6dof_bringup hardware.launch.py \
  port_name:=/dev/ttyUSB0 baud_rate:=1000000
```

The default device uses the stable FTDI `/dev/serial/by-id/...` path.

### Torque-off production-traffic diagnostic

Use `torque_off_diagnostic.launch.py` only to determine whether communication
errors appear under the real `ros2_control` read/write cadence while no motor
may produce torque. This is intentionally a separate entry point. It starts
the production Dynamixel hardware plugin and `joint_state_broadcaster`, but no
arm, gripper, forward-command, MoveIt, teleop, or pick controller.

The arm can fall as soon as torque is removed. Mechanically support every link
before stopping the normal owner, keep emergency power removal reachable, and
then run:

```bash
sudo systemctl stop om6dof-hardware.service
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch om6dof_bringup torque_off_diagnostic.launch.py \
  arm_supported:=true diagnostic_update_rate_hz:=100
```

The launch aborts before opening U2D2 unless the support acknowledgement is
exactly `true` and `om6dof-hardware.service` is conclusively loaded and
inactive. Diagnostic mode is hard-coded in the launch and cannot be disabled
with a launch argument. Inside the driver it:

- forces Torque Enable off during initialization and never enables it at
  activation;
- rejects torque-enable, generic register-write, and reboot service requests;
- checks fresh Torque Enable feedback for every actuator on activation and
  every successful cyclic read;
- requests Torque Enable off and terminates the stack if any actuator reports
  torque on or if torque feedback becomes incomplete.

Verify the invariant and the active controller set from another terminal:

```bash
ros2 control list_controllers
ros2 topic echo --once /dynamixel_hardware_interface/dxl_state
ros2 topic echo --once /dynamixel_hardware_interface/health
```

Expected results are only `joint_state_broadcaster` active, every
`torque_state` value `false`, and health values containing
`torque_all_enabled=false`, `torque_all_disabled=true`, and
`torque_enable_inhibited=true`. A clean bus is reported as
`OK (torque-off diagnostic mode)`; communication counters remain cumulative
for the life of that driver instance.

For rate isolation, repeat otherwise identical, separately restarted runs at
`diagnostic_update_rate_hz:=10`, `20`, `50`, then `100`. Only these four
values are accepted, and the override applies only to this diagnostic
`controller_manager`; `config/controllers.yaml` and the production 100 Hz
configuration are not modified. Compare equal transaction counts (rather
than equal wall-clock time) when calculating the read-error rate.

The commissioned hardware profile selects
`read_transport_mode: sequential_single_sync`. Each cycle issues seven
single-responder SyncRead transactions against the same 14-byte indirect block
and commits the state values only after all seven replies succeed. The generic
driver default remains `multi_sync`; OM6DOF opts in through
`config/hardware_safety.yaml`. The torque-off diagnostic validates this mode in
its rendered robot description before opening U2D2.

This diagnostic is not electrically read-only: the normal production loop
still transmits Goal Position packets and maintains Bus Watchdog while torque
is off. Initialization can also restore the configured mode, gains, profiles,
and watchdog values. The torque lock prevents those packets from moving the
robot, but it is not a certified safety function and cannot protect against a
separate process that directly owns the serial bus. Do not run any second U2D2
client. Stop the diagnostic with Ctrl-C before restarting the normal service.

### U2D2 / FTDI USB latency

The U2D2 uses an FTDI USB-to-serial interface. Ubuntu defaults its USB latency
timer to 16 ms, while OM6DOF's control cycle can time out in about 10 ms. This
can appear as intermittent `SYNC_READ_FAIL` messages even when the motors
recover immediately.

OM6DOF therefore sets `read_packet_timeout_ms: 30.0` in
`config/hardware_safety.yaml`. Both position and current-control descriptions
use it as the deadline for each requested single-servo response. Each servo's
14-byte indirect state block includes position, velocity, current, torque,
input voltage, and Hardware Error Status. A normal response still returns
immediately; the extra time is consumed only when a response is late.
This does not weaken the persistent failure counters or the ten-consecutive-
failure shutdown. Changing it requires rebuilding and a hardware-owner restart,
which must only be done with the arm supported and workspace clear.

At 1 Mbps, seven one-ID SyncRead request/status pairs occupy about 2.8 ms of
wire time before USB and host scheduling overhead. Commission the mode at
50 Hz first. Use 100 Hz only after a torque-off run demonstrates adequate
end-to-end timing margin for the complete read/write cycle; an exceptional
missing response can still consume the configured 30 ms deadline.

The position and current-control profiles both monitor Hardware Error Status on
all seven physical Dynamixels. The health topic reports the bitwise aggregate
plus expected/monitored coverage, so a missing interface is an error rather
than an apparent all-clear value of zero. It also reports voltage coverage and
the minimum input voltage/ID from the same cyclic packet for supply-sag
diagnosis; no extra service read is needed.

Install the ROBOTIS udev rule once on the host to set the timer to 1 ms on
every FTDI U2D2 reconnect or reboot.  The same rule marks the adapter with
`ID_MM_DEVICE_IGNORE=1`, preventing ModemManager from probing the Dynamixel
bus as if it were a modem:

```bash
source ~/ros2_ws/install/setup.bash
ros2 run dynamixel_hardware_interface create_udev_rules
```

For this OM6DOF setup, verify the active U2D2 device with:

```bash
OM6_DXL_TTY="$(basename "$(readlink -f \
  /dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FT5NUUIQ-if00-port0)")"
cat "/sys/bus/usb-serial/devices/${OM6_DXL_TTY}/latency_timer"
udevadm info -q property -n "/dev/${OM6_DXL_TTY}" | \
  grep '^ID_MM_DEVICE_IGNORE=1$'
```

The expected latency is `1`, and the udev property must be
`ID_MM_DEVICE_IGNORE=1`. The actual tty number may differ after reconnect;
the launch file uses the stable `/dev/serial/by-id/...` path, so it does not
need to be changed. If `SYNC_READ_FAIL` continues after the latency is `1`,
inspect the Dynamixel power supply, common ground, U2D2 cable, and daisy-chain
connectors for voltage drop or electrical noise.

#### Recorded validation — 2026-08-10

The following observation was recorded on the Jetson AGX using the OM6DOF
hardware service and the FTDI U2D2 (`FT5NUUIQ`) at 1 Mbps:

| Test condition | FTDI latency | Observation |
| --- | ---: | --- |
| Before udev rule | 16 ms | About 12 `SYNC_READ_FAIL` events per minute; each observed sequence recovered after 1–2 failed reads. |
| After udev rule, without service restart | 1 ms | 0 `Communication Fail` events during 40 seconds of observation. |
| After OM6DOF stack restart | 1 ms | 0 `Communication Fail` events during 60 seconds of observation; service remained `active`, with `NRestarts=0`. |

This is evidence that the default 16 ms FTDI buffering was the main source of
the observed read timeouts. It is not a guarantee against future bus faults:
if errors return while the timer remains `1`, check motor power, common ground,
and the Dynamixel cable chain.

#### Multi-responder isolation — 2026-09-04

Later torque-off diagnostics with the FTDI latency already at 1 ms found a
second failure mode. Multi-ID GroupSyncRead produced intermittent CRC/timeout
failures in natural, reverse, and sorted ID orders. Some early CRC failures
left 31–99 bytes queued, showing that later status packets continued after the
failed response; other failures reached the 30 ms deadline with an empty
queue. Exact individual reads completed 28,000/28,000 cleanly, and seven
one-ID GroupSyncRead runs completed 70,000/70,000 cleanly.

Consequently, changing ID order is not treated as a fix. OM6DOF now uses the
single-responder sequential transport while retaining the same ID-keyed state
mapping, fail-safe counters, and unchanged SyncWrite path. Physical inspection
of power, common ground, U2D2, and the daisy-chain remains required because the
software mode avoids the triggering traffic pattern rather than repairing its
electrical cause.

## Fake hardware

```bash
ros2 launch om6dof_bringup hardware.launch.py use_fake_hardware:=true
```

## MoveIt wrapper

`real.launch.py` can start hardware and ordinary MoveIt together:

```bash
ros2 launch om6dof_bringup real.launch.py
```

When `om6dof-hardware.service` already owns U2D2, start only MoveIt:

```bash
ros2 launch om6dof_bringup real.launch.py start_hardware:=false
```

MoveIt uses `arm_controller`. Remote control must be disabled before executing
a plan.

## Verification

```bash
ros2 control list_controllers
ros2 control list_hardware_interfaces
ros2 topic echo --once /joint_states
```

Expected initial states:

```text
joint_state_broadcaster     active
arm_controller              active
gripper_controller          active
forward_position_controller inactive
```

If `ros2_control_node` exits, `hardware.launch.py` shuts down its parent launch
so systemd can restart the complete hardware owner instead of leaving only
`robot_state_publisher` alive.

## Ownership rule

Never launch two copies of `hardware.launch.py`, or one `hardware.launch.py`
plus `om6dof_leader_controller/leader.launch.py`, against the same U2D2. A
second process cannot share the serial port and must not be used as a
controller-switch mechanism. Within the normal profile, controller switching
occurs inside the one canonical `controller_manager`. Changing between the
normal and leader profiles requires a supported arm, zero current, verified
Torque Enable OFF, and a complete hardware-owner restart.
