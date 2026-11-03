# Experimental position impedance, OM6DOF V2

Plugin: `om6dof_controllers/PositionImpedanceController`.
Instance: `om6dof_position_impedance_controller`.
This is joint-space position impedance using signed effort commands in Dynamixel
**Operating Mode 0**, not a servo position loop with a current ceiling (Mode 5).
It is separate from the teaching, leader, spring and trajectory controllers.

## Control law and units

At each controller update, using measured positions and velocities:

```text
g = KDL gravity from the current URDF (Nm)
absolute_limit = min(URDF joint effort, max_effort / effort_scale)
budget(q) = max(0, min(extra_torque_base + extra_torque_ratio * abs(g(q)),
                      extra_torque_max, absolute_limit - abs(g(q))))
reference moves toward final target at reference_velocity (rad/s)
reference stays within max_reference_error of measured position (rad)
requested_residual = stiffness * (reference - position) - damping * velocity
                    [+ URDF mechanical friction compensation, if enabled]
residual = rate-limited requested_residual, clamped to +/- budget(q)
command = effort_scale * (ramp * g + residual)
```

Stiffness is Nm/rad; artificial damping is Nm.s/rad. The residual slew limit is
Nm/s. The total command is also clamped to the absolute limit. A shrinking budget
overrides the slew limit immediately; **zero budget means zero extra torque**.
Residual motion torque is enabled only after the gravity ramp is complete.
There is no integrator. The reference cannot wind up into a large positional
error while the arm is blocked; the final target is kept separately.

The proposed gains/headroom in `config/position_impedance.yaml` are conservative
starting values, **not hardware-calibrated or certified safe values**. Nominal
raw-current/Nm scales and raw-current ceilings are copied from the current torque
teaching profile. They are approximations, not measured torque sensors.
Masses, CoMs, geometry, joint limits and mechanical friction come from URDF;
no copies of those physical parameters or fitted offsets are added to YAML.
Artificial impedance damping is a control gain and is intentionally separate
from URDF mechanical damping. Friction compensation defaults off and, when
enabled, consumes the same extra-torque budget rather than bypassing it.

## Important limitations

- A blocked arm can still apply the bounded residual force. This is **not** a
  contact detector or an emergency stop, and does not guarantee harmless contact.
  Gravity-model error, payload, current/torque calibration, inertia and Jacobian
  geometry affect actual contact force. No payload is assumed beyond URDF.
- A velocity threshold is a trip, not a guaranteed speed limit. Reference speed
  is not a physical speed bound. No collision checking, passivity guarantee,
  Cartesian stiffness, MoveIt trajectory action, or automatic obstacle bypass
  is implemented.
- The controller claims **effort only**. Mode-0 declaration is checked in URDF;
  this does not read back physical servo registers. The hardware driver must
  correctly apply/verify mode, transmission, sign and current units. Do not run
  a position/trajectory controller or another motion publisher alongside it.
- Activation captures the measured pose, discards commands from previous
  sessions, and ramps gravity from zero. Until a new valid target arrives it
  provides gravity plus bounded velocity damping, not a position hold. Support the arm during startup,
  shutdown, switching and faults. Zero torque can cause the arm to fall.

## Loading and commissioning

### Dashboard stack buttons (separate mode-switch and pose confirmations)

In **OM6DOF hardware/controller service → Position impedance stack**:

1. Support the arm and stop any other hardware stack first. **Start position
   impedance stack** refuses a busy/unknown serial owner or another controller
   manager. It starts its own normal position hardware and canonical command
   converter, without teleop, then requests the same READY command as Go to READY.
2. The supervisor reads the live converter's configured TRANSITION and READY.
   It must observe TRANSITION after the request, then measured READY within
   0.03 rad and speed <=0.05 rad/s for one second. Missing/stale/nonfinite state,
   a feedback gap or a timeout blocks advancement. This is not a collision check.
3. Once **READY confirmed** appears, physically support the arm, check the
   support checkbox and click **Arm supported at READY → prepare torque mode**. This is a
   separate, single-use confirmation valid for 60 seconds. Losing READY or fresh
   feedback invalidates it. An old browser form cannot confirm another run.
   A pre-switch READY/feedback failure or expired confirmation offers **Check
   READY again (no motion)**. This explicit read-only recheck requires the same
   owned position processes, an active position controller, and a new full
   one-second settling check without relaxing any thresholds. It issues a new
   support token; no activation or motion follows without another support
   confirmation. Status identifies the joint, speed, timestamp or feedback gap
   that blocked confirmation. Process/controller failures and faults after
   switching still require supported Stop, not the read-only recheck.
4. Only after that confirmation does the supervisor stop its own position
   processes, recheck port ownership, and start Mode-0 hardware with zero initial
   current. It loads impedance inactive with `enable_on_activation: false`.
   **Mode 0 entered does not mean impedance active.** It captures the actual
   supported pose after one continuous second of fresh stationary feedback
   (speed <=0.05 rad/s, drift <=0.03 rad). This pose may differ from READY.
   Review its displayed joint angles, check the NEW support checkbox and click
   **Confirm supported pose → enable gravity / impedance** within 60 seconds.
   Movement, feedback gaps or expiry invalidate the token; only the read-only
   **Capture supported pose again** action requests a new capture, never torque.
   The supervisor verifies inactive exclusive ownership, atomically sets the
   confirmed pose and opens the activation gate, then rechecks feedback/consent.
   The C++ plugin checks the confirmed pose at activation (unchanged 0.03 rad
   tolerance), speed <=min(max_velocity,0.05), URDF limits, gravity torque capacity
   and bus health BEFORE ramping torque. No position target is sent. Old plugins
   without supported-pose protocol version 1 are refused. If no supported pose
   settles within 60 seconds before any activation request, the supervisor
   offers **Recover position control → TRANSITION → READY (moves arm)** for 60
   seconds. It reports the joint/feedback reason and warns that gravity support
   has not started. Recovery requires two new acknowledgements (physical support
   and permission for the full motion path), a current single-use token, live
   stationary feedback, all owned processes alive, and impedance still inactive
   with no other position/velocity/effort arm owner. Only then are owned children
   stopped and the port rechecked. The canonical position preparation is run
   again, including TRANSITION observation and NEW mode-switch and pose consents.
   Expiry, unknown ownership, child failure, failed activation, or an active
   controller fault never causes an automatic restart. A limit-only active
   fault (32) may also offer the separately confirmed recovery below.
5. Wait for impedance readiness, then select Target mode and Run impedance
   target. Starting the stack never sends a target automatically.

The supervisor executable is `impedance_stack` in this package. The web process
owns it and presents its progress/log. All physical startup above happens only
after the user clicks the relevant confirmation, not on web monitor startup.
It never stops an externally launched stack or edits a systemd unit. Hardware
preflight is **not an atomic serial-device lock**: external launchers must remain
stopped throughout the session.

The original YAML is unchanged. A temporary copy preserves every tuning value
and sets `enable_on_activation: false`, `require_activation_pose: true`, initial
READY `activation_pose` and its 0.03-rad tolerance. Only NEW post-switch consent
updates the in-memory activation pose and opens its gate. The configured required
pose and tolerance cannot be loosened at runtime. The ordinary loader retains its default activation lock.
The parameter callback accepts only the atomic pose-plus-enable pair, with six
finite positions inside the configured soft limits, before the first activation
attempt. All gain/model/limit edits and pose edits after an activation attempt
require a supported reload. Failed activation does not authorize another pose.
Direct/manual activation can omit `activation_pose`; the stack button never does.

On failure there is no automatic retry, activation or mode fallback. Existing
owned processes are retained for inspection and **supported Stop**; in Mode 0
there may be zero holding torque if activation failed. Stop impedance stack (or
the main Stop stack button) stops this supervisor's children, not an unrelated
hardware service. Keep supporting until shutdown completes. Closing/restarting
the web process also tears down its owned children; do not restart the web while
the robot is unsupported. **Stop target** is different: it retracts only the
target stream and retains gravity plus bounded damping while the controller stays healthy.

This handoff remains **support-dependent, not bumpless**. It stops one hardware
instance before opening another; there is a zero-current interval. Recovery does
not fix inadequate physical support or guarantee a subsequent handoff will pass.
Keep supporting through the handoff, actual-pose capture and gravity ramp. A
shifted pose is NEVER accepted automatically: fresh stationary capture and NEW
operator confirmation are mandatory. Do not enable from an unsupported falling
pose. The browser's
position-mode buttons are disabled while the supervisor owns the arm. An older
already-running supervisor cannot gain recovery via browser refresh; supported
shutdown and reloading the updated web/supervisor are required.

### Manual loader

The launch **only loads/configures inactive** into an existing controller manager:

```bash
source ~/ros2_ws/install/setup.bash
ros2 launch om6dof_controllers position_impedance.launch.py
```

It also starts the `impedance_target` web bridge, which remains non-commanding
until the plugin is explicitly activated and an operator sends a leased target.
It opens no hardware, changes no operating mode, and never commands READY.
On position-mode hardware configuration is expected to fail. The YAML activation
gate `enable_on_activation: false` additionally prevents accidental activation.
This loader deliberately has no hardware startup or automatic mode switch.

Before a separately supervised hardware trial:

1. Use the existing position-control **Go to READY** path: TRANSITION, then READY,
   with measured settling confirmed. Do not attempt this positioning in Mode 0.
2. Support the arm, clear the workspace, and have an independent stop available.
   Safely stop the position stack and establish exactly one Mode-0 hardware owner
   using the existing supported-arm procedure. Do not restart services remotely
   as part of an unsupervised trial.
3. Verify V2 robot_description, motor mode/current sign/conversion, gravity
   holding capability at the planned poses, current ceilings, and no competing
   arm controller. Inspect/configure the new YAML before lifting the activation
   gate. Parameters are startup-only; changes require a supported unload/reload.
4. Activation additionally requires finite in-limit measured state, acceptable
   speed, enough absolute torque capacity for modeled gravity, and a fresh,
   healthy bus stream for `health_clean_window` seconds. Then begin with a small
   reviewed target and continuous operator supervision. Increasing limits/gains
   is a separate tuning task, not a workaround for a fault.

`allow_mock_hardware` is only honored for `mock_components/GenericSystem`.
Disabling bus health is rejected for real Dynamixel hardware. GenericSystem is
useful for interface tests, **not** a physical effort-dynamics simulation.

## Position targets and watchdog

### Web monitor Target mode

The web monitor's **Target mode — position impedance** now uses this plugin,
not the trajectory controller. Keep the above TRANSITION → READY and supported
Mode-0 commissioning sequence. Start `position_impedance.launch.py` and explicitly
activate the configured plugin; this launch does **not** activate it for you.
Do not run torque teaching or a competing arm controller.

In the web panel select target coordinates (JOINT, CARTESIAN or CYLINDRICAL), use
**Use current position**, adjust a small reviewed target, and press **Run impedance
target**. JOINT angles and orientation are shown in degrees, XYZ/radius in mm;
the backend uses radians/metres. Coordinate selection does not send servo mode
or remote/READY commands. There is no need to press Enable JOINT control in this
mode. Cartesian inputs use the existing gripper orientation conventions and the
live `/robot_description` world → end_effector_link chain, not a second xacro
render. A changed live model locks the bridge until a supported controller and
bridge reload, avoiding targets planned against a different configured model.
The default bridge soft margin is 0.05 rad; the plugin independently
enforces its own configured limits. It refuses approximate IK solutions and
checks the approximate link-capsule straight joint path. Actual compliant motion
may differ from that path; there is no environment collision or swept-volume
guarantee.

`Run` is enabled only with fresh bridge readiness. The bridge independently
checks plugin type, six effort claims, absence of other active arm position,
velocity or effort claims, current telemetry, fault state and URDF limits.
Targets are published as complete stamped JointStates at 20 Hz; the C++ effort
loop remains configured at 100 Hz. Reach time is deliberately not offered:
reference speed comes from `reference_velocity`, and blockage may prevent arrival.
The same final target remains while blocked; at arrival, compliant holding
continues while this browser's lease remains valid.

The browser which clicked Run receives a private lease token and renews it every
250 ms. Merely opening/polling the status page cannot authorize movement.
**Stop target**, hiding/closing/reloading that tab, changing input source, a lost
connection, stale controller status or a controller fault retracts the stream.
Lease expiry is 1 s (source age counts), followed by the plugin's 0.25 s default
command watchdog. Stop normally retracts the stream immediately, then the same
watchdog applies. Reconnection does not resume: press Run again. This removes
the target spring/friction residual, **not gravity or healthy idle damping**, while the plugin's health
checks pass; it is not a protective or emergency stop. Support the arm whenever
deactivating the plugin or stopping hardware.

Bridge protocol (separate from the legacy position converter):

- `/om6dof/impedance_target/command`: String JSON, `move` with `request_id`,
  increasing `stamp_ns`, coordinate `mode` and six SI `values`; `renew` with
  matching id and fresh stamp; `stop` always retracts.
- `/om6dof/impedance_target/status`: String JSON, readiness, actual FK/current
  joint feedback, target state, errors and request id. No browser lease token.
- `/om6dof_position_impedance_controller/target`: bridge's stamped JointStates.

The old `/om6dof/target_cmd` converter remains for non-web clients, but the web
panel no longer publishes there. No fallback to trajectory motion is allowed.

### Direct ROS targets

Topic `~/target`: `sensor_msgs/msg/JointState` with all configured joint names
exactly once (any order), positions in radians, and a fresh ROS header timestamp.
Velocity and effort arrays must be empty. Targets outside the URDF soft limits,
NaN, wrong/duplicate names, stale, future, zero or replayed timestamps are rejected
and latch a motion fault. Messages received while inactive are ignored.

The sender must renew the target, for example at 20 Hz, with a **new timestamp**
each time. Receipt freshness is measured with a monotonic clock; the source
message age also consumes the timeout. On timeout (default 0.25 s), the target
spring and friction feedforward are removed, reference resets to measurement,
gravity plus bounded velocity damping remain, and the final target remains visible in telemetry. Tracking
resumes only when a new valid command is received. A one-shot target is not a
persistent authorization to move.

Healthy WAITING, TIMEOUT and RAMPING apply only opposing velocity damping
`-ramp * damping[i] * measured_velocity[i]`, using the existing YAML damping
coefficients and the same gravity-relative extra-torque and absolute-current
caps. No idle position spring or friction feedforward is applied. Damping
magnitude increases within `residual_torque_rate`; shrinking budgets, velocity
reversal and zero speed immediately discard stale-sign/excess torque, overriding
slew-down if needed. The computed idle damping satisfies `tau_damping * v <= 0`
for the measured sample; this is not a whole-system passivity or safety guarantee
under communication delay, sampling, noise or model error. At rest the added
torque is zero, so this does not correct static gravity-model error or hold pose.
The tracking residual is never carried into idle. FAULT behavior remains
unchanged: **no residual**, no automatic clear; hard faults latch zero commands.
This change is offline-tested, not a hardware-validated cure for joint3 drift.

While blocked and the command stream remains valid, the final target is retained;
the arm can continue toward it as the obstruction is removed, but only within
the residual budget. Stop the command stream if that resumption is unwanted.

## Fault behavior and telemetry

Bus diagnostics use `/dynamixel_hardware_interface/health`, status
`dynamixel_hardware_interface/BusHealth`. Require schema 1, fresh increasing
timestamps, enabled/uninhibited torque, complete voltage/error monitoring, no
current bus/hardware errors. Persistent read/write counters and driver-instance
changes latch a fault even if an intermediate error message was missed.

- Nonfinite state, missing/stale/unhealthy bus, or gravity-solver failure:
  latch **zero commands**, with no automatic reenergizing when samples recover.
- Invalid target, excessive update interval, measured overspeed/soft-limit
  violation, changed persistent health counters, or insufficient modeled gravity
  capacity: latch **no residual**, retain bounded gravity while state/health are
  valid. If gravity exceeds the absolute cap it is clipped and the arm may sag.
- Faults require supported deactivation/reactivation after addressing the cause;
  a new target alone cannot clear a fault. Deactivation writes zero effort.

`~/state` is `Float64MultiArray`, attempted once each update with a nonblocking
realtime publisher. For N joints, it contains `2 + 7*N` numbers:

```text
[status, fault_flags, measured_q[N], final_target[N], reference[N], gravity_Nm[N],
 budget_Nm[N], residual_Nm[N], commanded_raw_effort[N]]
```

Status: 0 waiting, 1 tracking, 2 command timeout, 3 latched fault, 4 gravity ramp.
Fault flags (bitwise sum): 1 nonfinite state, 2 unavailable/unhealthy bus,
4 gravity solver failure, 8 invalid target, 16 update-period fault,
32 measured position/speed limit, 64 insufficient gravity capacity,
128 nonfinite command, 256 changed persistent bus counters/instance or stream gap.

`~/limit_event` is a separate fixed-size `Float64MultiArray`; the existing state
layout is unchanged. It records the **first** limit trip since activation:
`[version=1, kind (1=position, 2=speed), zero-based joint index, measured value,
lower bound, upper bound, current_limits_ok]`. Position is radians and speed is rad/s. Kind 0 means
no limit event. The bridge shows the joint, measurement and bound for fresh
events, decodes all other fault bits, and distinguishes ramping from faults.
Historical faults from an older binary cannot be reconstructed from this topic.

For **only fault 32**, the supervisor offers the existing explicitly confirmed
**Recover position control → TRANSITION → READY (moves arm)** action, including
when the trip occurs during the initial gravity ramp. It requires two operator
acknowledgements, an expiring token, fresh limit-only fault feedback, exclusive
active impedance ownership, current positions/speeds within the plugin's limits,
and a new continuous one-second stationary supported-pose capture **before**
issuing a consent token. This captures the actual supported pose, not READY.
The checks are repeated after the manager query. A moving/unsettled pose,
stale feedback or expired consent cancels permission and offers **Capture
recovery pose again (no motion)** instead of terminating the supervisor.
Recapture is read-only and requires an explicit click; it never renews motion
consent. A new displayed pose and BOTH new acknowledgements are mandatory.
Additional faults, lost ownership, process/service failure still require Stop.
This is a full owned-stack shutdown/reload, **not** an in-place fault clear.
Gravity support disappears during shutdown: keep physically supporting the arm.
The canonical TRANSITION → READY path and BOTH new support confirmations still
apply. No old target resumes, and speed/current/position limits are unchanged.
Other active faults require supported Stop and diagnosis. Existing running
processes must be reloaded to use this workflow; browser refresh is insufficient.

The YAML requests 100 Hz; controller_manager and the hardware read/write loop
must also sustain it. Diagnostic messages may be dropped under contention; the
publication rate is not proof of motor-write rate. Timing and real hardware
behavior have not been validated by the offline tests.

## Offline tests and references

Tests use stack-backed interfaces on ROS domain 192, not real motors. They cover
pose-dependent bounds, signed gravity, zero budget, blocked references, source
timestamps, watchdog, activation gate, faults and lifecycle replay protection.

```bash
colcon build --symlink-install --packages-select om6dof_controllers --cmake-args -DBUILD_TESTING=ON
ROS_DOMAIN_ID=192 ROS_LOCALHOST_ONLY=1 colcon test --packages-select om6dof_controllers --return-code-on-test-failure
```

References: [ROBOTIS XM430-W350 operating modes/current](https://emanual.robotis.com/docs/en/dxl/x/xm430-w350/)
and [ROS 2 Humble realtime_tools](https://control.ros.org/humble/doc/realtime_tools/doc/index.html).
