# Experimental Mode-5 joint admittance

This opt-in path keeps current-based position mode, the existing motor PID
gains, the pose-dependent current budget, and the strict TRANSITION → READY
sequence. It adds a bounded virtual spring/damper displacement around the
original absolute target. A current ceiling alone does not provide this behavior.

It is not an energy-optimal or certified force/safety controller. The current
model estimates external joint torque, not a directly measured Cartesian force.
Real-arm commissioning remains necessary before enabling this path.

## Control and ownership

After a new target is explicitly requested, the converter requests a STRICT
controller-manager switch from `arm_controller` to
`forward_position_controller`. It sends no direct position command before a
successful response. The initial direct reference is a fresh measured pose,
not the final target. Mode 5, PID registers, and the current-budget controller
are not switched.

The outer loop publishes rate- and acceleration-limited joint references on
`/forward_position_controller/commands`, with a corresponding motion reference
on `/om6dof/admittance/reference`. It does not stream interpolated JTC
trajectories or overwrite the operator offset. A fresh controller listing,
zero operator offset, and exclusive direct/offset publishers are required
throughout the session.

The dynamics are conceptually:

```
tau_external = M(q) qdd + C(q, qd) + g(q) + friction(qd)
               - current_raw / raw_per_nm - torque_bias
Mv * offset_dd + Dv * offset_d + Kv * offset = filtered(tau_external)
command = bounded(nominal_reference + offset)
```

Contact uses per-joint on/off hysteresis and dwell times. While contact is
estimated, nominal progress decelerates to a pause; the original absolute
target is retained. After release, the displacement relaxes and nominal
progress resumes. A tracking-error governor prevents the command accumulating
far ahead of an arm that cannot follow. Displacement and joint, speed, and
acceleration limits remain in effect.

Missing/stale/invalid force information freezes the reference and contact
state. It is **not** interpreted as release. At current saturation, gravity
model error, insufficient support, and actual contact cannot be reliably
distinguished; this implementation conservatively freezes rather than
claiming a valid external-force estimate. Thus compliance at the current cap
is not guaranteed. Resolve inadequate support/model/caps before commissioning.

Stop immediately disables return to the original target, makes a bounded
transition to a captured fresh measured hold, and waits for measured settling.
It reports `stopped` only after a successful STRICT switch back to
`arm_controller`. A failed/unknown ownership transition stays blocked; it
does not silently retry, restart hardware, or change motor mode. Stop is not
torque-off or an emergency stop. If a switch response never returns, its
pending request cannot safely be assumed canceled: commands stay blocked,
including after Target Stop. Support the arm and use the stack Stop controls;
do not launch a second controller to bypass unresolved ownership.
Loss of the converter leaves the direct
controller's last position reference; use the supervised stack's fault handling
and physical support, not this software as a safety mechanism.

## Observer data contract

`GravityCurrentBudgetController` reads positions, measured velocities, signed
raw Present Current, and the driver's `gravity_budget/read_ok` in the same
ros2_control cycle. Arm `joint_states.effort` in this driver is raw current,
**not Nm or mA**. It must not be interpreted using the gripper's conversion.

The physical model uses the active URDF inertial/dynamics values with filtered
measured velocity and acceleration. It excludes the current budget's gravity
multiplier, holding reserve, and motion reserve. Unmodeled gearbox friction,
inertial/CoM errors, and nominal current-to-torque errors contaminate the
estimate and must be measured, not assumed to be contact.

`/gravity_current_budget_controller/observer_sample` publishes a versioned
32-double `Float64MultiArray` packet:

| Indices | Meaning |
|---|---|
| 0 | ROS source timestamp, seconds |
| 1 | Quality code |
| 2–7 | Measured joint position, rad |
| 8–13 | Measured joint velocity, rad/s |
| 14–19 | Signed raw Present Current |
| 20–25 | Physical model torque, Nm |
| 26–31 | Commanded Goal Current ceiling, raw |

Quality 0 is usable; 1 is startup, 2 bus/current-budget fault, 3 invalid motor
telemetry, 4 dynamics failure, and 5 current saturation/insufficient headroom.
Both source timestamp and monotonic receipt age are checked by the converter.
Invalid force estimates are omitted from target status, not displayed as live
zero force. A saturated but fresh position sample can support the explicit
Stop workflow, but cannot enable compliant progress.

## Configuration and commissioning

`config/controller.yaml` contains the `admittance` block. Defaults are
`enabled: false` and `calibration_verified: false`; ordinary targets retain
their existing action-based behavior. The candidate gains and contact
thresholds are **not real-arm calibrated values**. `raw_per_nm` is a nominal
conversion; verify motor current polarity before use. Do not auto-zero bias
while the arm is supported by hand or in contact: that would hide a real force.

Before setting `calibration_verified: true`, record no-contact static and slow
motion residuals across the intended workspace, separately verify positive
and negative pushes on each joint, and select fixed bias and hysteresis above
the measured model/noise envelope. If that envelope overlaps the intended
contact torque, current-only contact sensing is insufficient in that region.
Do not compensate a bad model by continually learning away a sustained push.

Only after an explicitly supervised commissioning decision, the launch profile
is:

```bash
ros2 launch om6dof_teleop full_stack.launch.py \
  current_control:=false gravity_current_budget:=true \
  start_go2w_teleop:=true input_source:=web admittance_control:=true
```

This command starts hardware and can run the existing READY workflow; it is
not an offline test. Stop the old stack and support the arm before an approved
restart. The input adapter is required for the existing fresh source-ACK gate;
the gate is not bypassed. READY tolerances are not loosened. Restoring ordinary
targets requires restoring the position owner and an approved restart without
`admittance_control:=true`.

Acceptance is based on recorded measured behavior: no-contact drift/noise;
positive and negative push response for each joint; sustained contact without
nominal-target windup; bounded release/return; original-target retention;
stale/bus/saturation freeze; Stop during pending switching; and confirmed
position-owner restoration. Check actual current ceilings, measured tracking,
and unchanged PID/EEPROM limits. Offline tests cover the mathematical and
asynchronous state-machine contracts but cannot establish physical stability,
minimum energy consumption, force accuracy, or stopping distance. Energy
claims need synchronized supply voltage/current measurements over the same
motion, not just smaller Goal Current numbers.
