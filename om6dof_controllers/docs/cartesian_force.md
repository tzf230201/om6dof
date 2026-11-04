# Cartesian wrench mode

`FORCE` is an opt-in, Mode-0 effort-stack feature. It maps a requested TCP
wrench in the **URDF/KDL base frame** into joint torque using virtual work:

```text
tau_motor = g(q) + J(q)^T [Fx, Fy, Fz, Mx, My, Mz] - D*q_dot
```

The controller consumes N, N, N, Nm, Nm, Nm in base axes. The shipped bridge
profile uses `input_frame: custom`: inputs are radial force, tangential force,
vertical force, gripper-forward roll moment, pitch moment, and unused yaw.
Fresh joint feedback and FK rotate this request into the controller's base
frame. Missing/stale feedback or an undefined radial direction sends zero.
The default bridge configuration remains `base` for explicit Cartesian users.

F710 FORCE matches CUSTOM: left vertical stick controls Z; right vertical
stick controls R; right horizontal stick controls theta; X/B select negative/
positive roll and Y/A negative/positive pitch. Left horizontal and trigger
axes do not command a wrench. FORCE consumes signs instead of jog speeds.

The regular position-offset controller never interprets `FORCE` as motion. It
zeros its output. The `cartesian_force_bridge` turns the six values arriving
on `/om6dof/control_cmd` into signs only: `-1`, `0`, or `+1`. Its
`wrench_limits` then supplies the magnitude. Stick deflection and speed-scale
cannot increase force. The shipped Start force mode profile requests a
**1 N resultant translation force** (`force_newtons: 1.0`), including diagonal
directions. Rotational channels retain their independent 0.2 Nm limits.
Release removes the commanded wrench; a healthy active controller continues
its existing gravity compensation and bounded damping.

The dashboard's **Start force mode** starts from the measured supported pose.
It does not send READY in the current supervisor. The same activation checks
and gravity ramp apply. Start FORCE sets `force_native_current_limits: true`.
The driver reads each motor's stored Current Limit with torque off and exports
it as `dxlN/stored_current_limit`. FORCE refuses activation without a complete,
valid readback. It uses that native ceiling instead of the provisional
`max_effort`, `extra_torque_*` and URDF effort caps. Position impedance retains
its previous limits. No EEPROM Current Limit is changed.

The residual-torque headroom is the native ceiling converted to Nm minus the
magnitude of gravity compensation. A common wrench scale preserves direction
when a native ceiling is reached, and final raw current remains clamped to
that ceiling. A changed/invalid readback stops commands. Native-current FORCE
rejects position targets so the higher authority cannot drive a position spring.
Consequently 1 N is a request, not a guarantee that every pose/direction has
enough torque. Gravity model/current conversion
calibration must be verified on hardware before interpreting it as actual force.

The FORCE trial uses the URDF joint position bounds directly, with no extra
inward position margin. The dashboard precheck and effort controller use the
same zero-margin profile. Position impedance retains its configured margin;
speed, bus health, gravity capacity and current limits still apply to FORCE.
The target bridge reports `force_ready` separately from position-target
`ready`. FORCE activation and its readiness watchdog consume `force_ready`,
so the position planner's 0.05 rad inward margin cannot block an otherwise
healthy wrench controller. Fresh telemetry, exclusive effort ownership, a
live URDF, completed gravity ramp and zero controller faults remain required.

The wrench speed governor reduces the common wrench scale when a commanded
joint torque assists motion above 40% of its speed limit, reaching zero at
70%. The profile now uses ROBOTIS factory Velocity Limit defaults: 200 raw
for XM430-W350 (joints 1,2,3,5), 330 for XM430-W210 (joints 4,6), converted
with 0.229 rpm per count. These correspond to 4.7962 and 7.9137 rad/s.
The former 0.3 rad/s commissioning trip has been removed. These defaults
are not a readback of the motors' current EEPROM settings. Joint 4 and 6
URDF velocity bounds also match the W210 default; other joints retain 4.8.
See [W350](https://emanual.robotis.com/docs/en/dxl/x/xm430-w350/#velocity-limit44)
and [W210](https://emanual.robotis.com/docs/en/dxl/x/xm430-w210/#velocity-limit44).
In Mode 0, firmware controls current regardless of speed and position; the
software speed trip remains necessary independently of firmware current limits.
Gravity compensation and damping remain active; reversed commands can brake.
Over the same speed range, damping progressively uses up to the existing
residual-torque budget in either direction, including zero or stale wrench
input. This also brakes motion caused by external pushes against the command.
The stored firmware Current Limit, torque slew limits and gravity allocation
remain in effect.
There is no measured external-force threshold and no automatic switch to an
admittance controller. A low external force does not itself cause a fault.
This does not guarantee protection against external motion or model errors.
The hard speed trip is unchanged and faults still require supported recovery.

`/om6dof_position_impedance_controller/wrench_status` publishes 18 doubles:
`[tracking, combined_scale, requested_wrench[6], limited_wrench[6], ramp, 2, torque_scale, speed_scale]`.
Index 15 is the protocol version. The dashboard also accepts legacy version 1
messages and distinguishes torque and speed limiting. The displayed resultant
is a model estimate before damping and slew, not measured contact force.

## Mandatory commissioning

This is **commanded wrench**, not measured contact force. The arm has no F/T
sensor in this stack, and the current-to-torque conversion is not a certified
calibration. The bridge's code defaults are zero; the shipped 1 N profile is
an operator-selected experimental profile. Verify motor
polarity, torque calibration, gravity model, payload, soft limits, emergency
stop, and a supported activation pose.

Set matching reviewed values in both places:

1. `om6dof_position_impedance_controller.max_cartesian_wrench` and
   `enable_cartesian_wrench: true` in the Mode-0 impedance configuration.
2. `om6dof_controllers/config/cartesian_force.yaml` (`wrench_limits`).

The controller rejects components above its limits, bounds all joint residual
torques by its existing pose-dependent current budget, ramps force increases at
`residual_torque_rate`, drops stale-sign force immediately on release or
reversal, and falls back to gravity support plus damping when its 0.25-second
command watchdog expires. Simultaneous position-target and wrench streams
latch a fault rather than resolve ownership by last writer.

The impedance stack remains the activation authority. `FORCE` does not switch
motor mode, start hardware, activate a controller, or bypass the supported
pose, bus-health, speed, or joint-limit gates.
