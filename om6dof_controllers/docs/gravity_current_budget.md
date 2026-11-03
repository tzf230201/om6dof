# Mode-5 pose-dependent Goal Current

Experimental, opt-in. This is a magnitude ceiling for the Dynamixel position
loop, not signed gravity feed-forward, obstacle detection, or full admittance.
The normal position controller and Target mode continue owning position.

At the controller-manager rate (100 Hz in the bringup configuration):

```text
support = gravity_scale * abs(g(q_measured)) * raw_per_nm + holding_current_raw
motion  = velocity_current_gain * abs(v_reference)
        + acceleration_current_gain * abs(a_reference)
cap     = min(max_current_raw, stored Current Limit, URDF effort limit * raw_per_nm)
request = min(cap, startup_ramp * (support + motion))
raw     = floor(slew_limit(request))        # cap overrides slew immediately
```

Reference motion comes from named joints in the fresh trajectory-controller
state (reference, or legacy desired). Measured speed and position error never
increase the motion reserve: being pushed or blocked does not wind up a growing
current request. These coefficients provide headroom for the motor's existing
position loop; they are not an identified inverse-dynamics model.

The live startup `/robot_description` supplies the V2 geometry, masses and CoMs.
The six `raw_per_nm` startup parameters use the same datasheet-derived XM430
conversion estimates as teaching; they are not measured torque calibration.
The model is captured on configure; editing a Xacro does not hot-reload it.

The old gravity-only ceiling could be zero on a vertical axis and left no
headroom for stiction, model error, acceleration or servo tracking. Mode 0
teaching sends signed current directly; Mode 5 only limits the motor's internal
position servo. Equating those two commands does not guarantee equal holding
or tracking behavior.

## Commissioning parameters

Edit the `gravity_current_budget_controller.ros__parameters` block under `/**`
in `om6dof_bringup/config/controllers.yaml`. The initial candidate values are:

| Parameter | J1 | J2 | J3 | J4 | J5 | J6 |
|---|---:|---:|---:|---:|---:|---:|
| `holding_current_raw` | 10 | 20 | 20 | 8 | 12 | 8 |
| `velocity_current_gain`, raw/(rad/s) | 40 | 120 | 100 | 40 | 80 | 40 |
| `acceleration_current_gain`, raw/(rad/s²) | 5 | 20 | 15 | 5 | 10 | 5 |
| `max_current_raw` | 30 | 360 | 260 | 30 | 220 | 30 |
| `current_slew_raw_per_second` | 40 | 150 | 150 | 40 | 100 | 40 |

`gravity_scale` is 1.2: 20% gravity headroom, in addition to the fixed holding
reserve. For example, with zero gravity torque, J1 still receives 10 raw holding
headroom. At a reference speed of 0.1 rad/s and acceleration of 0.5 rad/s², its
request becomes 16.5 raw before ramp/slew/quantization, below its 30-raw cap.

These are **unvalidated commissioning values**, not friction measurements or
proof that the robot can hold every pose. The per-joint caps match the existing
teaching caps; the verified EEPROM and URDF limits can reduce them further.
The current V2 URDF specifies 1 Nm for each arm joint: with the nominal
conversion this also limits W350 joints to about 208 raw, even when YAML permits
260 or 360. Activation logs report all three ceilings and the effective cap.
Tune static holding first with physical support, then use small, slow targets
and inspect tracking/current saturation. If a joint sags at its cap, diagnose
model/conversion/cap adequacy before increasing it. Larger reserves increase
available contact force. This mechanism alone does not implement admittance.

All parameters above are read on configure. Parameter edits require a supported
controller reload/restart; a live `ros2 param set` does not apply them. The plugin
defaults keep margins at zero and gravity_scale at 1.0; the bringup YAML enables
the commissioning reserve only when this opt-in controller is launched. To
recover the gravity-only calculation, set scale to 1.0 and the three holding,
velocity and acceleration arrays to zeros. Slew limiting still applies.

## What is preserved

- The normal Mode-5 PID configuration is unchanged. In this budget profile the
  driver also skips ALL gain writes and does not rewrite Operating Mode.
  Motors must already read back Mode 5; otherwise startup fails.
- EEPROM Current Limit is read and verified, never changed. Driver writes
  clamp to each verified motor limit even if a controller requests more.
- Gripper current remains 45 raw; no gripper changes.
- Existing normal/teaching/impedance profiles are unchanged with the new flag off.
- Generic services cannot override mode, Goal Current, Current Limit, watchdog,
  or PID while this profile owns the hardware. No second serial client allowed.

## Startup and faults

Physically support the arm throughout startup and first tests. The new profile
starts with Goal Current ZERO, then ramps the requested budget over one second.
The slew limit can extend the time needed to reach that budget. It does NOT
move through TRANSITION/READY, send a new target, or guarantee position holding.

Motion reserve requires fresh trajectory state, at most 0.25 seconds old by
both message stamp on reception and monotonic receipt age during updates.
Missing accelerations mean zero acceleration reserve. Missing velocity,
duplicate/missing joints, nonfinite data, old/future stamps or a silent topic
remove the motion request, while static support remains. Reserve changes use
the same slew limit, so removal of motion reserve is gradual. Pre-activation
reference data is discarded. A reference that keeps requesting motion while
blocked can still use its bounded reserve; this is not obstacle detection.

Missing/failed encoder read, a control gap over 100 ms, invalid joint position,
invalid runtime Current Limit, or gravity solver failure latches zero budgets.
Deactivation also writes zero;
there is NO full-current fallback. Zero current can let the arm fall: support
before stopping or recovering. Existing driver bus watchdog/fault shutdowns
still apply. A failed bus write cannot guarantee that a zero command reached
the motors. Recovery is explicit; cancel old position trajectories before
reactivation so they cannot resume unexpectedly.

## Deliberate activation, after deployment and support confirmation

Never run this beside an existing hardware owner. Stop the old stack only with
the arm supported, deploy both the new driver and controller, then use:

```bash
ros2 launch om6dof_teleop full_stack.launch.py \
  current_control:=false gravity_current_budget:=true start_go2w_teleop:=false
```

Do not use `current_control:=true`: that is the separate effort-mode profile.
The new flag defaults to false, including on an ordinary service restart.

The controller owns only `joint1..joint6/effort`, interpreted as raw positive
Goal Current ceilings in this Mode-5 profile. Values being commanded, ordered
J1 through J6, are published at about 10 Hz:

```bash
ros2 topic echo /gravity_current_budget_controller/goal_current_raw
```

These are calculated commands, not register readback or measured motor current.
2.69 mA corresponds to one raw tick on these XM430 motors. Controller state
interfaces `dxlN/stored_current_limit` contain verified startup EEPROM readback.
