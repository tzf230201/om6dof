# Cartesian wrench mode

`FORCE` is an opt-in, Mode-0 effort-stack feature. It maps a requested TCP
wrench in the **URDF/KDL base frame** into joint torque using virtual work:

```text
tau = J(q)^T [Fx, Fy, Fz, Mx, My, Mz]
```

The components are N, N, N, Nm, Nm, Nm. `Mx`, `My`, and `Mz` rotate about the
base-frame X, Y, and Z axes; they are not gripper-relative axes.

The regular position-offset controller never interprets `FORCE` as motion. It
zeros its output. The `cartesian_force_bridge` turns the six values arriving
on `/om6dof/control_cmd` into signs only: `-1`, `0`, or `+1`. Its
`wrench_limits` then supplies the magnitude. Stick deflection and speed-scale
cannot increase force.

## Mandatory commissioning

This is **commanded wrench**, not measured contact force. The arm has no F/T
sensor in this stack, and the current-to-torque conversion is not a certified
calibration. Keep both defaults at zero until an operator has verified motor
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
