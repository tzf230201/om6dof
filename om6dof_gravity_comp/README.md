# om6dof_gravity_comp

Experimental identification of the gravity and friction currents of the
OM6DOF arm, and the online estimator that uses the result.

Everything here is **read-only with respect to the motors**. Nothing in this
package commands current, and nothing changes the Dynamixel operating mode.

---

## 1. Theory

At the speeds a person moves an arm by hand, inertia and Coriolis terms are
small, so the joint model collapses to

    tau_m  ≈  g(q) + r(qd)

Working in the servo's own current units instead of newton-metres:

    I_measured  =  I_gravity(q) + I_friction(qd) + I_external

and once the first two are known, whatever is left is the third:

    I_external  =  I_measured − I_model(q, qd)

Current rather than torque is deliberate. The effective torque constant of
these servos is not calibrated, and converting first would inject that
unknown scale into the data. Fitting in current lets the scale land inside
the identified parameters instead.

### The fitted model, per joint

    I_i  =  a_i · tau_g_nominal_i(q)     gravity shape from the URDF, via KDL
          + b_i · sign(qd_i)             Coulomb friction
          + c_i · qd_i                   viscous friction
          + d_i                          bias

`a_i` absorbs the current-to-torque constant, the gearing, and any uniform
error in the link masses. It **cannot** fix a wrong mass *distribution* — and
an `a_i` far from 1 is itself evidence that the distribution is wrong, which
is a useful diagnostic rather than a failure.

Four parameters per joint, six joints, so **24 in this regressor**. That
number comes from the implementation, not from any paper.

### Coulomb friction near zero velocity

`sign(qd)` is discontinuous at zero, and near zero the measured velocity is
mostly quantisation noise, so its sign is close to random. Two options:

| mode | what it does | why you'd pick it |
|---|---|---|
| `smooth` (default) | `tanh(qd / deadzone)` | keeps every sample, including the slow ones where gravity dominates; the Coulomb term fades to zero instead of chattering |
| `exclude` | drops samples with `abs(qd) < deadzone` | truer to Coulomb friction being undefined at rest, but throws away the low-speed data |

The percentage removed is reported either way. The online estimator uses the
same feature the fit used — mixing them would be wrong at low speed with
nothing to flag it.

---

## 2. Units in this repository

Verified from the repository, not assumed:

| signal | unit | source |
|---|---|---|
| `/joint_states.effort`, joints 1–6 | **raw Dynamixel current ticks** | `xm430_w350.model` / `xm430_w210.model`: `Present Current  1.0  raw` |
| `/joint_states.effort`, gripper | **milliamps** | per-device override in `om6dof.ros2_control.xacro`: `Present Current,2.69,mA` |
| `/joint_states.velocity` | rad/s | `Present Velocity  0.0239691227  rad/s` |
| tick size | 1 tick = **2.69 mA** | the same override, whose comment reads "45 raw ticks * 2.69 mA/tick = 121.05 mA" |

`dynamixel_hardware_interface/dxl_state` carries only `comm_state`, `id`,
`torque_state` and `dxl_hw_state` — **no current**, so `/joint_states` is the
only source.

Servo per joint, from the table in `om6dof.ros2_control.xacro`:

    joint1 XM430-W350   joint4 XM430-W210
    joint2 XM430-W350   joint5 XM430-W350
    joint3 XM430-W350   joint6 XM430-W210

Joint axes: J1 Z, J2 Y, J3 Y, J4 Z, J5 Y, J6 Z. Only the Y axes carry gravity
load, so joints 1, 4 and 6 have no gravity term to identify.

---

## 3. Collecting data

The logger never commands the arm, so it is safe to start at any time.

    ros2 run om6dof_gravity_comp identification_logger --rate 100

It writes `~/om6dof_identification/identification_<timestamp>.csv` with a
metadata header (joint order, current unit, tick size, sample rate, URDF
source, commit) and one row per sample: wall and ROS time, mode, q, qd, raw
current, and mA.

---

## 4. Dry-run excitation

Always look at the plan before letting it move anything.

    ros2 run om6dof_gravity_comp excitation --joint joint2 --duration 60

This prints the position range, the conservative band, the hard limits, and
the peak velocity and acceleration, then stops. It moves nothing without
`--execute`, and refuses outright if any limit would be crossed.

To actually run it, the arm must be in **AUTONOMOUS** so `arm_controller`
owns the joints — the tool checks, and refuses if `forward_position_controller`
is still active, because two writers on the same joints is not something to
discover halfway through a trajectory.

    ros2 run om6dof_gravity_comp excitation --joint joint2 --duration 60 --execute

Start the logger first, in another terminal.

---

## 5. Identification

    ros2 run om6dof_gravity_comp identify ~/om6dof_identification/identification_XXXX.csv \
        --plots ~/om6dof_identification/plots \
        --compare-ridge

Prints per-joint coefficients, RMSE, MAE, R², max absolute error, sample
counts and the deadzone percentage, for both ordinary least squares and
ridge. Writes `config/identified_gravity_friction.yaml` with the dataset
name, fitting date, model version, current unit and joint order.

Train and validation are split **by time**, never at random: neighbouring
samples at 100 Hz are near-copies, and a random split reports an error far
below what the model would show on a fresh run.

---

## 6. Validation on a second dataset

The split above is a held-out tail of the *same* run. A separately recorded
dataset is the honest test:

    ros2 run om6dof_gravity_comp evaluate \
        config/identified_gravity_friction.yaml \
        ~/om6dof_identification/identification_YYYY.csv

---

## 7. Online estimator

    ros2 run om6dof_gravity_comp current_estimator --ros-args \
        -p model_file:=/absolute/path/to/identified_gravity_friction.yaml

Publishes six-element arrays:

| topic | meaning |
|---|---|
| `/om6dof/current_model` | what the model expects to see |
| `/om6dof/current_residual` | measured − model |
| `/om6dof/gravity_component` | the gravity term, bias included |
| `/om6dof/friction_component` | the Coulomb and viscous terms |

It refuses to start if the model's units or joint order disagree with what it
reads.

---

## 8. Reading the residual

A residual near zero means the model accounts for what the motors are doing.
A sustained non-zero residual on one joint, at low speed, is the signature of
something pushing the arm.

Two things it is **not**:

- **It is not newton-metres.** It is raw current ticks. Converting needs a
  calibrated `Kt`, which this arm does not have.
- **It is not trustworthy during fast motion.** The whole model assumes
  inertia and Coriolis terms are negligible, which stops being true as speed
  rises.

Later, once `Kt` is calibrated and the model's accuracy is good enough:

    tau_external = Kt · I_external
    F_external   = J^{-T} · tau_external

Neither step is implemented, and neither should be until the residual on a
second dataset is small enough to be worth converting.

---

## 9. Baseline: the nominal KDL model

`gravity_model.py` and `gravity_comp_node` remain, unchanged, as the physics
baseline. Comparing the two answers whether the experimental identification
actually earns its keep:

    ros2 run om6dof_gravity_comp gravity_comp_node     # nominal, from the URDF
    ros2 run om6dof_gravity_comp current_estimator ... # identified

---

## 10. Not implemented, on purpose

Commanding current. Before that is ever switched on it needs: an explicit
current operating mode, per-motor current saturation, a global scale starting
well below 1.0, ramp up and down, a watchdog, a communication-failure
shutdown, an emergency zero-current path, joint-limit and velocity
protection, command sanity checks, and hardware error monitoring.

Nothing in this package changes a Dynamixel operating mode.
