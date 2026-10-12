# om6dof_controllers

Unified OM6DOF package: Python command conversion/kinematics and four C++
ros2_control controller plugins.

## High-level command converter

The former `om6dof_controller` package is merged here. Its Python modules are
imported as `om6dof_controllers.*`; launch files and dependencies now use this
package as well:

```bash
ros2 launch om6dof_controllers controller.launch.py
```

This is the normal position-stack command converter, not a torque-teaching
launch. The ROS node name stays `/om6dof_controller`, and its topics, READY
sequence, and `/om6dof_controller/get_parameters` service are unchanged so
existing dashboard and identification clients keep their runtime contract.
The full-stack launch in `om6dof_teleop` starts it automatically.

See [command converter details](docs/command_converter.md) and the
[combined reference preview](config/COMBINED_REFERENCE.md). Executables are
`controller_node` and `combined_preview_node`. Do not start the position
converter alongside the isolated Mode 0 torque-teaching stack.

## ros2_control plugins

| Plugin | What it does | Command interface |
|---|---|---|
| `om6dof_controllers/TrajectoryController` | follows joint trajectories | `position` or `effort` |
| `om6dof_controllers/LeaderArmController` | legacy Mode 5 lead-by-hand experiment | `position` + `effort` |
| `om6dof_controllers/GravityCompensationController` | caps the current from the weight model | `effort` |
| `om6dof_controllers/SpringActuatorController` | virtual spring-damper to a rest pose | `effort` |

The four entries above are controller_manager plugins, not Python nodes. In the normal/legacy profile,
`om6dof_bringup` owns the hardware and `ros2_control_node`; nothing here opens
U2D2 directly.

Combining the packages does not change controller ownership or the motor
control laws. The normal bringup still uses ROS's `JointTrajectoryController`
as `arm_controller`; merging packages does not select a different plugin.

> **Leader-arm status:** `LeaderArmController` in this package is retained as a
> historical Operating Mode 5 implementation. It is not the commissioned
> leader path. Physical testing showed that its remaining position loop could
> make the arm stiff and pull toward a setpoint. Use the separate
> `om6dof_leader_controller` package for Mode 0 signed-current gravity support
> and GUIDE. Do not run both U2D2 owners. See the
> [complete research and commissioning record](../docs/leader_arm_gravity_compensation.md).

## Read this before energising anything on the real arm

The `effort` command interface does not mean the same thing on both of this
repo's descriptions, and `GravityCompensationController` has to be told which
one it is talking to via `command_semantics`.

| `command_semantics` | What `effort` is | Dynamixel mode | Zero means |
|---|---|---|---|
| `torque` | a commanded torque | 0 (pure current) | neutral |
| `current_limit` | a ceiling on the servo's own position loop | 5 (current-based position) | **may pull nothing -- the arm drops** |

`om6dof_bringup/urdf/om6dof.ros2_control.current.xacro` configures **mode 5**,
so that is what the shipped config selects. Under it:

- The command is a **magnitude**. The sign of g(q) stops mattering, and the arm
  cannot go slack as gravity torque crosses zero.
- It is floored at `min_effort`, which must be greater than zero. A zero
  ceiling is a permanently slack joint; configure refuses both.
- It ramps **down** from `max_effort` to the computed limit, so activation goes
  from firmly held to compliant and never passes through limp. Measured on
  mock hardware: joint2 eases 496 -> 120, joint3 398 -> 218, monotonically.
- What the operator feels is set by `headroom`: the current the position loop
  gets on top of what merely holding the pose costs. Small is light, large is
  stiff. It is the first knob to tune.

`SpringActuatorController` and `TrajectoryController`'s effort mode still assume
a **torque** interface and have no `current_limit` equivalent, so they are not
correct against mode 5. `TrajectoryController` in its default `position` mode is
unaffected by any of this.

The former `om6dof_gravity_comp` Python package reached the same interface with
its own arming interlock and ramp. That package is currently removed from the
working tree; its datasets and method are historical inputs, not an active
runtime dependency. Never resurrect or run it alongside another effort/current
controller without a fresh safety review.

## Which description you need

`GravityCompensationController` and `SpringActuatorController` write efforts, so
they need the `effort` command interface, which only
`om6dof_bringup/urdf/om6dof.ros2_control.current.xacro` exposes. That file also
puts the servos in operating mode 5 (current-based position), which keeps the
position loop alive and merely caps how hard the servo may pull -- so a wrong
model makes the arm heavy or floaty, not falling.

`TrajectoryController` in its default `position` mode works against the standard
description and claims the same six position interfaces as `arm_controller` and
`forward_position_controller`; only one of the three can be active at a time.

## Gravity model

### V2 teaching calibration status (2026-10-06)

The temporary blanket motion lock was removed at the user's request after
replacing the old direct-ready path. The dashboard enables calibration when
its worker is idle; a running optional passive recorder does not block it.
Runtime READY configuration and fresh-feedback checks remain mandatory. This
code change does not certify bus reliability or validate the physical calibration.

The calibration implementation now requests `READY` on the live
controller's `operation_mode_topic`, exactly as **Go to READY** does. That
controller owns the sequence **current pose → TRANSITION → READY**, including
timing and offset reset. The runner no longer generates its own ready trajectory
or uses the old hardcoded target. It reads `transition_pose_degrees` and
`ready_pose_degrees` from `/om6dof_controller/get_parameters` and refuses missing
or invalid configuration without requesting motion. It must observe fresh
TRANSITION feedback first, then measured READY positions within 0.03 rad and
speeds below 0.03 rad/s for one second before recording/excitation. Missing
TRANSITION, timeout, missing feedback or failure to settle cancels calibration.
`--move-to-ready` remains for compatibility; `--ready-speed` no longer controls
the canonical controller's waypoint timing.
Preview remains non-actuating and checks the excitation around the ready pose.
The dashboard's excitation route uses this same runner and records its own CSV;
the legacy separate recorder is not required for that run. This does not certify
the path to ready as collision-free or automatically start a hardware stack.

The optional dashboard **Start recording** button now uses
`om6dof_controllers multisine_identification.py --record-only --rate 100`
with a unique output filename. This mode only subscribes to joint states and
does not send a trajectory. The rate is an upper bound; it cannot exceed the
incoming joint-state rate. Existing CSV files are never overwritten.

The dashboard **Fit model** button uses `--model v2 --auto-joints` and fits only
joints with sufficient bidirectional motion in the selected dataset. Its result
is `~/om6dof_identification/identified_v2_candidate.yaml`, separate from historical
fits. The result table shows Coulomb current, viscous current per rad/s, bias,
training RMSE and R². These current coefficients are not friction values in Nm.
**Check on another recording** evaluates saved coefficients without refitting;
it refuses the training CSV or a changed URDF and writes a separate evaluation
report. Neither fitting nor evaluation deploys coefficients to the controller.

For slow calibration recordings, the fitter now defaults to
`--velocity-source position`: an offline centered quadratic derivative of the
recorded positions against `ros_stamp_s`, using a 1 s window. It rejects
non-increasing timestamps, does not bridge gaps above 0.2 s, and discards
incomplete windows at segment boundaries. The CSV is unchanged. The dashboard
fits all retained samples (`--stride 1`) so decimation cannot discard the few
nonzero motor-velocity readings. `--velocity-source reported` remains available
for comparison. Saved candidates record preprocessing; evaluation reuses it.

Rank-deficient, ill-conditioned, one-direction-only, or negative-friction fits
are marked `fit_quality: rejected` with reasons shown on the dashboard. A fit
without those flags still requires independent validation. Smoothing cannot
prove that Coulomb and viscous friction are separately identifiable: compare
window/deadzone sensitivity and use a separate recording. Chronological
holdouts must be split before centered differentiation to prevent leakage.

#### Experimental combined six-joint multisine

The web monitor's **Preview ranges — all joints** and **Transition → Ready →
Move all 6 joints** use `teaching_combined`, with distinct three-frequency
patterns per joint. The last coefficient enforces zero initial velocity.
The web fixes amplitude/frequency scales to 0.5 and allows 30–120 seconds.
Each joint's absolute displacement bound is below 0.065 rad at these scales;
this is a small experimental profile, not a validated excitation design.

The server requires a successful preview of the same duration within ten
minutes, plus explicit path confirmation. Each execution consumes that
preview. The existing canonical TRANSITION → READY → settled-feedback gate
still applies, and the existing **Stop step** targets the same worker. Stop
step is not an emergency stop and the runner does not own the separate
canonical READY controller's goal. Keep the physical stopping procedure ready.
Joint-range preview does **not** check collisions, self-collisions or tool
clearance. Distinct frequencies do not prove parameter identifiability. All
six joints are recorded, but no fitted parameters are deployed automatically.
No web/hardware process is restarted by installing this code; the running
web monitor must be restarted separately, with operator approval, to show
the new buttons. Existing single-joint controls remain unchanged.

#### Goal-specific tracking and current/torque checks

Every new multisine `--execute` run automatically creates three files:

- `RUN.csv`: original joint-state schema, including raw Present Current.
- `RUN.csv.tracking.csv`: controller action feedback for that calibration goal,
  with goal UUID, source timestamp, receive time, desired/actual/error position
  and velocity. Missing fields are NaN, not artificial zeroes.
- `RUN.csv.tracking.json`: automatic diagnostic after a successful trajectory,
  reporting tracking RMSE/max error, timing-match coverage and data quality.

This also works for calibration launched by the existing web button; no new
motion button or separate passive recording is required. The runner logs a
`TRACKING_REPORT` summary for the dashboard log. TRANSITION → READY → settled
feedback remains mandatory. No controller/servo settings are changed. Passive
`--record-only` still records only joint states, not unrelated action feedback.
Aborted runs retain raw files but do not get an automatic success-path report.
Zero/missing action feedback is warned about and is not a successful tracking
measurement. Feedback rate is set by the controller, not the CSV rate cap.

Controller desired and actual values come from the same feedback message.
Motor current comes from the nearest JointState **source timestamp**, with a
25 ms maximum difference; out-of-window rows remain unmatched, and coverage
below 95% is flagged. This is not atomic hardware sampling. Nearest matching
may reuse a joint-state sample; do not count such rows as independent current
measurements. Duplicate/backwards/zero timestamps are rejected. Header stamps
must share a clock. Old CSVs cannot acquire missing target feedback retroactively.

Reanalyze offline or export the aligned samples (new output paths only):

```bash
ros2 run om6dof_controllers identification_tracking.py /path/to/RUN.csv \
  --output /path/to/NEW_tracking.json --aligned-output /path/to/NEW_aligned.csv
```

`check_current_torque.py` prepares a signed linear current-to-torque candidate
from independently supplied reference measurements, not from URDF predictions.
Use the header in `references/torque_reference_template.csv`. Each row is one
averaged reference observation with a unique `reference_id`, raw measured
current, measured velocity, and reference actuator joint torque in Nm. The
reference must include the total required joint torque (including gravity),
not just an added external load. Document the torque sign convention, method,
instrument/load uncertainty, pose and payload in `--reference-source`.
Static friction/hysteresis can still bias this fit; a good regression is not
physical validation. This tool does not specify or execute a loading procedure.

```bash
ros2 run om6dof_controllers check_current_torque.py TRAIN_REFERENCE.csv TEST_REFERENCE.csv \
  --joint joint3 --reference-source 'Describe independent measurement and sign convention' \
  --output NEW_current_torque.json
```

Each dataset requires at least six unique reference points, at least three
positive and three negative torques and four distinct torque levels; training
and test reference IDs must be disjoint. Nonfinite values, insufficient current
variation and speeds above 0.005 rad/s are rejected. Holdout predictions use
unchanged training coefficients. A negative gain is reported as candidate
polarity, not silently flipped. All results are **not deployable** and no
calibration is claimed without trustworthy external reference data. Do not
invent reference readings to make the checker pass.

#### Offline comparison with SI2017 Li/Takesue/Nakamura

`compare_paper_identification.py` compares `sign(velocity)` (paper equation 10)
against the existing tanh approximation, on identical moving samples. It fits
the training recording only and evaluates unchanged coefficients on a different
recording. Defaults are position-derived velocity, a 1 s window, a 0.005 rad/s
moving threshold, and tanh deadzone 0.005 rad/s. These are analysis settings,
not parameters stated in the paper. Neither the dashboard fitter nor its
defaults are changed by this comparison tool.

```bash
ros2 run om6dof_controllers compare_paper_identification.py TRAIN.csv TEST.csv \
  --model v2 --joint joint3 --output NEW_REPORT.yaml
```

Two gravity representations are compared: a single nominal-URDF scale, and
an analytic URDF regressor with per-link `[mass, mass*cx, mass*cy, mass*cz]`
columns. The latter includes fixed branches/payloads and reconstructs nominal
gravity when multiplied by URDF inertial parameters. It is a **full redundant
link basis**, not an identified minimal base-parameter model. Unit-column SVD
reports rank and nonunique coefficients; its minimum-norm output must not be
interpreted as physical masses or centers of mass. Unrecorded gripper joints
are held at zero and gravity points along base -Z. Runtime payload/orientation
and URDF identity at recording time are not established by the CSV.

All outputs remain `offline_diagnostic_not_deployable`. Current-domain fits
are independent per joint. A shared physical parameter fit as in the paper
requires known signed current-to-joint-torque gains; no such calibration is
inferred here. Negative friction is exposed, not clipped to pass a gate.
Existing output paths and identical training/validation CSV hashes are refused.

Source audit: `DynamixelHardware` maps ROS effort to Present Current;
`xm430_w350.model` and `xm430_w210.model` specify signed raw current with scale
1 and offset 0. The standard arm transmission matrix is identity. This is
not an Nm conversion or a physical polarity verification, and does not prove
which settings were active during a past recording. Gripper mA overrides
must not be applied to arm data. No hardware configuration is changed here.

The hardware wrapper currently includes `om6dof_v2.urdf.xacro`. The leader
controller still has the historical 2026-08-28 gains/scales, not validated on
V2. Torque teaching now uses nominal URDF gravity without fitted gains, with
approximate datasheet-based current conversion (see "URDF-only teaching
baseline" below); this is not hardware validation either. V2 changes joint3's
mounting transform and the wrist payload; selecting the V2 description alone
does not validate the fitted current coefficients. Link3 CoM and the camera
payload assumptions also need physical verification.

Fit only a newly recorded dataset from the intended physical configuration.
Select the model explicitly, for example:

```bash
ros2 run om6dof_controllers fit_gravity_friction.py /path/to/new_v2_run.csv \
  --model v2 --joints joint2 joint3 joint5 --output /path/to/v2_candidate.yaml
```

The output records the expanded URDF and input CSV SHA-256 hashes and is marked
`candidate_unvalidated`. This is an offline candidate, not automatic deployment
or a validation result. Do not relabel the August dataset as V2. Check current
units/polarity, independent held-out poses, residuals, and current saturation
before updating the controller. Keep friction/bias disabled until separately
validated. Collect identification under the position controller, before the
supported transition to pure-current teaching. The multisine preview only
checks joint limits; it does not check self-collision or workspace clearance.

The dashboard now checks for a local serial-port owner and an existing
`ros2_control_node` before spawning teaching. This check is not an atomic
device lock and does not protect direct CLI launches. A running launch process
is not proof that the teaching controller is active.

`g(q)` comes from the URDF via KDL (`kdl_parser` -> `KDL::ChainDynParam`), for
the chain `base_link` -> `tip_link`.

### Mass that hangs off the chain

`KDL::Tree::getChain` walks one path and drops every branch. On this arm that
meant the two gripper fingers and the wrist payload, all three bolted to link7,
all three invisible to the model: 0.114 kg at the far end of the longest lever,
about 28% of joint2's peak torque, gone without a word. The model now folds
every off-chain massive link into the nearest chain link it is rigidly attached
to, and reports what it folded at configure time:

    folded 0.1140 kg of off-chain mass into the gravity model:
    d405_payload_link, gripper_left_link, gripper_right_link

Movable joints on the way out to a branch are taken at zero, which for the
gripper fingers is a centimetre of travel and nothing next to being absent. There are no identified parameters inside
it: the link masses and inertias in the description are taken at face value,
and everything that the description cannot know is a controller parameter:

- `gain` (or `gravity_gain`) scales the model per joint,
- `friction.coulomb` / `friction.viscous` add a friction feed-forward, with
  `sign(qd)` softened to `tanh(qd / deadzone)` so it does not chatter at rest,
- `effort_scale` converts newton-metres into command-interface units,
- `max_effort` bounds the result no matter what the model says.

The former `om6dof_gravity_comp` package identified per-joint values from
measured current. Nothing here read its output automatically; selected values
were copied into this YAML by hand. The package is no longer present in the
working tree, so treat its files and plots as historical research artifacts,
not a runnable calibration dependency.

### The shipped numbers, and where they came from

The legacy mode-5 `om6dof_gravity_compensation_controller` block in
`config/om6dof_controllers.yaml` carries measured values, not datasheet ones,
taken from `~/om6dof_identification/identified_gravity_friction.yaml` (fitted
2026-08-20). That fit expresses everything in raw Dynamixel current ticks
(2.69 mA each) per unit of the KDL URDF-nominal gravity torque -- which is
exactly what this controller computes -- so its coefficients transfer directly
into `effort_scale`.

| Joint | effort_scale | why |
|---|---|---|
| joint2 | 285.4 | measured, r2 0.93 -- **stale, see below** |
| joint3 | 384.7 | measured, r2 0.93 -- **stale, see below** |
| joint1, 4, 5, 6 | 1.0, `gain: 0.0` | not compensated -- see below |

Do not compare these raw-tick coefficients directly with the new leader
controller's provisional mA/N·m values. The interface units and controller
semantics differ, and stall ratios are not a substitute for a measured fit.

**These scales no longer match the model.** They were fitted against a `g(q)`
that was missing 0.114 kg of off-chain mass and had placeholder centres of mass
on link3, link4 and link5. Both are fixed now, and the model's output moved a
long way: at the ready pose joint3 went from -0.306 to -0.531 Nm and joint5 from
-0.093 to -0.199 Nm. A fitted scale absorbs magnitude error, so the old numbers
were partly compensating for the old model's absence of mass. Re-run the
identification through a new, versioned pipeline before trusting these on
hardware, and watch whether `gravity_correlation` on joint2 improves on 0.87 --
that number tests whether the model's *shape* actually got better.

Joints 1, 4 and 6 are yaw/roll axes: their gravity torque comes out around
1e-7 Nm at any pose, so there is nothing to compensate and the identification
could not fit them for want of signal. Joint 5 fitted with a gravity
correlation of -0.13 and is flagged `gravity_identifiable: false`. All four run
at `gain: 0.0` -- deliberately not compensated, rather than compensated with a
number nobody measured.

`viscous` came out negative on every joint in that fit, which would mean
friction adding energy instead of removing it. It stays at zero; only
`gravity_nominal` and `coulomb` are transcribed.

### deactivate_effort

Written to every joint when the controller deactivates, and when joint state
reads back non-finite. Left unset it follows `command_semantics`: zero under
`torque`, and `max_effort` under `current_limit` -- the fail-safe direction,
handing the joints back fully able to hold themselves.

### bias

Only meaningful under `torque` semantics; a signed offset has no meaning for a
magnitude limit, and the controller warns and ignores it under `current_limit`.

The identification also fitted a constant per-joint current offset (-40.7 ticks
on joint2, +24.7 on joint3) that it could attribute to neither gravity nor
friction. `bias` is where that goes. It is in command-interface units and is
added after `effort_scale`, before `max_effort` clamps.

It ships at zero. A constant current on a joint standing still makes that joint
creep in one direction, so applying it is a decision, not a default. Set it only
when reproducing the fitted model exactly, and watch the arm hold still
afterwards.

### First run on real hardware

The historical coefficients in this package predate gravity-model corrections
and must not be treated as a current first-run recipe. The former comparison
node has been removed from the working tree, and the Mode 5 leader behavior was
superseded by the isolated Mode 0 package.

If this legacy controller is studied again, keep all gains at zero, support the
arm, verify model sign independently, use conservative current limits, and
collect new pose/current/temperature data. Do not copy the old 340-tick J2
observation into the Mode 0 controller: the legacy package uses raw ticks and a
current ceiling, whereas the commissioned leader interface uses signed mA.

## Getting the robot description

Humble's controller_manager does not push `robot_description` down to
controllers. Each controller here first looks at its own `robot_description`
parameter and, if that is empty, waits up to `robot_description_timeout` seconds
for a latched message on `robot_description_topic` (default `/robot_description`,
which `robot_state_publisher` already publishes transient-local). The wait runs
on a throwaway node, never on the controller's own, whose executor belongs to
controller_manager.

`TrajectoryController` only needs the description when `gravity_feedforward` is
on, and `SpringActuatorController` only when `gravity_compensation` is on.

## Starting

With `om6dof_bringup` already up:

```bash
ros2 launch om6dof_controllers om6dof_controllers.launch.py
```

Pick which ones to load, and whether to activate them:

```bash
ros2 launch om6dof_controllers om6dof_controllers.launch.py controllers:="om6dof_gravity_compensation_controller om6dof_spring_actuator_controller" activate:=false
```

Or load one by hand. Note `--controller-type`: the manager looks a controller's
type up in its *own* parameters, and `om6dof_bringup` starts it knowing only its
own `controllers.yaml`. `--param-file` does not cover this -- that lands on the
controller's node, not on the manager -- so without the type the load fails with
"The 'type' param was not defined". The launch file above passes it for you.

```bash
ros2 run controller_manager spawner om6dof_gravity_compensation_controller -c /controller_manager --controller-type om6dof_controllers/GravityCompensationController --param-file $(ros2 pkg prefix om6dof_controllers)/share/om6dof_controllers/config/om6dof_controllers.yaml
```

The alternative is to declare the three types in `om6dof_bringup`'s
`controllers.yaml`, which this package deliberately does not do to itself.

The effort controllers claim the same six `effort` interfaces, so they cannot be
active together; switch between them.

## TrajectoryController

Trajectories arrive two ways:

- `~/follow_joint_trajectory` (`control_msgs/action/FollowJointTrajectory`) --
  what MoveIt uses, and the only path that reports back.
- `~/joint_trajectory` (`trajectory_msgs/msg/JointTrajectory`) -- fire and
  forget, for scripting and bring-up.

A trajectory may list its joints in any order but must name all six. Points are
reordered into the controller's joint order once, outside the update loop. A
zero `header.stamp` means "start now"; any other stamp is honoured.

Sampling is cubic Hermite where both ends of a segment carry velocities and
linear otherwise. Point accelerations are ignored -- resample more densely
upstream if they matter. Whatever the trajectory's first point says, the first
segment starts from the measured position, so accepting a trajectory never steps
the arm.

State goes out on `~/controller_state`
(`control_msgs/msg/JointTrajectoryControllerState`) at `state_publish_rate`.

### Tolerances

- `constraints.<joint>.trajectory` -- violated mid-motion, the goal aborts with
  `PATH_TOLERANCE_VIOLATED`.
- `constraints.<joint>.goal` plus `constraints.stopped_velocity_tolerance` --
  checked from the end of the trajectory on.
- `constraints.goal_time` -- how long past the end the arm may take to settle.
  **Zero means no deadline**, so a goal that never settles never finishes; set
  it to something on hardware.

Zero on any per-joint tolerance means that check is off. Tolerances named in the
goal message override the parameters, per the action's own contract.

A new trajectory preempts whatever is running: the old goal is finished as
aborted with `error_string` saying so.

### Effort mode

`command_interface: effort` closes the loop here instead of in the servo:

    tau = Kp (q_des - q) + Kd (qd_des - qd) [+ gravity feed-forward]

`gains.p` must be non-zero somewhere or the controller refuses to configure.
`gravity_feedforward: true` is ignored (with a warning) in position mode, where
the servo's own loop already deals with gravity.

## LeaderArmController

### Legacy Mode 5 experiment — not the current leader implementation

This section documents why the historical controller was built and how it
worked. It must not be read as the recommended procedure for the physical
leader arm. The commissioned path is the companion
`om6dof_leader_controller`, which uses Operating Mode 0, explicit ARM and GUIDE
deadman, signed current, saturation, slew limiting, and zero-before-enable
interlocks.

Gravity compensation alone did not give lead-by-hand behavior while a separate
position controller retained a fixed setpoint, which motivated the combined
legacy controller below.

`arm_controller` holds one fixed position setpoint. It owns `jointN/position`,
so it decides where the arm goes; a controller that only writes `jointN/effort`
merely decides how *hard* it may go there. Push the arm and you are fighting
the servo pulling back to that stale setpoint; let go and it springs back. No
amount of current-limit tuning fixes that, and lowering the limit far enough to
make the fight winnable just means the arm can no longer hold itself up and
folds.

So this controller claims **both** interfaces and replaces `arm_controller`
while active. Switch between them; they cannot be stacked -- trying to activate
it while `arm_controller` holds the position interfaces fails with a resource
conflict, which is the manager doing its job.

Load it first (the launch file leaves it configured), then switch atomically, so
there is never a moment when nothing owns the arm:

```bash
ros2 launch om6dof_controllers om6dof_controllers.launch.py controllers:="om6dof_leader_arm_controller" activate:=false
```

```bash
ros2 control switch_controllers --deactivate arm_controller --activate om6dof_leader_arm_controller
```

To hand the legacy profile back to MoveIt, switch the other way. This operation
does not convert a Mode 5 stack into the Mode 0 leader profile; changing
Dynamixel operating mode requires a safe hardware restart with torque OFF.

Each cycle it writes:

- **position** -- the setpoint, dragged along so it never sits further than
  `setpoint_deadband` from where the arm actually is. Gravity pulls the arm to
  the edge of that band and the servo holds it there; a hand moving the arm
  pushes the band along, so release leaves nothing to spring back to.
- **effort** -- the current ceiling, from the same measured gravity model
  `GravityCompensationController` uses.

### setpoint_deadband is the knob for how heavy it feels

Bounding the error is what makes this light. The position loop never sees more
than `setpoint_deadband`, so the force it pushes back with is bounded and
smooth.

An earlier version gated on measured velocity instead -- freeze when still,
track when moving -- and it was worse in two ways. It had to guess whether
motion came from a hand or from gravity. And while still, error built up
unopposed until the joint broke away, so the shoulder felt stuck: the peak force
you had to overcome was the whole current ceiling, not the holding force. The
band removes both problems, and makes the velocity signal's quantisation
(0.02397 rad/s on an XM430, often one or two counts on a joint standing still)
irrelevant.

Too small a band and the loop cannot generate enough current to carry the arm:
it sinks, the band follows it down, and the arm walks to the floor. Too large
and it droops noticeably before catching, and pushes back harder. Size it up
from the shipped 0.03 rad until the arm holds still when released, then stop.

### Why max_effort is not a comfort knob here

The band follows a sagging arm as readily as a pushed one, so the arm holds only
while the servo can carry it at `setpoint_deadband` of error. Drop the ceiling
below what holding costs and it cannot, the band follows the sag down, and the
arm walks itself to the floor. Configure refuses a non-positive ceiling, and
update() warns whenever the cap binds.

Tune the feel with `setpoint_deadband`, then `headroom`. Never with
`max_effort`.

### Driving a follower

`~/lead` publishes `sensor_msgs/msg/JointState` with the leader's measured
position and velocity, at `publish_rate`.

## GravityCompensationController

Commands the effort that cancels gravity and holds no position setpoint at all,
which is exactly what makes the arm leadable by hand. Output ramps in over
`ramp_seconds` on activation and is set to zero on deactivation, so switching
controllers does not step the current.

The model's raw output, in newton-metres and before `gain`, `effort_scale` and
`max_effort`, is published on `~/gravity_torque` at `publish_rate` -- that is
the topic to watch while trimming the gains.

### URDF-only teaching baseline

#### Control and diagnostic rates

Torque teaching explicitly sets `update_rate: 100` Hz, matching the hardware
controller manager's 100 Hz read/update/write loop, and `publish_rate: 100.0`.
Effort command interfaces are written on every control update regardless of
whether diagnostics are published. With publication rate at or above the
controller rate, `commanded_effort` and `gravity_torque` each get one
**nonblocking publish attempt per update**. This avoids the old timestamp gate
skipping a cycle when a nominal 10 ms interval is slightly short due to jitter.
It does not increase servo firmware's internal loop rate or command faster than
controller_manager, nor add catch-up bursts after a delay.

A requested publication rate below the controller rate keeps timestamp gating;
zero disables diagnostics. Busy realtime publishers, DDS or subscriber load can
still drop/delay diagnostics: 100 Hz is a target, not a hard delivery guarantee.
Publication failure never blocks the motor-command path. Changes require a
safely supported controller reload; changing `publish_rate` live is rejected.
The full-rate behavior requires the updated binary, not just YAML. Verify
actual topic frequency again after deployment; offline timing tests are not a
measurement on the running robot.

#### Physical parameter sources

The shipped `om6dof_torque_teaching_controller` block uses the URDF published
on `/robot_description` as its sole source of link masses, CoMs and joint
geometry. Normal hardware bringup expands
`om6dof_description/urdf/om6dof_v2.urdf.xacro`. To correct a measured CoM, edit
the appropriate link's `<inertial><origin xyz="...">` there, not a joint origin
or visual origin. No physical URDF values were changed in this configuration
cleanup. Incorrect inertial values still require measurement/correction.

YAML keeps the joint/controller setup, base/tip selection, installation gravity
vector, motor Nm-to-raw-current conversion, current limits, startup ramp and
diagnostic publish rate. It no longer supplies `com_offsets`, `gain`, physical
friction coefficients or `bias` for torque teaching. Tested defaults are no CoM
overlay, gain **1 on all six joints**, and bias **0**. In particular J1/J4/J6
are no longer gravity-disabled; the solver determines their torque and the
existing current caps still apply.

Teaching selects `friction.source: urdf`: each controlled joint's
`<dynamics friction="C" damping="B">` supplies Coulomb and viscous coefficients.
For revolute joints these are Nm and Nm*s/rad respectively. Missing dynamics or
missing attributes mean zero; negative/nonfinite coefficients, unknown joints,
unsupported joint types and duplicate names fail configuration. Explicit YAML
`friction.coulomb` / `friction.viscous` arrays are rejected in URDF mode rather
than silently overriding the source. No URDF physical values are changed.

`friction.deadzone: 0.05` (rad/s) remains a controller smoothing setting, not a
measured physical coefficient. The output before the existing ramp/current cap
is `effort_scale * (g_URDF(q) + C*tanh(v/deadzone) + B*v)`. The friction terms
**assist motion to compensate mechanical resistance**, including viscous
resistance described by URDF damping. They are not artificial braking `-D*v`.
There is no static breakaway kick at zero velocity, no double application of
URDF damping by KDL gravity, and no inertia/Coriolis feed-forward. Runtime uses
measured servo velocity, so quantization can affect the current command.

Configure logs each joint's effective C/B/smoothing. Model/friction changes via
parameter services are rejected after initialization to avoid misleading or
sudden live torque changes. To return to gravity-only teaching set
`friction.source: parameters` with no coefficient arrays and reload safely.
The gravity-only pre-change YAML is archived at
`calibration_reports/urdf_friction_teaching/baseline_controllers.yaml`.
Other legacy controller blocks keep their parameter-based friction behavior.

The URDF numbers may be simulation defaults, not measured friction of this arm;
reading them successfully does not validate them or solve a wrong-CoM drift.
Conversion remains the approximate nominal-12-V datasheet conversion, not a verified
torque calibration. The previous YAML is preserved at
`calibration_reports/urdf_only_teaching/baseline_controllers.yaml`. Other
controller blocks and previous calibration artifacts have not been changed.

Editing YAML/URDF does **not** update a running solver. YAML must be reloaded;
URDF changes also require the robot-description publisher to reload the xacro
and the controller to rebuild its model. Physically support the arm before any
stop/restart: mode 0 has no position hold and deactivation writes zero current.
Unlike the earlier gravity-only YAML cleanup, reading URDF joint dynamics
**requires installing the newly compiled controller binary**. An old binary
does not implement `friction.source` and must not be assumed to apply it just
because YAML has been edited. Build/test in separate directories while hardware
may be running; do not overwrite a loaded controller shared library.

### Optional CoM-overlay capability (not used by the shipped teaching YAML)

`om6dof_torque_teaching_controller.ros__parameters.com_offsets` in
an experimental controller configuration adds an XYZ displacement to each
named link's original URDF inertial origin, **in metres along the link frame axes**:

```yaml
com_offsets:
  links: [link3, link7]
  xyz_m: [
    0.0, 0.0, 0.0,  # link3: dx, dy, dz
    0.0, 0.0, 0.0   # link7: dx, dy, dz
  ]
```

For example, changing link3's triple to `[0.001, 0.0, 0.0]` means an additive
**1 mm along link3 +X**, not +X of the world, not necessarily "forward", and
not a 1 mm change to the joint origin. This is a syntax example, not a
recommended correction. One link's CoM can affect several upstream joints.
Offsets are always relative to a fresh URDF copy, so reloads do not accumulate
them. All zeros restore the baseline; the URDF file is never overwritten.

The shipped teaching YAML omits these parameters, so no overlay is applied.
The code and tests are retained for historical experiments, not as a second
source of CoM values for the URDF-only workflow. This feature changes only this
controller's gravity model, including off-chain gripper/camera masses before folding.
It does not change masses, mesh poses, joint zero offsets, robot_description,
MoveIt/RViz/web kinematics, friction, gains, effort scales, or current limits.
The leader and spring controller configurations are unaffected. As before,
off-chain moving joints are approximated at zero by the gravity model.

Each entry needs exactly three finite numbers, a unique existing massive link
that contributes to the selected chain, and a displacement norm <= 0.05 m.
Invalid input fails configuration. The 5 cm bound only catches gross unit
mistakes; it is **not** a safety envelope. Configure logs the original, delta
and effective CoM for each entry in link-local metres.

These parameters are **startup-only**. `ros2 param set` is explicitly rejected
instead of reporting success while leaving the solver unchanged. Editing YAML
also does not alter a running controller: a controlled unload/reload (or launch
restart) is needed. Before any stop/reload, physically support the arm safely:
mode 0 has no position hold and deactivation writes zero current. Never adjust
by repeatedly allowing an unsupported arm to drift. Do not increase current
limits to hide a model mismatch. Verify against several poses/load measurements,
not just the pose where it happens to balance.

Build the changed C++ package only when the hardware stack is safely stopped,
or use separate build/install directories for offline validation. A new shared
library must not overwrite one mapped by a running hardware process. The YAML
alone cannot add this feature to an old controller binary. Existing offline
identification scripts still use nominal URDF CoMs and do **not** consume this
overlay; retain the offset YAML with any experiment and do not call such fits
validation of the adjusted model.

## SpringActuatorController

    tau = K (q_rest - q) - D qd [+ g(q)]

With `gravity_compensation: true` the spring is the only thing the operator
feels. With it false, the arm settles where the spring balances its own weight.
Negative stiffness or damping is rejected at configure time.

The rest pose starts at the measured position when `capture_rest_on_activate` is
true, so activation is bump-free, and then slews towards `rest_position` (if
set) at `rest_slew_rate` rad/s. Publishing `std_msgs/msg/Float64MultiArray` on
`~/rest_position` moves it afterwards; the same slew limit applies, so a jump in
the reference cannot become a jump in torque. Commanded effort is echoed on
`~/commanded_effort`.

## Parameters

Every per-joint list accepts an empty list (use the default everywhere), a
single value (broadcast to all joints), or one value per joint. Any other length
is a configure-time error rather than something padded silently.

Shared by all three: `joints`, `base_link`, `tip_link`, `gravity`,
`effort_scale`, `max_effort`, `robot_description`, `robot_description_topic`,
`robot_description_timeout`. `GravityCompensationController` adds `bias`. See `config/om6dof_controllers.yaml` for the rest,
with the OM6DOF values filled in.

## Tests

```bash
colcon test --packages-select om6dof_controllers && colcon test-result --verbose
```

The gtest suites cover the pieces that hold the arithmetic -- trajectory
interpolation, the KDL gravity chain against a hand-computed two-link case, and
per-joint parameter expansion. Configuration tests also load the actual teaching
YAML, check URDF-only defaults and motor limits, and reject runtime CoM edits.
Hardware activation/switching and realtime paths are not covered; those need a
controller_manager fixture and are still to do.
