# om6dof_controllers

Three ros2_control controller plugins for the OM6DOF arm:

| Plugin | What it does | Command interface |
|---|---|---|
| `om6dof_controllers/TrajectoryController` | follows joint trajectories | `position` or `effort` |
| `om6dof_controllers/LeaderArmController` | lead-by-hand leader arm | `position` + `effort` |
| `om6dof_controllers/GravityCompensationController` | caps the current from the weight model | `effort` |
| `om6dof_controllers/SpringActuatorController` | virtual spring-damper to a rest pose | `effort` |

These are controller_manager plugins, not nodes. `om6dof_bringup` stays the
single owner of the hardware and of `ros2_control_node`; nothing here opens
U2D2.

This is a different package from `om6dof_controller` (singular), which is the
Python jog-command converter that publishes to `forward_position_controller`.
The two do not overlap and do not talk to each other.

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

`om6dof_gravity_comp`'s Python node reaches the same interface with its own
arming interlock and ramp; it is the path that has the most hours on this arm.
Never run it and this controller at the same time.

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

`om6dof_gravity_comp` is a separate effort that identifies these per-joint
numbers from measured current. Nothing here reads its results automatically;
they have been transcribed into this package's YAML by hand -- see below.

It is also a **second, independent implementation of gravity compensation** on
the same arm: its node publishes to `/forward_effort_controller/commands`, while
this controller claims `jointN/effort` directly. Never run both at once. Check
whether that node is alive before activating this controller.

### The shipped numbers, and where they came from

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

The datasheet route would have given ~609 for a W350, more than double the
measured 285. Guessing this was never acceptable.

**These scales no longer match the model.** They were fitted against a `g(q)`
that was missing 0.114 kg of off-chain mass and had placeholder centres of mass
on link3, link4 and link5. Both are fixed now, and the model's output moved a
long way: at the ready pose joint3 went from -0.306 to -0.531 Nm and joint5 from
-0.093 to -0.199 Nm. A fitted scale absorbs magnitude error, so the old numbers
were partly compensating for the old model's absence of mass. Re-run the
identification in `om6dof_gravity_comp` before trusting these on hardware, and
watch whether `gravity_correlation` on joint2 improves on 0.87 -- that number is
the test of whether the model's *shape* actually got better.

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

The one thing that cannot be checked away from the arm is the **sign** of the
model. This controller takes gravity as a vector in the base frame
(`gravity: [0, 0, -9.80665]` about `base_link`); a sign that comes out inverted
would make it push with gravity instead of against it. Before trusting it:

1. Load it with `gain` all zeros. Output is then exactly zero -- the arm cannot
   move -- but `~/gravity_torque` still publishes the raw model.
2. Move the arm by hand to a few poses and compare that topic against the
   Python `om6dof_gravity_comp` node, which is already known-good on this rig.
   Signs and magnitudes should agree.
3. Only then restore `gain` to `[0.0, 1.0, 1.0, 0.0, 0.0, 0.0]`, and bring it up
   with the arm supported.

`max_effort` is a real limit, not a formality. joint2 needs about 340 ticks at
its worst pose, which is already roughly 85% of the W350 stall figure: held out
horizontally, it will run hot.

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

The one to use for a leader arm. Gravity compensation alone does not give you
lead-by-hand on this rig, and it took a while to see why.

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

To hand the arm back to MoveIt, switch the other way. The controller writes the
measured position and a full current ceiling on the way out, so whatever takes
over gets the arm where it stands and able to hold it.

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
per-joint parameter expansion. The controllers' lifecycle and realtime paths are
not covered; those need a controller_manager fixture and are still to do.
