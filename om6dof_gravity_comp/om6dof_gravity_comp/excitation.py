"""Multi-sine excitation for identification, dry-run unless told otherwise.

Why a sum of sines
------------------
A single sine visits the same few configurations over and over, and a fit on
that data is only valid there. Summing three incommensurate frequencies makes
the joint wander over its range without repeating, so one run covers many
gravity configurations and many speeds -- which is what the regression needs
to separate the gravity term from the friction term.

Safety
------
The trajectory is checked against *conservative* limits, not the URDF's hard
stops: a fit is worthless if the run ends against an end stop, and the hard
limits leave no margin for tracking error. Amplitude is windowed by a raised
cosine so the arm starts and stops at rest rather than stepping into motion.

Execution goes through the existing ``arm_controller``
(JointTrajectoryController). This never opens the serial port and never
becomes a second owner of the hardware.
"""

from __future__ import annotations

import argparse
import math
import sys
from dataclasses import dataclass
from typing import List, Optional, Sequence, Tuple

import numpy as np

from om6dof_gravity_comp.units import JOINT_NAMES, match_stack_rmw

# Hard stops from the URDF, in radians. Kept here so the checker can run
# without a ROS graph; verified against om6dof.urdf.xacro.
URDF_LIMITS = {
    "joint1": (-2.827, 2.827),
    "joint2": (-2.042, 2.105),
    "joint3": (-1.885, 2.136),
    "joint4": (-2.827, 2.827),
    "joint5": (-1.979, 2.105),
    "joint6": (-2.827, 2.827),
}

# Fraction of the hard range the excitation is allowed to use, centred on the
# chosen midpoint. Deliberately timid: this is a first run on a real arm.
DEFAULT_RANGE_FRACTION = 0.35
DEFAULT_VELOCITY_LIMIT = 0.6      # rad/s
DEFAULT_ACCELERATION_LIMIT = 2.0  # rad/s^2
# The first waypoint has to be strictly in the future. A trajectory whose
# first point sits at time_from_start = 0 is dropped by
# JointTrajectoryController, and dropped silently from the publisher's side --
# which is exactly how the first version reported "done" while the arm never
# moved. This also gives the controller a moment to pick the message up.
LEAD_IN_S = 0.5


def trajectory_times(times, lead_in: float = LEAD_IN_S):
    """Waypoint times as the controller must receive them.

    Kept separate from the planning times so the dry-run still describes the
    motion itself, and so this rule can be tested without a robot.
    """
    shifted = [float(t) + float(lead_in) for t in times]
    if shifted and shifted[0] <= 0.0:
        raise ValueError("first waypoint must be strictly in the future")
    for earlier, later in zip(shifted, shifted[1:]):
        if later <= earlier:
            raise ValueError("waypoint times must strictly increase")
    return shifted


@dataclass
class SineComponent:
    amplitude: float
    frequency: float   # Hz
    phase: float


def default_components(base_frequency: float = 0.05) -> List[SineComponent]:
    """Three incommensurate components, decreasing in amplitude.

    The ratios are irrational-ish on purpose: 1 : 2.3 : 3.7 does not close
    into a short repeating pattern, so the joint keeps visiting new places
    for the whole run.
    """
    return [
        SineComponent(1.0, base_frequency, 0.0),
        SineComponent(0.5, base_frequency * 2.3, math.pi / 3),
        SineComponent(0.3, base_frequency * 3.7, 2 * math.pi / 5),
    ]


def ramp(t: float, duration: float, ramp_time: float) -> float:
    """Raised-cosine window: zero at both ends, one in the middle."""
    if duration <= 0.0:
        return 0.0
    ramp_time = max(1e-6, min(ramp_time, duration / 2.0))
    if t < ramp_time:
        return 0.5 * (1.0 - math.cos(math.pi * t / ramp_time))
    if t > duration - ramp_time:
        return 0.5 * (1.0 - math.cos(math.pi * (duration - t) / ramp_time))
    return 1.0


def evaluate(
    t: float,
    centre: float,
    scale: float,
    components: Sequence[SineComponent],
    duration: float,
    ramp_time: float,
) -> Tuple[float, float, float]:
    """Position, velocity and acceleration of the windowed sum at time t.

    Derivatives are taken numerically over the *windowed* signal rather than
    analytically over the bare sines, because the window is part of what the
    arm actually follows and ignoring it understates the peak acceleration at
    the ends of the run.
    """
    step = 1e-4

    def position(time_value: float) -> float:
        if time_value < 0.0 or time_value > duration:
            return centre
        total = sum(
            component.amplitude
            * math.sin(2 * math.pi * component.frequency * time_value
                       + component.phase)
            for component in components
        )
        return centre + scale * ramp(time_value, duration, ramp_time) * total

    here = position(t)
    ahead = position(t + step)
    behind = position(t - step)
    velocity = (ahead - behind) / (2 * step)
    acceleration = (ahead - 2 * here + behind) / (step * step)
    return here, velocity, acceleration


def plan(
    joint: str,
    centre: float,
    duration: float,
    sample_rate: float,
    base_frequency: float,
    range_fraction: float = DEFAULT_RANGE_FRACTION,
    ramp_time: float = 3.0,
) -> dict:
    """Build the trajectory and everything needed to judge whether it is safe."""
    if joint not in URDF_LIMITS:
        raise ValueError(f"unknown joint {joint!r}")
    lower, upper = URDF_LIMITS[joint]
    span = (upper - lower) * float(range_fraction)
    allowed_low = centre - span / 2.0
    allowed_high = centre + span / 2.0

    components = default_components(base_frequency)
    peak = sum(component.amplitude for component in components)
    scale = (span / 2.0) / peak if peak > 0 else 0.0

    # linspace, not arange: arange stops one step short of the duration, so
    # the last waypoint still had the window open and carried a non-zero
    # velocity. arm_controller sets allow_nonzero_velocity_at_trajectory_end
    # to false, so that one number made it reject the entire trajectory --
    # silently, with the arm simply not moving.
    count = max(2, int(round(duration * sample_rate)) + 1)
    times = np.linspace(0.0, duration, count)
    q, qd, qdd = [], [], []
    for t in times:
        position, velocity, acceleration = evaluate(
            float(t), centre, scale, components, duration, ramp_time)
        q.append(position)
        qd.append(velocity)
        qdd.append(acceleration)

    q = np.array(q)
    qd = np.array(qd)
    qdd = np.array(qdd)
    # The window makes both of these true analytically; pin them so floating
    # point cannot leave a few micro-rad/s behind for the controller to
    # object to.
    q[0] = centre
    q[-1] = centre
    qd[0] = 0.0
    qd[-1] = 0.0
    violations = []
    if q.min() < lower or q.max() > upper:
        violations.append(
            f"position {q.min():.3f}..{q.max():.3f} leaves the hard limits "
            f"{lower:.3f}..{upper:.3f}")
    if q.min() < allowed_low - 1e-9 or q.max() > allowed_high + 1e-9:
        violations.append(
            f"position {q.min():.3f}..{q.max():.3f} leaves the conservative "
            f"band {allowed_low:.3f}..{allowed_high:.3f}")
    if np.abs(qd).max() > DEFAULT_VELOCITY_LIMIT:
        violations.append(
            f"velocity peaks at {np.abs(qd).max():.3f} rad/s, over the "
            f"{DEFAULT_VELOCITY_LIMIT} rad/s cap")
    if np.abs(qdd).max() > DEFAULT_ACCELERATION_LIMIT:
        violations.append(
            f"acceleration peaks at {np.abs(qdd).max():.3f} rad/s^2, over the "
            f"{DEFAULT_ACCELERATION_LIMIT} rad/s^2 cap")

    return {
        "joint": joint,
        "times": times,
        "q": q,
        "qd": qd,
        "qdd": qdd,
        "centre": centre,
        "hard_limits": (lower, upper),
        "conservative_band": (allowed_low, allowed_high),
        "violations": violations,
        "components": components,
        "scale": scale,
    }


# Each joint gets its base frequency nudged by a different factor so the
# joints do not sweep in lockstep. Moving together in phase would revisit the
# same handful of arm configurations, which is the thing simultaneous
# excitation exists to avoid.
FREQUENCY_SPREAD = (1.0, 1.31, 1.67, 2.11, 2.53, 3.07)


def plan_many(
    joints: Sequence[str],
    centres: dict,
    duration: float,
    sample_rate: float,
    base_frequency: float,
    range_fraction: float = DEFAULT_RANGE_FRACTION,
    ramp_time: float = 3.0,
) -> dict:
    """Sweep several joints at once, each at its own frequency.

    Sweeping one joint at a time leaves the rest of the arm in a single
    configuration, so the gravity torque on that joint barely varies -- on
    this arm, measured, roughly a third of what moving three joints together
    produces. The regression needs that variation; without it the gravity
    coefficient is not identifiable no matter how long the run.
    """
    plans = {}
    violations = []
    for index, joint in enumerate(joints):
        factor = FREQUENCY_SPREAD[index % len(FREQUENCY_SPREAD)]
        # Peak speed goes as amplitude times frequency, so a joint given a
        # higher frequency has its travel cut to match. Without this the last
        # joint in the list trips the velocity cap and the whole run is
        # refused -- and shrinking every joint instead would throw away the
        # gravity variation this mode exists to gain.
        single = plan(joint, centres[joint], duration, sample_rate,
                      base_frequency * factor, range_fraction / factor,
                      ramp_time)
        plans[joint] = single
        violations += [f"{joint}: {v}" for v in single["violations"]]
    return {
        "joints": list(joints),
        "plans": plans,
        "times": plans[joints[0]]["times"],
        "violations": violations,
    }


def describe_many(result: dict) -> str:
    lines = [f"joints           : {', '.join(result['joints'])}"]
    for joint in result["joints"]:
        single = result["plans"][joint]
        lines.append(
            f"  {joint}: {math.degrees(single['q'].min()):+7.1f} .. "
            f"{math.degrees(single['q'].max()):+7.1f} deg   "
            f"peak |v| {np.abs(single['qd']).max():.3f} rad/s   "
            f"peak |a| {np.abs(single['qdd']).max():.3f} rad/s^2")
    lines.append(f"duration         : {result['times'][-1]:.1f} s, "
                 f"{len(result['times'])} points")
    if result["violations"]:
        lines.append("VIOLATIONS:")
        lines += [f"  - {v}" for v in result["violations"]]
    else:
        lines.append("within every limit checked")
    return "\n".join(lines)


def describe(result: dict) -> str:
    q, qd, qdd = result["q"], result["qd"], result["qdd"]
    low, high = result["conservative_band"]
    hard_low, hard_high = result["hard_limits"]
    lines = [
        f"joint            : {result['joint']}",
        f"centre           : {math.degrees(result['centre']):+.1f} deg",
        f"position range   : {math.degrees(q.min()):+.1f} .. "
        f"{math.degrees(q.max()):+.1f} deg",
        f"conservative band: {math.degrees(low):+.1f} .. "
        f"{math.degrees(high):+.1f} deg",
        f"hard limits      : {math.degrees(hard_low):+.1f} .. "
        f"{math.degrees(hard_high):+.1f} deg",
        f"peak |velocity|  : {np.abs(qd).max():.3f} rad/s "
        f"(cap {DEFAULT_VELOCITY_LIMIT})",
        f"peak |accel|     : {np.abs(qdd).max():.3f} rad/s^2 "
        f"(cap {DEFAULT_ACCELERATION_LIMIT})",
        f"duration         : {result['times'][-1]:.1f} s, "
        f"{len(result['times'])} points",
    ]
    if result["violations"]:
        lines.append("VIOLATIONS:")
        lines += [f"  - {v}" for v in result["violations"]]
    else:
        lines.append("within every limit checked")
    return "\n".join(lines)


# --------------------------------------------------------------------------- #
#  Execution. Everything above is pure and testable without a robot.           #
# --------------------------------------------------------------------------- #
def _print_environment(node) -> None:
    """Dump what this process is actually talking over, on failure.

    The same call succeeds or times out depending on which shell it is run
    from, and guessing at the difference across a terminal has cost several
    rounds. This makes the next failure carry its own diagnosis.
    """
    import os as _os
    print("\n--- what this process is using ---", file=sys.stderr)
    for name in ("ROS_DOMAIN_ID", "RMW_IMPLEMENTATION",
                 "FASTRTPS_DEFAULT_PROFILES_FILE", "CYCLONEDDS_URI",
                 "ROS_LOCALHOST_ONLY", "AMENT_PREFIX_PATH"):
        value = _os.environ.get(name)
        if name == "AMENT_PREFIX_PATH" and value:
            value = value.split(":")[0] + " (first entry)"
        print(f"  {name} = {value!r}", file=sys.stderr)
    try:
        names = sorted(f"{ns}{n}" for n, ns in node.get_node_names_and_namespaces())
        print(f"  nodes discovered ({len(names)}):", file=sys.stderr)
        for entry in names:
            print(f"    {entry}", file=sys.stderr)
    except Exception as exc:
        print(f"  could not list nodes: {exc}", file=sys.stderr)
    print("--- end ---\n", file=sys.stderr)


def _execute_many(result: dict, hold: dict, controller: str) -> int:
    """Send one trajectory that moves every planned joint together."""
    combined = {
        "joint": result["joints"][0],
        "times": result["times"],
        "q": result["plans"][result["joints"][0]]["q"],
        "qd": result["plans"][result["joints"][0]]["qd"],
        "_multi": result,
    }
    return _execute(combined, hold, controller)


def _execute(result: dict, hold: dict, controller: str) -> int:
    import rclpy
    from builtin_interfaces.msg import Duration
    from controller_manager_msgs.srv import ListControllers
    from rclpy.node import Node
    from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

    rclpy.init()
    node = Node("om6dof_excitation")
    try:
        # Refuse unless the trajectory controller is the one holding the arm.
        # Publishing while forward_position_controller is active would put two
        # writers on the same joints.
        client = node.create_client(ListControllers,
                                    "/controller_manager/list_controllers")
        # Discovery on this graph routinely takes more than five seconds --
        # eleven nodes, and stale shm segments slow FastDDS's startup -- so a
        # short wait here refused runs whose only fault was asking too early.
        if not client.wait_for_service(timeout_sec=20.0):
            print("controller_manager not reachable after 20 s -- is "
                  "om6dof-hardware.service running?", file=sys.stderr)
            _print_environment(node)
            return 1
        future = client.call_async(ListControllers.Request())
        rclpy.spin_until_future_complete(node, future, timeout_sec=20.0)
        if future.result() is None:
            print("controller_manager found but did not answer within 20 s",
                  file=sys.stderr)
            _print_environment(node)
            return 1
        states = {c.name: c.state for c in future.result().controller}
        if states.get(controller) != "active":
            print(f"{controller} is '{states.get(controller, 'missing')}', not "
                  "active. Put the arm in AUTONOMOUS so the trajectory "
                  "controller owns it, then retry.", file=sys.stderr)
            return 1
        if states.get("forward_position_controller") == "active":
            print("forward_position_controller is still active; two writers "
                  "would fight. Switch the arm to AUTONOMOUS first.",
                  file=sys.stderr)
            return 1

        publisher = node.create_publisher(
            JointTrajectory, f"/{controller}/joint_trajectory", 10)
        message = JointTrajectory()
        message.joint_names = list(JOINT_NAMES)
        multi = result.get("_multi")
        moving = result["joint"]
        # One trajectory carries every joint; the ones not being excited hold
        # where they are rather than being left out, because a partial goal
        # is refused (allow_partial_joints_goal is false).
        tracks = ({j: multi["plans"][j] for j in multi["joints"]} if multi
                  else {moving: result})
        stamps = trajectory_times(result["times"])
        for index, t in enumerate(stamps):
            point = JointTrajectoryPoint()
            point.positions = [
                float(tracks[name]["q"][index]) if name in tracks
                else float(hold[name])
                for name in JOINT_NAMES
            ]
            point.velocities = [
                float(tracks[name]["qd"][index]) if name in tracks else 0.0
                for name in JOINT_NAMES
            ]
            seconds = int(t)
            point.time_from_start = Duration(
                sec=seconds, nanosec=int((t - seconds) * 1e9))
            message.points.append(point)

        print(f"sending {len(message.points)} points to {controller}...")
        # Give discovery a moment; a trajectory dropped because the subscriber
        # was not matched yet looks exactly like a controller fault.
        deadline = 3.0
        while publisher.get_subscription_count() == 0 and deadline > 0:
            rclpy.spin_once(node, timeout_sec=0.1)
            deadline -= 0.1
        if publisher.get_subscription_count() == 0:
            print(f"nothing is listening on /{controller}/joint_trajectory",
                  file=sys.stderr)
            return 1
        # Watch the joint while it runs. Publishing succeeds whether or not
        # the controller accepts the trajectory, so the only honest way to
        # report success is to see the arm move.
        from sensor_msgs.msg import JointState
        seen = []

        def on_js(msg):
            for index, name in enumerate(msg.name):
                if name == moving and index < len(msg.position):
                    seen.append(float(msg.position[index]))

        node.create_subscription(JointState, "/joint_states", on_js, 20)
        import time as _time
        settle = _time.monotonic() + 1.0
        while rclpy.ok() and _time.monotonic() < settle:
            rclpy.spin_once(node, timeout_sec=0.05)
        before = len(seen)

        publisher.publish(message)
        end = stamps[-1] + 2.0
        print(f"running for {end:.0f} s -- start the logger now if you have "
              "not already")
        finish = _time.monotonic() + end
        while rclpy.ok() and _time.monotonic() < finish:
            rclpy.spin_once(node, timeout_sec=0.05)

        travelled = seen[before:]
        if len(travelled) < 10:
            print("no joint feedback during the run", file=sys.stderr)
            return 1
        span = max(travelled) - min(travelled)
        if span < math.radians(1.0):
            print(f"\nthe trajectory was sent but {moving} moved only "
                  f"{math.degrees(span):.2f} deg -- the controller did not "
                  "run it. Check that arm_controller is active and that "
                  "nothing else is holding the joints.", file=sys.stderr)
            return 1
        print(f"done -- {moving} covered {math.degrees(span):.1f} deg")
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


def _current_positions(timeout: float = 10.0) -> Optional[dict]:
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import JointState
    from om6dof_gravity_comp.units import order_by_joint
    import time as _time

    rclpy.init()
    node = Node("om6dof_excitation_probe")
    seen = {}

    def on_js(msg):
        for index, name in enumerate(msg.name):
            if index < len(msg.position):
                seen[name] = float(msg.position[index])

    node.create_subscription(JointState, "/joint_states", on_js, 20)
    deadline = _time.monotonic() + timeout
    try:
        while _time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
            values = order_by_joint(list(seen), [seen[n] for n in seen])
            if all(v is not None for v in values):
                return dict(zip(JOINT_NAMES, values))
        return None
    finally:
        node.destroy_node()
        rclpy.shutdown()


def main(argv=None) -> int:
    match_stack_rmw()
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--joint", choices=list(JOINT_NAMES),
                        help="one joint to excite")
    parser.add_argument("--joints", default=None,
                        help="several joints at once, comma separated, e.g. "
                             "joint2,joint3,joint5 -- gives each joint far "
                             "more gravity variation than sweeping alone")
    parser.add_argument("--duration", type=float, default=60.0)
    parser.add_argument("--rate", type=float, default=20.0,
                        help="trajectory points per second")
    parser.add_argument("--base-frequency", type=float, default=0.05,
                        help="Hz of the slowest component (default 0.05)")
    parser.add_argument("--range-fraction", type=float,
                        default=DEFAULT_RANGE_FRACTION,
                        help="share of the joint's hard range to use")
    parser.add_argument("--centre-deg", type=float, default=None,
                        help="midpoint; defaults to where the joint is now")
    parser.add_argument("--ramp", type=float, default=3.0)
    parser.add_argument("--controller", default="arm_controller")
    parser.add_argument("--execute", action="store_true",
                        help="actually move the arm; without this it only "
                             "reports what it would do")
    args = parser.parse_args(argv)

    if not args.joint and not args.joints:
        print("give --joint or --joints", file=sys.stderr)
        return 2
    selected = ([j.strip() for j in args.joints.split(",") if j.strip()]
                if args.joints else [args.joint])
    unknown = [j for j in selected if j not in JOINT_NAMES]
    if unknown:
        print(f"unknown joints: {unknown}", file=sys.stderr)
        return 2

    hold = _current_positions()
    if hold is None:
        print("could not read /joint_states", file=sys.stderr)
        return 1
    centres = dict(hold)
    if args.centre_deg is not None and len(selected) == 1:
        centres[selected[0]] = math.radians(args.centre_deg)

    if len(selected) == 1:
        result = plan(selected[0], centres[selected[0]], args.duration,
                      args.rate, args.base_frequency, args.range_fraction,
                      args.ramp)
        print(describe(result))
        executor = _execute
    else:
        result = plan_many(selected, centres, args.duration, args.rate,
                           args.base_frequency, args.range_fraction, args.ramp)
        print(describe_many(result))
        executor = _execute_many

    if result["violations"]:
        print("\nrefusing to run: fix the settings above", file=sys.stderr)
        return 1
    if not args.execute:
        print("\ndry run only. Add --execute to move the arm.")
        return 0
    return executor(result, hold, args.controller)


if __name__ == "__main__":
    sys.exit(main())
