"""Visit a grid of poses, pause at each, and let the logger record them.

Why this exists
---------------
The moving-excitation model holds up across recordings while the joints are
turning (R2 ~0.92) and falls apart when they stop (as low as -0.62). A leader
arm spends much of its life nearly still, so that is the half that matters
most and the half no data covers.

Standing still, the motor current is gravity plus whatever the gearbox is
holding by friction, with no velocity term to help separate them. The way out
is the one the SI2017 work uses: arrive at the same pose from both
directions. Friction opposes the last motion, so it flips sign between the
two visits while gravity does not:

    gravity  = (I_from_above + I_from_below) / 2
    friction = (I_from_above - I_from_below) / 2

So each pose is visited twice, with a dwell long enough for the arm to settle
and for the logger to collect a steady stretch.

Motion goes through arm_controller, the same as the excitation tool, and is
dry-run unless told otherwise.
"""

from __future__ import annotations

import argparse
import itertools
import math
import sys
import time
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np

from om6dof_gravity_comp.excitation import (
    DEFAULT_VELOCITY_LIMIT,
    LEAD_IN_S,
    URDF_LIMITS,
    _current_positions,
    trajectory_times,
)
from om6dof_gravity_comp.units import JOINT_NAMES, match_stack_rmw

# Only these three carry gravity on this arm; the others have axes along it.
GRAVITY_JOINTS = ("joint2", "joint3", "joint5")
DEFAULT_GRID = 4
DEFAULT_DWELL_S = 3.0
DEFAULT_APPROACH_RAD = 0.12   # how far back to step before coming in again
DEFAULT_MOVE_SPEED = 0.25     # rad/s between poses
DEFAULT_RANGE_FRACTION = 0.30


def pose_grid(
    centres: Dict[str, float],
    joints: Sequence[str] = GRAVITY_JOINTS,
    steps: int = DEFAULT_GRID,
    range_fraction: float = DEFAULT_RANGE_FRACTION,
) -> List[Dict[str, float]]:
    """Every combination of a few values per gravity-carrying joint.

    A grid rather than a random sample so the coverage is even and the run
    length is predictable; the point is to span gravity configurations, not
    to be statistically clever about it.
    """
    axes = []
    for joint in joints:
        lower, upper = URDF_LIMITS[joint]
        span = (upper - lower) * float(range_fraction)
        low = max(lower + 0.05, centres[joint] - span / 2)
        high = min(upper - 0.05, centres[joint] + span / 2)
        axes.append(np.linspace(low, high, steps))

    poses = []
    for combination in itertools.product(*axes):
        pose = dict(centres)
        for joint, value in zip(joints, combination):
            pose[joint] = float(value)
        poses.append(pose)
    return poses


def check_poses(poses: Sequence[Dict[str, float]]) -> List[str]:
    problems = []
    for index, pose in enumerate(poses):
        for joint, value in pose.items():
            if joint not in URDF_LIMITS:
                continue
            lower, upper = URDF_LIMITS[joint]
            if not (lower + 0.02 <= value <= upper - 0.02):
                problems.append(
                    f"pose {index}: {joint} at {math.degrees(value):+.1f} deg "
                    f"is outside {math.degrees(lower):+.1f}.."
                    f"{math.degrees(upper):+.1f}")
    return problems


def plan_visits(
    poses: Sequence[Dict[str, float]],
    approach: float = DEFAULT_APPROACH_RAD,
    joints: Sequence[str] = GRAVITY_JOINTS,
) -> List[Tuple[str, Dict[str, float]]]:
    """Each pose reached twice, once from each side, with a dwell between.

    Labelled so the fitting step can tell the two visits apart without having
    to infer direction from the trajectory afterwards.
    """
    visits: List[Tuple[str, Dict[str, float]]] = []
    for pose in poses:
        for direction, sign in (("from_above", +1.0), ("from_below", -1.0)):
            backed_off = dict(pose)
            for joint in joints:
                lower, upper = URDF_LIMITS[joint]
                backed_off[joint] = float(np.clip(
                    pose[joint] + sign * approach, lower + 0.02, upper - 0.02))
            visits.append(("approach", backed_off))
            visits.append((direction, dict(pose)))
    return visits


def estimate_duration(
    visits: Sequence[Tuple[str, Dict[str, float]]],
    dwell: float,
    speed: float,
    start: Dict[str, float],
) -> float:
    total = 0.0
    previous = dict(start)
    for label, pose in visits:
        distance = max(
            (abs(pose[j] - previous[j]) for j in JOINT_NAMES if j in pose),
            default=0.0)
        total += max(0.5, distance / max(speed, 1e-6))
        if label != "approach":
            total += dwell
        previous = pose
    return total


def describe(poses, visits, duration: float) -> str:
    lines = [
        f"gravity joints   : {', '.join(GRAVITY_JOINTS)}",
        f"poses            : {len(poses)}",
        f"visits           : {len(visits)} "
        f"({len(poses)} poses x 2 directions, plus approach moves)",
        f"estimated time   : {duration / 60:.1f} minutes",
    ]
    for joint in GRAVITY_JOINTS:
        values = sorted({round(math.degrees(p[joint]), 1) for p in poses})
        lines.append(f"  {joint}: {values} deg")
    return "\n".join(lines)


def _execute(visits, dwell, speed, controller, start) -> int:
    import rclpy
    from builtin_interfaces.msg import Duration
    from controller_manager_msgs.srv import ListControllers
    from rclpy.node import Node
    from sensor_msgs.msg import JointState
    from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

    rclpy.init()
    node = Node("om6dof_static_sweep")
    try:
        client = node.create_client(
            ListControllers, "/controller_manager/list_controllers")
        if not client.wait_for_service(timeout_sec=20.0):
            print("controller_manager not reachable", file=sys.stderr)
            return 1
        future = client.call_async(ListControllers.Request())
        rclpy.spin_until_future_complete(node, future, timeout_sec=20.0)
        if future.result() is None:
            print("controller_manager did not answer", file=sys.stderr)
            return 1
        states = {c.name: c.state for c in future.result().controller}
        if states.get(controller) != "active":
            print(f"{controller} is '{states.get(controller, 'missing')}'. "
                  "Put the arm in AUTONOMOUS first.", file=sys.stderr)
            return 1
        if states.get("forward_position_controller") == "active":
            print("forward_position_controller is still active; switch the "
                  "arm to AUTONOMOUS first.", file=sys.stderr)
            return 1

        publisher = node.create_publisher(
            JointTrajectory, f"/{controller}/joint_trajectory", 10)
        seen = {}

        def on_js(msg):
            for index, name in enumerate(msg.name):
                if index < len(msg.position):
                    seen[name] = float(msg.position[index])

        node.create_subscription(JointState, "/joint_states", on_js, 20)
        deadline = time.monotonic() + 5.0
        while publisher.get_subscription_count() == 0 and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        if publisher.get_subscription_count() == 0:
            print(f"nothing is listening on /{controller}/joint_trajectory",
                  file=sys.stderr)
            return 1

        previous = dict(start)
        dwelled = 0
        for index, (label, pose) in enumerate(visits):
            distance = max(
                (abs(pose[j] - previous[j]) for j in JOINT_NAMES if j in pose),
                default=0.0)
            travel = max(0.6, distance / max(speed, 1e-6))

            message = JointTrajectory()
            message.joint_names = list(JOINT_NAMES)
            point = JointTrajectoryPoint()
            point.positions = [float(pose[name]) for name in JOINT_NAMES]
            # Zero velocity at the end, or arm_controller refuses the goal:
            # allow_nonzero_velocity_at_trajectory_end is false.
            point.velocities = [0.0] * len(JOINT_NAMES)
            stamp = trajectory_times([travel])[0]
            point.time_from_start = Duration(
                sec=int(stamp), nanosec=int((stamp - int(stamp)) * 1e9))
            message.points.append(point)
            publisher.publish(message)

            finish = time.monotonic() + stamp + 0.3
            while rclpy.ok() and time.monotonic() < finish:
                rclpy.spin_once(node, timeout_sec=0.05)

            if label != "approach":
                # Hold still so the logger collects a settled stretch.
                dwelled += 1
                hold = time.monotonic() + dwell
                while rclpy.ok() and time.monotonic() < hold:
                    rclpy.spin_once(node, timeout_sec=0.05)
                if dwelled % 8 == 0:
                    print(f"  {dwelled} dwells done "
                          f"({index + 1}/{len(visits)} moves)")
            previous = pose

        print(f"done -- {dwelled} dwells recorded")
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


def main(argv=None) -> int:
    match_stack_rmw()
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--grid", type=int, default=DEFAULT_GRID,
                        help="values per gravity joint (default 4 -> 64 poses)")
    parser.add_argument("--dwell", type=float, default=DEFAULT_DWELL_S)
    parser.add_argument("--approach", type=float, default=DEFAULT_APPROACH_RAD,
                        help="rad to back off before re-approaching a pose")
    parser.add_argument("--speed", type=float, default=DEFAULT_MOVE_SPEED)
    parser.add_argument("--range-fraction", type=float,
                        default=DEFAULT_RANGE_FRACTION)
    parser.add_argument("--controller", default="arm_controller")
    parser.add_argument("--execute", action="store_true")
    args = parser.parse_args(argv)

    if not (0.0 < args.speed <= DEFAULT_VELOCITY_LIMIT):
        print(f"--speed must be within 0..{DEFAULT_VELOCITY_LIMIT} rad/s",
              file=sys.stderr)
        return 2

    start = _current_positions()
    if start is None:
        print("could not read /joint_states", file=sys.stderr)
        return 1

    poses = pose_grid(start, steps=args.grid,
                      range_fraction=args.range_fraction)
    problems = check_poses(poses)
    visits = plan_visits(poses, args.approach)
    duration = estimate_duration(visits, args.dwell, args.speed, start)
    print(describe(poses, visits, duration))

    if problems:
        print("\nrefusing to run:", file=sys.stderr)
        for problem in problems[:10]:
            print(f"  - {problem}", file=sys.stderr)
        return 1
    if not args.execute:
        print("\ndry run only. Add --execute to move the arm.")
        return 0
    return _execute(visits, args.dwell, args.speed, args.controller, start)


if __name__ == "__main__":
    sys.exit(main())
