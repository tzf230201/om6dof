"""Check gravity compensation on one joint, at one pose.

The full model spans six joints and every configuration, which makes a
disagreement hard to attribute. This narrows it to a single number: park the
arm at a chosen pose, hold still, and compare three things on one joint.

    predicted   what the identified model says the motor should draw
    measured    what it actually draws
    gap         the difference

At the all-zero pose the arm stands nearly upright, so gravity on joint 2 is
only about 0.12 Nm and almost all of the prediction is the model's bias term.
That makes zero a poor place to judge the gravity part; the load appears as
joint 2 comes down, reaching about 1.1 Nm near -80 degrees. The default is
still zero because that is the pose people mean by "straight", and the tool
says so rather than quietly moving somewhere more flattering.

Motion goes through arm_controller, and nothing moves without --execute.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
import time
from typing import Dict, List, Optional

import numpy as np

from om6dof_gravity_comp.excitation import (
    URDF_LIMITS,
    _current_positions,
    trajectory_times,
)
from om6dof_gravity_comp.units import CURRENT_TICK_MA, JOINT_NAMES, match_stack_rmw

SETTLE_S = 3.0
SAMPLE_S = 3.0
MOVE_SPEED = 0.25          # rad/s


def load_model(path: str) -> Dict[str, dict]:
    import yaml
    with open(path) as handle:
        document = yaml.safe_load(handle)
    root = (document.get("om6dof_static_model")
            or document.get("om6dof_current_model"))
    if root is None:
        raise ValueError(f"{path} is not a fitted model")
    return root["joints"]


def predict(entry: dict, nominal_torque: float) -> Optional[float]:
    """Predicted holding current, or None when the model cannot say.

    A gravity coefficient fitted where gravity barely varied is noise, and
    reporting it as a prediction would invite trusting it.
    """
    if not entry.get("gravity_identifiable", False):
        return None
    if "gravity" in entry:                       # static model
        return entry["gravity"] * nominal_torque + entry.get("bias", 0.0)
    coefficients = entry.get("coefficients") or {}
    if "gravity_nominal" not in coefficients:
        return None
    return (coefficients["gravity_nominal"] * nominal_torque
            + coefficients.get("bias", 0.0))


def _gravity_model():
    import os
    import subprocess
    from ament_index_python.packages import get_package_share_directory
    from om6dof_gravity_comp.gravity_model import GravityModel
    share = get_package_share_directory("om6dof_description")
    path = os.path.join(share, "urdf", "om6dof.urdf.xacro")
    result = subprocess.run(["xacro", path], capture_output=True, text=True,
                            timeout=60)
    if result.returncode != 0:
        raise RuntimeError(f"xacro failed: {result.stderr.strip()}")
    return GravityModel(result.stdout)


def _move_and_measure(target: Dict[str, float], joint: str, controller: str
                      ) -> Optional[Dict[str, float]]:
    import rclpy
    from builtin_interfaces.msg import Duration
    from controller_manager_msgs.srv import ListControllers
    from rclpy.node import Node
    from sensor_msgs.msg import JointState
    from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

    rclpy.init()
    node = Node("om6dof_joint_check")
    try:
        client = node.create_client(
            ListControllers, "/controller_manager/list_controllers")
        if not client.wait_for_service(timeout_sec=20.0):
            print("controller_manager not reachable", file=sys.stderr)
            return None
        future = client.call_async(ListControllers.Request())
        rclpy.spin_until_future_complete(node, future, timeout_sec=20.0)
        if future.result() is None:
            print("controller_manager did not answer", file=sys.stderr)
            return None
        states = {c.name: c.state for c in future.result().controller}
        if states.get(controller) != "active":
            print(f"{controller} is '{states.get(controller, 'missing')}'. "
                  "Put the arm in AUTONOMOUS first.", file=sys.stderr)
            return None

        samples: List[Dict[str, float]] = []

        def on_js(msg):
            row = {}
            for index, name in enumerate(msg.name):
                if index < len(msg.position):
                    row[f"q_{name}"] = float(msg.position[index])
                if index < len(msg.effort):
                    row[f"i_{name}"] = float(msg.effort[index])
                if index < len(msg.velocity):
                    row[f"v_{name}"] = float(msg.velocity[index])
            samples.append(row)

        node.create_subscription(JointState, "/joint_states", on_js, 20)
        publisher = node.create_publisher(
            JointTrajectory, f"/{controller}/joint_trajectory", 10)
        deadline = time.monotonic() + 5.0
        while publisher.get_subscription_count() == 0 and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        if publisher.get_subscription_count() == 0:
            print(f"nothing listening on /{controller}/joint_trajectory",
                  file=sys.stderr)
            return None

        start = _current_positions()
        if start is None:
            print("could not read /joint_states", file=sys.stderr)
            return None
        distance = max(abs(target[n] - start[n]) for n in JOINT_NAMES)
        travel = max(1.0, distance / MOVE_SPEED)

        message = JointTrajectory()
        message.joint_names = list(JOINT_NAMES)
        point = JointTrajectoryPoint()
        point.positions = [float(target[n]) for n in JOINT_NAMES]
        point.velocities = [0.0] * len(JOINT_NAMES)
        stamp = trajectory_times([travel])[0]
        point.time_from_start = Duration(
            sec=int(stamp), nanosec=int((stamp - int(stamp)) * 1e9))
        message.points.append(point)
        print(f"moving there over {stamp:.1f} s...")
        publisher.publish(message)

        finish = time.monotonic() + stamp + SETTLE_S
        while rclpy.ok() and time.monotonic() < finish:
            rclpy.spin_once(node, timeout_sec=0.05)

        # Only the settled stretch: the tail of the move still carries the
        # current that decelerated the arm, which is not holding current.
        mark = len(samples)
        window = time.monotonic() + SAMPLE_S
        while rclpy.ok() and time.monotonic() < window:
            rclpy.spin_once(node, timeout_sec=0.05)
        settled = [r for r in samples[mark:]
                   if abs(r.get(f"v_{joint}", 1.0)) < 0.01]
        if len(settled) < 10:
            print(f"only {len(settled)} still samples; did the arm stop?",
                  file=sys.stderr)
            return None
        return {
            "position": float(np.mean([r[f"q_{joint}"] for r in settled])),
            "current": float(np.mean([r[f"i_{joint}"] for r in settled])),
            "spread": float(np.std([r[f"i_{joint}"] for r in settled])),
            "samples": len(settled),
        }
    finally:
        node.destroy_node()
        rclpy.shutdown()


def main(argv=None) -> int:
    match_stack_rmw()
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--model", required=True,
                        help="YAML from identify or identify_static")
    parser.add_argument("--joint", default="joint2", choices=list(JOINT_NAMES))
    parser.add_argument("--angle-deg", type=float, default=0.0,
                        help="where to put that joint (default 0, arm upright)")
    parser.add_argument("--controller", default="arm_controller")
    parser.add_argument("--json", action="store_true",
                        help="emit one JSON object instead of prose, so the "
                             "dashboard does not have to parse sentences")
    parser.add_argument("--execute", action="store_true",
                        help="actually move the arm; without this it only "
                             "reports what it would do")
    args = parser.parse_args(argv)

    joints = load_model(args.model)
    entry = joints.get(args.joint) or {}
    if "coefficients" not in entry and "gravity" not in entry:
        print(f"{args.joint} has no fitted gravity term in {args.model}",
              file=sys.stderr)
        return 1

    target = {name: 0.0 for name in JOINT_NAMES}
    target[args.joint] = math.radians(args.angle_deg)
    lower, upper = URDF_LIMITS[args.joint]
    if not (lower + 0.02 <= target[args.joint] <= upper - 0.02):
        print(f"{args.angle_deg} deg is outside {args.joint}'s limits",
              file=sys.stderr)
        return 1

    model = _gravity_model()
    nominal = model.torques([target[n] for n in JOINT_NAMES])
    index = list(JOINT_NAMES).index(args.joint)
    torque = float(nominal[index])
    predicted = predict(entry, torque)

    report = {
        "joint": args.joint,
        "angle_deg": args.angle_deg,
        "torque_nm": torque,
        "predicted": predicted,
        "identifiable": predicted is not None,
        "low_load": abs(torque) < 0.2,
        "executed": bool(args.execute),
    }
    if args.json and not args.execute:
        print(json.dumps(report))
        return 0

    print(f"pose            : every joint 0, {args.joint} at "
          f"{args.angle_deg:+.1f} deg")
    print(f"gravity torque  : {torque:+.4f} Nm on {args.joint}")
    if predicted is None:
        print(f"prediction      : unavailable -- the model marks "
              f"{args.joint}'s gravity term as not identifiable")
    else:
        print(f"predicted current: {predicted:+.1f} raw "
              f"({predicted * CURRENT_TICK_MA:+.0f} mA)")
    if abs(torque) < 0.2:
        print(f"\nNOTE: only {abs(torque):.3f} Nm of gravity here, so most of "
              "the prediction is the model's bias rather than its gravity\n"
              "      term. Try --angle-deg -60 or -80 for a loaded pose.")

    if not args.execute:
        print("\ndry run. Add --execute to move there and measure.")
        return 0

    result = _move_and_measure(target, args.joint, args.controller)
    if result is None:
        if args.json:
            report["error"] = "could not move or measure"
            print(json.dumps(report))
        return 1
    report.update({
        "settled_deg": math.degrees(result["position"]),
        "measured": result["current"],
        "spread": result["spread"],
        "samples": result["samples"],
        "gap": (result["current"] - predicted) if predicted is not None else None,
    })
    if args.json:
        print(json.dumps(report))
        return 0
    print(f"\nsettled at      : {math.degrees(result['position']):+.2f} deg "
          f"({result['samples']} samples)")
    print(f"measured current: {result['current']:+.1f} raw "
          f"(spread {result['spread']:.1f})")
    if predicted is not None:
        gap = result["current"] - predicted
        print(f"gap             : {gap:+.1f} raw "
              f"({gap * CURRENT_TICK_MA:+.0f} mA)")
        print("\nThe gap is what the gearbox is holding on its own, plus "
              "whatever the model has wrong.\nIt is not zero even for a "
              "perfect model: friction carries part of the load at rest.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
