"""Measure what actually resists your hand when the arm is in FLOAT.

The arm being hard to push has two possible causes, and they lead to opposite
next steps:

  the servo fighting     Position error builds faster than FLOAT's command
                         can follow, so the motor pushes back. Shows up as
                         large |effort| while you push. Fixed by letting the
                         command follow faster, or by softening the servo's
                         position gain.

  the gearbox            353:1 is not comfortably backdriveable on its own.
                         Shows up as effort staying near zero while the arm
                         still resists. No amount of tuning the position loop
                         helps; that needs current control with friction
                         compensation.

This records both while you push, and says which one it saw. It only watches
and switches mode -- it never commands motion.
"""

from __future__ import annotations

import argparse
import math
import sys
import time
from typing import Dict, List

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, String

JOINT_NAMES = [f"joint{i}" for i in range(1, 7)]
CURRENT_UNIT_MA = 2.69
# Below this the motor is barely doing anything, so whatever is resisting is
# not the servo. Roughly 50 mA on an XM430.
QUIET_EFFORT_RAW = 20.0
MOVED_RAD = math.radians(1.0)


class PushProbe(Node):
    def __init__(self) -> None:
        super().__init__("om6dof_push_test")
        self.positions: Dict[str, float] = {}
        self.efforts: Dict[str, float] = {}
        self.mode = ""
        self.remote = None
        self._stamp = 0.0
        self.samples: List[tuple] = []

        latched = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.pub_mode = self.create_publisher(
            String, "/om6dof/operation_mode", 10)
        self.create_subscription(JointState, "/joint_states", self._on_js, 20)
        self.create_subscription(
            String, "/om6dof/operation_mode/state",
            lambda m: setattr(self, "mode", str(m.data).strip().upper()), latched)
        self.create_subscription(
            Bool, "/om6dof/remote_enabled/state",
            lambda m: setattr(self, "remote", bool(m.data)), latched)

    def _on_js(self, msg: JointState) -> None:
        for index, name in enumerate(msg.name):
            if index < len(msg.position):
                self.positions[name] = float(msg.position[index])
            if index < len(msg.effort):
                self.efforts[name] = float(msg.effort[index])
        self._stamp = time.monotonic()

    def fresh(self) -> bool:
        return bool(self._stamp) and (time.monotonic() - self._stamp) < 0.5

    def wait_ready(self, timeout: float = 15.0) -> bool:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.fresh() and self.mode and self.remote is not None:
                return True
        return False

    def set_mode(self, mode: str) -> None:
        self.pub_mode.publish(String(data=mode))

    def record(self, seconds: float) -> None:
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.02)
            if not self.fresh():
                continue
            q = [self.positions.get(n) for n in JOINT_NAMES]
            e = [self.efforts.get(n) for n in JOINT_NAMES]
            if any(v is None for v in q) or any(v is None for v in e):
                continue
            self.samples.append((np.array(q), np.array(e)))


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--seconds", type=float, default=20.0,
                        help="how long to record while you push")
    parser.add_argument("--keep-float", action="store_true",
                        help="stay in FLOAT afterwards instead of returning "
                             "to JOINT")
    args = parser.parse_args(argv)

    rclpy.init()
    node = PushProbe()
    entered_float = False
    try:
        if not node.wait_ready():
            print("No arm state. Is the stack running and control claimed?",
                  file=sys.stderr)
            return 1
        if node.remote is not True:
            print("Enable streaming control first.", file=sys.stderr)
            return 1

        if node.mode != "FLOAT":
            print("Entering FLOAT...")
            node.set_mode("FLOAT")
            deadline = time.monotonic() + 5.0
            while node.mode != "FLOAT" and time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.05)
            if node.mode != "FLOAT":
                print(f"Arm did not enter FLOAT (still {node.mode}).",
                      file=sys.stderr)
                return 1
            entered_float = True

        print(f"\nPush the arm around for {args.seconds:g} seconds. "
              "Try the joints that felt heaviest.\n")
        node.record(args.seconds)
        if len(node.samples) < 20:
            print("Too few samples; is /joint_states publishing?",
                  file=sys.stderr)
            return 1

        q = np.array([s[0] for s in node.samples])
        e = np.array([s[1] for s in node.samples])
        travel = q.max(axis=0) - q.min(axis=0)
        peak = np.abs(e).max(axis=0)
        median = np.median(np.abs(e), axis=0)

        print(f"{'joint':7} {'moved(deg)':>11} {'peak|effort|':>13} "
              f"{'median':>8} {'peak(mA)':>10}")
        for i in range(6):
            print(f"joint{i+1:<2} {math.degrees(travel[i]):11.1f} "
                  f"{peak[i]:13.0f} {median[i]:8.0f} "
                  f"{peak[i] * CURRENT_UNIT_MA:10.0f}")

        pushed = [i for i in range(6) if travel[i] > MOVED_RAD]
        print()
        if not pushed:
            print("Nothing moved by more than a degree, so there is nothing "
                  "to conclude. Push harder, or push a different joint.")
            return 0

        fighting = [i for i in pushed if peak[i] > QUIET_EFFORT_RAW]
        if fighting:
            names = ", ".join(f"joint{i+1}" for i in fighting)
            print(f"VERDICT: the servo is pushing back on {names} "
                  f"(peak {peak[fighting].max():.0f} raw = "
                  f"{peak[fighting].max() * CURRENT_UNIT_MA:.0f} mA).")
            print("  Try raising float_follow_velocity so the command keeps "
                  "up with your hand:")
            print("    ros2 param set /om6dof_controller float_follow_velocity 1.0")
        else:
            print("VERDICT: the motors stayed quiet while you pushed "
                  f"(peak {peak[pushed].max():.0f} raw = "
                  f"{peak[pushed].max() * CURRENT_UNIT_MA:.0f} mA), so the "
                  "resistance is the gearbox, not the servo.")
            print("  Softening the position loop will not help. Making this a "
                  "leader arm needs current control with friction "
                  "compensation -- friction being the larger term.")
        return 0
    except KeyboardInterrupt:
        return 130
    finally:
        if entered_float and not args.keep_float:
            node.set_mode("JOINT")
            deadline = time.monotonic() + 2.0
            while time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.05)
            print("\nReturned to JOINT.")
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
