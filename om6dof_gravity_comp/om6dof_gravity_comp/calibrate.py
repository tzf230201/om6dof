"""Measure what the OM6DOF's motors actually have to supply, one joint at a time.

Why a sweep and not a static reading
------------------------------------
Standing still, this arm is held up mostly by its own gearboxes. At 353:1 the
static friction referred to the joint is comparable to the gravity load, so
the measured current sits far below what gravity alone would need and tells
you almost nothing about the model.

Moving resolves it. Drive the same joint through the same positions in both
directions and the friction term flips sign while the gravity term does not:

    gravity  = (effort_forward + effort_reverse) / 2
    friction = (effort_forward - effort_reverse) / 2

That separation is the whole point of this script. It then fits the measured
gravity term against the model's prediction to give a per-joint scale, and
reports the friction term directly.

Safety
------
Motion goes through om6dof_controller in JOINT mode, so its joint limits,
velocity ceilings and collision checks all still apply -- this never writes to
the hardware interface directly. It moves one joint, slowly, over a small
range, and only after you pass --confirm. Ctrl-C sends a stop before exiting.
"""

from __future__ import annotations

import argparse
import math
import os
import subprocess
import sys
import time
from typing import Dict, List, Optional, Tuple

import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float64MultiArray, String

from om6dof_gravity_comp.gravity_model import GravityModel

JOINT_NAMES = [f"joint{i}" for i in range(1, 7)]
CURRENT_UNIT_MA = 2.69
SETTLE_S = 0.6


class Calibrator(Node):
    def __init__(self, urdf: str) -> None:
        super().__init__("om6dof_gravity_calibrate")
        self.model = GravityModel(urdf)
        self.positions: Dict[str, float] = {}
        self.efforts: Dict[str, float] = {}
        self.mode = ""
        self.remote: Optional[bool] = None
        self._stamp = 0.0

        latched = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.pub = self.create_publisher(
            Float64MultiArray, "/om6dof/control_cmd", 10)
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

    # -- helpers ----------------------------------------------------------
    def spin(self, seconds: float) -> None:
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.01)

    def fresh(self) -> bool:
        return bool(self._stamp) and (time.monotonic() - self._stamp) < 0.5

    def wait_for_state(self, timeout: float = 15.0) -> bool:
        """Block until joint feedback and the latched mode topics have arrived.

        A fixed short spin was not enough: the arm's graph carries enough
        participants that discovery plus transient-local delivery can take
        several seconds, and the checks below then read an unset mode and
        refused a perfectly ready arm.
        """
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.fresh() and self.mode and self.remote is not None:
                return True
        return False

    def q(self) -> Optional[np.ndarray]:
        values = [self.positions.get(n) for n in JOINT_NAMES]
        if any(v is None for v in values):
            return None
        return np.array(values, dtype=float)

    def stop(self) -> None:
        self.pub.publish(Float64MultiArray(data=[0.0] * 6))

    def drive(self, index: int, rate: float) -> None:
        command = [0.0] * 6
        command[index] = float(rate)
        self.pub.publish(Float64MultiArray(data=command))

    def sweep(
        self, index: int, rate: float, span: float, timeout: float
    ) -> List[Tuple[float, float]]:
        """Move until the joint has covered ``span`` radians, sampling as it goes."""
        start = self.q()
        if start is None:
            raise RuntimeError("no joint feedback")
        origin = start[index]
        samples: List[Tuple[float, float]] = []
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            self.drive(index, rate)
            rclpy.spin_once(self, timeout_sec=0.02)
            if not self.fresh():
                raise RuntimeError("joint feedback went stale mid-sweep")
            current = self.q()
            effort = self.efforts.get(JOINT_NAMES[index])
            if current is None or effort is None:
                continue
            samples.append((float(current[index]), float(effort)))
            if abs(current[index] - origin) >= span:
                break
        else:
            raise RuntimeError(
                "joint did not cover the requested span before the timeout; "
                "it may be at a limit"
            )
        self.stop()
        self.spin(SETTLE_S)
        return samples

    # -- the measurement --------------------------------------------------
    def measure(
        self, index: int, rate: float, span: float, bins: int
    ) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        forward = self.sweep(index, +rate, span, timeout=span / rate + 20.0)
        reverse = self.sweep(index, -rate, span, timeout=span / rate + 20.0)
        return separate_gravity_and_friction(forward, reverse, bins)


def separate_gravity_and_friction(
    forward: List[Tuple[float, float]],
    reverse: List[Tuple[float, float]],
    bins: int,
) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Split paired sweeps into their position-dependent and direction-dependent parts.

    Compared at the same positions, the gravity term is identical in both
    directions and the friction term is opposite, so the half-sum isolates one
    and the half-difference the other. Only positions covered by both sweeps
    are usable, which is why this bins rather than pairing samples directly:
    the two passes never land on exactly the same encoder values.
    """
    if not forward or not reverse:
        raise RuntimeError("a sweep returned no samples")
    low = max(min(p for p, _ in forward), min(p for p, _ in reverse))
    high = min(max(p for p, _ in forward), max(p for p, _ in reverse))
    if high - low < 1e-3:
        raise RuntimeError("the two sweeps do not overlap")
    edges = np.linspace(low, high, bins + 1)
    centres, gravity, friction = [], [], []
    for i in range(bins):
        a, b = edges[i], edges[i + 1]
        # The final bin takes its upper edge, or the endpoint is dropped.
        upper_inclusive = i == bins - 1
        f = [e for p, e in forward if a <= p <= b] if upper_inclusive else \
            [e for p, e in forward if a <= p < b]
        r = [e for p, e in reverse if a <= p <= b] if upper_inclusive else \
            [e for p, e in reverse if a <= p < b]
        if not f or not r:
            continue
        centres.append(0.5 * (a + b))
        gravity.append(0.5 * (np.mean(f) + np.mean(r)))
        friction.append(0.5 * (np.mean(f) - np.mean(r)))
    return (np.array(centres), np.array(gravity), np.array(friction))


def _load_urdf() -> str:
    share = get_package_share_directory("om6dof_description")
    path = os.path.join(share, "urdf", "om6dof.urdf.xacro")
    result = subprocess.run(["xacro", path], capture_output=True, text=True,
                            timeout=60)
    if result.returncode != 0:
        raise RuntimeError(f"xacro failed: {result.stderr.strip()}")
    return result.stdout


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--joint", type=int, required=True, choices=range(1, 7),
                        help="which joint to calibrate, 1-6")
    parser.add_argument("--span-deg", type=float, default=20.0,
                        help="how far to sweep, in degrees (default 20)")
    parser.add_argument("--rate", type=float, default=0.15,
                        help="sweep speed in rad/s (default 0.15)")
    parser.add_argument("--bins", type=int, default=8)
    parser.add_argument("--torque-constant", type=float, default=0.61,
                        help="Nm per amp for this joint's servo")
    parser.add_argument("--confirm", action="store_true",
                        help="required: the arm will move")
    parser.add_argument("--dry-run", action="store_true",
                        help="check readiness and exit without moving")
    args = parser.parse_args(argv)

    if not args.confirm and not args.dry_run:
        print("This moves the arm. Re-run with --confirm once the workspace "
              "is clear, or --dry-run to check readiness first.",
              file=sys.stderr)
        return 2
    if args.rate <= 0 or args.rate > 0.5:
        print("--rate must be between 0 and 0.5 rad/s", file=sys.stderr)
        return 2

    rclpy.init()
    node = Calibrator(_load_urdf())
    index = args.joint - 1
    try:
        if not node.wait_for_state():
            print("Timed out waiting for the arm's state. "
                  f"joint_states={'ok' if node.fresh() else 'missing'}, "
                  f"mode={node.mode or 'not received'}, "
                  f"remote={node.remote if node.remote is not None else 'not received'}. "
                  "Is the arm stack running?", file=sys.stderr)
            return 1
        if node.remote is not True or node.mode != "JOINT":
            print(f"Arm must be in JOINT mode with streaming control enabled "
                  f"(now: mode={node.mode or 'UNKNOWN'}, remote={node.remote}).",
                  file=sys.stderr)
            return 1
        if args.dry_run:
            q = node.q()
            print(f"Ready. mode={node.mode}, remote={node.remote}, "
                  f"joint{args.joint} at {math.degrees(q[index]):.1f} deg.")
            print(f"Would sweep {args.span_deg:g} deg at {args.rate:g} rad/s, "
                  "both directions. Re-run with --confirm to do it.")
            return 0

        span = math.radians(args.span_deg)
        print(f"Sweeping joint{args.joint} by {args.span_deg:g} deg at "
              f"{args.rate:g} rad/s, both directions...")
        centres, gravity_raw, friction_raw = node.measure(
            index, args.rate, span, args.bins)
        if centres.size < 2:
            print("Not enough overlapping samples; try a larger --span-deg.",
                  file=sys.stderr)
            return 1

        # What the model says the same positions should need, in raw units.
        predicted = []
        base = node.q()
        for centre in centres:
            q = base.copy()
            q[index] = centre
            torque = node.model.torques(q)[index]
            predicted.append(torque / args.torque_constant * 1000.0 / CURRENT_UNIT_MA)
        predicted = np.array(predicted)

        # Least squares through the origin: how much of the model's prediction
        # the motor actually supplies.
        denominator = float(np.dot(predicted, predicted))
        scale = float(np.dot(predicted, gravity_raw) / denominator) \
            if denominator > 1e-9 else float("nan")
        friction = float(np.mean(np.abs(friction_raw)))
        residual = gravity_raw - scale * predicted
        spread = float(np.std(residual))

        print(f"\n{'position(deg)':>14} {'predicted(raw)':>15} "
              f"{'measured(raw)':>14} {'friction(raw)':>14}")
        for c, p, g, f in zip(centres, predicted, gravity_raw, friction_raw):
            print(f"{math.degrees(c):14.1f} {p:15.1f} {g:14.1f} {f:14.1f}")

        print(f"\njoint{args.joint}:")
        print(f"  torque_scaling_factors[{index}] = {scale:.3f}")
        print(f"  kinetic_friction_scalars[{index}] = "
              f"{friction * CURRENT_UNIT_MA / 1000.0 * args.torque_constant:.3f} Nm "
              f"({friction:.0f} raw)")
        print(f"  residual spread = {spread:.1f} raw")
        if denominator <= 1e-9:
            print("  NOTE: gravity barely loads this joint in this range, so "
                  "the scale is meaningless here. Sweep a joint that carries "
                  "weight, or pick a range where this one does.")
        return 0
    except KeyboardInterrupt:
        print("\ninterrupted", file=sys.stderr)
        return 130
    except RuntimeError as exc:
        print(f"aborted: {exc}", file=sys.stderr)
        return 1
    finally:
        try:
            node.stop()
            node.spin(0.3)
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
