"""Record joint angle, velocity and motor current for offline identification.

Read-only with respect to the motors: this node subscribes and writes a file.
It publishes nothing and commands nothing, so it is safe to run alongside
whatever is driving the arm.

Both the raw current and its milliamp equivalent are written. The raw column
is the one the driver actually reports; the mA column is derived through the
tick size stated in the hardware description, and is there for reading
convenience only. Fitting is done on raw so that no conversion error enters
the identified parameters.
"""

from __future__ import annotations

import argparse
import csv
import os
import subprocess
import sys
import time
from datetime import datetime
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, String

from om6dof_gravity_comp.units import (
    match_stack_rmw,
    CURRENT_TICK_MA,
    CURRENT_UNIT_RAW,
    JOINT_NAMES,
    JOINT_SERVO_MODELS,
    order_by_joint,
    raw_to_ma,
)

DEFAULT_DIRECTORY = os.path.expanduser("~/om6dof_identification")


def _git_describe(path: str) -> str:
    try:
        result = subprocess.run(
            ["git", "-C", path, "rev-parse", "--short", "HEAD"],
            capture_output=True, text=True, timeout=10)
        return result.stdout.strip() or "unknown"
    except (OSError, subprocess.SubprocessError):
        return "unknown"


class IdentificationLogger(Node):
    def __init__(self, path: str, rate_hz: float, description_source: str):
        super().__init__("om6dof_identification_logger")
        self.path = path
        self.rate_hz = rate_hz
        self.description_source = description_source

        self.positions = {}
        self.velocities = {}
        self.efforts = {}
        self._stamp = 0.0
        self.mode = ""
        self.remote: Optional[bool] = None
        self.rows = 0
        self.skipped = 0

        latched = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(JointState, "/joint_states", self._on_js, 50)
        self.create_subscription(
            String, "/om6dof/operation_mode/state",
            lambda m: setattr(self, "mode", str(m.data).strip().upper()), latched)
        self.create_subscription(
            Bool, "/om6dof/remote_enabled/state",
            lambda m: setattr(self, "remote", bool(m.data)), latched)

        os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
        self._handle = open(path, "w", newline="")
        self._writer = csv.writer(self._handle)
        self._write_header()
        self.create_timer(1.0 / rate_hz, self._sample)

    def _write_header(self) -> None:
        # Commented metadata lines, so the file stays a valid CSV for anything
        # that skips '#' and stays self-describing for anything that does not.
        meta = [
            f"# om6dof identification dataset",
            f"# written_at_iso: {datetime.now().astimezone().isoformat()}",
            f"# joint_order: {','.join(JOINT_NAMES)}",
            f"# servo_models: {','.join(JOINT_SERVO_MODELS[j] for j in JOINT_NAMES)}",
            f"# current_unit: {CURRENT_UNIT_RAW}",
            f"# current_tick_ma: {CURRENT_TICK_MA}",
            f"# velocity_unit: rad_per_s",
            f"# position_unit: rad",
            f"# sample_rate_hz: {self.rate_hz}",
            f"# robot_description_source: {self.description_source}",
            f"# repo_commit: {_git_describe(os.path.dirname(__file__))}",
        ]
        for line in meta:
            self._handle.write(line + "\n")
        columns = ["t_wall", "t_ros", "operation_mode", "remote_enabled"]
        columns += [f"q_{name}" for name in JOINT_NAMES]
        columns += [f"qd_{name}" for name in JOINT_NAMES]
        columns += [f"i_raw_{name}" for name in JOINT_NAMES]
        columns += [f"i_ma_{name}" for name in JOINT_NAMES]
        self._writer.writerow(columns)

    def _on_js(self, msg: JointState) -> None:
        for index, name in enumerate(msg.name):
            if index < len(msg.position):
                self.positions[name] = float(msg.position[index])
            if index < len(msg.velocity):
                self.velocities[name] = float(msg.velocity[index])
            if index < len(msg.effort):
                self.efforts[name] = float(msg.effort[index])
        self._stamp = time.monotonic()

    def _sample(self) -> None:
        if not self._stamp or time.monotonic() - self._stamp > 0.5:
            self.skipped += 1
            return
        names = list(self.positions.keys())
        q = order_by_joint(names, [self.positions[n] for n in names])
        qd = order_by_joint(
            names, [self.velocities.get(n, float("nan")) for n in names])
        raw = order_by_joint(
            names, [self.efforts.get(n, float("nan")) for n in names])
        # A partial frame would silently become a hole in the regression, so
        # drop it and count it rather than writing blanks.
        if any(value is None for value in q + qd + raw):
            self.skipped += 1
            return

        row = [
            f"{time.time():.6f}",
            f"{self.get_clock().now().nanoseconds * 1e-9:.6f}",
            self.mode or "UNKNOWN",
            "" if self.remote is None else int(self.remote),
        ]
        row += [f"{v:.6f}" for v in q]
        row += [f"{v:.6f}" for v in qd]
        row += [f"{v:.3f}" for v in raw]
        row += [f"{raw_to_ma(v):.2f}" for v in raw]
        self._writer.writerow(row)
        self.rows += 1
        if self.rows % 200 == 0:
            self._handle.flush()
            self.get_logger().info(f"{self.rows} samples")

    def close(self) -> None:
        try:
            self._handle.flush()
            self._handle.close()
        except OSError:
            pass


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", default=None,
                        help="CSV path; defaults to a timestamped file under "
                             f"{DEFAULT_DIRECTORY}")
    parser.add_argument("--rate", type=float, default=100.0,
                        help="samples per second (default 100)")
    parser.add_argument("--seconds", type=float, default=0.0,
                        help="stop after this long; 0 means until Ctrl-C")
    parser.add_argument("--description-source",
                        default="om6dof_description/urdf/om6dof.urdf.xacro")
    args = parser.parse_args(argv)

    path = args.output or os.path.join(
        DEFAULT_DIRECTORY,
        f"identification_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv")

    match_stack_rmw()
    rclpy.init()
    node = IdentificationLogger(path, args.rate, args.description_source)
    node.get_logger().info(f"logging to {path}")
    deadline = time.monotonic() + args.seconds if args.seconds > 0 else None
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.05)
            if deadline and time.monotonic() > deadline:
                break
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        print(f"\nwrote {node.rows} samples to {path}")
        if node.skipped:
            print(f"skipped {node.skipped} incomplete or stale frames")
        node.destroy_node()
        rclpy.shutdown()
    return 0 if node.rows else 1


if __name__ == "__main__":
    sys.exit(main())
