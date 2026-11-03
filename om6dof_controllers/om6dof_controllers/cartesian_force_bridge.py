"""Convert canonical FORCE directions into a bounded base-frame TCP wrench.

This bridge never changes motor operating mode, loads a controller, or claims
an interface.  It can only feed an already active PositionImpedanceController
whose ``enable_cartesian_wrench`` gate is explicitly enabled.  A stale input
or a mode change stops publishing, allowing the controller-side watchdog to
fall back to gravity support and bounded damping.
"""

from __future__ import annotations

import math
import time
from typing import Sequence

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, String

from .control_math import MODE_FORCE, normalize_operation_mode, validated_control_command


def directional_wrench(direction: Sequence[float], limits: Sequence[float]) -> list[float]:
    """Return a sign-only wrench; magnitude is always the configured limit."""
    values = validated_control_command(direction)
    maximum = validated_control_command(limits)
    if any(value < 0.0 for value in maximum):
        raise ValueError("wrench_limits must be nonnegative")
    return [
        (limit if value > 0.0 else -limit if value < 0.0 else 0.0)
        for value, limit in zip(values, maximum)
    ]


class CartesianForceBridge(Node):
    def __init__(self) -> None:
        super().__init__("om6dof_cartesian_force_bridge")
        self.declare_parameter("operation_mode_topic", "/om6dof/operation_mode")
        self.declare_parameter("control_cmd_topic", "/om6dof/control_cmd")
        # The supervised impedance stack deliberately does not run the normal
        # position teleop converter.  The dashboard therefore has its own
        # direction-only input topic; it is still subject to the same mode,
        # limit, and timeout gates as control_cmd.
        self.declare_parameter(
            "direction_topic", "/om6dof/cartesian_force/direction"
        )
        self.declare_parameter(
            "wrench_topic", "/om6dof_position_impedance_controller/wrench"
        )
        # Deliberately zero by default.  A calibrated, reviewed profile is
        # required before the bridge can command any physical wrench.
        self.declare_parameter("wrench_limits", [0.0] * 6)
        self.declare_parameter("input_timeout_seconds", 0.20)
        self.declare_parameter("publish_rate_hz", 50.0)

        self.limits = [float(value) for value in self.get_parameter("wrench_limits").value]
        self.timeout = float(self.get_parameter("input_timeout_seconds").value)
        self.rate = float(self.get_parameter("publish_rate_hz").value)
        if (len(self.limits) != 6 or not all(math.isfinite(value) and value >= 0.0 for value in self.limits)
                or not math.isfinite(self.timeout) or self.timeout <= 0.0
                or not math.isfinite(self.rate) or self.rate <= 0.0):
            raise RuntimeError("invalid Cartesian force bridge parameters")

        self.mode = ""
        self.direction = [0.0] * 6
        self.direction_at = 0.0
        self.sent_stop = False
        self.publisher = self.create_publisher(
            Float64MultiArray, str(self.get_parameter("wrench_topic").value), 1
        )
        self.create_subscription(
            String, str(self.get_parameter("operation_mode_topic").value), self._on_mode, 1
        )
        self.create_subscription(
            Float64MultiArray, str(self.get_parameter("control_cmd_topic").value), self._on_direction, 1
        )
        self.create_subscription(
            Float64MultiArray, str(self.get_parameter("direction_topic").value), self._on_direction, 1
        )
        self.create_timer(1.0 / self.rate, self._tick)

    def _on_mode(self, msg: String) -> None:
        try:
            self.mode = normalize_operation_mode(msg.data)
        except ValueError:
            self.mode = ""
        if self.mode != MODE_FORCE:
            self.direction = [0.0] * 6
            self.direction_at = 0.0

    def _on_direction(self, msg: Float64MultiArray) -> None:
        if self.mode != MODE_FORCE:
            return
        try:
            # The input is intentionally reduced to sign here too, so a rogue
            # publisher cannot turn a larger Float64 value into more force.
            self.direction = [
                1.0 if value > 0.0 else -1.0 if value < 0.0 else 0.0
                for value in validated_control_command(msg.data)
            ]
            self.direction_at = time.monotonic()
        except ValueError:
            self.direction_at = 0.0

    def _tick(self) -> None:
        fresh = self.mode == MODE_FORCE and 0.0 <= time.monotonic() - self.direction_at <= self.timeout
        if fresh:
            self.publisher.publish(Float64MultiArray(data=directional_wrench(self.direction, self.limits)))
            self.sent_stop = False
        elif not self.sent_stop:
            # One explicit zero makes a released stick immediate; stopping the
            # stream afterwards preserves the controller's watchdog semantics.
            self.publisher.publish(Float64MultiArray(data=[0.0] * 6))
            self.sent_stop = True


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CartesianForceBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

