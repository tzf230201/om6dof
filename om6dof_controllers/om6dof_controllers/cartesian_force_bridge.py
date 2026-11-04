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

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float64MultiArray, String
from sensor_msgs.msg import JointState

from .control_math import (MODE_FORCE, normalize_operation_mode, validated_control_command,
                           tip_rotation_to_cartesian_rpy, rotation_from_zyx)


def directional_wrench(direction: Sequence[float], limits: Sequence[float],
                       force_newtons: float = 0.0) -> list[float]:
    """Return fixed force/moment magnitudes from direction signs, within caps."""
    values = validated_control_command(direction)
    maximum = validated_control_command(limits)
    if any(value < 0.0 for value in maximum):
        raise ValueError("wrench_limits must be nonnegative")
    if not math.isfinite(force_newtons) or force_newtons < 0.0:
        raise ValueError('force_newtons must be finite and nonnegative')
    result = [
        (limit if value > 0.0 else -limit if value < 0.0 else 0.0)
        for value, limit in zip(values, maximum)
    ]
    if force_newtons > 0.0:
        signs = [1.0 if v > 0.0 else -1.0 if v < 0.0 else 0.0 for v in values[:3]]
        norm = math.sqrt(sum(v * v for v in signs))
        if norm:
            force = [force_newtons * v / norm for v in signs]
            scale = min([1.0] + [limit / abs(v) for limit, v in zip(maximum[:3], force) if v])
            result[:3] = [v * scale for v in force]
    return result


def custom_directional_wrench(direction, limits, force_newtons, position, rotation,
                              origin_xy=(.012, 0.)):
    """CUSTOM R/tangential/Z force and gripper roll/pitch moment, in base axes."""
    direction = validated_control_command(direction)
    direction[5] = 0.0
    local = np.asarray(directional_wrench(direction, limits, force_newtons))
    position, rotation = np.asarray(position), np.asarray(rotation)
    origin_xy = np.asarray(origin_xy)
    if (position.shape != (3,) or rotation.shape != (3, 3) or origin_xy.shape != (2,)
            or not all(np.all(np.isfinite(v)) for v in (position, rotation, origin_xy))):
        raise ValueError('finite TCP pose and cylindrical origin required')
    radial = position[:2] - origin_xy
    radius = float(np.linalg.norm(radial))
    if radius < 1e-6:
        raise ValueError('CUSTOM direction is undefined at the cylindrical axis')
    x, y = radial / radius
    basis = np.array([[x, -y, 0.], [y, x, 0.], [0., 0., 1.]])
    forward_rotation = rotation_from_zyx(*tip_rotation_to_cartesian_rpy(rotation))
    result = np.r_[basis @ local[:3], forward_rotation @ local[3:]]
    # Rotation must never turn two allowed local components into an oversized
    # base component. Scale each vector as a whole to preserve its direction.
    for start in (0, 3):
        scale = min([1.] + [limits[i] / abs(result[i]) for i in range(start, start + 3)
                            if abs(result[i]) > 1e-12])
        result[start:start + 3] *= scale
    return result.tolist()


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
        self.declare_parameter('force_newtons', 0.0)
        self.declare_parameter('input_frame', 'base')
        self.declare_parameter('base_link', 'link1')
        self.declare_parameter('tip_link', 'end_effector_link')
        self.declare_parameter('cylindrical_origin_xy', [.012, 0.])
        self.declare_parameter("input_timeout_seconds", 0.20)
        self.declare_parameter("publish_rate_hz", 50.0)

        self.limits = [float(value) for value in self.get_parameter("wrench_limits").value]
        self.force_newtons = float(self.get_parameter('force_newtons').value)
        self.timeout = float(self.get_parameter("input_timeout_seconds").value)
        self.rate = float(self.get_parameter("publish_rate_hz").value)
        self.input_frame = str(self.get_parameter('input_frame').value)
        self.origin_xy = list(self.get_parameter('cylindrical_origin_xy').value)
        if (self.input_frame not in ('base', 'custom') or len(self.origin_xy) != 2
                or not all(math.isfinite(v) for v in self.origin_xy)):
            raise RuntimeError('invalid FORCE input frame or cylindrical origin')
        self.ik = None
        self.joints = None
        self.joints_at = 0.0
        if self.input_frame == 'custom':
            from .ik_solver import IKSolver
            self.ik = IKSolver(base_link=str(self.get_parameter('base_link').value),
                               tip_link=str(self.get_parameter('tip_link').value))
            self.create_subscription(JointState, '/joint_states', self._on_joints,
                QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
        if (not math.isfinite(self.force_newtons) or self.force_newtons < 0.0
                or len(self.limits) != 6 or not all(math.isfinite(value) and value >= 0.0 for value in self.limits)
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
            Float64MultiArray, str(self.get_parameter("control_cmd_topic").value), self._on_direction,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
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

    def _on_joints(self, msg: JointState) -> None:
        positions = dict(zip(msg.name, msg.position))
        try:
            joints = [float(positions[f'joint{i}']) for i in range(1, 7)]
            if not all(math.isfinite(q) for q in joints):
                raise ValueError('non-finite joint position')
            self.joints, self.joints_at = joints, time.monotonic()
        except (KeyError, ValueError):
            self.joints, self.joints_at = None, 0.0

    def _tick(self) -> None:
        fresh = self.mode == MODE_FORCE and 0.0 <= time.monotonic() - self.direction_at <= self.timeout
        if self.input_frame == 'custom':
            fresh = fresh and self.joints is not None and 0.0 <= time.monotonic() - self.joints_at <= self.timeout
        if fresh:
            try:
                if self.input_frame == 'custom':
                    position, rotation = self.ik.fk_pose(np.asarray(self.joints))
                    wrench = custom_directional_wrench(self.direction, self.limits,
                        self.force_newtons, position, rotation, self.origin_xy)
                else:
                    wrench = directional_wrench(self.direction, self.limits, self.force_newtons)
            except (ValueError, RuntimeError) as exc:
                self.get_logger().warn(f'FORCE direction held: {exc}', throttle_duration_sec=2.)
                self.publisher.publish(Float64MultiArray(data=[0.] * 6))
                self.sent_stop = True
                return
            self.publisher.publish(Float64MultiArray(data=wrench))
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

