"""Publish the identified current model and what is left over after it.

    I_residual = I_measured - I_model(q, qd)

The residual is what the model cannot account for. Under the low-speed
assumption that is external contact -- a hand on the arm, or something it has
run into.

Read-only. This node subscribes and publishes; it holds no command interface
and writes nothing to the motors.

The residual is in raw Dynamixel current ticks, not newton-metres. Turning it
into a torque needs a calibrated current-to-torque constant, which this arm
does not have yet, so nothing here pretends to output force.
"""

from __future__ import annotations

import math
import os
import subprocess
import sys
from typing import Dict, Optional

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

from om6dof_gravity_comp.identify import FEATURE_NAMES, load_yaml
from om6dof_gravity_comp.units import (
    JOINT_NAMES,
    match_stack_rmw,
    order_by_joint,
)

STATE_STALE_S = 0.5


def coulomb_feature(velocity: float, deadzone: float, mode: str) -> float:
    """One sample of the Coulomb column, matching what the fit used.

    Kept identical to identify.friction_features on purpose: a model fitted
    with the smooth feature and evaluated with the hard one would be
    systematically wrong at low speed, and nothing would flag it.
    """
    if mode == "smooth":
        return math.tanh(float(velocity) / max(float(deadzone), 1e-9))
    if abs(float(velocity)) < float(deadzone):
        return 0.0
    return math.copysign(1.0, float(velocity))


def evaluate_model(
    coefficients: Dict[str, "np.ndarray"],
    nominal_gravity,
    velocity,
    deadzone: float,
    mode: str,
):
    """Split the modelled current into its gravity and friction parts.

    The bias rides with the gravity part: it is a standing offset, not
    something that appears when the joint moves.
    """
    gravity_part = np.zeros(len(JOINT_NAMES))
    friction_part = np.zeros(len(JOINT_NAMES))
    for index, joint in enumerate(JOINT_NAMES):
        a, b, c, d = coefficients[joint]
        gravity_part[index] = a * float(nominal_gravity[index]) + d
        friction_part[index] = (
            b * coulomb_feature(velocity[index], deadzone, mode)
            + c * float(velocity[index])
        )
    return gravity_part + friction_part, gravity_part, friction_part


class CurrentModelEstimator(Node):
    def __init__(self) -> None:
        super().__init__("om6dof_current_estimator")

        self.declare_parameter("model_file", "")
        self.declare_parameter("joint_state_topic", "/joint_states")
        self.declare_parameter("publish_rate_hz", 50.0)

        path = str(self.get_parameter("model_file").value).strip()
        if not path:
            raise RuntimeError(
                "model_file is required: point it at the YAML that "
                "om6dof_gravity_comp.identify wrote")
        self.model = load_yaml(path)
        self._check_units()

        self.coefficients = {}
        for joint in JOINT_NAMES:
            entry = self.model["joints"].get(joint, {})
            if "coefficients" not in entry:
                self.get_logger().warn(
                    f"{joint} has no fitted coefficients; it will publish zero")
                self.coefficients[joint] = np.zeros(len(FEATURE_NAMES))
            else:
                self.coefficients[joint] = np.array(
                    [entry["coefficients"][k] for k in FEATURE_NAMES])

        self.deadzone = float(self.model.get("velocity_deadzone", 0.02))
        self.deadzone_mode = str(self.model.get("deadzone_mode", "smooth"))
        self.gravity = self._load_gravity_model()

        self._positions: Dict[str, float] = {}
        self._velocities: Dict[str, float] = {}
        self._efforts: Dict[str, float] = {}
        self._stamp = 0.0

        self.pub_model = self.create_publisher(
            Float64MultiArray, "/om6dof/current_model", 10)
        self.pub_residual = self.create_publisher(
            Float64MultiArray, "/om6dof/current_residual", 10)
        self.pub_gravity = self.create_publisher(
            Float64MultiArray, "/om6dof/gravity_component", 10)
        self.pub_friction = self.create_publisher(
            Float64MultiArray, "/om6dof/friction_component", 10)
        self.create_subscription(
            JointState, str(self.get_parameter("joint_state_topic").value),
            self._on_js, 20)
        rate = max(1.0, float(self.get_parameter("publish_rate_hz").value))
        self.create_timer(1.0 / rate, self._tick)
        self.get_logger().info(
            f"current model loaded from {path} "
            f"(version {self.model.get('model_version', '?')})")

    def _check_units(self) -> None:
        """Refuse a model fitted in units this node does not speak."""
        from om6dof_gravity_comp.units import CURRENT_UNIT_RAW
        unit = self.model.get("current_unit")
        if unit and unit != CURRENT_UNIT_RAW:
            raise RuntimeError(
                f"model was fitted in {unit!r}, this node reads "
                f"{CURRENT_UNIT_RAW!r}")
        order = self.model.get("joint_order")
        if order and list(order) != list(JOINT_NAMES):
            raise RuntimeError(
                f"model joint order {order} does not match {list(JOINT_NAMES)}")

    def _load_gravity_model(self):
        from ament_index_python.packages import get_package_share_directory
        from om6dof_gravity_comp.gravity_model import GravityModel
        share = get_package_share_directory("om6dof_description")
        path = os.path.join(share, "urdf", "om6dof.urdf.xacro")
        result = subprocess.run(["xacro", path], capture_output=True,
                                text=True, timeout=60)
        if result.returncode != 0:
            raise RuntimeError(f"xacro failed: {result.stderr.strip()}")
        return GravityModel(result.stdout)

    def _on_js(self, msg: JointState) -> None:
        for index, name in enumerate(msg.name):
            if index < len(msg.position):
                self._positions[name] = float(msg.position[index])
            if index < len(msg.velocity):
                self._velocities[name] = float(msg.velocity[index])
            if index < len(msg.effort):
                self._efforts[name] = float(msg.effort[index])
        self._stamp = self.get_clock().now().nanoseconds * 1e-9

    def _vector(self, source: Dict[str, float]) -> Optional[np.ndarray]:
        names = list(source.keys())
        values = order_by_joint(names, [source[n] for n in names])
        if any(v is None for v in values):
            return None
        return np.array(values, dtype=float)

    def _tick(self) -> None:
        now = self.get_clock().now().nanoseconds * 1e-9
        if not self._stamp or now - self._stamp > STATE_STALE_S:
            return
        q = self._vector(self._positions)
        qd = self._vector(self._velocities)
        measured = self._vector(self._efforts)
        if q is None or qd is None or measured is None:
            return
        if not (np.all(np.isfinite(q)) and np.all(np.isfinite(qd))
                and np.all(np.isfinite(measured))):
            return

        nominal = self.gravity.torques(q)
        model, gravity_part, friction_part = evaluate_model(
            self.coefficients, nominal, qd, self.deadzone, self.deadzone_mode)
        residual = measured - model

        self.pub_model.publish(Float64MultiArray(data=model.tolist()))
        self.pub_residual.publish(Float64MultiArray(data=residual.tolist()))
        self.pub_gravity.publish(Float64MultiArray(data=gravity_part.tolist()))
        self.pub_friction.publish(Float64MultiArray(data=friction_part.tolist()))


def main(args=None) -> int:
    match_stack_rmw()
    rclpy.init(args=args)
    try:
        node = CurrentModelEstimator()
    except RuntimeError as exc:
        print(f"cannot start: {exc}", file=sys.stderr)
        rclpy.shutdown()
        return 1
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
