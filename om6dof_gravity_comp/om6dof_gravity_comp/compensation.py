"""Gravity and friction compensation, for using the arm as a leader.

    I_command = scale * ( I_gravity(q) + I_friction(qd) )

What this does not do on its own
--------------------------------
It publishes a current command. It does not put the servos into current
mode, and it cannot: the arm's ros2_control description exposes a position
command interface only, and the XM430s run in Operating Mode 3. Switching
that is a deliberate hardware change with real consequences, documented in
the README, and left to a person.

Until then this node is useful for watching what it *would* command, against
the current the arm is actually drawing.

Why the default scale is zero
-----------------------------
Because the model is not ready, and the node should not be the thing that
decides otherwise. Measured on this arm:

  * it holds up across recordings while the joints are moving (R2 ~0.92)
  * it falls apart when they are not (R2 0.37 down to -0.62)

A leader arm spends much of its time nearly still -- someone holding it,
pausing, thinking -- which is exactly where the model has never been shown to
work. Commanding current there could as easily drop the arm as hold it. So
the scale starts at 0.0, every command is zero, and raising it is an explicit
act by someone watching the arm.

Every guard from the plan is here: saturation per motor, ramping, a watchdog
on the joint feed, joint-limit and velocity cut-outs, sanity checks on the
numbers, and a zero-current path that runs on any of them tripping.
"""

from __future__ import annotations

import math
import os
import subprocess
import sys
import time
from typing import Dict, List, Optional

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float64MultiArray, String
from std_srvs.srv import Trigger

from om6dof_gravity_comp.estimator import coulomb_feature
from om6dof_gravity_comp.identify import DEFAULT_STRIBECK_SCALE, feature_names, load_yaml
from om6dof_gravity_comp.units import (
    CURRENT_TICK_MA,
    CURRENT_UNIT_RAW,
    JOINT_NAMES,
    match_stack_rmw,
    order_by_joint,
)

# XM430 stall current is about 2.3 A ~ 855 raw ticks. A quarter of that is
# well inside continuous rating and still more than the arm's own weight
# needs; nothing here should ever approach the stall figure.
DEFAULT_CURRENT_LIMIT_RAW = 200.0
STATE_STALE_S = 0.2          # tighter than the estimator: this one acts
RAMP_SECONDS = 2.0
LIMIT_MARGIN_RAD = 0.10      # stop pushing this far from a joint limit
MAX_SAFE_VELOCITY = 1.0      # rad/s; above this something is wrong


class GravityCompensation(Node):
    def __init__(self) -> None:
        super().__init__("om6dof_gravity_compensation")

        self.declare_parameter("model_file", "")
        self.declare_parameter(
            "command_topic",
            "/forward_effort_controller/commands")
        self.declare_parameter("require_controller", True)
        self.declare_parameter("joint_state_topic", "/joint_states")
        self.declare_parameter("publish_rate_hz", 100.0)
        # The one number that matters. Zero means every command is zero.
        self.declare_parameter("scale", 0.0)
        self.declare_parameter("current_limit_raw",
                               [DEFAULT_CURRENT_LIMIT_RAW] * 6)
        self.declare_parameter("joint_lower", [-2.8, -2.0, -1.9, -2.8, -2.0, -2.8])
        self.declare_parameter("joint_upper", [2.8, 2.1, 2.1, 2.8, 2.1, 2.8])
        self.declare_parameter("compensate_friction", True)

        # Which joints may receive current at all. The default is joint2 alone
        # because that is the only joint tested on hardware; widening this is
        # a deliberate act, not something that happens by loading a file.
        self.declare_parameter("enabled_joints", ["joint2"])
        # I_cmd = sign * alpha * G(q) / Kt, then amperes -> register ticks.
        # Kt here is the initial approximation, not a calibrated value; alpha
        # is the per-joint correction meant to absorb the difference once it
        # has been identified. Neither hides missing payload physics: the
        # payload lives in the URDF, where it belongs.
        self.declare_parameter("torque_constants_nm_per_a",
                               [0.61, 0.61, 0.61, 0.61, 0.40, 0.40])
        self.declare_parameter("alpha", [1.0] * 6)
        self.declare_parameter("joint_signs", [1.0] * 6)

        path = str(self.get_parameter("model_file").value).strip()
        if path:
            self.model = load_yaml(path)
            self._check_model()
        else:
            # Model-only operation: pure G(q) from the URDF, no identified
            # friction. Without this the node refused to start at all until an
            # identification run existed for every joint, which made the whole
            # six-joint posture path unreachable even though the gravity model
            # itself was ready.
            self.model = {"joints": {}}
            self.get_logger().info(
                "no model_file: running model-only, gravity from the URDF "
                "with no friction compensation")

        self.stribeck = any(
            "stribeck" in (entry.get("coefficients") or {})
            for entry in self.model["joints"].values())
        self.features = feature_names(self.stribeck)
        self.stribeck_scale = float(
            self.model.get("stribeck_scale", DEFAULT_STRIBECK_SCALE))
        self.deadzone = float(self.model.get("velocity_deadzone", 0.02))
        self.deadzone_mode = str(self.model.get("deadzone_mode", "smooth"))

        self.coefficients: Dict[str, Optional[np.ndarray]] = {}
        self.identifiable: Dict[str, bool] = {}
        for joint in JOINT_NAMES:
            entry = self.model["joints"].get(joint) or {}
            coefficients = entry.get("coefficients")
            if not coefficients:
                self.coefficients[joint] = None
                self.identifiable[joint] = False
                continue
            self.coefficients[joint] = np.array(
                [coefficients[name] for name in self.features])
            # A gravity term fitted where gravity barely varies is noise; it
            # must not be pushed into a motor.
            self.identifiable[joint] = bool(entry.get("gravity_identifiable", False))

        enabled = [str(v) for v in self.get_parameter("enabled_joints").value]
        unknown = sorted(set(enabled) - set(JOINT_NAMES))
        if unknown:
            raise RuntimeError(f"enabled_joints names no such joint: {unknown}")
        self.enabled = np.array([j in enabled for j in JOINT_NAMES], dtype=bool)
        self.kt = np.array(
            self.get_parameter("torque_constants_nm_per_a").value, float)
        if np.any(self.kt <= 0.0):
            raise RuntimeError("torque_constants_nm_per_a must all be positive")
        self.alpha = np.array(self.get_parameter("alpha").value, float)
        self.signs = np.array(self.get_parameter("joint_signs").value, float)
        for name, value in (("alpha", self.alpha), ("joint_signs", self.signs),
                            ("torque_constants_nm_per_a", self.kt)):
            if value.shape != (6,):
                raise RuntimeError(f"{name} must have six entries")

        self.gravity = self._load_gravity_model()
        self.joint_lower = np.array(self.get_parameter("joint_lower").value, float)
        self.joint_upper = np.array(self.get_parameter("joint_upper").value, float)

        self._positions: Dict[str, float] = {}
        self._velocities: Dict[str, float] = {}
        self._stamp = 0.0
        self._ramp = 0.0
        self._armed = False
        self._halt_reason = ""

        self.pub_command = self.create_publisher(
            Float64MultiArray, str(self.get_parameter("command_topic").value), 10)
        self.pub_status = self.create_publisher(
            String, "/om6dof/compensation_status", 10)
        self.create_subscription(
            JointState, str(self.get_parameter("joint_state_topic").value),
            self._on_js, 20)
        self.create_service(Trigger, "/om6dof/compensation/arm", self._on_arm)
        self.create_service(Trigger, "/om6dof/compensation/stop", self._on_stop)

        rate = max(1.0, float(self.get_parameter("publish_rate_hz").value))
        self.period = 1.0 / rate
        self.create_timer(self.period, self._tick)

        self._announce()
        if bool(self.get_parameter("require_controller").value):
            self._warn_if_no_listener()

    def _warn_if_no_listener(self) -> None:
        """Say plainly when the commands are going nowhere.

        With the standard hardware description there is no effort interface,
        so forward_effort_controller cannot load and every command published
        here is discarded. That is the safe state, but it should not look
        like the arm is being compensated when it is not.
        """
        if self.pub_command.get_subscription_count() > 0:
            return
        self.get_logger().warn(
            f"nothing is subscribed to {self.get_parameter('command_topic').value}. "
            "The arm is running the position-only description, so these "
            "commands reach no motor. To change that, bring the stack up "
            "with om6dof.ros2_control.current.xacro and activate "
            "forward_effort_controller.")

    # -- startup ----------------------------------------------------------
    def _check_model(self) -> None:
        unit = self.model.get("current_unit")
        if unit and unit != CURRENT_UNIT_RAW:
            raise RuntimeError(
                f"model is in {unit!r}, this node commands {CURRENT_UNIT_RAW!r}")
        order = self.model.get("joint_order")
        if order and list(order) != list(JOINT_NAMES):
            raise RuntimeError(f"model joint order {order} is not {list(JOINT_NAMES)}")

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

    def _announce(self) -> None:
        scale = float(self.get_parameter("scale").value)
        live = [j for j, on in zip(JOINT_NAMES, self.enabled) if on]
        self.get_logger().info(
            f"gravity compensation loaded, scale={scale:.2f}, "
            f"friction={'on' if self.get_parameter('compensate_friction').value else 'off'}")
        self.get_logger().info(
            f"joints that may receive current: {', '.join(live) or 'none'}; "
            "every other joint is held at exactly zero")
        model_only = [j for j, on in zip(JOINT_NAMES, self.enabled)
                      if on and self.coefficients[j] is None]
        if model_only:
            self.get_logger().info(
                f"{', '.join(model_only)} run straight off the URDF gravity "
                "model: posture-dependent torque, no friction term, so they "
                "resist a push more than an identified joint would")
        blind = [j for j, on in zip(JOINT_NAMES, self.enabled)
                 if on and self.coefficients[j] is not None
                 and not self.identifiable[j]]
        if blind:
            self.get_logger().warn(
                f"{', '.join(blind)} have a fitted model whose gravity term "
                "was not identifiable; they get friction compensation only")
        if any(self.coefficients[j] is not None for j in JOINT_NAMES):
            self.get_logger().warn(
                "The fitted part of this model has only been shown to hold "
                "while the joints are moving. At rest it has not been "
                "validated and scored as low as R2 = -0.62 on a second "
                "recording. Raise scale only with a hand on the arm.")
        if scale <= 0.0:
            self.get_logger().info(
                "scale is 0.0, so every command is zero. Set it with: "
                "ros2 param set /om6dof_gravity_compensation scale 0.1")

    # -- inputs -----------------------------------------------------------
    def _on_js(self, msg: JointState) -> None:
        for index, name in enumerate(msg.name):
            if index < len(msg.position):
                self._positions[name] = float(msg.position[index])
            if index < len(msg.velocity):
                self._velocities[name] = float(msg.velocity[index])
        self._stamp = time.monotonic()

    def _on_arm(self, request, response):
        self._armed = True
        self._halt_reason = ""
        self.get_logger().warn("compensation armed")
        response.success = True
        response.message = "armed; output still follows the scale parameter"
        return response

    def _on_stop(self, request, response):
        self._armed = False
        self._ramp = 0.0
        self._publish(np.zeros(6))
        self.get_logger().warn("compensation stopped, zero current commanded")
        response.success = True
        response.message = "stopped"
        return response

    def _vector(self, source: Dict[str, float]) -> Optional[np.ndarray]:
        names = list(source.keys())
        values = order_by_joint(names, [source[n] for n in names])
        if any(value is None for value in values):
            return None
        return np.array(values, dtype=float)

    # -- the loop ---------------------------------------------------------
    def _halt(self, reason: str) -> None:
        if reason != self._halt_reason:
            self.get_logger().warn(f"holding at zero current: {reason}")
            self._halt_reason = reason
        self._ramp = 0.0
        self._publish(np.zeros(6))

    def _publish(self, command: np.ndarray) -> None:
        self.pub_command.publish(Float64MultiArray(data=command.tolist()))
        self.pub_status.publish(String(data=(
            f"armed={self._armed} ramp={self._ramp:.2f} "
            f"halt={self._halt_reason or 'none'}")))

    def _tick(self) -> None:
        scale = float(self.get_parameter("scale").value)
        if not self._armed or scale <= 0.0:
            self._ramp = 0.0
            self._publish(np.zeros(6))
            return

        # Watchdog: a stale or missing joint feed means the model is being
        # evaluated at a pose the arm may have left.
        if not self._stamp or time.monotonic() - self._stamp > STATE_STALE_S:
            return self._halt("joint feedback is stale")

        q = self._vector(self._positions)
        qd = self._vector(self._velocities)
        if q is None or qd is None:
            return self._halt("incomplete joint state")
        if not (np.all(np.isfinite(q)) and np.all(np.isfinite(qd))):
            return self._halt("non-finite joint state")
        if np.max(np.abs(qd)) > MAX_SAFE_VELOCITY:
            return self._halt(
                f"joint moving at {np.max(np.abs(qd)):.2f} rad/s")

        try:
            nominal = self.gravity.torques(q)
        except Exception as exc:
            return self._halt(f"gravity model failed: {exc}")

        command = np.zeros(6)
        friction_on = bool(self.get_parameter("compensate_friction").value)
        for index, joint in enumerate(JOINT_NAMES):
            if not self.enabled[index]:
                continue
            coefficients = self.coefficients[joint]
            if coefficients is None:
                # No identification for this joint: use the model directly.
                # Same posture, same G(q) -- what is missing is only the
                # friction term, so the arm resists a push more than a fully
                # identified joint would. That is the safe direction to err.
                command[index] = (
                    self.signs[index] * self.alpha[index] * nominal[index]
                    / self.kt[index] * 1000.0 / CURRENT_TICK_MA)
                continue
            values = dict(zip(self.features, coefficients))
            term = 0.0
            # A gravity coefficient fitted where gravity barely varies is
            # noise; leave it out rather than push it into a motor.
            if self.identifiable[joint]:
                term += values["gravity_nominal"] * nominal[index] + values["bias"]
            if friction_on:
                term += values["coulomb"] * coulomb_feature(
                    qd[index], self.deadzone, self.deadzone_mode)
                term += values["viscous"] * qd[index]
                if "stribeck" in values:
                    term += values["stribeck"] * math.copysign(
                        math.exp(-abs(qd[index]) / self.stribeck_scale),
                        qd[index]) if qd[index] != 0.0 else 0.0
            command[index] = (self.signs[index] * self.alpha[index] * term)

        if not np.all(np.isfinite(command)):
            return self._halt("model produced a non-finite command")

        # Do not push a joint further into its own end stop.
        near_low = q <= self.joint_lower + LIMIT_MARGIN_RAD
        near_high = q >= self.joint_upper - LIMIT_MARGIN_RAD
        command[near_low & (command < 0)] = 0.0
        command[near_high & (command > 0)] = 0.0

        self._ramp = min(1.0, self._ramp + self.period / RAMP_SECONDS)
        command = command * scale * self._ramp

        limits = np.array(self.get_parameter("current_limit_raw").value, float)
        command = np.clip(command, -np.abs(limits), np.abs(limits))

        self._halt_reason = ""
        self._publish(command)


def main(args=None) -> int:
    match_stack_rmw()
    rclpy.init(args=args)
    try:
        node = GravityCompensation()
    except RuntimeError as exc:
        print(f"cannot start: {exc}", file=sys.stderr)
        rclpy.shutdown()
        return 1
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node._publish(np.zeros(6))
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
