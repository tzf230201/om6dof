"""Publish the gravity torque the OM6DOF needs to hold itself up.

Groundwork for using this arm as a leader: an operator can only backdrive it
comfortably once the motors carry the arm's own weight, leaving only the
inertia and whatever the gearboxes resist with.

What this node does NOT do
--------------------------
It does not command the servos. It cannot: the arm's ros2_control description
exposes only a ``position`` command interface and the XM430s run in Operating
Mode 3 (position). Applying these torques needs Mode 0 (current) and an
``effort`` command interface, which is a hardware-configuration change with
real consequences -- get the model wrong in current mode and the arm falls
under its own weight. That switch is left to a deliberate act, not a side
effect of running this.

So this publishes what it computes, in newton-metres and in the Dynamixel
current units the servos would take, and it is useful on its own for checking
the model against the arm's measured effort before anything is energised.

Approach follows ROBOTIS's om_gravity_compensation_controller for the same
hardware: gravity from the URDF's masses, plus velocity-dependent friction
compensation, because the gearbox drag on these joints is not small next to
the gravity term.
"""

from __future__ import annotations

import os
import subprocess
from typing import Dict, List, Optional

import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

from om6dof_gravity_comp.gravity_model import (
    DEFAULT_BASE_LINK,
    DEFAULT_TIP_LINK,
    GRAVITY,
    GravityModel,
    friction_compensation,
)

STATE_STALE_S = 0.5


class GravityCompNode(Node):
    def __init__(self) -> None:
        super().__init__("om6dof_gravity_comp")

        self.declare_parameter("urdf_package", "om6dof_description")
        self.declare_parameter("urdf_file", "urdf/om6dof.urdf.xacro")
        self.declare_parameter("base_link", DEFAULT_BASE_LINK)
        self.declare_parameter("tip_link", DEFAULT_TIP_LINK)
        self.declare_parameter("gravity", GRAVITY)
        self.declare_parameter("joint_state_topic", "/joint_states")
        self.declare_parameter("torque_topic", "/om6dof/gravity_torque")
        self.declare_parameter("current_topic", "/om6dof/gravity_current_ma")
        # The effort interface on this driver is neither Nm nor mA: the
        # XM430 model file declares "Present Current"/"Goal Current" with
        # scale 1.0 and unit "raw", so it carries the register value
        # directly. Anything that ever commands effort has to send that,
        # which is why it is published alongside the physical units.
        self.declare_parameter("raw_topic", "/om6dof/gravity_effort_raw")
        self.declare_parameter("publish_rate_hz", 50.0)
        # Straight from ROBOTIS's controller for this hardware: joints 2 and 3
        # carry the arm and drag the most, the wrist barely at all.
        self.declare_parameter(
            "kinetic_friction_scalars", [0.4, 0.8, 0.8, 0.1, 0.1, 0.1])
        self.declare_parameter(
            "friction_velocity_thresholds", [1.0] * 6)
        self.declare_parameter("torque_scaling_factors", [1.0] * 6)
        # XM430 stall torque over stall current, near enough for a first pass.
        # Per joint because W350 and W210 differ, and it wants calibrating
        # against measured effort before anyone trusts it.
        self.declare_parameter(
            "torque_constants_nm_per_a", [0.61, 0.61, 0.61, 0.61, 0.40, 0.40])
        self.declare_parameter("current_unit_ma", 2.69)

        self.model = self._load_model()
        self.joint_names = self.model.joint_names
        self.dof = self.model.dof
        self.get_logger().info(
            f"gravity model over {self.joint_names}, "
            f"{self.model.total_mass():.3f} kg on the chain"
        )

        self._positions: Dict[str, float] = {}
        self._velocities: Dict[str, float] = {}
        self._stamp = 0.0

        self.pub_torque = self.create_publisher(
            Float64MultiArray, self._p("torque_topic"), 10)
        self.pub_current = self.create_publisher(
            Float64MultiArray, self._p("current_topic"), 10)
        self.pub_raw = self.create_publisher(
            Float64MultiArray, self._p("raw_topic"), 10)
        self.create_subscription(
            JointState, self._p("joint_state_topic"), self._on_joint_state, 20)
        period = 1.0 / max(1.0, float(self._p("publish_rate_hz")))
        self.create_timer(period, self._tick)

    def _p(self, name: str):
        return self.get_parameter(name).value

    def _load_model(self) -> GravityModel:
        share = get_package_share_directory(str(self._p("urdf_package")))
        path = os.path.join(share, str(self._p("urdf_file")))
        if path.endswith(".xacro"):
            result = subprocess.run(
                ["xacro", path], capture_output=True, text=True, timeout=60)
            if result.returncode != 0:
                raise RuntimeError(f"xacro failed: {result.stderr.strip()}")
            urdf = result.stdout
        else:
            with open(path, "r") as handle:
                urdf = handle.read()
        return GravityModel(
            urdf,
            base_link=str(self._p("base_link")),
            tip_link=str(self._p("tip_link")),
            gravity=float(self._p("gravity")),
        )

    def _on_joint_state(self, msg: JointState) -> None:
        for index, name in enumerate(msg.name):
            if index < len(msg.position):
                self._positions[name] = float(msg.position[index])
            if index < len(msg.velocity):
                self._velocities[name] = float(msg.velocity[index])
        self._stamp = self.get_clock().now().nanoseconds * 1e-9

    def _vector(self, source: Dict[str, float]) -> Optional[np.ndarray]:
        """Joint values in chain order, by name.

        /joint_states does not arrive in chain order, and reading it
        positionally would silently compute torque for the wrong joints.
        """
        values = [source.get(name) for name in self.joint_names]
        if any(value is None for value in values):
            return None
        return np.array(values, dtype=float)

    def _tick(self) -> None:
        now = self.get_clock().now().nanoseconds * 1e-9
        if not self._stamp or now - self._stamp > STATE_STALE_S:
            return
        positions = self._vector(self._positions)
        if positions is None or not np.all(np.isfinite(positions)):
            self.get_logger().warn(
                "joint state incomplete; not publishing torque",
                throttle_duration_sec=5.0)
            return
        velocities = self._vector(self._velocities)
        if velocities is None or not np.all(np.isfinite(velocities)):
            velocities = np.zeros(self.dof)

        torque = self.model.torques(positions)
        torque = torque + friction_compensation(
            velocities,
            self._p("kinetic_friction_scalars"),
            self._p("friction_velocity_thresholds"),
        )
        torque = torque * np.asarray(
            self._p("torque_scaling_factors"), dtype=float)

        constants = np.asarray(
            self._p("torque_constants_nm_per_a"), dtype=float)
        unit = float(self._p("current_unit_ma"))
        # Guard the division: a zero constant in a params file would otherwise
        # publish infinities straight at whatever reads this next.
        safe = np.where(np.abs(constants) < 1e-6, np.nan, constants)
        current_ma = torque / safe * 1000.0        # Nm / (Nm/A) -> A -> mA

        # Raw register units are what the effort interface speaks, and what
        # the measured effort in /joint_states is already in, so this is the
        # figure the two can actually be compared on.
        raw = np.nan_to_num(current_ma) / unit     # mA -> register units

        self.pub_torque.publish(Float64MultiArray(data=torque.tolist()))
        self.pub_current.publish(
            Float64MultiArray(data=np.nan_to_num(current_ma).tolist()))
        self.pub_raw.publish(Float64MultiArray(data=raw.tolist()))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = GravityCompNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
