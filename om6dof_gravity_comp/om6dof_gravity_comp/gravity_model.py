"""Gravity torque for the OM6DOF, from the URDF's own mass data.

Why this exists separately from ik_solver
-----------------------------------------
``om6dof_controller.ik_solver`` builds its KDL chain with bare segments --
``Segment(name, joint, frame)`` and no ``RigidBodyInertia``. That is fine for
kinematics, but every dynamics solver would then read the arm as massless and
hand back zero torque. This module builds the same chain again with the link
inertias attached.

Only mass and centre of mass matter here. Gravity torque is the gradient of
potential energy, so the inertia tensors never enter it -- which is just as
well, since several in this URDF are placeholders (ixx = 0.1 on an 82 g link).
Zeros are passed for them deliberately rather than propagating fiction.

Approach follows ROBOTIS's om_gravity_compensation_controller for the same
XM430 hardware: recursive Newton-Euler with gravity, then per-joint scaling
and friction compensation, because the Dynamixel gearboxes have enough
stiction that a textbook gravity model alone does not feel weightless.
"""

from __future__ import annotations

import math
from typing import List, Optional, Sequence, Tuple

import numpy as np
import PyKDL
from urdf_parser_py.urdf import URDF

GRAVITY = 9.80665
DEFAULT_BASE_LINK = "link1"
DEFAULT_TIP_LINK = "end_effector_link"


def _rigid_body_inertia(link) -> PyKDL.RigidBodyInertia:
    """Mass and centre of mass of a URDF link, as KDL sees it.

    The rotational inertia is deliberately zero: it plays no part in gravity
    torque, and the values in this URDF are not trustworthy.
    """
    inertial = getattr(link, "inertial", None)
    if inertial is None or not inertial.mass:
        return PyKDL.RigidBodyInertia(0.0, PyKDL.Vector(0, 0, 0),
                                      PyKDL.RotationalInertia())
    origin = getattr(inertial, "origin", None)
    xyz = origin.xyz if (origin and origin.xyz) else [0.0, 0.0, 0.0]
    return PyKDL.RigidBodyInertia(
        float(inertial.mass),
        PyKDL.Vector(*[float(v) for v in xyz]),
        PyKDL.RotationalInertia(),
    )


def _lumped_tip_inertia(
    robot, chain_links: Sequence[str], tip_link: str
) -> Tuple[float, PyKDL.Vector]:
    """Mass hanging off the chain that the chain itself does not carry.

    The gripper fingers branch off the wrist, so a chain that stops at the
    tool tip would ignore them. They are real load on joints 2 and 3, so
    their mass is folded into the tip as a single equivalent point.
    """
    total = 0.0
    moment = PyKDL.Vector(0, 0, 0)
    for name, link in robot.link_map.items():
        if name in chain_links:
            continue
        inertial = getattr(link, "inertial", None)
        if inertial is None or not inertial.mass:
            continue
        origin = getattr(inertial, "origin", None)
        xyz = origin.xyz if (origin and origin.xyz) else [0.0, 0.0, 0.0]
        mass = float(inertial.mass)
        total += mass
        moment += PyKDL.Vector(*[float(v) for v in xyz]) * mass
    if total <= 0.0:
        return 0.0, PyKDL.Vector(0, 0, 0)
    return total, moment / total


def build_chain(
    urdf_str: str,
    base_link: str = DEFAULT_BASE_LINK,
    tip_link: str = DEFAULT_TIP_LINK,
    include_offchain_mass: bool = True,
) -> Tuple[PyKDL.Chain, List[str]]:
    """A KDL chain carrying the URDF's masses, plus the actuated joint names.

    Segment construction mirrors ``ik_solver._kdl_chain_from_urdf`` exactly --
    the axis rotated into the parent frame, the joint reference at the origin
    -- because a chain that disagrees with the kinematics would produce
    torques for a different robot.
    """
    robot = URDF.from_xml_string(urdf_str)
    joint_names = robot.get_chain(base_link, tip_link, joints=True, links=False)
    link_names = robot.get_chain(base_link, tip_link, joints=False, links=True)

    extra_mass, extra_com = (
        _lumped_tip_inertia(robot, link_names, tip_link)
        if include_offchain_mass else (0.0, PyKDL.Vector(0, 0, 0))
    )

    chain = PyKDL.Chain()
    actuated: List[str] = []
    for index, jname in enumerate(joint_names):
        joint = robot.joint_map[jname]
        origin = joint.origin
        xyz = origin.xyz if (origin and origin.xyz) else [0.0, 0.0, 0.0]
        rpy = origin.rpy if (origin and origin.rpy) else [0.0, 0.0, 0.0]
        frame = PyKDL.Frame(PyKDL.Rotation.RPY(*rpy), PyKDL.Vector(*xyz))

        if joint.type == "fixed":
            kdl_joint = PyKDL.Joint(joint.name, PyKDL.Joint.Fixed)
        else:
            axis = joint.axis if getattr(joint, "axis", None) else [0.0, 0.0, 1.0]
            axis_parent = frame.M * PyKDL.Vector(*axis)
            if joint.type in ("revolute", "continuous"):
                kind = PyKDL.Joint.RotAxis
            elif joint.type == "prismatic":
                kind = PyKDL.Joint.TransAxis
            else:
                raise ValueError(f"unsupported joint type: {joint.type}")
            kdl_joint = PyKDL.Joint(joint.name, frame.p, axis_parent, kind)
            actuated.append(joint.name)

        inertia = _rigid_body_inertia(robot.link_map[joint.child])
        if extra_mass > 0.0 and index == len(joint_names) - 1:
            # Fold the branch mass into the last segment: same total weight,
            # same first moment about that frame.
            combined = inertia.getMass() + extra_mass
            centre = (
                (inertia.getCOG() * inertia.getMass() + extra_com * extra_mass)
                / combined
            )
            inertia = PyKDL.RigidBodyInertia(
                combined, centre, PyKDL.RotationalInertia()
            )
        chain.addSegment(PyKDL.Segment(joint.name, kdl_joint, frame, inertia))
    return chain, actuated


class GravityModel:
    """Per-joint torque needed to hold the arm still against gravity."""

    def __init__(
        self,
        urdf_str: str,
        base_link: str = DEFAULT_BASE_LINK,
        tip_link: str = DEFAULT_TIP_LINK,
        gravity: float = GRAVITY,
        include_offchain_mass: bool = True,
    ) -> None:
        self.chain, self.joint_names = build_chain(
            urdf_str, base_link, tip_link, include_offchain_mass
        )
        self.gravity = float(gravity)
        # Base-frame gravity. The chain starts at link1, whose frame is the
        # world orientation, so -Z is down.
        self._solver = PyKDL.ChainDynParam(
            self.chain, PyKDL.Vector(0.0, 0.0, -self.gravity)
        )
        self.dof = self.chain.getNrOfJoints()

    def torques(self, positions: Sequence[float]) -> np.ndarray:
        """Gravity torque, in newton-metres, at this joint configuration."""
        q = np.asarray(positions, dtype=float)
        if q.shape != (self.dof,):
            raise ValueError(f"expected {self.dof} joint positions, got {q.shape}")
        if not np.all(np.isfinite(q)):
            raise ValueError("joint positions must be finite")
        jnt = PyKDL.JntArray(self.dof)
        for index in range(self.dof):
            jnt[index] = float(q[index])
        out = PyKDL.JntArray(self.dof)
        self._solver.JntToGravity(jnt, out)
        return np.array([out[i] for i in range(self.dof)])

    def potential_energy(self, positions: Sequence[float]) -> float:
        """Total gravitational potential energy, for checking the torques.

        Gravity torque is the gradient of this, so differentiating it gives an
        independent answer that shares no code with ChainDynParam.
        """
        q = np.asarray(positions, dtype=float)
        jnt = PyKDL.JntArray(self.dof)
        for index in range(self.dof):
            jnt[index] = float(q[index])
        solver = PyKDL.ChainFkSolverPos_recursive(self.chain)
        energy = 0.0
        for segment in range(self.chain.getNrOfSegments()):
            frame = PyKDL.Frame()
            solver.JntToCart(jnt, frame, segment + 1)
            inertia = self.chain.getSegment(segment).getInertia()
            mass = inertia.getMass()
            if mass <= 0.0:
                continue
            centre = frame * inertia.getCOG()
            energy += mass * self.gravity * centre.z()
        return energy

    def total_mass(self) -> float:
        return sum(
            self.chain.getSegment(i).getInertia().getMass()
            for i in range(self.chain.getNrOfSegments())
        )


def friction_compensation(
    velocities: Sequence[float],
    kinetic_scalars: Sequence[float],
    velocity_thresholds: Sequence[float],
) -> np.ndarray:
    """Torque added in the direction of motion to cancel gearbox drag.

    Without it the arm still resists being pushed: the XM430 gearboxes have
    enough stiction that a correct gravity model alone does not feel
    weightless. Scaled by how fast the joint is already moving so it fades out
    at rest instead of buzzing around zero.
    """
    velocity = np.asarray(velocities, dtype=float)
    scalars = np.asarray(kinetic_scalars, dtype=float)
    thresholds = np.asarray(velocity_thresholds, dtype=float)
    ramp = np.clip(np.abs(velocity) / np.maximum(thresholds, 1e-6), 0.0, 1.0)
    return np.sign(velocity) * scalars * ramp
