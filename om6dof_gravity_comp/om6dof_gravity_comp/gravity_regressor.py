"""Gravity torque as a linear regressor over mass parameters (Approach B).

The derivation
--------------
Gravity torque is the gradient of potential energy:

    U(q)   = sum_i  m_i * g * z_i(q)
    tau_g  = dU/dq

For a serial chain the world-frame centre of mass of link i is

    p_i(q) = o_i(q) + R_i(q) * c_i

where o_i and R_i come from the kinematics alone, and c_i is the centre of
mass expressed in link i's own frame. Substituting,

    U(q) = g * sum_i [ m_i * o_iz(q) + (m_i c_i) . R_i(q)^T_z ]

which is **linear** in the four numbers per link

    phi_i = [ m_i,  m_i c_ix,  m_i c_iy,  m_i c_iz ]

Differentiating a linear form leaves it linear, so

    tau_g(q) = Y(q) . phi          Y is 6 x 24, phi is 24 x 1

Y depends only on link geometry -- joint origins and axes -- which the URDF
does describe correctly. The masses and centres of mass, which it does not,
live entirely in phi and are identified from the robot instead.

Why this is built numerically rather than symbolically
------------------------------------------------------
Because tau_g is linear in phi, column k of Y is just the gravity torque of a
robot whose only mass is a unit value in parameter k. That is exactly what
KDL already computes, so each column comes from one call against a chain with
one parameter set and the rest zeroed. No hand-derived trigonometry, and
nothing to get subtly wrong the way the 2017 paper's closed form would be if
copied onto a different kinematic structure.

The construction is checked against the nominal model: Y(q) . phi_urdf must
reproduce ChainDynParam's gravity torque for the URDF's own parameters, at
arbitrary configurations. If it does, Y is right and only phi is in question.

Identifiability
---------------
Not all 24 parameters are recoverable. Mass on a link whose axis is parallel
to gravity never produces torque, and consecutive links trade off against one
another, so Y is rank deficient -- the classic base-parameter situation. This
module reports the rank rather than pretending otherwise, and the fit uses a
pseudo-inverse so the answer is the minimum-norm one rather than an arbitrary
point on a solution ridge.
"""

from __future__ import annotations

import math
from typing import List, Optional, Sequence, Tuple

import numpy as np
import PyKDL
from urdf_parser_py.urdf import URDF

from om6dof_gravity_comp.gravity_model import (
    DEFAULT_BASE_LINK,
    DEFAULT_TIP_LINK,
    GRAVITY,
    _frame_of,
)

# Four numbers per link: the mass, then the three components of the first
# moment m*c. Named so a fitted vector can be read back.
PARAMETERS_PER_LINK = ("m", "m_cx", "m_cy", "m_cz")


def _chain_with(
    robot,
    joint_names: Sequence[str],
    loaded_joint: Optional[str],
    mass: float,
    centre: PyKDL.Vector,
) -> PyKDL.Chain:
    """The kinematic chain with mass on exactly one segment, or none at all."""
    chain = PyKDL.Chain()
    for name in joint_names:
        joint = robot.joint_map[name]
        frame = _frame_of(joint)
        if joint.type == "fixed":
            kdl_joint = PyKDL.Joint(joint.name, PyKDL.Joint.Fixed)
        else:
            axis = joint.axis if getattr(joint, "axis", None) else [0.0, 0.0, 1.0]
            axis_parent = frame.M * PyKDL.Vector(*[float(v) for v in axis])
            if joint.type in ("revolute", "continuous"):
                kind = PyKDL.Joint.RotAxis
            elif joint.type == "prismatic":
                kind = PyKDL.Joint.TransAxis
            else:
                raise ValueError(f"unsupported joint type: {joint.type}")
            kdl_joint = PyKDL.Joint(joint.name, frame.p, axis_parent, kind)

        if name == loaded_joint:
            inertia = PyKDL.RigidBodyInertia(
                mass, centre, PyKDL.RotationalInertia())
        else:
            inertia = PyKDL.RigidBodyInertia(
                0.0, PyKDL.Vector(0, 0, 0), PyKDL.RotationalInertia())
        chain.addSegment(PyKDL.Segment(joint.name, kdl_joint, frame, inertia))
    return chain


class GravityRegressor:
    """Y(q) for the arm, plus the URDF's own phi for checking it."""

    def __init__(
        self,
        urdf_str: str,
        base_link: str = DEFAULT_BASE_LINK,
        tip_link: str = DEFAULT_TIP_LINK,
        gravity: float = GRAVITY,
    ) -> None:
        self.robot = URDF.from_xml_string(urdf_str)
        self.joint_names = self.robot.get_chain(
            base_link, tip_link, joints=True, links=False)
        self.actuated = [
            name for name in self.joint_names
            if self.robot.joint_map[name].type != "fixed"
        ]
        self.gravity = float(gravity)
        self._gravity_vector = PyKDL.Vector(0.0, 0.0, -self.gravity)

        # One parameter block per segment that carries a child link, which is
        # every joint in the chain including the fixed ones -- a fixed joint's
        # child still has mass and still loads the joints behind it.
        self.segments = list(self.joint_names)
        self.parameter_names = [
            f"{joint}:{suffix}"
            for joint in self.segments
            for suffix in PARAMETERS_PER_LINK
        ]

        self._solvers = self._build_solvers()

    def _build_solvers(self) -> List[Tuple[PyKDL.ChainDynParam, PyKDL.ChainDynParam]]:
        """One solver pair per parameter: unit mass at origin, and offset.

        The offset chain carries the same unit mass one metre along an axis,
        so subtracting the two isolates the first-moment column from the mass
        column.
        """
        # The chains are kept alive alongside their solvers. ChainDynParam
        # holds the chain by reference, so letting it fall out of scope leaves
        # the solver reading freed memory -- which shows up as every column
        # coming back zero and the regressor having rank 0, not as a crash.
        self._chains = []
        solvers = []
        for joint in self.segments:
            entry = []
            for centre in (PyKDL.Vector(0, 0, 0), PyKDL.Vector(1, 0, 0),
                           PyKDL.Vector(0, 1, 0), PyKDL.Vector(0, 0, 1)):
                chain = _chain_with(self.robot, self.joint_names, joint,
                                    1.0, centre)
                self._chains.append(chain)
                entry.append(PyKDL.ChainDynParam(chain, self._gravity_vector))
            solvers.append(entry)
        return solvers

    @property
    def dof(self) -> int:
        return len(self.actuated)

    @property
    def parameter_count(self) -> int:
        return len(self.parameter_names)

    def _torque(self, solver: PyKDL.ChainDynParam, q: np.ndarray) -> np.ndarray:
        jnt = PyKDL.JntArray(self.dof)
        for index in range(self.dof):
            jnt[index] = float(q[index])
        out = PyKDL.JntArray(self.dof)
        solver.JntToGravity(jnt, out)
        return np.array([out[i] for i in range(self.dof)])

    def regressor(self, positions: Sequence[float]) -> np.ndarray:
        """Y(q): rows are joints, columns are the 24 mass parameters."""
        q = np.asarray(positions, dtype=float)
        if q.shape != (self.dof,):
            raise ValueError(f"expected {self.dof} joint positions")
        columns = []
        for entry in self._solvers:
            mass_column = self._torque(entry[0], q)
            columns.append(mass_column)
            for axis_solver in entry[1:]:
                # Unit mass one metre along the axis, minus the same mass at
                # the origin, leaves the pure first-moment contribution.
                columns.append(self._torque(axis_solver, q) - mass_column)
        return np.column_stack(columns)

    def urdf_parameters(self) -> np.ndarray:
        """phi as the URDF states it, for validating Y."""
        values = []
        for joint in self.segments:
            link = self.robot.link_map[self.robot.joint_map[joint].child]
            inertial = getattr(link, "inertial", None)
            if inertial is None or not inertial.mass:
                values += [0.0, 0.0, 0.0, 0.0]
                continue
            mass = float(inertial.mass)
            origin = getattr(inertial, "origin", None)
            xyz = origin.xyz if (origin and origin.xyz) else [0.0, 0.0, 0.0]
            values += [mass] + [mass * float(v) for v in xyz]
        return np.array(values)

    def rank(self, samples: Sequence[Sequence[float]]) -> int:
        """How many parameter combinations the data can actually separate."""
        stacked = np.vstack([self.regressor(q) for q in samples])
        return int(np.linalg.matrix_rank(stacked, tol=1e-8))
