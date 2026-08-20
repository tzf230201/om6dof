"""A distal payload must show up in G(q), and show up in the right frame.

These tests exist because of a specific worry: the gravity chain stops at
end_effector_link, so a payload bolted on after it could be present in the
URDF, visible in RViz, and still contribute nothing at all to the torque the
compensation asks for. That failure is silent -- the arm just sags -- so it is
worth pinning down with numbers rather than with reading.
"""
import math
import re
import subprocess

import numpy as np
import pytest

from om6dof_gravity_comp.gravity_model import GRAVITY, GravityModel
from om6dof_gravity_comp.payload_check import render_urdf, strip_payload

PAYLOAD_MASS = 0.150
PAYLOAD_REACH = 0.10


def _urdf() -> str:
    """The real URDF with any declared payload removed.

    Tests here are about the mechanism -- does distal mass reach G(q) -- not
    about the value currently in payload.yaml. Taking the stripped robot as
    the baseline keeps them true whether the camera has been weighed or not.
    """
    return strip_payload(
        render_urdf("om6dof_description", "urdf/om6dof.urdf.xacro"))


def _with_payload(urdf: str, mass: float, com, mount=(0.0, 0.0, 0.0)) -> str:
    """The real URDF with a payload of our choosing, for a controlled test."""
    inertial = (
        f'<inertial><origin xyz="{com[0]} {com[1]} {com[2]}" rpy="0 0 0"/>'
        f'<mass value="{mass}"/>'
        '<inertia ixx="1e-6" ixy="0" ixz="0" iyy="1e-6" iyz="0" izz="1e-6"/>'
        "</inertial>"
    )
    # Replace the whole origin element: xacro emits rpy before xyz, so a
    # pattern anchored on attribute order silently matches nothing.
    origin = (f'<origin xyz="{mount[0]} {mount[1]} {mount[2]}" '
              f'rpy="0 0 0"/>')
    urdf = re.sub(
        r'(<joint name="d405_payload_joint"[^>]*>)\s*<origin[^/]*/>',
        lambda m: m.group(1) + origin, urdf, flags=re.S)
    return re.sub(
        r'(<link name="d405_payload_link">)(.*?)(</link>)',
        lambda m: m.group(1) + inertial + m.group(3),
        urdf, flags=re.S)


def test_stripping_the_payload_removes_exactly_the_declared_mass():
    """The A/B the comparison tool relies on has to be clean: stripping must
    take the payload's mass and nothing else."""
    real = render_urdf("om6dof_description", "urdf/om6dof.urdf.xacro")
    assert "d405_payload_link" in real
    declared = _declared_payload_mass(real)
    assert GravityModel(real).total_mass() - GravityModel(
        strip_payload(real)).total_mass() == pytest.approx(declared, abs=1e-12)


def _declared_payload_mass(urdf: str) -> float:
    block = re.search(r'<link name="d405_payload_link">.*?</link>', urdf,
                      flags=re.S).group(0)
    found = re.search(r'<mass value="([^"]+)"', block)
    return float(found.group(1)) if found else 0.0


def test_distal_payload_raises_j2_and_j3_torque():
    urdf = _urdf()
    loaded = _with_payload(urdf, PAYLOAD_MASS, (PAYLOAD_REACH, 0.0, 0.0))
    bare = GravityModel(urdf).torques(np.zeros(6))
    heavy = GravityModel(loaded).torques(np.zeros(6))

    # Straight up, the only horizontal offset is the payload's own reach, so
    # the added torque is m*g*reach exactly -- an independent hand calculation.
    expected = PAYLOAD_MASS * GRAVITY * PAYLOAD_REACH
    for joint in (1, 2, 4):          # joint2, joint3, joint5: the Y axes
        assert abs(heavy[joint] - bare[joint]) == pytest.approx(expected,
                                                               rel=1e-9)
    assert abs(heavy[1]) > abs(bare[1])
    assert abs(heavy[2]) > abs(bare[2])


def test_payload_reaches_the_chain_despite_hanging_off_the_tip():
    """The whole point: mass past end_effector_link is not silently dropped."""
    urdf = _urdf()
    loaded = _with_payload(urdf, PAYLOAD_MASS, (0.0, 0.0, 0.0))
    assert GravityModel(loaded).total_mass() - GravityModel(urdf).total_mass() \
        == pytest.approx(PAYLOAD_MASS, rel=1e-9)


def test_mount_offset_and_com_offset_are_equivalent():
    """Frames are composed, not confused.

    Moving the bracket 10 cm out and moving the mass 10 cm inside the bracket
    describe the same physical robot, so they must give the same torque. If a
    transform were dropped or applied twice these would differ.
    """
    urdf = _urdf()
    a = _with_payload(urdf, PAYLOAD_MASS, (PAYLOAD_REACH, 0.0, 0.0))
    b = _with_payload(urdf, PAYLOAD_MASS, (0.0, 0.0, 0.0),
                      mount=(PAYLOAD_REACH, 0.0, 0.0))
    rng = np.random.default_rng(3)
    for _ in range(20):
        q = rng.uniform(-1.5, 1.5, 6)
        np.testing.assert_allclose(GravityModel(a).torques(q),
                                   GravityModel(b).torques(q), atol=1e-12)


def test_payload_torque_matches_energy_gradient():
    """Cross-check against potential energy, which shares no code with
    ChainDynParam -- so an error in the lumping cannot hide in both."""
    model = GravityModel(_with_payload(_urdf(), PAYLOAD_MASS,
                                       (PAYLOAD_REACH, 0.0, 0.0)))
    rng = np.random.default_rng(11)
    q = rng.uniform(-1.0, 1.0, 6)
    step = 1e-6
    numeric = np.empty(6)
    for i in range(6):
        up, down = q.copy(), q.copy()
        up[i] += step
        down[i] -= step
        numeric[i] = ((model.potential_energy(up)
                       - model.potential_energy(down)) / (2 * step))
    np.testing.assert_allclose(model.torques(q), numeric, atol=1e-6)
