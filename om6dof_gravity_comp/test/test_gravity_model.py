import math

import numpy as np
import pytest

from om6dof_gravity_comp.gravity_model import (
    GravityModel,
    friction_compensation,
)

# Two links, 1 m each, 1 kg at each far end, rotating about Y so the arm
# swings in the X-Z plane. Small enough to check torques by hand.
TWO_LINK_URDF = """<?xml version="1.0"?>
<robot name="twolink">
  <link name="base"/>
  <link name="l1">
    <inertial><origin xyz="1 0 0"/><mass value="1.0"/>
      <inertia ixx="0" ixy="0" ixz="0" iyy="0" iyz="0" izz="0"/></inertial>
  </link>
  <link name="l2">
    <inertial><origin xyz="1 0 0"/><mass value="1.0"/>
      <inertia ixx="0" ixy="0" ixz="0" iyy="0" iyz="0" izz="0"/></inertial>
  </link>
  <link name="tip"/>
  <joint name="j1" type="revolute">
    <parent link="base"/><child link="l1"/>
    <origin xyz="0 0 0"/><axis xyz="0 1 0"/>
    <limit lower="-3.14" upper="3.14" effort="10" velocity="1"/>
  </joint>
  <joint name="j2" type="revolute">
    <parent link="l1"/><child link="l2"/>
    <origin xyz="2 0 0"/><axis xyz="0 1 0"/>
    <limit lower="-3.14" upper="3.14" effort="10" velocity="1"/>
  </joint>
  <joint name="j3" type="fixed">
    <parent link="l2"/><child link="tip"/><origin xyz="2 0 0"/>
  </joint>
</robot>
"""


def _model():
    return GravityModel(TWO_LINK_URDF, base_link="base", tip_link="tip",
                        include_offchain_mass=False)


def test_two_link_arm_matches_hand_computed_torque():
    """Straight out horizontally, both masses pull on joint 1."""
    model = _model()
    g = model.gravity
    # Mass 1 sits 1 m out, mass 2 sits 3 m out. Torque about j1 is the sum of
    # m*g*lever; about j2 only the outer mass, 1 m from it.
    expected_j1 = -(1.0 * g * 1.0 + 1.0 * g * 3.0)
    expected_j2 = -(1.0 * g * 1.0)
    torque = model.torques([0.0, 0.0])
    assert torque[0] == pytest.approx(expected_j1, abs=1e-6)
    assert torque[1] == pytest.approx(expected_j2, abs=1e-6)


def test_hanging_straight_down_needs_no_torque():
    """Nothing to hold up when the arm hangs along gravity."""
    model = _model()
    torque = model.torques([math.pi / 2, 0.0])
    assert np.allclose(torque, 0.0, atol=1e-9)


def test_torque_is_the_gradient_of_potential_energy():
    """Cross-check against a route that shares no code with ChainDynParam.

    Gravity torque is dU/dq by definition, so differentiating the potential
    energy numerically is an independent answer. If the chain were built with
    the wrong frames both would still agree with each other -- but the hand
    computation above pins the absolute value, so together they cover it.
    """
    model = _model()
    step = 1e-6
    for q in ([0.0, 0.0], [0.3, -0.7], [1.1, 0.4], [-0.9, 1.3]):
        analytic = model.torques(q)
        numeric = np.zeros(2)
        for index in range(2):
            high = list(q)
            low = list(q)
            high[index] += step
            low[index] -= step
            numeric[index] = (
                model.potential_energy(high) - model.potential_energy(low)
            ) / (2 * step)
        assert np.allclose(analytic, numeric, atol=1e-4), (
            f"at {q}: {analytic} vs {numeric}"
        )


def test_rotational_inertia_does_not_change_gravity_torque():
    """Justifies passing zeros for the URDF's placeholder inertia tensors."""
    absurd = TWO_LINK_URDF.replace(
        'ixx="0" ixy="0" ixz="0" iyy="0" iyz="0" izz="0"',
        'ixx="5" ixy="0" ixz="0" iyy="5" iyz="0" izz="5"',
    )
    plain = _model().torques([0.4, -0.2])
    loaded = GravityModel(absurd, base_link="base", tip_link="tip",
                          include_offchain_mass=False).torques([0.4, -0.2])
    assert np.allclose(plain, loaded, atol=1e-12)


def test_rejects_wrong_sized_or_non_finite_input():
    model = _model()
    with pytest.raises(ValueError):
        model.torques([0.0])
    with pytest.raises(ValueError):
        model.torques([0.0, float("nan")])


def test_friction_compensation_pushes_with_the_motion_and_fades_at_rest():
    scalars = [0.4, 0.8]
    thresholds = [1.0, 1.0]
    moving = friction_compensation([0.5, -0.5], scalars, thresholds)
    assert moving[0] > 0 and moving[1] < 0
    # Never more than the scalar, and nothing at all when standing still.
    assert abs(moving[0]) <= scalars[0] + 1e-12
    assert np.allclose(friction_compensation([0.0, 0.0], scalars, thresholds), 0.0)
    saturated = friction_compensation([10.0, 10.0], scalars, thresholds)
    assert saturated == pytest.approx(scalars)
