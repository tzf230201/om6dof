import math

import numpy as np
import pytest

from urdf_parser_py.urdf import URDF

from om6dof_gravity_comp.gravity_model import (
    GravityModel,
    _lumped_tip_inertia,
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


# A branch hanging off the last chain link, with its COM offset inside its
# own frame -- the case the first version of the lumping got wrong.
BRANCH_URDF = """<?xml version="1.0"?>
<robot name="branch">
  <link name="base"/>
  <link name="arm">
    <inertial><origin xyz="0 0 0"/><mass value="1.0"/>
      <inertia ixx="0" ixy="0" ixz="0" iyy="0" iyz="0" izz="0"/></inertial>
  </link>
  <link name="tip"/>
  <link name="finger">
    <inertial><origin xyz="0 0 0.01"/><mass value="0.02"/>
      <inertia ixx="0" ixy="0" ixz="0" iyy="0" iyz="0" izz="0"/></inertial>
  </link>
  <joint name="j1" type="revolute">
    <parent link="base"/><child link="arm"/>
    <origin xyz="0 0 0"/><axis xyz="0 1 0"/>
    <limit lower="-3" upper="3" effort="10" velocity="1"/>
  </joint>
  <joint name="j_tip" type="fixed">
    <parent link="arm"/><child link="tip"/><origin xyz="0 0 0.5"/>
  </joint>
  <joint name="j_finger" type="fixed">
    <parent link="arm"/><child link="finger"/><origin xyz="0 0 0.2"/>
  </joint>
</robot>
"""


def test_branch_mass_is_placed_in_the_tip_frame_not_its_own():
    """A branch COM must be carried into the tip frame before it is lumped.

    Using the branch link's own inertial origin directly put the gripper
    44 mm too far out on the real arm, inflating every torque it contributes.
    """
    robot = URDF.from_xml_string(BRANCH_URDF)
    joints = robot.get_chain("base", "tip", joints=True, links=False)
    links = robot.get_chain("base", "tip", joints=False, links=True)
    mass, com = _lumped_tip_inertia(robot, joints, links)

    assert mass == pytest.approx(0.02)
    # finger sits 0.2 along arm, its own COM another 0.01 -> 0.21 from arm;
    # the tip frame is 0.5 along arm, so 0.21 - 0.5 = -0.29 in tip frame.
    assert com.z() == pytest.approx(-0.29, abs=1e-9)
    assert com.x() == pytest.approx(0.0, abs=1e-12)


def test_branch_mass_actually_changes_the_torque():
    """Guards the lumping from quietly becoming a no-op."""
    with_branch = GravityModel(BRANCH_URDF, base_link="base", tip_link="tip")
    without = GravityModel(BRANCH_URDF, base_link="base", tip_link="tip",
                           include_offchain_mass=False)
    # Away from zero: at q = 0 the branch sits directly above the joint axis,
    # so its lever arm vanishes and the comparison would prove nothing.
    q = [0.6]
    assert abs(with_branch.torques(q)[0] - without.torques(q)[0]) > 1e-3
    assert with_branch.total_mass() == pytest.approx(without.total_mass() + 0.02)
