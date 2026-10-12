import math

import pytest
from urdf_parser_py.urdf import URDF

from om6dof_description.vectors import LinkMass, link_masses, transform_point, vector_endpoints


def test_gravity_remains_world_aligned_when_link_rotates():
    link = LinkMass('wrist', 0.1, (0.1, 0.0, 0.0))
    half = math.sqrt(0.5)
    start, end = vector_endpoints(
        link, (1.0, 2.0, 3.0), (0.0, half, 0.0, half),
        (0.0, 0.0, -9.81), 0.05, 'gravity')
    assert start == pytest.approx((1.0, 2.0, 2.9))
    assert end == pytest.approx((1.0, 2.0, 2.85095))


def test_force_scales_with_mass_and_configured_gravity():
    link = LinkMass('payload', 2.0, (0.0, 0.0, 0.0))
    start, end = vector_endpoints(
        link, (0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0),
        (1.0, -2.0, -3.0), 0.2, 'gravity')
    assert start == (0.0, 0.0, 0.0)
    assert end == pytest.approx((0.4, -0.8, -1.2))


def test_com_offset_rotates_with_link_without_force_scaling():
    link = LinkMass('link', 100.0, (0.1, 0.0, 0.0))
    half = math.sqrt(0.5)
    start, end = vector_endpoints(
        link, (1.0, 2.0, 3.0), (0.0, 0.0, half, half),
        (0.0, 0.0, -9.81), 10.0, 'com_offset')
    assert start == (1.0, 2.0, 3.0)
    assert end == pytest.approx((1.0, 2.1, 3.0))


def test_payload_branches_and_inertial_origin_rotation():
    robot = URDF.from_xml_string('''
        <robot name="fixture">
          <link name="base"/>
          <link name="payload">
            <inertial>
              <origin xyz="0.1 0.2 0.3" rpy="1 2 3"/>
              <mass value="0.08"/>
              <inertia ixx="1" ixy="0" ixz="0" iyy="1" iyz="0" izz="1"/>
            </inertial>
          </link>
          <joint name="mount" type="fixed">
            <parent link="base"/><child link="payload"/>
          </joint>
        </robot>''')
    assert link_masses(robot) == [LinkMass('payload', 0.08, (0.1, 0.2, 0.3))]


@pytest.mark.parametrize('mass', [-0.1, float('nan'), float('inf')])
def test_invalid_mass_is_rejected(mass):
    robot = URDF.from_xml_string('''
        <robot name="fixture"><link name="link"><inertial>
          <mass value="1"/>
          <inertia ixx="1" ixy="0" ixz="0" iyy="1" iyz="0" izz="1"/>
        </inertial></link></robot>''')
    robot.links[0].inertial.mass = mass
    with pytest.raises(ValueError, match='invalid mass'):
        link_masses(robot)
    robot.links[0].inertial.mass = 0.0
    assert link_masses(robot) == []


def test_zero_quaternion_is_rejected():
    with pytest.raises(ValueError, match='zero norm'):
        transform_point((1.0, 0.0, 0.0), (0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 0.0))
