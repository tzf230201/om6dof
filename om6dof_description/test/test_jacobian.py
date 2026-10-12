from pathlib import Path
import subprocess

from ament_index_python.packages import get_package_share_directory
import numpy as np
import pytest
from urdf_parser_py.urdf import URDF

from om6dof_description.jacobian import GeometricJacobian, ellipsoid_segments, rotation


@pytest.fixture(scope='module')
def v2_model():
    share = Path(get_package_share_directory('om6dof_description'))
    xml = subprocess.check_output(['xacro', str(share / 'urdf' / 'om6dof_v2.urdf.xacro')], text=True)
    return GeometricJacobian(URDF.from_xml_string(xml), 'end_effector_link')


@pytest.mark.parametrize('q', [
    [0.0] * 6,
    [0.2, -0.5, 0.7, -0.3, 0.4, 0.1],
    [-0.7, -1.0, 1.2, 0.4, -0.2, -0.6],
])
def test_v2_jacobian_matches_central_difference(v2_model, q):
    assert v2_model.names == ['joint%d' % i for i in range(1, 7)]
    positions = dict(zip(v2_model.names, q))
    pose, matrix = v2_model.evaluate(positions)
    step = 1e-6
    for index, name in enumerate(v2_model.names):
        plus, minus = dict(positions), dict(positions)
        plus[name] += step
        minus[name] -= step
        positive, _ = v2_model.evaluate(plus)
        negative, _ = v2_model.evaluate(minus)
        linear = (positive[:3, 3] - negative[:3, 3]) / (2 * step)
        derivative = (positive[:3, :3] - negative[:3, :3]) / (2 * step)
        skew = derivative @ pose[:3, :3].T
        angular = np.array([skew[2, 1], skew[0, 2], skew[1, 0]])
        np.testing.assert_allclose(matrix[:3, index], linear, atol=1e-8)
        np.testing.assert_allclose(matrix[3:, index], angular, atol=1e-8)


def test_reference_frame_rotation_and_translation(v2_model):
    positions = dict(zip(v2_model.names, [0.2, -0.5, 0.7, -0.3, 0.4, 0.1]))
    pose, jacobian = v2_model.evaluate(positions)
    root = np.eye(4)
    root[:3, :3] = rotation([1, 2, 3], 0.6)
    root[:3, 3] = [0.2, 0.3, 0.4]
    transformed_pose, transformed_jacobian = v2_model.evaluate(positions, root)
    np.testing.assert_allclose(transformed_pose, root @ pose)
    np.testing.assert_allclose(transformed_jacobian[:3], root[:3, :3] @ jacobian[:3], atol=1e-12)
    np.testing.assert_allclose(transformed_jacobian[3:], root[:3, :3] @ jacobian[3:], atol=1e-12)


def test_prismatic_column_and_fixed_tip_offset():
    robot = URDF.from_xml_string('''<robot name="fixture">
      <link name="world"/><link name="slide"/><link name="tip"/>
      <joint name="slider" type="prismatic"><parent link="world"/><child link="slide"/>
        <origin xyz="0.1 0.2 0.3" rpy="0 0 1.5707963267948966"/>
        <axis xyz="2 0 0"/><limit lower="0" upper="1" effort="1" velocity="1"/>
      </joint>
      <joint name="tool" type="fixed"><parent link="slide"/><child link="tip"/>
        <origin xyz="0 0 0.115"/>
      </joint>
    </robot>''')
    model = GeometricJacobian(robot, 'tip')
    pose, matrix = model.evaluate({'slider': 0.4})
    np.testing.assert_allclose(pose[:3, 3], [0.1, 0.6, 0.415], atol=1e-12)
    np.testing.assert_allclose(matrix[:, 0], [0, 1, 0, 0, 0, 0], atol=1e-12)


def test_missing_motor_position_is_not_replaced_with_zero(v2_model):
    with pytest.raises(ValueError, match='joint1'):
        v2_model.evaluate({})


def test_tcp_roll_can_have_zero_translation_and_nonzero_rotation(v2_model):
    _, matrix = v2_model.evaluate(dict.fromkeys(v2_model.names, 0.0))
    np.testing.assert_allclose(matrix[:3, 5], np.zeros(3), atol=1e-12)
    assert np.linalg.norm(matrix[3:, 5]) == pytest.approx(1.0)


def test_singular_ellipsoid_keeps_collapsed_axis():
    center = np.array([0.1, 0.2, 0.3])
    segments, singular_values = ellipsoid_segments(center, np.diag([2, 1, 0]), 0.4)
    np.testing.assert_allclose(singular_values, [2, 1, 0])
    assert len(segments) % 2 == 0
    np.testing.assert_allclose(np.array(segments)[:, 2], 0.3)
    assert max(point[0] for point in segments) == pytest.approx(0.9)
