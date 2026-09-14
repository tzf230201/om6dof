"""Projection tests are offline: no ROS nodes, camera, serial port, or robot."""

from concurrent.futures import ThreadPoolExecutor
import json
import math
import os
from pathlib import Path
import shutil
import subprocess

import numpy as np
import pytest

from om6dof_perception.world_projection import (
    ProjectionError, UrdfKinematics, WorldProjector, load_model_urdf,
)


XML = '''<robot name="test">
  <link name="base_link"/><link name="pedestal"/><link name="arm"/>
  <link name="camera"/><link name="d435_color_optical_frame"/>
  <joint name="base_mount" type="fixed"><parent link="base_link"/><child link="pedestal"/>
    <origin xyz="1 2 3" rpy="0 0 1.5707963267948966"/></joint>
  <joint name="arm_yaw" type="revolute"><parent link="pedestal"/><child link="arm"/>
    <origin xyz="0 0 .2"/><axis xyz="0 0 2"/></joint>
  <joint name="camera_mount" type="fixed"><parent link="arm"/><child link="camera"/>
    <origin xyz=".5 0 0"/></joint>
  <joint name="color_optical" type="fixed"><parent link="camera"/><child link="d435_color_optical_frame"/>
    <origin xyz="0 .1 .2" rpy="-1.5707963267948966 0 -1.5707963267948966"/></joint>
</robot>'''
NOW = 10_000_000_000
STAMP = 9_800_000_000
FRAME = 'd435_color_optical_frame'


@pytest.fixture
def kinematics():
    return UrdfKinematics(XML)


@pytest.fixture
def projector(kinematics):
    return WorldProjector(kinematics)


@pytest.mark.parametrize('angle,origin,forward_point', [
    (0., [.9, 2.5, 3.4], [.9, 3.5, 3.4]),
    (math.pi / 2., [.5, 1.9, 3.4], [-.5, 1.9, 3.4]),
])
def test_generic_fixed_origins_and_joint_frame_rotation(kinematics, angle, origin, forward_point):
    transform = kinematics.transform({'arm_yaw': angle})
    np.testing.assert_allclose(transform[:3, 3], origin, atol=1e-12)
    np.testing.assert_allclose(transform @ [0., 0., 1., 1.], [*forward_point, 1.], atol=1e-12)
    np.testing.assert_allclose(transform[:3, :3].T @ transform[:3, :3], np.eye(3), atol=1e-12)
    assert kinematics.joint_names == ('arm_yaw',)


def test_generic_prismatic_joint_has_no_robot_specific_offsets():
    chain = UrdfKinematics(XML.replace('type="revolute"', 'type="prismatic"'))
    transform = chain.transform({'arm_yaw': .3})
    np.testing.assert_allclose(transform[:3, 3], [.9, 2.5, 3.7], atol=1e-12)


def test_valid_projection_preserves_nominal_metadata(projector):
    projector.add_joint_state(['unrelated', 'arm_yaw'], [123., 0.], STAMP, NOW)
    point, status = projector.project([0, 0, 1], FRAME, STAMP + 10_000_000, NOW)
    np.testing.assert_allclose(point, [.9, 3.5, 3.4], atol=1e-12)
    assert status['point_stamp_ns'] == STAMP + 10_000_000
    assert status['joint_stamp_ns'] == STAMP
    assert status['joint_point_delta_sec'] == .01
    assert status['transform_source'] == 'urdf_nominal'
    assert status['calibration_verified'] is False
    assert status['hardware_time_synchronized'] is False
    assert status['base_frame'] == 'base_link'
    assert status['model_version'] == 'v2'
    json.dumps(status, allow_nan=False)


@pytest.mark.parametrize('stamp,code', [
    (0, 'point_stamp_invalid'), (-1, 'point_stamp_invalid'),
    (float('nan'), 'point_stamp_invalid'),
    (NOW + 1, 'point_stamp_future'), (NOW - 500_000_001, 'point_stamp_stale'),
])
def test_invalid_point_timestamps_are_rejected(projector, stamp, code):
    with pytest.raises(ProjectionError, match=code):
        projector.project([0, 0, 1], FRAME, stamp, NOW)


@pytest.mark.parametrize('stamp,code', [
    (0, 'joint_stamp_invalid'), (-1, 'joint_stamp_invalid'),
    (float('nan'), 'joint_stamp_invalid'),
    (NOW + 1, 'joint_stamp_future'), (NOW - 500_000_001, 'joint_stamp_stale'),
])
def test_invalid_joint_timestamps_are_rejected(projector, stamp, code):
    with pytest.raises(ProjectionError, match=code):
        projector.add_joint_state(['arm_yaw'], [0], stamp, NOW)


@pytest.mark.parametrize('names,positions,code', [
    ([], [], 'missing_joint_positions'),
    (['other'], [0], 'missing_joint_positions'),
    (['arm_yaw'], [], 'invalid_joint_state_layout'),
    (['arm_yaw', 'arm_yaw'], [0, 0], 'invalid_joint_state_layout'),
    (['arm_yaw'], [float('nan')], 'nonfinite_joint_position'),
    (['arm_yaw'], [float('inf')], 'nonfinite_joint_position'),
])
def test_incomplete_or_nonfinite_joint_state_never_enters_buffer(projector, names, positions, code):
    with pytest.raises(ProjectionError, match=code):
        projector.add_joint_state(names, positions, STAMP, NOW)
    with pytest.raises(ProjectionError, match='no_fresh_complete_joint_state'):
        projector.project([0, 0, 1], FRAME, STAMP, NOW)


@pytest.mark.parametrize('point,frame,code', [
    ([0, 0, 1], 'camera_color_optical_frame', 'wrong_optical_frame'),
    ([0, 0, 1], '', 'wrong_optical_frame'),
    ([0, float('nan'), 1], FRAME, 'nonfinite_point'),
    ([0, 0, float('inf')], FRAME, 'nonfinite_point'),
    ([0, 0], FRAME, 'nonfinite_point'),
    ([0, 0, 0], FRAME, 'nonpositive_optical_depth'),
    ([0, 0, -.1], FRAME, 'nonpositive_optical_depth'),
])
def test_frame_and_point_validation(projector, point, frame, code):
    with pytest.raises(ProjectionError, match=code):
        projector.project(point, frame, STAMP, NOW)


def test_nearest_state_is_selected_not_latest_and_arrival_order_does_not_matter(projector):
    projector.add_joint_state(['arm_yaw'], [math.pi / 2], STAMP + 40_000_000, NOW)
    projector.add_joint_state(['arm_yaw'], [0], STAMP, NOW)
    point, status = projector.project([0, 0, 1], FRAME, STAMP + 10_000_000, NOW)
    assert status['joint_stamp_ns'] == STAMP
    np.testing.assert_allclose(point, [.9, 3.5, 3.4], atol=1e-12)
    _, tied = projector.project([0, 0, 1], FRAME, STAMP + 20_000_000, NOW)
    assert tied['joint_stamp_ns'] == STAMP


def test_later_joint_sample_can_match_point_only_after_it_arrives(projector):
    with pytest.raises(ProjectionError, match='no_fresh_complete_joint_state'):
        projector.project([0, 0, 1], FRAME, STAMP, NOW)
    projector.add_joint_state(['arm_yaw'], [0], STAMP + 20_000_000, NOW)
    _, status = projector.project([0, 0, 1], FRAME, STAMP, NOW)
    assert status['joint_stamp_ns'] > status['point_stamp_ns']
    assert status['joint_point_delta_sec'] == .02


def test_no_latest_state_fallback_for_time_mismatch(projector):
    projector.add_joint_state(['arm_yaw'], [0], STAMP + 100_000_000, NOW)
    with pytest.raises(ProjectionError, match='joint_point_time_mismatch'):
        projector.project([0, 0, 1], FRAME, STAMP, NOW)


def test_state_expires_even_if_valid_at_arrival(projector):
    projector.add_joint_state(['arm_yaw'], [0], STAMP, NOW)
    with pytest.raises(ProjectionError, match='no_fresh_complete_joint_state'):
        projector.project([0, 0, 1], FRAME, NOW + 200_000_000, NOW + 400_000_000)


def test_clock_rollback_does_not_use_future_buffer_samples(projector):
    projector.add_joint_state(['arm_yaw'], [0], STAMP, NOW)
    with pytest.raises(ProjectionError, match='no_fresh_complete_joint_state'):
        projector.project([0, 0, 1], FRAME, STAMP - 10_000_000, STAMP - 1)


def test_liveness_initially_waits_and_is_always_uncalibrated(projector):
    status = projector.liveness_status(NOW)
    assert status['status'] == 'waiting_for_data'
    assert status['accepted'] is False
    assert status['calibration_verified'] is False
    assert projector.liveness_status(0)['reason'] == 'clock_invalid'


def test_liveness_accepts_only_fresh_last_projection(projector):
    projector.add_joint_state(['arm_yaw'], [0], STAMP, NOW)
    projector.project([0, 0, 1], FRAME, STAMP + 10_000_000, NOW)
    status = projector.liveness_status(NOW + 10_000_000)
    assert status['accepted'] is True
    assert status['status'] == 'projected'
    assert status['point_stamp_ns'] == STAMP + 10_000_000
    assert status['calibration_verified'] is False


def test_liveness_expires_if_camera_stops_even_while_joints_continue(projector):
    projector.add_joint_state(['arm_yaw'], [0], STAMP, NOW)
    projector.project([0, 0, 1], FRAME, STAMP, NOW)
    later = NOW + 510_000_000
    projector.add_joint_state(['arm_yaw'], [0], later, later)
    status = projector.liveness_status(later)
    assert status['accepted'] is False
    assert status['status'] == 'stale'
    assert status['reason'] == 'point_stream_stale'
    assert status['calibration_verified'] is False


def test_liveness_expires_by_point_capture_age_before_receipt_age(projector):
    projector.add_joint_state(['arm_yaw'], [0], STAMP, NOW)
    projector.project([0, 0, 1], FRAME, STAMP, NOW)
    later = NOW + 310_000_000
    projector.add_joint_state(['arm_yaw'], [0], later, later)
    status = projector.liveness_status(later)
    assert status['accepted'] is False
    assert status['reason'] == 'point_stamp_stale'


def test_liveness_expires_if_joint_stream_stops(projector):
    projector.add_joint_state(['arm_yaw'], [0], STAMP, NOW)
    projector.project([0, 0, 1], FRAME, STAMP + 40_000_000, NOW)
    status = projector.liveness_status(NOW + 310_000_000)
    assert status['accepted'] is False
    assert status['reason'] == 'joint_state_stream_stale'


def test_rejected_point_does_not_restore_previous_accepted_status(projector):
    projector.add_joint_state(['arm_yaw'], [0], STAMP, NOW)
    projector.project([0, 0, 1], FRAME, STAMP, NOW)
    with pytest.raises(ProjectionError, match='nonpositive_optical_depth'):
        projector.project([0, 0, -1], FRAME, STAMP, NOW)
    status = projector.liveness_status(NOW)
    assert status['accepted'] is False
    assert status['reason'] == 'nonpositive_optical_depth'


def test_liveness_rejects_clock_rollback(projector):
    projector.add_joint_state(['arm_yaw'], [0], STAMP, NOW)
    projector.project([0, 0, 1], FRAME, STAMP, NOW)
    status = projector.liveness_status(NOW - 1)
    assert status['accepted'] is False
    assert status['reason'] == 'clock_rollback'


def test_buffer_bounded_sorted_and_thread_safe(kinematics):
    projector = WorldProjector(kinematics, buffer_size=3)
    with ThreadPoolExecutor(max_workers=4) as pool:
        list(pool.map(lambda i: projector.add_joint_state(['arm_yaw'], [i / 100],
                      STAMP + i * 1_000_000, NOW), reversed(range(20))))
    with pytest.raises(ProjectionError, match='joint_point_time_mismatch'):
        projector.project([0, 0, 1], FRAME, STAMP - 40_000_000, NOW)
    projector.add_joint_state(['arm_yaw'], [0], STAMP + 19_000_000, NOW)
    point, _ = projector.project([0, 0, 1], FRAME, STAMP + 19_000_000, NOW)
    np.testing.assert_allclose(point, [.9, 3.5, 3.4], atol=1e-12)


@pytest.mark.parametrize('kwargs', [
    {'model_version': 'v1'}, {'max_joint_delta_sec': 0},
    {'max_age_sec': float('nan')}, {'buffer_size': 0}, {'buffer_size': 1.5},
])
def test_invalid_projection_configuration_fails(kinematics, kwargs):
    with pytest.raises(ValueError):
        WorldProjector(kinematics, **kwargs)


def test_v1_loader_fails_before_resolving_or_rendering_any_model():
    with pytest.raises(ValueError, match='v2 only'):
        load_model_urdf('v1')


@pytest.mark.parametrize('xml', [
    XML.replace('name="base_link"', 'name="missing_base"'),
    XML.replace('axis xyz="0 0 2"', 'axis xyz="0 0 0"'),
    XML.replace('type="revolute"', 'type="floating"'),
    XML.replace('<axis xyz="0 0 2"/>', '<mimic joint="other"/>'),
    XML.replace('xyz=".5 0 0"', 'xyz="nan 0 0"'),
])
def test_invalid_or_unsupported_urdf_fails_closed(xml):
    with pytest.raises(ValueError):
        UrdfKinematics(xml)


def test_rendered_current_v2_matches_independent_native_kdl(tmp_path):
    """Optional real-model cross-check against C++ KDL, not this FK code."""
    pytest.importorskip('xacro', reason='Source the ROS environment for the optional native model test')
    binary = Path('/tmp/om6dof-workspace-cpp-build/workspace_scan_cpp')
    if not shutil.which('xacro') or not binary.is_file():
        pytest.skip('xacro and the offline C++ KDL probe are optional prerequisites')
    description = Path(__file__).resolve().parents[2] / 'om6dof_description'
    prefix = tmp_path / 'prefix'
    index = prefix / 'share/ament_index/resource_index/packages'
    index.mkdir(parents=True)
    (index / 'om6dof_description').symlink_to(description / 'package.xml')
    (prefix / 'share/om6dof_description').symlink_to(description, target_is_directory=True)
    env = dict(os.environ, AMENT_PREFIX_PATH=f'{prefix}:{os.environ.get("AMENT_PREFIX_PATH", "")}')
    xml = subprocess.run(['xacro', str(description / 'urdf/om6dof_v2.urdf.xacro')],
                         check=True, capture_output=True, text=True, env=env).stdout
    chain = UrdfKinematics(xml)
    assert chain.joint_names == tuple(f'joint{i}' for i in range(1, 7))
    model = tmp_path / 'model.urdf'
    model.write_text(xml)
    for joints in ([0] * 6, [0, -.68, 1.36, 0, .89, 0], [.2, -.4, .7, .3, -.5, .6]):
        result = subprocess.run([str(binary), '--urdf', str(model), '--base-link', 'base_link',
                                 '--tip-link', FRAME, '--probe', *map(str, joints)],
                                check=True, capture_output=True, text=True)
        reference = json.loads(result.stdout)
        transform = chain.transform(dict(zip(chain.joint_names, joints)))
        np.testing.assert_allclose(transform[:3, 3], reference['position'], atol=1e-12)
        np.testing.assert_allclose(transform[:3, :3], np.asarray(reference['rotation']).reshape(3, 3), atol=1e-12)
