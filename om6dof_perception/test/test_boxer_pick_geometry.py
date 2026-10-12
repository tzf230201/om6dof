import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from boxer_pick_geometry import (capture_world_camera, rigid_transform,
    rotation_quaternion, world_detection_message)


def calibration():
    return {'camera_calibration_color_xyz': [.05, .02, -.03],
            'camera_calibration_color_quaternion_xyzw': [0, 0, np.sqrt(.5), np.sqrt(.5)],
            'camera_calibration_sha256': 'a'*64}


def scene():
    return {'boxes': [
        {'accepted': True, 'label': 'bottle', 'score': .9, 'yolo_score': .95,
         'translation_local_object_m': [.1, .2, .3], 'size_m': [.05, .06, .2],
         'rotation_local_object': np.eye(3), 'inside_point_count': 900},
        {'accepted': False, 'label': 'keyboard'},
    ]}


def snapshot():
    return {'receipt_ns': 4_000_000_001,
            'R_local_camera': rigid_transform([0, 0, 0], [np.sqrt(.5), 0, 0, np.sqrt(.5)])[:3, :3],
            'T_world_camera': rigid_transform([1, 2, 3], [0, 0, np.sqrt(.5), np.sqrt(.5)])}


def test_measured_hand_eye_composes_in_end_effector_frame_once():
    world_eoe = rigid_transform([1, 2, 3], [0, 0, np.sqrt(.5), np.sqrt(.5)])
    actual = capture_world_camera(world_eoe, calibration())
    assert np.allclose(actual[:3, 3], [.98, 2.05, 2.97])
    assert np.allclose(actual[:3, :3], np.diag([-1, -1, 1]))


def test_delayed_detection_uses_captured_pose_and_inverts_local_rotation():
    data = world_detection_message(scene(), snapshot(), calibration(), 'serial', 14_000_000_001)
    assert data['source_stamp_ns'] == 4_000_000_001
    assert data['result_stamp_ns'] == 14_000_000_001
    assert data['schema'] == 'om6dof.boxer3d_detections.v1'
    assert data['frame_id'] == 'world' and data['calibration_verified'] is True
    assert len(data['boxes']) == 1
    box = data['boxes'][0]
    # Inverse local rotation maps (.1,.2,.3) -> (.1,.3,-.2).
    # World camera yaw then maps this to (-.3,.1,-.2), plus translation.
    assert box['center'] == pytest.approx([.7, 2.1, 2.8])
    assert box['size'] == [.05, .06, .2]
    actual_rotation = rigid_transform([0, 0, 0], box['quaternion_xyzw'])[:3, :3]
    assert np.allclose(actual_rotation,
                       snapshot()['T_world_camera'][:3, :3] @ snapshot()['R_local_camera'].T)
    assert box['label'] == 'bottle' and box['inside_point_count'] == 900


@pytest.mark.parametrize('axis', [[1, 0, 0], [0, 1, 0], [0, 0, 1], [.6, .8, 0]])
def test_box_orientation_round_trip_at_pi(axis):
    rotation = rigid_transform([0, 0, 0], [*axis, 0])[:3, :3]
    recovered = rigid_transform([0, 0, 0], rotation_quaternion(rotation))[:3, :3]
    assert np.allclose(recovered, rotation, atol=1.e-9)


def test_empty_result_clears_boxes_while_retaining_capture_provenance():
    data = world_detection_message({'boxes': []}, snapshot(), calibration(), 'serial',
                                   8_000_000_000, reason='No accepted Boxer boxes')
    assert data['boxes'] == []
    assert data['source_stamp_ns'] == int(snapshot()['receipt_ns'])
    assert data['calibration_sha256'] == 'a'*64


@pytest.mark.parametrize('field,value', [('size_m', [.1, 0, .2]),
                                        ('translation_local_object_m', [float('nan'), 0, 1]),
                                        ('rotation_local_object', np.diag([1, 1, -1]))])
def test_invalid_accepted_geometry_is_not_published(field, value):
    result = scene(); result['boxes'][0][field] = value
    with pytest.raises(ValueError):
        world_detection_message(result, snapshot(), calibration(), 'serial', 8_000_000_000)


def test_improper_capture_rotation_and_future_source_timestamp_are_rejected():
    captured = snapshot(); captured['R_local_camera'] = np.diag([1, 1, -1])
    with pytest.raises(ValueError):
        world_detection_message(scene(), captured, calibration(), 'serial', 8_000_000_000)
    with pytest.raises(ValueError):
        world_detection_message(scene(), snapshot(), calibration(), 'serial', 1_000_000_000)
