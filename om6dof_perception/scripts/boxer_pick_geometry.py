"""Calibrated, capture-time geometry for the optional Boxer pickup interface."""
import importlib.util
from pathlib import Path

import numpy as np


def rigid_transform(xyz, quaternion_xyzw):
    xyz = np.asarray(xyz, dtype=float)
    quaternion = np.asarray(quaternion_xyzw, dtype=float)
    if (xyz.shape != (3,) or quaternion.shape != (4,) or
            not np.isfinite(xyz).all() or not np.isfinite(quaternion).all() or
            abs(np.linalg.norm(quaternion) - 1.) > 1.e-3):
        raise ValueError('Expected finite xyz and normalized quaternion')
    x, y, z, w = quaternion / np.linalg.norm(quaternion)
    transform = np.eye(4)
    transform[:3, :3] = [
        [1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
        [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
        [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)],
    ]
    transform[:3, 3] = xyz
    return transform


def checked_rotation(rotation):
    rotation = np.asarray(rotation, dtype=float)
    if (rotation.shape != (3, 3) or not np.isfinite(rotation).all() or
            not np.allclose(rotation.T @ rotation, np.eye(3), atol=1.e-4) or
            not np.isclose(np.linalg.det(rotation), 1., atol=1.e-4)):
        raise ValueError('Expected a rigid rotation')
    return rotation


def rotation_quaternion(rotation):
    """Stable quaternion conversion, including rotations near pi."""
    r = checked_rotation(rotation)
    eigen_matrix = np.array([
        [r[0,0]-r[1,1]-r[2,2], r[0,1]+r[1,0], r[0,2]+r[2,0], r[2,1]-r[1,2]],
        [r[0,1]+r[1,0], r[1,1]-r[0,0]-r[2,2], r[1,2]+r[2,1], r[0,2]-r[2,0]],
        [r[0,2]+r[2,0], r[1,2]+r[2,1], r[2,2]-r[0,0]-r[1,1], r[1,0]-r[0,1]],
        [r[2,1]-r[1,2], r[0,2]-r[2,0], r[1,0]-r[0,1], r.trace()],
    ]) / 3.
    _, vectors = np.linalg.eigh(eigen_matrix)
    quaternion = vectors[:, -1]
    if quaternion[3] < 0:
        quaternion = -quaternion
    return quaternion.tolist()


def load_pick_calibration(path, camera_serial):
    """Reuse the measured hand-eye artifact contract used by DD-GNG."""
    if not path:
        raise ValueError('Pickup detections require camera_calibration_file')
    from ament_index_python.packages import get_package_share_directory
    loader_path = Path(get_package_share_directory('om6dof_dd_gng')) / 'launch/_model_inputs.py'
    spec = importlib.util.spec_from_file_location('boxer_hand_eye_contract', loader_path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    calibration = module.load_hand_eye_calibration(path)
    if calibration['camera_calibration_serial'] != str(camera_serial):
        raise ValueError('Calibration serial does not match selected camera_serial')
    return calibration


def capture_world_camera(world_end_effector, calibration):
    """Compose world<-EoE with the measured EoE<-RGB transform exactly once."""
    transform = np.asarray(world_end_effector, dtype=float)
    if (transform.shape != (4, 4) or not np.isfinite(transform).all() or
            not np.allclose(transform[3], [0, 0, 0, 1])):
        raise ValueError('Invalid world end-effector transform')
    checked_rotation(transform[:3, :3])
    measured = rigid_transform(calibration['camera_calibration_color_xyz'],
                               calibration['camera_calibration_color_quaternion_xyzw'])
    return transform @ measured


def world_detection_message(result, snapshot, calibration, camera_serial, result_stamp_ns,
                            reason=''):
    """Transform accepted OBBs using the frozen pose, never a current camera pose."""
    source_stamp = int(snapshot['receipt_ns'])
    if source_stamp <= 0 or int(result_stamp_ns) < source_stamp:
        raise ValueError('Invalid capture/result timestamp ordering')
    world_camera = np.asarray(snapshot['T_world_camera'], dtype=float)
    if (world_camera.shape != (4, 4) or not np.isfinite(world_camera).all() or
            not np.allclose(world_camera[3], [0, 0, 0, 1])):
        raise ValueError('Snapshot has no valid captured world camera transform')
    rotation = checked_rotation(world_camera[:3, :3]) @ checked_rotation(snapshot['R_local_camera']).T
    boxes = []
    for index, box in enumerate(result.get('boxes', [])):
        if not box.get('accepted', False):
            continue
        center = np.asarray(box['translation_local_object_m'], dtype=float)
        size = np.asarray(box['size_m'], dtype=float)
        confidence = float(box['score'])
        if (center.shape != (3,) or size.shape != (3,) or
                not np.isfinite(center).all() or not np.isfinite(size).all() or
                np.any(size <= 0) or not np.isfinite(confidence)):
            raise ValueError('Accepted Boxer detection contains invalid geometry')
        orientation = rotation @ checked_rotation(box['rotation_local_object'])
        boxes.append({
            'id': index, 'label': str(box['label']), 'score': confidence,
            'yolo_score': float(box.get('yolo_score', confidence)),
            'center': (rotation @ center + world_camera[:3, 3]).tolist(),
            'size': size.tolist(), 'quaternion_xyzw': rotation_quaternion(orientation),
            'inside_point_count': int(box.get('inside_point_count', 0)),
        })
    return {
        'schema': 'om6dof.boxer3d_detections.v1', 'frame_id': 'world',
        'source_stamp_ns': source_stamp, 'result_stamp_ns': int(result_stamp_ns),
        'camera_serial': str(camera_serial), 'calibration_verified': True,
        'calibration_sha256': calibration['camera_calibration_sha256'],
        'boxes': boxes, 'reason': reason,
    }
