import cv2
import numpy as np
import pytest

from om6dof_pick_and_place.hand_eye_calibrate import (
    average_transform, matrix_quaternion, quaternion_matrix, rotation_angle,
    solve_hand_eye, transform_dispersion,
)


def _transform(rotation_vector, translation):
    result = np.eye(4)
    result[:3, :3], _ = cv2.Rodrigues(np.asarray(rotation_vector, dtype=float))
    result[:3, 3] = translation
    return result


def test_quaternion_matrix_round_trip_near_pi():
    rotation, _ = cv2.Rodrigues(np.array([3.05, -0.12, 0.08]))
    quaternion = matrix_quaternion(rotation)
    assert np.linalg.norm(quaternion) == pytest.approx(1.0)
    np.testing.assert_allclose(quaternion_matrix(*quaternion), rotation, atol=1.0e-8)


def test_stationary_transform_aggregate_rejects_translation_outlier():
    expected = _transform([0.1, -0.2, 0.05], [0.03, -0.01, 0.2])
    samples = [expected.copy() for _ in range(9)]
    samples[-1][:3, 3] += [0.05, 0.0, 0.0]
    center = average_transform(samples)
    np.testing.assert_allclose(center[:3, 3], expected[:3, 3], atol=1.0e-12)
    translation_rms, translation_max, rotation_rms, rotation_max = \
        transform_dispersion(samples, center)
    assert translation_rms > 0.0
    assert translation_max > 0.04
    assert rotation_rms < 1.0e-8
    assert rotation_max < 1.0e-8


def test_hand_eye_solver_recovers_eye_in_hand_transform():
    gripper_camera = _transform([0.25, -0.18, 0.12], [0.035, -0.022, 0.081])
    base_target = _transform([-0.12, 0.08, 0.2], [0.42, 0.03, 0.16])
    base_gripper = []
    camera_target = []
    for index in range(12):
        base_from_gripper = _transform(
            [0.08 * index, -0.035 * (index % 4), 0.05 * ((index * 2) % 5)],
            [0.16 + 0.012 * index, -0.06 + 0.018 * (index % 5), 0.24 + 0.01 * (index % 3)],
        )
        base_gripper.append(base_from_gripper)
        camera_target.append(
            np.linalg.inv(gripper_camera) @ np.linalg.inv(base_from_gripper) @ base_target)
    solved, translation_rmse, rotation_rmse, translation_errors, rotation_errors = solve_hand_eye(
        base_gripper, camera_target)
    assert np.linalg.norm(solved[:3, 3] - gripper_camera[:3, 3]) < 1.0e-5
    assert rotation_angle(solved[:3, :3].T @ gripper_camera[:3, :3]) < 1.0e-5
    assert translation_rmse < 1.0e-8
    assert rotation_rmse < 1.0e-8
    assert max(translation_errors) < 1.0e-8
    assert max(rotation_errors) < 1.0e-8
