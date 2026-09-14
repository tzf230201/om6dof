import numpy as np
import pytest

from om6dof_pick_and_place.center_depth_urdf_check import (
    apply_world_z_offset, quaternion_matrix, robust_depth, transform_point,
)


def test_robust_depth_ignores_invalid_and_out_of_range_values():
    values = [0.0, float("nan"), 0.40, 0.42, 9.0, 0.41]
    assert robust_depth(values, 0.08, 3.0, 3) == pytest.approx(0.41)
    assert robust_depth(values, 0.08, 3.0, 4) is None


def test_quaternion_matrix_rotates_optical_point():
    half = np.sqrt(0.5)
    rotation = quaternion_matrix(0.0, 0.0, half, half)
    np.testing.assert_allclose(rotation @ [1.0, 0.0, 0.0],
                               [0.0, 1.0, 0.0], atol=1.0e-12)


def test_transform_point_applies_rotation_then_translation():
    half = np.sqrt(0.5)
    result = transform_point(
        [1.0, 0.0, 0.0], [0.1, 0.2, 0.3], [0.0, 0.0, half, half])
    np.testing.assert_allclose(result, [0.1, 1.2, 0.3], atol=1.0e-12)


def test_quaternion_matrix_rejects_zero_quaternion():
    with pytest.raises(ValueError, match="quaternion"):
        quaternion_matrix(0.0, 0.0, 0.0, 0.0)


def test_world_z_offset_is_explicit_and_does_not_mutate_raw_point():
    raw = np.array([0.2951, -0.0688, 0.1485])
    corrected = apply_world_z_offset(raw, -0.0185)
    np.testing.assert_allclose(corrected, [0.2951, -0.0688, 0.1300])
    np.testing.assert_allclose(raw, [0.2951, -0.0688, 0.1485])
