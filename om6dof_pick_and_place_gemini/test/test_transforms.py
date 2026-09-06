"""Frame algebra: the tool convention and the wrist-camera chain."""

import math

import numpy as np
import pytest

from om6dof_pick_and_place_gemini import transforms as tf


def test_tool_rotation_matches_the_pitch_convention():
    """pitch=pi is "straight down" for the other OM6DOF pick nodes; a grasp
    approaching along -Z with the jaws on Y must produce the same matrix."""
    assert np.allclose(tf.tool_rotation([0, 0, -1], [0, 1, 0]),
                       tf.rpy_to_matrix(0.0, math.pi, 0.0), atol=1e-12)


def test_tool_rotation_is_right_handed_and_orthonormal():
    R = tf.tool_rotation([0.3, 0.1, -0.9], [1.0, 0.4, 0.2])
    assert np.allclose(R.T @ R, np.eye(3), atol=1e-9)
    assert pytest.approx(1.0, abs=1e-9) == float(np.linalg.det(R))


def test_tool_rotation_puts_approach_on_z_and_closing_on_y():
    approach = np.array([0.0, 0.0, -1.0])
    closing = np.array([0.7, 0.7, 0.0])
    R = tf.tool_rotation(approach, closing)
    assert np.allclose(R[:, 2], approach)
    assert np.allclose(R[:, 1], closing / np.linalg.norm(closing))


def test_tool_rotation_survives_a_closing_axis_parallel_to_approach():
    R = tf.tool_rotation([0, 0, -1], [0, 0, 1])
    assert np.allclose(R.T @ R, np.eye(3), atol=1e-9)


def test_quaternion_round_trip():
    R = tf.tool_rotation([0.2, -0.5, -0.84], [0.9, 0.1, 0.0])
    assert np.allclose(tf.quat_to_matrix(*tf.matrix_to_quat(R)), R, atol=1e-9)


def test_camera_pose_in_base_with_an_identity_wrist():
    p_wc, R_wc = tf.camera_pose_in_base(
        np.array([0.3, 0.0, 0.2]), np.eye(3), [0.0, 0.0, -0.05], np.eye(3))
    assert np.allclose(p_wc, [0.3, 0.0, 0.15])
    assert np.allclose(R_wc, np.eye(3))


def test_optical_axes_point_where_the_body_frame_says():
    """Body x is forward, so the optical z (view direction) must be body x."""
    R = tf.optical_from_parent([0.0, 0.0, 0.0])
    assert np.allclose(R[:, 2], [1.0, 0.0, 0.0])   # optical +Z = body forward
    assert np.allclose(R[:, 1], [0.0, 0.0, -1.0])  # optical +Y = body down


def test_deproject_and_project_are_inverses():
    intr = (600.0, 600.0, 320.0, 240.0)
    point = tf.deproject(400.0, 300.0, 0.35, intr)
    pixel = tf.project(point[np.newaxis], intr)[0]
    assert np.allclose(pixel, [400.0, 300.0], atol=1e-6)


def test_points_to_base_applies_rotation_then_translation():
    R = tf.rpy_to_matrix(0.0, 0.0, math.pi / 2)
    out = tf.points_to_base(np.array([[1.0, 0.0, 0.0]]), np.array([0.1, 0.2, 0.3]), R)
    assert np.allclose(out[0], [0.1, 1.2, 0.3], atol=1e-9)


@pytest.mark.parametrize("approach,expected_deg", [
    ([0, 0, -1], 0.0),
    ([1, 0, 0], 90.0),
    ([0, 0, 1], 180.0),
])
def test_approach_tilt_from_vertical(approach, expected_deg):
    got = math.degrees(tf.approach_tilt_from_vertical(approach))
    assert pytest.approx(expected_deg, abs=1e-6) == got
