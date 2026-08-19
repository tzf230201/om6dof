import math

import numpy as np
import pytest

from om6dof_controller.control_math import (
    MODE_AUTONOMOUS,
    MODE_CARTESIAN,
    MODE_CYLINDRICAL,
    MODE_JOINT,
    MODE_SEMI_CYLINDRICAL,
    clamp_positions,
    integrate_cylindrical_position,
    next_motion_mode,
    normalize_operation_mode,
    rotation_error,
    rotation_from_rotvec,
    rotation_from_zyx,
    rotation_to_zyx,
    semi_cylindrical_rotation,
    step_toward,
    validated_control_command,
)


def test_operation_mode_aliases_and_cycle():
    assert normalize_operation_mode("auto") == MODE_AUTONOMOUS
    assert normalize_operation_mode("joint") == MODE_JOINT
    assert normalize_operation_mode("ik") == MODE_CARTESIAN
    assert normalize_operation_mode("silinder") == MODE_CYLINDRICAL
    assert next_motion_mode(MODE_JOINT) == MODE_CARTESIAN
    assert next_motion_mode(MODE_CARTESIAN) == MODE_CYLINDRICAL
    assert next_motion_mode(MODE_CYLINDRICAL) == MODE_SEMI_CYLINDRICAL
    assert next_motion_mode(MODE_SEMI_CYLINDRICAL) == MODE_JOINT
    assert normalize_operation_mode("semi") == MODE_SEMI_CYLINDRICAL
    with pytest.raises(ValueError):
        normalize_operation_mode("twist")


@pytest.mark.parametrize(
    "values",
    ([0.0] * 5, [0.0] * 7, [0.0, 0.0, math.nan, 0.0, 0.0, 0.0]),
)
def test_control_command_requires_six_finite_values(values):
    with pytest.raises(ValueError):
        validated_control_command(values)
    assert validated_control_command([0.0] * 6).shape == (6,)


def test_joint_clamp_and_pose_step():
    assert clamp_positions([-2, 0, 2], [-1, -1, -1], [1, 1, 1]) == [
        -1.0, 0.0, 1.0
    ]
    assert step_toward([0.0, 0.0], [1.0, -1.0], 0.1) == pytest.approx(
        [0.1, -0.1]
    )


def test_rotation_helpers_recover_rotation_vector():
    vector = np.array([0.1, -0.2, 0.05])
    assert rotation_error(rotation_from_rotvec(vector), np.eye(3)) == pytest.approx(
        vector, abs=1e-9
    )


def test_cylindrical_theta_does_not_drift_radius():
    result, theta = integrate_cylindrical_position(
        [0.212, 0.0, 0.3],
        radial_velocity=0.0,
        theta_velocity=math.pi / 2.0,
        vertical_velocity=0.0,
        dt=1.0,
        origin_xy=[0.012, 0.0],
        minimum_radius=0.03,
    )
    assert result == pytest.approx([0.012, 0.2, 0.3], abs=1e-9)
    assert theta == pytest.approx(math.pi / 2.0)


def test_cylindrical_min_radius_does_not_jump_on_entry():
    result, _ = integrate_cylindrical_position(
        [0.022, 0.0, 0.3],
        radial_velocity=0.0,
        theta_velocity=0.0,
        vertical_velocity=0.0,
        dt=0.02,
        origin_xy=[0.012, 0.0],
        minimum_radius=0.03,
        theta_hint=0.0,
    )
    assert result == pytest.approx([0.022, 0.0, 0.3])


def test_zyx_round_trip_recovers_the_angles():
    for roll, pitch, yaw in [
        (0.0, 0.0, 0.0),
        (0.3, -0.4, 1.2),
        (-1.1, 0.2, -2.5),
        (0.0, 1.0, math.pi / 2),
    ]:
        matrix = rotation_from_zyx(roll, pitch, yaw)
        assert np.allclose(matrix @ matrix.T, np.eye(3), atol=1e-9)
        assert math.isclose(float(np.linalg.det(matrix)), 1.0, abs_tol=1e-9)
        back = rotation_to_zyx(matrix)
        assert np.allclose(back, (roll, pitch, yaw), atol=1e-9)


def test_zyx_survives_gimbal_lock():
    """Pitch at +/-90 degrees leaves roll and yaw degenerate.

    The decomposition must still return a usable triple rather than a NaN,
    because this runs inside the control loop.
    """
    for pitch in (math.pi / 2, -math.pi / 2):
        matrix = rotation_from_zyx(0.4, pitch, 0.9)
        roll, recovered_pitch, yaw = rotation_to_zyx(matrix)
        assert all(math.isfinite(v) for v in (roll, recovered_pitch, yaw))
        assert math.isclose(recovered_pitch, pitch, abs_tol=1e-7)
        # Only the sum (or difference) is observable when locked, so check
        # the rotation itself round-trips rather than the individual angles.
        assert np.allclose(rotation_from_zyx(roll, recovered_pitch, yaw),
                           matrix, atol=1e-7)


def test_semi_cylindrical_yaw_follows_theta():
    """Yaw tracks theta; pitch and roll stay where they were put."""
    roll, pitch, yaw_offset = 0.2, -0.3, 0.5
    for theta in (0.0, 0.7, -1.9, math.pi):
        matrix = semi_cylindrical_rotation(theta, roll, pitch, yaw_offset)
        got_roll, got_pitch, got_yaw = rotation_to_zyx(matrix)
        assert math.isclose(got_roll, roll, abs_tol=1e-9)
        assert math.isclose(got_pitch, pitch, abs_tol=1e-9)
        expected = math.atan2(
            math.sin(theta + yaw_offset), math.cos(theta + yaw_offset)
        )
        assert math.isclose(got_yaw, expected, abs_tol=1e-9)


def test_semi_cylindrical_sweep_leaves_pitch_and_roll_untouched():
    """Swinging a quarter turn must not tilt the tool.

    This is the whole point of the mode: theta changes the heading and
    nothing else.
    """
    start = semi_cylindrical_rotation(0.0, 0.15, -0.6, 0.0)
    swept = semi_cylindrical_rotation(math.pi / 2, 0.15, -0.6, 0.0)
    assert math.isclose(rotation_to_zyx(start)[0], rotation_to_zyx(swept)[0],
                        abs_tol=1e-9)
    assert math.isclose(rotation_to_zyx(start)[1], rotation_to_zyx(swept)[1],
                        abs_tol=1e-9)
    delta = rotation_to_zyx(swept)[2] - rotation_to_zyx(start)[2]
    assert math.isclose(delta, math.pi / 2, abs_tol=1e-9)
