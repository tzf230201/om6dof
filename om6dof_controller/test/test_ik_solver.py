import math

import numpy as np
import pytest

from om6dof_controller.ik_solver import IKSolver


def _rotation_from_rotvec(rotvec: np.ndarray) -> np.ndarray:
    """Small Rodrigues helper kept local so this test only exercises IK code."""
    vector = np.asarray(rotvec, dtype=float)
    angle = float(np.linalg.norm(vector))
    if angle < 1e-15:
        return np.eye(3)
    axis = vector / angle
    skew = np.array([
        [0.0, -axis[2], axis[1]],
        [axis[2], 0.0, -axis[0]],
        [-axis[1], axis[0], 0.0],
    ])
    return np.eye(3) + math.sin(angle) * skew + (1.0 - math.cos(angle)) * (
        skew @ skew
    )


def test_orientation_difference_recovers_small_base_frame_rotation():
    present = _rotation_from_rotvec(np.array([0.2, -0.1, 0.4]))
    expected = np.array([0.03, -0.04, 0.02])
    target = _rotation_from_rotvec(expected) @ present

    error = IKSolver.orientation_difference(target, present)

    assert error == pytest.approx(expected, abs=1e-12)


@pytest.mark.parametrize(
    "axis",
    [
        np.array([1.0, 0.0, 0.0]),
        np.array([0.0, 1.0, 0.0]),
        np.array([0.0, 0.0, 1.0]),
        np.array([1.0, -2.0, 3.0]) / math.sqrt(14.0),
    ],
)
def test_orientation_difference_does_not_collapse_at_180_degrees(axis):
    target = _rotation_from_rotvec(axis * math.pi)

    error = IKSolver.orientation_difference(target, np.eye(3))

    assert np.all(np.isfinite(error))
    assert np.linalg.norm(error) == pytest.approx(math.pi, abs=1e-10)
    # Axis sign is ambiguous at exactly pi, so compare the represented rotation.
    assert _rotation_from_rotvec(error) == pytest.approx(target, abs=1e-10)


@pytest.mark.parametrize("offset", [1e-8, 1e-6, 1e-4])
def test_orientation_difference_is_stable_just_below_180_degrees(offset):
    axis = np.array([-2.0, 1.0, 3.0]) / math.sqrt(14.0)
    expected = axis * (math.pi - offset)
    target = _rotation_from_rotvec(expected)

    error = IKSolver.orientation_difference(target, np.eye(3))

    assert error == pytest.approx(expected, abs=2e-9)


def test_pose_ik_does_not_report_false_convergence_for_half_turn():
    """Regression: the former sin(theta) error was zero at theta == pi."""
    solver = IKSolver.__new__(IKSolver)
    solver.n_joints = 6
    solver.q_min = np.full(6, -math.pi)
    solver.q_max = np.full(6, math.pi)
    solver.fk_pose = lambda _q: (np.zeros(3), np.eye(3))
    solver.jacobian = lambda _q: np.eye(6)

    _, converged = solver.solve_pose_ik(
        np.zeros(6),
        np.zeros(3),
        _rotation_from_rotvec(np.array([math.pi, 0.0, 0.0])),
        max_iter=1,
    )

    assert not converged


def test_orientation_difference_rejects_wrong_matrix_shape():
    with pytest.raises(ValueError, match="shape"):
        IKSolver.orientation_difference(np.eye(4), np.eye(3))
