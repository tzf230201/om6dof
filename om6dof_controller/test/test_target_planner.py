import math
from types import SimpleNamespace

import numpy as np
import pytest

from om6dof_controller.ik_solver import IKSolver
from om6dof_controller.control_math import cartesian_rpy_to_tip_rotation
from om6dof_controller.target_planner import (
    plan_pose, path_is_clear, POSITION_TOLERANCE, ORIENTATION_TOLERANCE,
)


def test_real_urdf_150_0_300_target_escapes_negative_elbow_local_minimum():
    ik = IKSolver()
    ik.q_min += 0.02
    ik.q_max -= 0.02
    start = np.array([0, 0.2868, -0.8958, 0, 2.08486, 0])
    rotation = cartesian_rpy_to_tip_rotation(0, 0, 0)
    position = np.array([0.15, 0, 0.3])
    old, converged = ik.solve_pose_ik(start, position, rotation, max_iter=100)
    assert not converged
    assert np.linalg.norm(ik.fk_pose(old)[0] - position) > 0.01
    plan = plan_pose(ik, start, position, rotation, ik.q_min, ik.q_max,
                     [0, -0.6806, 1.3613, 0, 0.8901, 0], 0.025)
    assert not plan.approximate
    assert plan.position_error < POSITION_TOLERANCE
    assert plan.orientation_error < ORIENTATION_TOLERANCE
    assert np.all(plan.joints >= ik.q_min)
    assert np.all(plan.joints <= ik.q_max)
    assert path_is_clear(ik, start, plan.joints, 0.025)


def test_clear_endpoints_do_not_hide_collision_in_middle_of_path():
    ik = SimpleNamespace(self_collides=lambda q, radius: 0.4 < q[0] < 0.6)
    assert not path_is_clear(ik, np.zeros(6), np.array([1, 0, 0, 0, 0, 0]), .025)


def test_candidate_error_is_evaluated_after_effective_limit_clamping():
    ik = SimpleNamespace(
        solve_pose_ik=lambda *args, **kwargs: (np.array([1, 0, 0, 0, 0, 0]), True),
        fk_pose=lambda q: (q[:3], np.eye(3)),
    )
    plan = plan_pose(ik, np.zeros(6), [1, 0, 0], np.eye(3),
                     [-.5]*6, [.5]*6, [.1]*6)
    assert plan.approximate
    assert plan.joints[0] == .5
    assert plan.position_error == pytest.approx(.5)


def test_approximation_that_does_not_improve_pose_is_rejected():
    ik = SimpleNamespace(
        solve_pose_ik=lambda *args, **kwargs: (np.array([-.1, 0, 0, 0, 0, 0]), False),
        fk_pose=lambda q: (q[:3], np.eye(3)),
    )
    with pytest.raises(ValueError, match='No improving pose'):
        plan_pose(ik, np.zeros(6), [1, 0, 0], np.eye(3), [-1]*6, [1]*6, [.1]*6)


def test_collision_check_exception_fails_closed():
    def failed(*args):
        raise RuntimeError('collision checker failure')
    ik = SimpleNamespace(
        solve_pose_ik=lambda *args, **kwargs: (np.zeros(6), True),
        fk_pose=lambda q: (q[:3], np.eye(3)), self_collides=failed,
    )
    with pytest.raises(RuntimeError, match='collision checker failure'):
        plan_pose(ik, np.zeros(6), [0, 0, 0], np.eye(3), [-1]*6, [1]*6, [.1]*6, .025)
