from types import SimpleNamespace

import numpy as np
import pytest

from workspace_scan import conditioning, grid_points, main, parser, scan_point
from workspace_scan import IKSolver, cartesian_rpy_to_tip_rotation


def fake_ik(position=(0, 0, 0), collision=False, singular=False):
    return SimpleNamespace(
        q_min=np.full(6, -1.), q_max=np.full(6, 1.),
        solve_position_ik=lambda *a, **kw: (np.zeros(6), True),
        solve_pose_ik=lambda *a, **kw: (np.zeros(6), True),
        fk_pose=lambda q: (np.array(position), np.eye(3)),
        self_collides=lambda *a: collision,
        jacobian=lambda q: np.diag([.3, .3, .3, 1, 1, 0 if singular else 1]),
    )


def test_grid_contains_both_hemispheres_and_only_sphere_centres():
    points = grid_points(.2, .1)
    assert len(points) == 33
    assert np.all(np.linalg.norm(points, axis=1) <= .2 + 1e-12)
    assert any(np.allclose(p, [0, 0, -.2]) for p in points)
    assert any(np.allclose(p, [0, 0, 0]) for p in points)


@pytest.mark.parametrize('collision,singular,status', [
    (False, False, 'pose_found'),
    (False, True, 'pose_near_singular'),
    (True, False, 'collision_candidates_only'),
])
def test_classification(collision, singular, status):
    args = parser().parse_args([])
    result = scan_point(fake_ik(collision=collision, singular=singular), np.zeros(3),
                        [np.zeros(6)], np.eye(3), args)
    assert result['status'] == status
    assert result['pose_found'] == (not collision)


def test_pose_solver_success_flag_is_not_evidence_of_reachability():
    result = scan_point(fake_ik(), np.ones(3), [np.zeros(6)], np.eye(3), parser().parse_args([]))
    assert result['status'] == 'position_unresolved'
    assert not result['position_found']


def test_position_and_orientation_failures_are_distinct():
    rotation = np.diag([-1, -1, 1])
    result = scan_point(fake_ik(), np.zeros(3), [np.zeros(6)], rotation, parser().parse_args([]))
    assert result['status'] == 'orientation_unresolved'
    assert result['position_found']
    assert not result['pose_found']


def test_conditioning_uses_dimensionless_scaled_jacobian():
    assert conditioning(fake_ik(), np.zeros(6), .3) == pytest.approx(1.)
    assert conditioning(fake_ik(singular=True), np.zeros(6), .3) == 0


def test_nonfinite_solver_candidate_is_not_accepted():
    ik = fake_ik()
    ik.solve_pose_ik = ik.solve_position_ik = lambda *a, **kw: (np.full(6, np.nan), True)
    result = scan_point(ik, np.zeros(3), [np.zeros(6)], np.eye(3), parser().parse_args([]))
    assert result['status'] == 'position_unresolved'
    assert result['best_position_error_mm'] is None


def test_real_robot_known_target_has_noncolliding_pose_witness():
    ik = IKSolver()
    ik.q_min += .02
    ik.q_max -= .02
    args = parser().parse_args(['--iterations', '160'])
    result = scan_point(ik, np.array([.15, 0, .3]),
                        [np.array([0, -.6806, 1.3613, 0, .8901, 0])],
                        cartesian_rpy_to_tip_rotation(0, 0, 0), args)
    assert result['pose_found']
    assert result['position_error_mm'] <= 1
    assert result['orientation_error_deg'] <= .5
    witness = np.array([result[f'q{i+1}_rad'] for i in range(6)])
    assert not ik.self_collides(witness, .025)
    assert np.all(witness >= ik.q_min)
    assert np.all(witness <= ik.q_max)


def test_out_of_limit_solver_candidate_is_not_reachable():
    ik = fake_ik()
    ik.solve_pose_ik = ik.solve_position_ik = lambda *a, **kw: (np.full(6, 2.), True)
    result = scan_point(ik, np.zeros(3), [np.zeros(6)], np.eye(3), parser().parse_args([]))
    assert not result['position_found']
    assert not result['pose_found']


def test_existing_experiment_directory_is_not_overwritten(tmp_path):
    with pytest.raises(SystemExit):
        main(['--output', str(tmp_path)])
    assert list(tmp_path.iterdir()) == []


@pytest.mark.parametrize('argument,value', [
    ('--spacing-mm', '0'), ('--radius-mm', 'nan'), ('--samples', '-1'),
    ('--joint-margin-rad', '-.1'), ('--iterations', '0'),
])
def test_invalid_scan_settings_are_rejected(argument, value):
    with pytest.raises(SystemExit):
        main([argument, value])
