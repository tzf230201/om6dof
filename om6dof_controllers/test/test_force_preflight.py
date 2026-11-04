import math

from om6dof_controllers.force_preflight import force_pose_problem, load_force_limits


def test_reported_joint3_start_pose_is_inside_updated_urdf_range():
    limits = load_force_limits()
    q = list(map(math.radians, [-5.71, -114.96, 86.57, 3.52, 12.83, -3.25]))
    assert not force_pose_problem(q, [0.] * 6, limits)


def test_joint3_beyond_ninety_degrees_is_still_rejected():
    limits = load_force_limits()
    q = [0.] * 6
    q[2] = math.radians(90.1)
    problem = force_pose_problem(q, [0.] * 6, limits)
    assert 'joint3' in problem
    assert '90.10' in problem
    assert '90.00' in problem


def test_reported_joint2_pose_is_allowed_without_extra_margin():
    q = list(map(math.radians, [0, -114.96, 15, 0, 60, 0]))
    assert not force_pose_problem(q, [0.] * 6, load_force_limits())


def test_ready_pose_is_inside_limits_and_speed_still_checked():
    limits = load_force_limits()
    q = list(map(math.radians, [0, -60, 15, 0, 60, 0]))
    assert not force_pose_problem(q, [0.] * 6, limits)
    assert 'joint2 speed' in force_pose_problem(q, [0, .06, 0, 0, 0, 0], limits)


def test_missing_feedback_never_passes_preflight():
    assert force_pose_problem([0.] * 6, [], ([], [], []))
