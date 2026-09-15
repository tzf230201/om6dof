"""Geometry and planning contracts for the separate Boxer-center pickup path."""

import math
from types import SimpleNamespace

import pytest

pytest.importorskip('rclpy')
from geometry_msgs.msg import Pose  # noqa: E402
from moveit_msgs.msg import CollisionObject, RobotTrajectory  # noqa: E402
from moveit_msgs.srv import GetCartesianPath  # noqa: E402
from sensor_msgs.msg import JointState  # noqa: E402
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint  # noqa: E402

from om6dof_pick_and_place.boxer_pick_planning import (  # noqa: E402
    ARM_JOINTS, GRIPPER_JOINTS, BoxObstacle, PickupPlanner, PlanningError,
    joint_limits_from_urdf, retime_stopped_waypoints, side_grasp_poses,
    state_with_joints, tool_forward, trajectory_endpoint,
)


def trajectory(points):
    return JointTrajectory(joint_names=list(ARM_JOINTS), points=[
        JointTrajectoryPoint(positions=[float(v) for v in q]) for q in points])


def state(q=None):
    return JointState(name=list(ARM_JOINTS) + list(GRIPPER_JOINTS),
                      position=list(q or [0.0] * 6) + [0.019, 0.019])


def bare_planner():
    planner = object.__new__(PickupPlanner)
    planner.reference_frame = 'world'
    planner.group_name = 'arm'
    planner.ee_link = 'end_effector_link'
    planner.planning_time = 3.0
    planner.validation_timeout = 5.0
    planner._scene_ids = set()
    planner._scene = object()
    planner._valid = object()
    planner.joint_limits = {name: (-1.5, 1.5) for name in ARM_JOINTS}
    planner.joint_limits.update({name: (-0.011, 0.020) for name in GRIPPER_JOINTS})
    return planner


@pytest.mark.parametrize('center', [(0.30, 0.0, 0.12), (0.27, -0.09, 0.08),
                                    (-0.2, 0.15, 0.1), (-0.3, 0.0, 0.1)])
@pytest.mark.parametrize('roll', [1, -1])
def test_gripper_closes_horizontally_and_approaches_box_center(center, roll):
    pre, grasp = side_grasp_poses(center, 0.075, roll_sign=roll)
    forward = tool_forward(grasp.orientation)
    assert forward[2] == pytest.approx(0.0, abs=1e-12)
    assert forward[:2] == pytest.approx(
        (center[0] / math.hypot(*center[:2]), center[1] / math.hypot(*center[:2])))
    assert pre.position.z == grasp.position.z == center[2]
    assert (grasp.position.x - pre.position.x, grasp.position.y - pre.position.y) \
        == pytest.approx([0.075 * value for value in forward[:2]])
    q = grasp.orientation
    # R's local-Y column is the actual V2 finger-joint direction.
    jaw_axis = (2 * (q.x * q.y - q.w * q.z),
                1 - 2 * (q.x * q.x + q.z * q.z),
                2 * (q.y * q.z + q.w * q.x))
    assert jaw_axis[2] == pytest.approx(0.0, abs=1e-12)
    assert sum(a * b for a, b in zip(jaw_axis, forward)) == pytest.approx(0.0, abs=1e-12)


@pytest.mark.parametrize('center', [(0.0, 0.0, 0.1), (float('nan'), 0.1, 0.1), (0.2, 0.1)])
def test_undefined_or_nonfinite_target_is_rejected(center):
    with pytest.raises(PlanningError):
        side_grasp_poses(center)


def test_yaw_alternative_remains_horizontal_and_reaches_same_center():
    pre, grasp = side_grasp_poses((0.3, 0.0, 0.1), yaw_offset=0.25)
    assert tool_forward(grasp.orientation) == pytest.approx((math.cos(0.25), math.sin(0.25), 0))
    assert (grasp.position.x, grasp.position.y, grasp.position.z) == (0.3, 0.0, 0.1)
    assert pre.position.z == 0.1


def test_frozen_state_updates_both_fingers_without_altering_measurement():
    measured = state()
    measured.header.stamp.sec = 123
    frozen = state_with_joints(measured, GRIPPER_JOINTS, [-0.01, -0.01])
    assert list(measured.position[-2:]) == [0.019, 0.019]
    assert list(frozen.joint_state.position[-2:]) == [-0.01, -0.01]
    assert not frozen.is_diff
    assert frozen.joint_state.header.stamp.sec == 0
    assert list(trajectory_endpoint(frozen, trajectory([[0.1] * 6])).joint_state.position[:6]) == [0.1] * 6


def test_duplicate_measured_joint_names_rejected():
    measured = state()
    measured.name[-1] = measured.name[-2]
    with pytest.raises(PlanningError, match='joint_state_invalid'):
        state_with_joints(measured, [], [])


def test_motion_request_uses_explicit_open_fingers_and_pose_constraint():
    planner = bare_planner()
    frozen = state_with_joints(state(), GRIPPER_JOINTS, [0.019, 0.019])
    pre, _ = side_grasp_poses((0.3, 0.0, 0.1))
    request = planner.motion_request(frozen, pre).motion_plan_request
    assert request.pipeline_id == 'ompl'
    assert request.planner_id == 'RRTConnect'
    assert list(request.start_state.joint_state.position[-2:]) == [0.019, 0.019]
    assert request.start_state is not frozen
    assert request.goal_constraints[0].position_constraints[0].link_name == 'end_effector_link'
    assert list(request.goal_constraints[0].position_constraints[0].constraint_region.primitives[0].dimensions) == [0.0015]
    assert 0 < request.max_velocity_scaling_factor < 1


def test_cartesian_request_uses_collision_check_absolute_jump_limit_and_center():
    planner = bare_planner()
    _, grasp = side_grasp_poses((0.3, 0.0, 0.1))
    frozen = state_with_joints(state(), [], [])
    request = planner.cartesian_request(frozen, grasp)
    assert request.avoid_collisions
    assert request.revolute_jump_threshold == 0.15
    assert request.max_step <= 0.003
    assert request.waypoints[-1].position.x == 0.3
    assert not request.start_state.is_diff


@pytest.mark.parametrize('fraction', [0.0, 0.99, 0.9999, float('nan')])
def test_partial_cartesian_path_cannot_enable_pickup(fraction):
    planner = bare_planner()
    response = GetCartesianPath.Response()
    response.error_code.val = 1
    response.fraction = fraction
    with pytest.raises(PlanningError, match='cartesian_path_incomplete'):
        planner._complete_insertion(response, state_with_joints(state(), [], []), Pose())


def test_success_code_and_full_fraction_still_require_actual_fk_at_center():
    planner = bare_planner()
    response = GetCartesianPath.Response()
    response.error_code.val = 1
    response.fraction = 1.0
    response.solution = RobotTrajectory(joint_trajectory=trajectory([[0.0] * 6, [0.05] * 6]))
    _, grasp = side_grasp_poses((0.30, 0.0, 0.10))
    wrong = Pose()
    wrong.position.x = 0.25
    wrong.position.z = 0.10
    wrong.orientation = grasp.orientation
    planner.fk = lambda _: wrong
    with pytest.raises(PlanningError, match='grasp_center_not_reached'):
        planner._complete_insertion(response, state_with_joints(state(), [], []), grasp)


def test_complete_path_rejects_joint_branch_jump_even_if_service_claims_success():
    planner = bare_planner()
    response = GetCartesianPath.Response()
    response.error_code.val = 1
    response.fraction = 1.0
    response.solution = RobotTrajectory(joint_trajectory=trajectory([[0.0] * 6, [0.8] * 6]))
    with pytest.raises(PlanningError, match='cartesian_joint_jump'):
        planner._complete_insertion(response, state_with_joints(state(), [], []), Pose())


def test_scene_keeps_other_bottle_and_removes_old_selected_box():
    planner = bare_planner()
    requests = []
    planner._call = lambda client, request: requests.append(request) or SimpleNamespace(success=True)
    pose = Pose()
    pose.orientation.w = 1.0
    first = BoxObstacle('bottle_1', pose, (0.06, 0.06, 0.15))
    second = BoxObstacle('bottle_2', pose, (0.06, 0.06, 0.15))
    planner.set_scene([first, second], selected_id='bottle_1')
    objects = requests[-1].scene.world.collision_objects
    assert [obj.id for obj in objects] == ['boxer_pick_box_bottle_2']
    planner.set_scene([first, second], selected_id='bottle_2')
    operations = {obj.id: obj.operation for obj in requests[-1].scene.world.collision_objects}
    assert operations['boxer_pick_box_bottle_1'] == CollisionObject.ADD
    assert operations['boxer_pick_box_bottle_2'] == CollisionObject.REMOVE


def test_rerendered_box_does_not_accumulate_in_world():
    planner = bare_planner()
    requests = []
    planner._call = lambda client, request: requests.append(request) or SimpleNamespace(success=True)
    pose = Pose()
    pose.orientation.w = 1.0
    planner.set_scene([BoxObstacle('keyboard', pose, (0.3, 0.1, 0.03))], 'target')
    planner.set_scene([], 'target')
    assert requests[-1].scene.world.collision_objects[0].operation == CollisionObject.REMOVE


def test_trajectory_revalidation_checks_intermediate_mesh_and_full_robot():
    planner = bare_planner()
    sampled = []
    def valid(client, request, deadline=None):
        assert request.group_name == ''
        sampled.append(request.robot_state.joint_state.position[0])
        return SimpleNamespace(valid=not 0.04 <= sampled[-1] <= 0.06, contacts=[])
    planner._call = valid
    with pytest.raises(PlanningError, match='state_collision_or_invalid'):
        planner.validate_segment(trajectory([[0.0] * 6, [0.10] * 6]), state(), 0.019)
    assert any(0.04 <= q <= 0.06 for q in sampled)


def test_joint_limit_failure_cannot_be_masked_by_valid_collision_response():
    planner = bare_planner()
    planner._call = lambda *args: SimpleNamespace(valid=True, contacts=[])
    with pytest.raises(PlanningError, match='joint_out_of_bounds'):
        planner.validate_segment(trajectory([[1.51] * 6]), state([1.51] * 6), 0.019)


def test_large_measured_drift_does_not_create_unplanned_return_to_start():
    planner = bare_planner()
    with pytest.raises(PlanningError, match='trajectory_start_changed'):
        planner.validate_segment(trajectory([[0.0] * 6]), state([0.2] * 6), 0.019)


def test_gripper_closure_sweeps_intermediate_finger_states():
    planner = bare_planner()
    positions = []
    def check(value, deadline=None):
        planner._check_bounds(value)
        positions.append(value.joint_state.position[-2:])
        if -0.001 <= positions[-1][0] <= 0.001:
            raise PlanningError('state_collision_or_invalid:finger__obstacle')
    planner._valid_state = check
    with pytest.raises(PlanningError, match='finger__obstacle'):
        planner.validate_gripper(state(), 0.019, -0.010)
    assert all(a == b for a, b in positions)
    assert len(positions) > 10


def test_quintic_retiming_bounds_speed_and_acceleration_and_preserves_edges():
    source = trajectory([[0.0] * 6, [0.10] * 6, [0.11] * 6])
    timed = retime_stopped_waypoints(source, velocity=0.2, acceleration=0.4)
    assert [p.positions for p in timed.points] == [p.positions for p in source.points]
    times = [p.time_from_start.sec + p.time_from_start.nanosec * 1e-9 for p in timed.points]
    for i in (1, 2):
        duration = times[i] - times[i - 1]
        delta = abs(timed.points[i].positions[0] - timed.points[i - 1].positions[0])
        assert 1.875 * delta / duration <= 0.2 + 1e-8
        assert (10 / math.sqrt(3)) * delta / duration ** 2 <= 0.4 + 1e-8
        assert list(timed.points[i].velocities) == [0.0] * 6
        assert list(timed.points[i].accelerations) == [0.0] * 6
    assert source.points[-1].time_from_start.sec == 0


def test_prefix_shortcut_removes_sampling_stops_after_dense_collision_validation():
    planner = bare_planner()
    sampled = []
    planner._valid_state = lambda state, deadline=None: sampled.append(state.joint_state.position[0])
    original = trajectory([[value] * 6 for value in (0.0, 0.02, 0.04, 0.06, 0.08, 0.10)])
    shortened = planner._shortcut_pregrasp(original, state_with_joints(state(), [], []))
    assert len(shortened.points) == 2
    assert len(original.points) == 6
    assert sampled == pytest.approx([0.025, 0.05, 0.075, 0.10])


def test_prefix_shortcut_cannot_cut_through_obstacle():
    planner = bare_planner()
    def check(value, deadline=None):
        q = value.joint_state.position
        # Straight joint-space chord cuts through the obstacle; the original
        # detour first raises joint2 and only then advances joint1.
        if 0.04 < q[0] < 0.16 and q[1] < 0.12:
            raise PlanningError('state_collision_or_invalid:arm__obstacle')
    planner._valid_state = check
    original = trajectory([[0, 0, 0, 0, 0, 0], [0, .2, 0, 0, 0, 0],
                           [.2, .2, 0, 0, 0, 0], [.2, 0, 0, 0, 0, 0]])
    shortened = planner._shortcut_pregrasp(original, state_with_joints(state(), [], []))
    assert len(shortened.points) > 2
    planner.validate_segment(shortened, state(), .019)


def test_fk_orientation_checks_jaw_roll_in_addition_to_forward_axis():
    planner = bare_planner()
    response = GetCartesianPath.Response()
    response.error_code.val = 1
    response.fraction = 1.0
    response.solution = RobotTrajectory(joint_trajectory=trajectory([[0.0] * 6, [0.05] * 6]))
    _, grasp = side_grasp_poses((.3, 0, .1), roll_sign=1)
    _, wrong_roll = side_grasp_poses((.3, 0, .1), roll_sign=-1)
    planner.fk = lambda _: wrong_roll
    with pytest.raises(PlanningError, match='grasp_orientation_misaligned'):
        planner._complete_insertion(response, state_with_joints(state(), [], []), grasp)


def test_urdf_limits_are_loaded_from_model_not_guessed():
    limits = joint_limits_from_urdf('<robot><joint name="joint3" type="revolute">'
                                  '<limit lower="-1.75" upper="1.47"/></joint></robot>')
    assert limits['joint3'] == (-1.75, 1.47)
