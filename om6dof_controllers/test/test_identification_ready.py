"""Exercise sequencing without constructing a ROS node or sending motor goals."""
import importlib.util
import csv
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))

spec = importlib.util.spec_from_file_location(
    'multisine', Path(__file__).resolve().parents[1] / 'scripts/multisine_identification.py')
runner = importlib.util.module_from_spec(spec)
spec.loader.exec_module(runner)
TEST_READY = (0.1, 0.2, 0.3, 0.1, 0.2, 0.1)


@pytest.mark.parametrize('failure', ['rejected', 'timeout', 'unsettled'])
def test_ready_failure_prevents_calibration_and_recording(tmp_path, failure):
    move = Mock(side_effect=RuntimeError(failure) if failure != 'unsettled' else None)
    settle = Mock(side_effect=RuntimeError('unsettled'))
    node = SimpleNamespace(
        _client=Mock(), wait_for_state=lambda _: True,
        get_logger=Mock(), _move_to_ready=move, _wait_ready_settled=settle,
        _ready_timeout=10.,
        _args=SimpleNamespace(ready_only=False, output=str(tmp_path / 'data.csv')),
        preview=Mock())
    with pytest.raises(RuntimeError, match=failure):
        runner.IdentificationRun.execute(node)
    node._client.send_goal_async.assert_not_called()
    node.preview.assert_not_called()
    assert not (tmp_path / 'data.csv').exists()


def test_success_requires_ready_and_settle_before_recording(tmp_path):
    events = []
    output = tmp_path / 'data.csv'
    def send(goal, feedback_callback):
        events.append('calibrate')
        assert output.exists()
        assert callable(feedback_callback)
        raise RuntimeError('end of offline test')
    node = SimpleNamespace(
        _client=SimpleNamespace(wait_for_server=lambda **kw: True, send_goal_async=send),
        wait_for_state=lambda _: True, get_logger=Mock(),
        _move_to_ready=lambda: events.append('ready'),
        _wait_ready_settled=lambda **kw: events.append('settled'), _ready_timeout=10.,
        preview=lambda: events.append('preview'), _trajectory=lambda: None,
        _args=SimpleNamespace(ready_only=False, output=str(output)),
        _active_handle=None, _active_result=None)
    node._begin_recording = lambda: runner.IdentificationRun._begin_recording(node)
    with pytest.raises(RuntimeError, match='offline test'):
        runner.IdentificationRun.execute(node)
    assert events == ['ready', 'settled', 'preview', 'calibrate']
    assert not node._recording


@pytest.mark.parametrize('fresh,at_ready', [(False, True), (True, False), (True, True)])
def test_measured_ready_gate(monkeypatch, fresh, at_ready):
    clock = [0.0]
    message = SimpleNamespace(name=list(runner.ARM_JOINTS),
        position=list(TEST_READY if at_ready else [0]*6), velocity=[0]*6)
    node = SimpleNamespace(_latest=message, _latest_received=-10., get_logger=Mock(),
                           _ready_pose=TEST_READY)
    monkeypatch.setattr(runner.time, 'monotonic', lambda: clock[0])
    monkeypatch.setattr(runner.rclpy, 'ok', lambda: True)
    def spin(*args, **kwargs):
        clock[0] += .1
        if fresh:
            node._latest_received = clock[0]
    monkeypatch.setattr(runner.rclpy, 'spin_once', spin)
    if fresh and at_ready:
        runner.IdentificationRun._wait_ready_settled(node, timeout=2)
        assert clock[0] >= 1.
    else:
        with pytest.raises(RuntimeError, match='not confirmed'):
            runner.IdentificationRun._wait_ready_settled(node, timeout=2)


def test_passive_recorder_writes_compatible_csv_without_motion(monkeypatch, tmp_path):
    output = tmp_path / 'passive.csv'
    node = SimpleNamespace(
        _args=SimpleNamespace(output=str(output), rate=100.), _client=Mock(),
        _last_recorded=-float('inf'), _samples=0,
        wait_for_state=lambda _: True, get_logger=Mock())
    node._begin_recording = lambda: runner.IdentificationRun._begin_recording(node)
    message = SimpleNamespace(
        name=list(runner.ARM_JOINTS), position=list(TEST_READY),
        velocity=[0.1]*6, effort=[20.]*6,
        header=SimpleNamespace(stamp=SimpleNamespace(sec=1, nanosec=0)))
    monkeypatch.setattr(runner.rclpy, 'spin',
                        lambda _: runner.IdentificationRun._on_joint_state(node, message))
    runner.IdentificationRun.record_only(node)
    node._client.send_goal_async.assert_not_called()
    with output.open() as handle:
        rows = list(csv.DictReader(handle))
    assert len(rows) == 1
    assert float(rows[0]['joint2_effort_raw']) == 20.
    assert not node._recording
    assert node._csv_file is None


def test_recorder_does_not_overwrite_existing_dataset(tmp_path):
    output = tmp_path / 'existing.csv'
    output.write_text('original measurement')
    node = SimpleNamespace(_args=SimpleNamespace(output=str(output)))
    with pytest.raises(FileExistsError):
        runner.IdentificationRun._begin_recording(node)
    assert output.read_text() == 'original measurement'


@pytest.mark.parametrize('observed', ['transition', 'ready_only', 'stale'])
def test_go_to_ready_command_requires_transition_and_never_sends_direct_goal(monkeypatch, observed):
    clock = [1.]
    transition = (0., 0., -1.57, 0., 0., 0.)
    publisher = Mock(get_subscription_count=lambda: 1)
    node = SimpleNamespace(
        _load_ready_configuration=Mock(), _ready_topic='/om6dof/operation_mode',
        _ready_timeout=1., _transition_pose=transition, _latest=None,
        _latest_received=0., _client=Mock(), get_logger=Mock(),
        create_publisher=Mock(return_value=publisher))
    monkeypatch.setattr(runner.time, 'monotonic', lambda: clock[0])
    monkeypatch.setattr(runner.rclpy, 'ok', lambda: True)
    def spin(*a, **kw):
        clock[0] += .1
        node._latest = SimpleNamespace(name=list(runner.ARM_JOINTS),
            position=list(transition if observed != 'ready_only' else TEST_READY))
        if observed != 'stale':
            node._latest_received = clock[0]
    monkeypatch.setattr(runner.rclpy, 'spin_once', spin)
    if observed == 'transition':
        runner.IdentificationRun._move_to_ready(node)
    else:
        with pytest.raises(RuntimeError, match='TRANSITION was not observed'):
            runner.IdentificationRun._move_to_ready(node)
    assert publisher.publish.call_args.args[0].data == 'READY'
    node._client.send_goal_async.assert_not_called()


def test_missing_ready_config_never_publishes_or_moves():
    node = SimpleNamespace(_load_ready_configuration=Mock(side_effect=RuntimeError('unavailable')),
                           create_publisher=Mock(), _client=Mock())
    with pytest.raises(RuntimeError, match='unavailable'):
        runner.IdentificationRun._move_to_ready(node)
    node.create_publisher.assert_not_called()
    node._client.send_goal_async.assert_not_called()


def test_ready_configuration_comes_from_live_controller(monkeypatch):
    from rcl_interfaces.msg import ParameterValue
    response = SimpleNamespace(values=[
        ParameterValue(double_array_value=[0., 0., -80., 0., 0., 0.]),
        ParameterValue(double_array_value=[1., 2., 3., 4., 5., 6.]),
        ParameterValue(string_value='/custom/operation_mode'),
        ParameterValue(double_value=6.),
        ParameterValue(string_value='/arm_controller/follow_joint_trajectory')])
    future = Mock(done=lambda: True, result=lambda: response)
    client = Mock(wait_for_service=lambda **kw: True, call_async=Mock(return_value=future))
    node = SimpleNamespace(create_client=Mock(return_value=client), destroy_client=Mock(),
        _args=SimpleNamespace(action='/arm_controller/follow_joint_trajectory'))
    monkeypatch.setattr(runner.rclpy, 'spin_until_future_complete', Mock())
    runner.IdentificationRun._load_ready_configuration(node)
    assert node.create_client.call_args.args[1] == '/om6dof_controller/get_parameters'
    assert 'pose_profile_duration_seconds' in client.call_async.call_args.args[0].names
    assert node._ready_pose == pytest.approx([runner.math.radians(v) for v in [1,2,3,4,5,6]])
    assert node._transition_pose[2] == pytest.approx(runner.math.radians(-80))
    assert node._ready_topic == '/custom/operation_mode'
    node.destroy_client.assert_called_once_with(client)


def test_execute_cli_enters_verified_workflow(monkeypatch):
    monkeypatch.setattr(runner.sys, 'argv', ['multisine_identification.py', '--execute'])
    node = Mock()
    monkeypatch.setattr(runner.rclpy, 'init', Mock())
    monkeypatch.setattr(runner.rclpy, 'ok', lambda: False)
    monkeypatch.setattr(runner, 'IdentificationRun', Mock(return_value=node))
    runner.main()
    node.execute.assert_called_once_with()
    node.record_only.assert_not_called()


def test_tracking_path_collision_blocks_before_ready(tmp_path):
    output = tmp_path/'run.csv'
    runner.tracking_path(output).write_text('preserve')
    node = SimpleNamespace(_args=SimpleNamespace(output=str(output)), _move_to_ready=Mock())
    with pytest.raises(RuntimeError, match='already exists'):
        runner.IdentificationRun.execute(node)
    node._move_to_ready.assert_not_called()
    assert runner.tracking_path(output).read_text() == 'preserve'


@pytest.mark.parametrize('succeeded', [True, False])
def test_goal_feedback_capture_and_automatic_report(monkeypatch, tmp_path, succeeded):
    import json
    from control_msgs.action import FollowJointTrajectory
    from sensor_msgs.msg import JointState
    output = tmp_path/'run.csv'
    logger = Mock()
    response = SimpleNamespace(status=runner.GoalStatus.STATUS_SUCCEEDED if succeeded else 6,
        result=SimpleNamespace(error_code=0 if succeeded else -4, error_string='test failure'))
    result_future = Mock(done=lambda: True, result=lambda: response)
    handle = Mock(accepted=True, get_result_async=lambda: result_future)
    node = SimpleNamespace(
        _args=SimpleNamespace(output=str(output), ready_only=False, rate=100.,
                              duration=1., joints=['joint3']),
        _ready_timeout=10., _move_to_ready=Mock(), _wait_ready_settled=Mock(),
        wait_for_state=lambda _: True, get_logger=lambda: logger, preview=Mock(),
        _trajectory=lambda: None, _active_handle=None, _active_result=None,
        _samples=0, _last_recorded=-float('inf'))
    node._begin_recording = lambda: runner.IdentificationRun._begin_recording(node)
    def send(goal, feedback_callback):
        joint_state = JointState()
        joint_state.header.stamp.sec = 1
        joint_state.name = list(runner.ARM_JOINTS)
        joint_state.position = [.1]*6
        joint_state.velocity = [.01]*6
        joint_state.effort = [20.]*6
        runner.IdentificationRun._on_joint_state(node, joint_state)
        feedback = FollowJointTrajectory.Feedback()
        feedback.header.stamp.sec = 1
        feedback.joint_names = list(runner.ARM_JOINTS)
        for point, q in ((feedback.desired,.2),(feedback.actual,.1),(feedback.error,.1)):
            point.positions = [q]*6
            point.velocities = [.01]*6
        feedback_callback(SimpleNamespace(feedback=feedback, goal_id=SimpleNamespace(uuid=[1]*16)))
        return Mock(result=lambda: handle)
    node._client = SimpleNamespace(wait_for_server=lambda **kw: True, send_goal_async=send)
    monkeypatch.setattr(runner.rclpy,'spin_until_future_complete',Mock())
    if succeeded:
        runner.IdentificationRun.execute(node)
        report = json.loads(runner.report_path(output).read_text())
        assert report['matched_samples'] == 1
        assert report['joints']['joint3']['tracking_rmse_rad'] == pytest.approx(.1)
    else:
        with pytest.raises(RuntimeError, match='trajectory failed'):
            runner.IdentificationRun.execute(node)
        assert not runner.report_path(output).exists()
    assert output.exists() and runner.tracking_path(output).exists()
    assert node._tracking is None and node._csv_file is None
