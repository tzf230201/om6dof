"""Regression tests for centre provenance and the independent pickup sequence."""
import copy
import threading
import time
from types import SimpleNamespace

import pytest
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from om6dof_pick_and_place.boxer_pick_node import (
    BoxerPickNode, choose_box, parse_boxes, collision_boxes)


def detection():
    return {'schema': 'om6dof.boxer3d_detections.v1', 'frame_id': 'world',
            'source_stamp_ns': 10_000_000_000, 'result_stamp_ns': 18_000_000_000,
            'calibration_verified': True, 'calibration_sha256': 'measured', 'camera_serial': 'test',
            'boxes': [{'id': 0, 'label': 'bottle', 'score': .9, 'inside_point_count': 2000,
                       'center': [.3, -.03, .17], 'size': [.06, .06, .15],
                       'quaternion_xyzw': [0., 0., 0., 1.]}]}


def parse(payload, now=19_000_000_000):
    return parse_boxes(payload, now_ns=now, calibration_sha256='measured', camera_serial='test')


def test_target_uses_box_center_and_retains_obb_dimensions():
    boxes = parse(detection())
    target = choose_box(boxes, 'bottle')
    assert target['center'] == (.3, -.03, .17)
    obstacles = collision_boxes(boxes)
    assert obstacles[0].pose.position.z == .17
    assert obstacles[0].size == (.06, .06, .15)


def test_source_capture_age_cannot_be_hidden_by_new_result_stamp():
    payload = detection()
    payload['result_stamp_ns'] = 50_000_000_000
    with pytest.raises(ValueError, match='kedaluwarsa'):
        parse(payload, 51_000_000_000)


@pytest.mark.parametrize('key,value', [('frame_id','boxer_local'), ('calibration_verified',False),
                                     ('calibration_sha256','nominal'), ('camera_serial','wrong')])
def test_uncalibrated_or_wrong_frame_detections_cannot_become_robot_targets(key, value):
    payload = detection(); payload[key] = value
    with pytest.raises(ValueError):
        parse(payload)


@pytest.mark.parametrize('key,value', [('center',[float('nan'),0,0]), ('size',[0,.1,.1]),
                                     ('quaternion_xyzw',[0,0,0,0]), ('score',float('inf'))])
def test_invalid_geometry_rejected(key, value):
    payload = detection(); payload['boxes'][0][key] = value
    with pytest.raises(ValueError):
        parse(payload)


def test_track_id_changes_are_matched_by_unique_physical_center():
    boxes = parse(detection())
    boxes[0]['id'] = 'new id'
    assert choose_box(boxes, 'bottle', (.3,-.03,.17))['id'] == 'new id'
    with pytest.raises(ValueError, match='ambigu'):
        choose_box(boxes + boxes, 'bottle', (.3,-.03,.17))
    with pytest.raises(ValueError):
        choose_box(boxes, 'bottle', (.35,-.03,.17))
    with pytest.raises(ValueError):
        choose_box(boxes, 'keyboard')


def test_unsupported_depth_cut_is_not_a_pick_target():
    payload = detection(); payload['boxes'][0]['inside_point_count'] = 0
    with pytest.raises(ValueError):
        choose_box(parse(payload), 'bottle')


def fake_worker(arm_failure=None, validation_failure=None):
    node = object.__new__(BoxerPickNode)
    node._lock = threading.RLock(); node._cancel = threading.Event()
    node._busy = True; node._phase = 'executing'; node._frozen = object()
    node._holding_object = False
    node._stage = lambda state, message: setattr(node, '_state', state)
    node._publish_status = lambda: None
    node.get_logger = lambda: SimpleNamespace(error=lambda _: None)
    params = {'gripper_open': .019, 'gripper_close': -.01}
    node.param = lambda key: params[key]
    node.gripper_joint_name = 'gripper_left_joint'
    node._joints = lambda: JointState(name=['gripper_left_joint'], position=[.01])
    node._segment_start_rejection = lambda _: None
    node._execution_interlocks_ready = lambda _: not node._cancel.is_set()
    calls = []
    node._refresh_scene = lambda _: calls.append('fresh scene')
    def validate(*_, **kwargs):
        if validation_failure == node._state:
            raise ValueError('collision')
        calls.append('validate ' + node._state)
    node.planner = SimpleNamespace(validate_gripper=validate, validate_segment=validate)
    def send_gripper(value, label):
        calls.append(label)
        node._last_gripper_result_state = 'contact_detected'
        return True
    def send_arm(trajectory, label):
        calls.append(label)
        node._message = 'arm failed'
        return node._state != arm_failure
    node._send_gripper, node._send_arm = send_gripper, send_arm
    trajectory = JointTrajectory(points=[JointTrajectoryPoint(positions=[0.]*6),
                                        JointTrajectoryPoint(positions=[.1]*6)])
    plan = SimpleNamespace(motion=SimpleNamespace(pregrasp=trajectory, insertion=trajectory))
    return node, plan, calls


def test_simple_pickup_continues_beyond_pregrasp_and_only_then_closes():
    node, plan, calls = fake_worker()
    node._pickup_worker(plan)
    assert calls == ['fresh scene', 'validate opening', 'open gripper',
                     'fresh scene', 'validate pregrasp', 'menuju pra-jepit',
                     'fresh scene', 'validate insertion', 'mendekati pusat kotak',
                     'fresh scene', 'validate closing', 'close gripper']
    assert node._holding_object
    assert node._frozen is None


@pytest.mark.parametrize('stage', ['pregrasp', 'insertion'])
def test_arm_failure_never_sends_gripper_close(stage):
    node, plan, calls = fake_worker(arm_failure=stage)
    node._pickup_worker(plan)
    assert 'close gripper' not in calls
    assert node._state == 'stopped'
    assert not node._holding_object


@pytest.mark.parametrize('stage', ['opening', 'pregrasp', 'insertion', 'closing'])
def test_new_collision_cannot_be_overridden_by_successful_preview(stage):
    node, plan, calls = fake_worker(validation_failure=stage)
    node._pickup_worker(plan)
    assert 'close gripper' not in calls
    assert node._state == 'stopped'


def test_execution_disabled_does_not_start_worker():
    node = object.__new__(BoxerPickNode)
    node._lock = threading.RLock()
    node._busy = node._faulted = False
    node._frozen = object(); node.execution_enabled = False
    response = node._execute(None, SimpleNamespace())
    assert not response.success


def test_moveit_joint_order_does_not_change_measured_endpoint_check():
    node = object.__new__(BoxerPickNode)
    node._lock = threading.RLock()
    node.joint_names = [f'joint{i}' for i in range(1, 7)]
    node._joint_state = JointState(name=node.joint_names, position=[.1,.2,.3,.4,.5,.6])
    node._joint_state_time = time.monotonic()
    node.get_parameter = lambda name: SimpleNamespace(value={
        'max_joint_state_age_sec': .5, 'max_start_joint_error_rad': .02}[name])
    trajectory = JointTrajectory(joint_names=list(reversed(node.joint_names)),
        points=[JointTrajectoryPoint(positions=[.6,.5,.4,.3,.2,.1])])
    assert node._segment_start_rejection(trajectory) is None
    trajectory.points[0].positions[-1] = .2
    assert node._segment_start_rejection(trajectory).startswith('arm_segment_start_mismatch')
