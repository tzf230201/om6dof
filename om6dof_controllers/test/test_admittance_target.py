"""Offline direct-position ownership and compliant-target integration tests."""

import json
from types import SimpleNamespace

import numpy as np
import pytest
from builtin_interfaces.msg import Time
from control_msgs.msg import JointTrajectoryControllerState
from controller_manager_msgs.srv import SwitchController
from std_msgs.msg import Float64MultiArray

from test_controller_node import _controller, INITIAL, _target
from om6dof_controllers.control_math import MODE_JOINT


class Future:
    def __init__(self):
        self.callbacks = []
        self.value = None
        self.error = None
        self.done = False

    def add_done_callback(self, callback):
        self.callbacks.append(callback)
        if self.done:
            callback(self)

    def result(self):
        if self.error:
            raise self.error
        assert self.done
        return self.value

    def resolve(self, value=None, error=None):
        assert not self.done
        self.value, self.error, self.done = value, error, True
        self.replay()

    def replay(self):
        for callback in self.callbacks:
            callback(self)


class Publisher:
    def __init__(self, topic):
        self.topic_name = topic
        self.messages = []
        self.subscribers = 1

    def publish(self, message):
        self.messages.append(message)

    def get_subscription_count(self):
        return self.subscribers


class Client:
    def __init__(self, automatic=None):
        self.automatic = automatic
        self.requests, self.futures = [], []
        self.ready = True
        self.error = None

    def service_is_ready(self):
        return self.ready

    def call_async(self, request):
        if self.error:
            raise self.error
        self.requests.append(request)
        future = Future()
        self.futures.append(future)
        if self.automatic is not None:
            future.resolve(self.automatic())
        return future


class Clock:
    def __init__(self):
        self.seconds = 1000.0

    def now(self):
        seconds = self.seconds
        return SimpleNamespace(nanoseconds=int(seconds * 1e9), to_msg=lambda: Time(
            sec=int(seconds), nanosec=int(round((seconds % 1.0) * 1e9))))


def _node(monkeypatch, calibrated=True):
    clock = Clock()
    monkeypatch.setattr('om6dof_controllers.admittance_target.time.monotonic', lambda: clock.seconds)
    node = _controller(ik=False)
    node.nominal_dt = 0.02
    node._clock = clock
    node.get_clock = lambda: clock
    params = {'admittance.enabled': True, 'admittance.calibration_verified': calibrated}
    node.declare_parameter = lambda name, default: params.setdefault(name, default)
    node.get_parameter = lambda name: SimpleNamespace(value=params[name])
    node.create_publisher = lambda _kind, topic, _qos: Publisher(topic)
    node.create_subscription = lambda *_args: None
    node._controllers = {'arm_controller': 'active', 'forward_position_controller': 'inactive',
                         'forward_offset_controller': 'active', 'gravity_current_budget_controller': 'active'}
    def listed():
        return SimpleNamespace(controller=[SimpleNamespace(name=k, state=v)
                                            for k, v in node._controllers.items()])
    node._switch = Client()
    node._list = Client(automatic=listed)
    node.create_client = lambda kind, _topic: node._switch if kind is SwitchController else node._list
    node.count_publishers = lambda _topic: 1
    node._initialize_admittance()
    node.target_command_joints = [q + 0.25 for q in INITIAL]
    node.target_goal = list(node.target_command_joints)
    node.target_mode = MODE_JOINT
    _refresh(node)
    return node


def _refresh(node, quality=0, force=0.0, measured=None, velocity=None):
    measured = tuple(node._admittance_last_command or INITIAL) if measured is None else measured
    velocity = [0.0] * 6 if velocity is None else velocity
    node.joint_positions = dict(zip(node.joint_names, measured))
    node.last_joint_state = node._clock.seconds
    model = [0.0] * 6
    current = [-force * scale for scale in node._admittance_config.raw_per_nm]
    node._on_admittance_sample(Float64MultiArray(data=[node._clock.seconds, float(quality)]
        + list(measured) + list(velocity) + current + model + [force] * 6))
    message = JointTrajectoryControllerState()
    message.header.stamp = node._clock.now().to_msg()
    node._on_admittance_arm_state(message)


def _ack(node, ok=True, error=None):
    request = node._switch.requests[-1]
    if ok and error is None:
        for name in request.deactivate_controllers:
            node._controllers[name] = 'inactive'
        for name in request.activate_controllers:
            node._controllers[name] = 'active'
    node._switch.futures[-1].resolve(SimpleNamespace(ok=ok), error=error)


def _start(node):
    assert node._start_admittance_target_locked(INITIAL)
    _ack(node)


def _tick(node, *, force=0.0, quality=0, measured=None, velocity=None, refresh=True, dt=0.02):
    node._clock.seconds += dt
    if refresh:
        _refresh(node, quality, force, measured, velocity)
    node._tick_admittance_locked(node._clock.seconds, dt)


def _status(node):
    return json.loads(node.target_status_pub.messages[-1].data)


def test_direct_commands_and_reference_wait_for_strict_switch_ack(monkeypatch):
    node = _node(monkeypatch)
    assert node._start_admittance_target_locked(INITIAL)
    request = node._switch.requests[0]
    assert request.strictness == SwitchController.Request.STRICT
    assert request.activate_controllers == ['forward_position_controller']
    assert request.deactivate_controllers == ['arm_controller']
    _tick(node)
    assert not node._admittance_direct_pub.messages
    assert not node._admittance_reference_pub.messages
    assert node.target_active
    _ack(node)
    assert list(node._admittance_direct_pub.messages[-1].data) == INITIAL
    assert list(node._admittance_reference_pub.messages[-1].reference.positions) == INITIAL
    assert node._admittance_owner == 'direct'
    assert not node.arm_client.goals


def test_verified_calibration_is_mandatory(monkeypatch):
    node = _node(monkeypatch, calibrated=False)
    assert not node._start_admittance_target_locked(INITIAL)
    assert 'verified' in _status(node)['message']
    assert not node._switch.requests


@pytest.mark.parametrize('failure', ['active_action', 'offset', 'publishers', 'consumer', 'controller'])
def test_start_requires_exclusive_pipeline(monkeypatch, failure):
    node = _node(monkeypatch)
    if failure == 'active_action':
        node._arm_action_active = True
    elif failure == 'offset':
        node.offset[0] = 0.1
    elif failure == 'publishers':
        node.count_publishers = lambda _topic: 2
    elif failure == 'consumer':
        node._admittance_direct_pub.subscribers = 0
    else:
        node._controllers['gravity_current_budget_controller'] = 'inactive'
    assert not node._start_admittance_target_locked(INITIAL)
    assert not node._switch.requests


@pytest.mark.parametrize('kind', ['receipt', 'source', 'arm_source'])
def test_start_checks_both_source_and_receipt_age(monkeypatch, kind):
    node = _node(monkeypatch)
    if kind == 'receipt':
        node._admittance_sample_time -= 1.0
    elif kind == 'source':
        node._admittance_sample[0] -= 1.0
    else:
        node._arm_state_stamp_seconds -= 1.0
    assert not node._start_admittance_target_locked(INITIAL)
    assert not node._switch.requests


def test_stop_during_pending_start_keeps_callback_and_never_sends_final_target(monkeypatch):
    node = _node(monkeypatch)
    node._start_admittance_target_locked(INITIAL)
    node._request_admittance_stop_locked('stop-pending')
    _ack(node)
    assert node._admittance_stopping
    assert list(node._admittance_direct_pub.messages[-1].data) == INITIAL
    assert node.target_active
    for _ in range(20):
        _tick(node)
    assert node._admittance_switch_pending
    assert node._switch.requests[-1].activate_controllers == ['arm_controller']
    assert node.target_active
    assert _status(node)['state'] != 'stopped'
    _ack(node)
    assert not node.target_active
    assert _status(node)['state'] == 'stopped'
    assert node.offset.tolist() == [0.0] * 6


def test_stop_decelerates_and_waits_for_measured_settling_and_restore_ack(monkeypatch):
    node = _node(monkeypatch)
    _start(node)
    for _ in range(30):
        _tick(node)
    node._request_admittance_stop_locked('stop-moving')
    original = list(node.target_command_joints)
    previous = tuple(node._admittance_last_command)
    previous_velocity = tuple(node._admittance_last_velocity)
    for _ in range(300):
        _tick(node)
        command = node._admittance_last_command
        speeds = [(a - b) / 0.02 for a, b in zip(command, previous)]
        accelerations = [(a - b) / 0.02 for a, b in zip(speeds, previous_velocity)]
        assert max(map(abs, speeds)) <= 0.1 + 1e-8
        assert max(map(abs, accelerations)) <= 0.3 + 1e-7
        previous, previous_velocity = command, speeds
        if node._admittance_switch_pending:
            break
    assert node._admittance_switch_pending
    assert node.target_active
    assert node.target_command_joints == original
    published = len(node._admittance_direct_pub.messages)
    _tick(node)
    assert len(node._admittance_direct_pub.messages) == published
    _ack(node)
    assert not node.target_active
    assert node._admittance_owner == 'position'


@pytest.mark.parametrize('error', [None, RuntimeError('transport lost')])
def test_switch_failure_keeps_unknown_ownership_and_mode_guard(monkeypatch, error):
    node = _node(monkeypatch)
    node._start_admittance_target_locked(INITIAL)
    _ack(node, ok=False, error=error)
    assert node.target_active
    assert node._admittance_target_active
    assert node._admittance_owner == 'uncertain'
    assert not node._admittance_direct_pub.messages
    assert _status(node)['state'] == 'blocked'


def test_late_switch_ack_after_timeout_is_processed_without_early_position_write(monkeypatch):
    node = _node(monkeypatch)
    node._start_admittance_target_locked(INITIAL)
    _tick(node, dt=3.2)
    assert node._admittance_owner == 'uncertain'
    assert node._admittance_switch_pending
    assert not node._admittance_direct_pub.messages
    node._request_admittance_stop_locked('stop-timeout')
    _refresh(node)
    _ack(node)
    assert node._admittance_stopping
    assert node._admittance_owner == 'direct'
    assert list(node._admittance_direct_pub.messages[-1].data) == INITIAL


def test_old_switch_callback_cannot_overwrite_later_restore(monkeypatch):
    node = _node(monkeypatch)
    _start(node)
    first_future = node._switch.futures[0]
    node._request_admittance_stop_locked('stop')
    for _ in range(20):
        _tick(node)
    _ack(node)
    first_future.replay()
    assert node._admittance_owner == 'position'
    assert not node.target_active


def test_foreign_action_race_does_not_drop_guard_or_start_direct_motion(monkeypatch):
    node = _node(monkeypatch)
    node._start_admittance_target_locked(INITIAL)
    node._arm_action_active = True
    node._admittance_preempted_locked()
    _ack(node)
    assert node.target_active
    assert node._admittance_owner == 'uncertain'
    assert not node._admittance_direct_pub.messages


@pytest.mark.parametrize('kind', ['receipt', 'source', 'saturation'])
def test_active_stale_or_saturated_observer_holds_last_direct_command(monkeypatch, kind):
    node = _node(monkeypatch)
    _start(node)
    for _ in range(15):
        _tick(node)
    previous = node._admittance_last_command
    published = len(node._admittance_direct_pub.messages)
    if kind == 'receipt':
        node._admittance_sample_time -= 1.0
    elif kind == 'source':
        node._admittance_sample[0] -= 1.0
    else:
        node._admittance_sample[1] = 5.0
    node._tick_admittance_locked(node._clock.seconds, 0.02)
    assert node._admittance_last_command == previous
    assert len(node._admittance_direct_pub.messages) == published
    assert not node._admittance_status_locked()['target_available']
    assert node._admittance_status_locked()['external_torque_nm'] is None


def test_pushing_and_release_preserve_original_absolute_target(monkeypatch):
    node = _node(monkeypatch)
    original = list(node.target_command_joints)
    _start(node)
    for _ in range(250):
        _tick(node, force=0.35)
    paused = node._admittance_result.nominal
    for _ in range(250):
        _tick(node, force=0.35)
    assert node._admittance_result.nominal == pytest.approx(paused)
    assert node.target_command_joints == original
    for _ in range(1500):
        _tick(node)
    assert node._admittance_last_command == pytest.approx(original, abs=0.001)
    assert node.target_active
    assert not node.arm_client.goals


def test_new_move_is_rejected_while_direct_session_owns_position(monkeypatch):
    node = _node(monkeypatch)
    _start(node)
    original = list(node.target_command_joints)
    _target(node, action='move', mode=MODE_JOINT, values=list(INITIAL), reach_duration=5.0)
    assert node.target_command_joints == original
    assert len(node._switch.requests) == 1


def test_failed_restore_keeps_guard_until_explicit_stop_recovery_ack(monkeypatch):
    node = _node(monkeypatch)
    _start(node)
    node._request_admittance_stop_locked('stop')
    for _ in range(20):
        _tick(node)
    assert node._admittance_switch_pending
    _ack(node, ok=False)
    assert node.target_active
    assert node._admittance_owner == 'uncertain'
    assert _status(node)['state'] != 'stopped'
    requests = len(node._switch.requests)
    for _ in range(30):
        _tick(node)
    assert len(node._switch.requests) == requests  # No automatic switch retries.
    node._request_admittance_stop_locked('stop-retry')
    for _ in range(15):
        _tick(node)
        if node._admittance_switch_pending:
            break
    assert node._admittance_switch_pending
    _ack(node)
    assert node.target_active
    for _ in range(30):
        _tick(node)
        if node._admittance_switch_pending:
            break
    assert node._admittance_switch_pending
    _ack(node)
    assert not node.target_active
    assert _status(node)['state'] == 'stopped'


def test_lost_start_ack_cannot_auto_begin_later_when_sample_recovers(monkeypatch):
    node = _node(monkeypatch)
    node._start_admittance_target_locked(INITIAL)
    node._admittance_sample[0] -= 1.0
    _ack(node)
    assert not node._admittance_direct_pub.messages
    for _ in range(20):
        _tick(node)
    assert not node._admittance_direct_pub.messages
    assert node.target_active
    node._request_admittance_stop_locked('initialize-supported-hold')
    _tick(node)
    assert node._admittance_direct_pub.messages
    assert node._admittance_direct_pub.messages[-1].data == pytest.approx(INITIAL)


def test_second_direct_publisher_mid_session_freezes_output(monkeypatch):
    node = _node(monkeypatch)
    _start(node)
    for _ in range(15):
        _tick(node)
    previous = node._admittance_last_command
    count = len(node._admittance_direct_pub.messages)
    node.count_publishers = lambda _topic: 2
    _tick(node)
    assert node._admittance_last_command == previous
    assert len(node._admittance_direct_pub.messages) == count
    assert not node._admittance_status_locked()['target_available']


def test_stop_does_not_report_old_force_estimate_as_current(monkeypatch):
    node = _node(monkeypatch)
    _start(node)
    for _ in range(15):
        _tick(node, force=0.35)
    assert node._admittance_status_locked()['external_torque_nm'] is not None
    node._request_admittance_stop_locked('stop')
    assert node._admittance_status_locked()['external_torque_nm'] is None


def test_actual_node_tick_does_not_hide_overlong_control_gap(monkeypatch):
    node = _node(monkeypatch)
    _start(node)
    for _ in range(15):
        _tick(node)
    _refresh(node)
    count = len(node._admittance_direct_pub.messages)
    node.last_tick = node._clock.seconds - 0.5
    node._tick()
    assert len(node._admittance_direct_pub.messages) == count
    assert not node._admittance_result.valid
    assert 'interval' in node._admittance_result.reason


def test_never_resolved_switch_cannot_be_reissued_or_falsely_report_stopped(monkeypatch):
    node = _node(monkeypatch)
    node._start_admittance_target_locked(INITIAL)
    _tick(node, dt=3.2)
    node._request_admittance_stop_locked('stop-unknown')
    for _ in range(30):
        _tick(node)
    assert node.target_active
    assert node._admittance_target_active
    assert node._admittance_switch_pending
    assert len(node._switch.requests) == 1
    assert not node._admittance_direct_pub.messages
    assert _status(node)['state'] == 'blocked'
    assert 'stop the stack' in _status(node)['message']


@pytest.mark.parametrize('active', [False, True])
def test_second_offset_publisher_blocks_start_or_freezes_active_session(monkeypatch, active):
    node = _node(monkeypatch)
    if active:
        _start(node)
        for _ in range(15):
            _tick(node)
    node.count_publishers = lambda topic: 2 if topic == node.offset_pub.topic_name else 1
    if not active:
        assert not node._start_admittance_target_locked(INITIAL)
        assert not node._switch.requests
        assert 'zero-offset publisher' in _status(node)['message']
    else:
        count = len(node._admittance_direct_pub.messages)
        previous = node._admittance_last_command
        _tick(node)
        assert len(node._admittance_direct_pub.messages) == count
        assert node._admittance_last_command == previous
        assert not node._admittance_status_locked()['target_available']
        assert 'zero-offset publisher' in node._admittance_status_locked()['target_block_reason']
