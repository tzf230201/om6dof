"""No hardware/service execution: all process operations are mocked."""
import copy
import math
from pathlib import Path
from types import SimpleNamespace as NS
from unittest.mock import Mock

import pytest
import yaml

from om6dof_controllers import impedance_stack as stack


def test_trial_preserves_all_tuning_and_original_yaml():
    original = yaml.safe_load((Path(__file__).parents[1] / 'config/position_impedance.yaml').read_text())
    saved = copy.deepcopy(original)
    ready = [0., -1., .2, 0., 1., 0.]
    trial = stack.trial_config(original, ready)
    assert original == saved
    before = saved['/**'][stack.CONTROLLER]['ros__parameters']
    after = trial['/**'][stack.CONTROLLER]['ros__parameters']
    assert after['enable_on_activation'] is False
    assert after['require_activation_pose'] is True
    assert after['activation_pose'] == ready
    assert after['activation_pose_tolerance'] == .03
    for key, value in before.items():
        if key not in ('enable_on_activation', 'activation_pose', 'activation_pose_tolerance'):
            assert after[key] == value
    before['require_bus_health'] = False
    with pytest.raises(ValueError):
        stack.trial_config(saved, ready)


def test_force_start_sets_one_newton_limits_and_native_current_ceiling():
    original = yaml.safe_load((Path(__file__).parents[1] / 'config/position_impedance.yaml').read_text())
    before = original['/**'][stack.CONTROLLER]['ros__parameters']
    trial = stack.trial_config(original, [0.] * 6, auto_force=True)
    after = trial['/**'][stack.CONTROLLER]['ros__parameters']
    assert after['max_cartesian_wrench'][:3] == [1., 1., 1.]
    assert after['joint_limit_margin'] == 0.0
    assert after['force_native_current_limits'] is True
    assert before.get('force_native_current_limits', False) is False
    assert before['joint_limit_margin'] == .05
    for key in ('gravity', 'effort_scale', 'max_effort', 'extra_torque_base',
                'extra_torque_ratio', 'extra_torque_max', 'damping'):
        assert after[key] == before[key]
    before['max_cartesian_wrench'][0] = .5
    with pytest.raises(ValueError, match='1 N'):
        stack.trial_config(original, [0.] * 6, auto_force=True)


def test_force_activation_uses_force_readiness_not_position_target_margin(monkeypatch):
    n = object.__new__(stack.ImpedanceStack)
    n.auto_force = True
    n.get_clock = lambda: NS(now=lambda: NS(nanoseconds=1_000_000_000))
    n.on_bridge(NS(data='{"stamp_ns":1000000000,"ready":false,"force_ready":true}'))
    assert n.bridge_ready
    n.on_bridge(NS(data='{"stamp_ns":1000000000,"ready":true,"force_ready":false}'))
    assert not n.bridge_ready
    n.auto_force = False
    n.on_bridge(NS(data='{"stamp_ns":1000000000,"ready":true,"force_ready":false}'))
    assert n.bridge_ready


def message(q=0., v=0., ns=1_000_000_000):
    return NS(name=stack.JOINTS, position=[q] * 6, velocity=[v] * 6,
              header=NS(stamp=NS(sec=ns // 1_000_000_000, nanosec=ns % 1_000_000_000)))


def test_ready_requires_fresh_positions_and_velocities_and_detects_gaps():
    p = stack.PoseWatch()
    assert not p.at([0.] * 6, 10.)
    p.update(message(), 10., 1_000_000_000)
    assert p.at([0.] * 6, 10.1)
    assert not p.at([0.] * 6, 10.3)
    epoch = p.epoch
    p.update(message(ns=1_300_000_000), 10.3, 1_300_000_000)
    assert p.epoch > epoch
    p.update(message(v=.1, ns=1_310_000_000), 10.31, 1_310_000_000)
    assert not p.at([0.] * 6, 10.32)
    assert p.at([0.] * 6, 10.32, settled=False)


@pytest.mark.parametrize('change', [lambda m: setattr(m, 'velocity', []),
    lambda m: setattr(m, 'position', [math.nan] * 6),
    lambda m: setattr(m, 'name', ['joint1'] * 6),
    lambda m: setattr(m.header.stamp, 'sec', 0),
    lambda m: setattr(m.header.stamp, 'sec', 2)])
def test_bad_feedback_never_confirms_ready(change):
    p = stack.PoseWatch()
    m = message()
    change(m)
    p.update(m, 10., 1_000_000_000)
    assert not p.at([0.] * 6, 10.)


def test_busy_port_blocks_even_before_position_start(monkeypatch):
    monkeypatch.setattr(stack.os.path, 'exists', lambda _: True)
    monkeypatch.setattr(stack.subprocess, 'run', Mock(return_value=NS(returncode=0, stderr='', stdout='123')))
    node = object.__new__(stack.ImpedanceStack)
    node.spawn = Mock()
    with pytest.raises(RuntimeError, match='Serial port'):
        node.prepare()
    node.spawn.assert_not_called()


def test_prepare_observes_transition_then_settled_ready_and_requires_matching_confirmation(monkeypatch):
    monkeypatch.setattr(stack, 'preflight', lambda: None)
    monkeypatch.setattr(stack.secrets, 'token_hex', lambda _: 'proof')
    monkeypatch.setattr(stack.time, 'monotonic', lambda: 10.)
    n = object.__new__(stack.ImpedanceStack)
    n.auto_force = False
    n.children = []
    events = []
    def spawn(command):
        events.append(command)
        p = NS(poll=lambda: None)
        n.children.append(p)
        return p
    n.spawn = spawn
    n.phase = lambda phase, message, *a: events.append(phase)
    n.pose = NS(q=[0.] * 6, received=10., epoch=1, at=lambda *a: True,
                problem=lambda *a: '')
    config = NS(values=[NS(double_array_value=[0.] * 6), NS(double_array_value=[0.] * 6),
                        NS(string_value='/om6dof/operation_mode'), NS(double_value=4.)])
    n.call = lambda typ, name, *a: config if name.endswith('get_parameters') else NS(
        controller=[NS(name='arm_controller', state='active')])
    n.create_publisher = lambda *a: NS(get_subscription_count=lambda: 1,
                                     publish=lambda msg: events.append(msg.data))
    n.wait = lambda predicate, timeout, reason: events.append(reason)
    n.stable_ready = lambda *a: events.append('settled_ready')
    answers = iter(['supported stale', 'supported proof'])
    n.pump = lambda: next(answers)
    position, ready = n.prepare()
    assert len(position) == 2 and ready == [0.] * 6
    assert events.index('READY') < events.index('TRANSITION was not observed; impedance will not start.')
    assert events.index('settled_ready') < events.index('await_support')
    assert not any('arm_operating_mode:=0' in e for e in events if isinstance(e, list))


def test_mode0_is_never_started_until_owned_position_stack_has_stopped_and_port_is_free(monkeypatch):
    events = []
    n = object.__new__(stack.ImpedanceStack)
    n.auto_force = False
    child = NS(poll=lambda: None)
    n.children = [child]
    n.phase = lambda *a: None
    n.spawn = Mock()
    monkeypatch.setattr(stack, 'stop_owned', lambda p: events.append('stopped'))
    def blocked():
        events.append('preflight')
        raise RuntimeError('occupied')
    monkeypatch.setattr(stack, 'preflight', blocked)
    with pytest.raises(RuntimeError, match='occupied'):
        n.activate([child], [0.] * 6, '/unused')
    assert events == ['stopped', 'preflight']
    n.spawn.assert_not_called()


def test_stop_targets_only_owned_process_group(monkeypatch):
    kill = Mock()
    monkeypatch.setattr(stack.os, 'killpg', kill)
    p = NS(pid=1234, poll=lambda: None, wait=Mock())
    stack.stop_owned(p)
    kill.assert_called_once_with(1234, stack.signal.SIGINT)


def test_mode0_activation_is_single_guarded_request_after_rechecking_ready(monkeypatch):
    events = []
    monkeypatch.setattr(stack, 'preflight', lambda: events.append('port_free'))
    monkeypatch.setattr(stack, 'stop_owned', lambda _: events.append('position_stopped'))
    monkeypatch.setattr(stack.time, 'monotonic', lambda: 10.)
    n = object.__new__(stack.ImpedanceStack)
    n.auto_force = False
    position = NS(poll=lambda: None)
    n.children = [position]
    n.bridge_ready, n.bridge_at = True, 10.
    n.phase = lambda phase, *_: events.append(phase)
    n.spawn = lambda command: events.append(command)
    n.confirm_supported_pose = lambda: (events.append('new_supported_pose_consent') or ([.2] * 6, 1, 70.))
    n.arm_confirmed_pose = lambda *a: events.append('confirmed_pose_armed')
    n.check_confirmed_pose = Mock()
    n.verify_supported_pose_gate = lambda: events.append('new_plugin_verified')
    def call(typ, name, request, *args, **kwargs):
        if kwargs.get('before_send'):
            kwargs['before_send']()
        if name.endswith('list_controllers'):
            return NS(controller=[NS(name=stack.CONTROLLER, state='inactive')])
        assert name.endswith('switch_controller')
        assert request.activate_controllers == [stack.CONTROLLER]
        assert request.deactivate_controllers == []
        assert request.strictness == request.STRICT
        events.append('activation_request')
        return NS(ok=True)
    n.call = call
    n.wait = lambda predicate, *_: predicate() or pytest.fail('gate not satisfied')
    n.pump = Mock(side_effect=KeyboardInterrupt)
    with pytest.raises(KeyboardInterrupt):
        n.activate([position], [0.] * 6, '/temporary.yaml')
    assert events.index('position_stopped') < events.index('port_free') < events.index('starting')
    assert events.index('new_supported_pose_consent') < events.index('confirmed_pose_armed') < events.index('activation_request') < events.index('active')
    assert events.count('activation_request') == 1
    commands = [e for e in events if isinstance(e, list)]
    assert 'arm_operating_mode:=0' in commands[0] and 'start_arm_controller:=false' in commands[0]
    assert 'position_impedance.launch.py' in commands[1]


@pytest.mark.parametrize('q,v,age,expected', [(.04, 0., 0., 'joint1 pose error'),
    (0., .06, 0., 'joint1 velocity'), (0., 0., .21, 'receipt age')])
def test_feedback_failure_reports_specific_gate(q, v, age, expected):
    pose = stack.PoseWatch()
    pose.update(message(q=q, v=v), 10., 1_000_000_000)
    assert expected in pose.problem([0.] * 6, 10. + age)
    assert not pose.at([0.] * 6, 10. + age)


def support_node(monkeypatch):
    monkeypatch.setattr(stack.time, 'monotonic', lambda: 10.)
    tokens = iter(['old', 'new'])
    monkeypatch.setattr(stack.secrets, 'token_hex', lambda _: next(tokens))
    n = object.__new__(stack.ImpedanceStack)
    n.auto_force = False
    n.pose = stack.PoseWatch()
    n.pose.update(message(), 10., 1_000_000_000)
    n.phase = Mock()
    n.arm_active = Mock(return_value=True)
    n.stable_ready = Mock()
    n.spawn = Mock()
    return n


def test_ready_recheck_requires_operator_request_and_new_confirmation(monkeypatch):
    n = support_node(monkeypatch)
    position = [NS(poll=lambda: None)]
    steps = iter(['bad', 'supported old', '', 'recheck_ready', 'supported old', 'supported new'])
    def pump():
        line = next(steps)
        if line == 'bad':
            n.pose.update(message(v=.1, ns=1_010_000_000), 10., 1_010_000_000)
            return 'supported old'  # Simultaneous stale confirmation must not win.
        if line == 'recheck_ready':
            n.pose.update(message(ns=1_020_000_000), 10., 1_020_000_000)
        return line
    n.pump = pump
    n.await_support(position, [0.] * 6)
    phases = [(c.args[0], c.args[2] if len(c.args) > 2 else '') for c in n.phase.call_args_list]
    assert phases == [('await_support', 'old'), ('ready_blocked', ''),
                      ('checking_ready', ''), ('await_support', 'new')]
    assert 'joint1 velocity' in n.phase.call_args_list[1].args[1]
    assert n.stable_ready.call_count == 2
    assert n.arm_active.call_count == 2
    n.spawn.assert_not_called()


def test_feedback_epoch_change_invalidates_confirmation_even_after_feedback_recovers(monkeypatch):
    n = support_node(monkeypatch)
    def pump():
        if n.phase.call_args.args[0] == 'ready_blocked':
            raise KeyboardInterrupt
        n.pose.epoch += 1
        n.pose.disruption = 'test feedback interruption'
        return 'supported old'
    n.pump = pump
    with pytest.raises(KeyboardInterrupt):
        n.await_support([NS(poll=lambda: None)], [0.] * 6)
    assert n.phase.call_args.args[0] == 'ready_blocked'
    assert 'test feedback interruption' in n.phase.call_args.args[1]
    n.arm_active.assert_not_called()
    n.spawn.assert_not_called()


def test_dead_position_launch_cannot_offer_recheck(monkeypatch):
    n = support_node(monkeypatch)
    n.pump = lambda: 'supported old'
    with pytest.raises(RuntimeError, match='launch exited'):
        n.await_support([NS(poll=lambda: 1)], [0.] * 6)
    assert [c.args[0] for c in n.phase.call_args_list] == ['await_support']


def test_confirm_rechecks_controller_ownership(monkeypatch):
    n = support_node(monkeypatch)
    n.pump = lambda: 'supported old'
    n.arm_active.return_value = False
    with pytest.raises(RuntimeError, match='no longer active'):
        n.await_support([NS(poll=lambda: None)], [0.] * 6)
    n.stable_ready.assert_not_called()
    n.spawn.assert_not_called()


def test_expired_confirmation_cannot_switch_mode(monkeypatch):
    n = support_node(monkeypatch)
    def pump():
        if n.phase.call_args.args[0] == 'ready_blocked':
            raise KeyboardInterrupt
        monkeypatch.setattr(stack.time, 'monotonic', lambda: 71.)
        n.pose.received = 71.
        return 'supported old'
    n.pump = pump
    with pytest.raises(KeyboardInterrupt):
        n.await_support([NS(poll=lambda: None)], [0.] * 6)
    assert 'expired' in n.phase.call_args.args[1]
    n.arm_active.assert_not_called()


def test_post_switch_pose_failure_never_activates_and_process_fault_is_not_recoverable(monkeypatch):
    n = object.__new__(stack.ImpedanceStack)
    n.auto_force = False
    n.children = []
    n.phase = Mock()
    n.spawn = Mock()
    n.wait = lambda predicate, *_: predicate()
    n.call = Mock(return_value=NS(controller=[NS(name=stack.CONTROLLER, state='inactive')]))
    n.confirm_supported_pose = Mock(side_effect=stack.PositionRecoveryRequired('NOT activated: supported pose did not settle'))
    n.verify_supported_pose_gate = Mock()
    monkeypatch.setattr(stack, 'preflight', lambda: None)
    with pytest.raises(stack.PositionRecoveryRequired, match='NOT activated'):
        n.activate([], [0.] * 6, '/unused')
    assert n.call.call_count == 1
    assert n.call.call_args.args[1].endswith('list_controllers')
    assert 'activating' not in [c.args[0] for c in n.phase.call_args_list]
    n.confirm_supported_pose.side_effect = RuntimeError('Owned launch exited unexpectedly')
    with pytest.raises(RuntimeError, match='Owned launch exited') as exc:
        n.activate([], [0.] * 6, '/unused')
    assert not isinstance(exc.value, stack.PositionRecoveryRequired)


def recovery_node(monkeypatch):
    n = support_node(monkeypatch)
    n.children = [NS(poll=lambda: None), NS(poll=lambda: None)]
    n.call = Mock(return_value=NS(controller=[NS(name=stack.CONTROLLER, state='inactive',
                                                 claimed_interfaces=[])]))
    n.pump = lambda: 'recover_position old'
    return n


def test_recovery_stops_only_owned_children_before_port_check_and_clears_feedback(monkeypatch):
    n = recovery_node(monkeypatch)
    children = list(n.children)
    events = []
    answers = iter(['supported old', 'recover_position stale', 'recover_position old'])
    n.pump = lambda: next(answers)
    monkeypatch.setattr(stack, 'stop_owned', lambda p: events.append(p))
    monkeypatch.setattr(stack, 'preflight', lambda: events.append('port_check'))
    n.recover_position('READY lost')
    assert events == [children[1], children[0], 'port_check']
    assert not n.children and n.pose.q is None and not n.bridge_ready
    assert [c.args[0] for c in n.phase.call_args_list] == ['await_recovery', 'recovering']
    n.spawn.assert_not_called()  # Main must run canonical prepare with new consent, not activate here.


@pytest.mark.parametrize('problem', ['active', 'other_owner', 'missing', 'stale', 'moving',
                                     'dead_child', 'expired', 'expires_during_query'])
def test_unsafe_recovery_never_stops_or_launches_hardware(monkeypatch, problem):
    n = recovery_node(monkeypatch)
    result = n.call.return_value
    if problem == 'active':
        result.controller[0].state = 'active'
    if problem == 'other_owner':
        result.controller.append(NS(name='other', state='active', claimed_interfaces=['joint2/effort']))
    if problem == 'missing':
        result.controller = []
    if problem == 'stale':
        n.pose.received = 0.
    if problem == 'moving':
        n.pose.v[2] = .1
    if problem == 'dead_child':
        n.children[0].poll = lambda: 1
    if problem == 'expired':
        def pump():
            monkeypatch.setattr(stack.time, 'monotonic', lambda: 71.)
            return 'recover_position old'
        n.pump = pump
    if problem == 'expires_during_query':
        def call(*_):
            monkeypatch.setattr(stack.time, 'monotonic', lambda: 71.)
            n.pose.received = 71.
            return result
        n.call = call
    stop = Mock()
    monkeypatch.setattr(stack, 'stop_owned', stop)
    with pytest.raises(RuntimeError):
        n.recover_position('READY lost')
    stop.assert_not_called()
    n.spawn.assert_not_called()


def test_recovery_port_conflict_refuses_new_stack(monkeypatch):
    n = recovery_node(monkeypatch)
    monkeypatch.setattr(stack, 'stop_owned', Mock())
    monkeypatch.setattr(stack, 'preflight', Mock(side_effect=RuntimeError('port busy')))
    with pytest.raises(RuntimeError, match='port busy'):
        n.recover_position('READY lost')
    n.spawn.assert_not_called()


def test_ready_timeout_diagnoses_joint_and_does_not_convert_process_failure(monkeypatch):
    n = support_node(monkeypatch)
    n.stable_ready = stack.ImpedanceStack.stable_ready.__get__(n)
    n.pose.q[1] = .1
    def timeout(predicate, _, reason):
        assert not predicate()
        raise RuntimeError(reason)
    n.wait = timeout
    with pytest.raises(stack.ReadyNotSettled, match='joint2 pose error'):
        n.stable_ready([0.] * 6)
    n.wait = Mock(side_effect=RuntimeError('Owned launch exited unexpectedly'))
    with pytest.raises(RuntimeError, match='Owned launch exited') as exc:
        n.stable_ready([0.] * 6)
    assert not isinstance(exc.value, stack.ReadyNotSettled)


@pytest.mark.parametrize('recoverable', [True, False, 'active_limit'])
def test_main_repeats_canonical_preparation_only_after_explicit_recovery(monkeypatch, recoverable):
    events = []
    node = NS(children=[], auto_force=False, phase=Mock(), destroy_node=Mock(), pump=Mock(side_effect=KeyboardInterrupt))
    def prepare():
        events.append('transition_ready_new_support_consent')
        return [], [0.] * 6
    def activate(*_):
        events.append('activate')
        if events.count('activate') == 1:
            error = (stack.ActiveLimitRecoveryRequired if recoverable == 'active_limit' else
                     stack.PositionRecoveryRequired if recoverable else RuntimeError)
            raise error('test failure')
        raise KeyboardInterrupt
    node.prepare, node.activate = prepare, activate
    def recover(_, **kwargs):
        assert kwargs == ({'active_limit_fault': True} if recoverable == 'active_limit' else {})
        events.append('explicit_recovery')
    node.recover_position = recover
    monkeypatch.setattr(stack, 'ImpedanceStack', lambda **kwargs: node)
    monkeypatch.setattr(stack.rclpy, 'init', Mock())
    monkeypatch.setattr(stack.rclpy, 'ok', lambda: False)
    monkeypatch.setattr(stack, 'get_package_share_directory', lambda _: str(Path(__file__).parents[1]))
    stack.main()
    expected = ['transition_ready_new_support_consent', 'activate']
    if recoverable:
        expected += ['explicit_recovery', 'transition_ready_new_support_consent', 'activate']
    assert events == expected
    node.destroy_node.assert_called_once()


def active_fault_node(monkeypatch):
    n = recovery_node(monkeypatch)
    n.controller_status, n.controller_fault, n.controller_at = 3, 32, 10.
    n.limit_detail, n.limit_at = 'joint2 speed +0.4500 rad/s exceeded limit', 10.
    n.current_limits_ok = True
    c = n.call.return_value.controller[0]
    c.state, c.type = 'active', 'om6dof_controllers/PositionImpedanceController'
    c.claimed_interfaces = [j + '/effort' for j in stack.JOINTS]
    n.clock = [10.]
    monkeypatch.setattr(stack.time, 'monotonic', lambda: n.clock[0])
    def pump():
        if n.phase.call_args.args[0] == 'recovery_pose_blocked':
            raise KeyboardInterrupt  # End an intentionally blocked test, not a retry.
        n.clock[0] += .1
        n.controller_at = n.limit_at = n.clock[0]
        n.pose.update(message(ns=int(n.clock[0] * 1e9)), n.clock[0], int(n.clock[0] * 1e9))
        return 'recover_position old'
    n.pump = pump
    return n


def test_startup_limit_fault_offers_explicit_recovery_not_generic_failure(monkeypatch):
    n = active_fault_node(monkeypatch)
    n.bridge_ready, n.bridge_at = False, 10.
    with pytest.raises(stack.ActiveLimitRecoveryRequired, match='joint2 speed'):
        n.activation_ready()
    n.controller_fault = 34
    assert not n.activation_ready()  # Other faults never authorize this recovery.


def test_supported_active_limit_recovery_rechecks_then_reloads_no_in_place_clear(monkeypatch):
    n = active_fault_node(monkeypatch)
    stop = Mock()
    monkeypatch.setattr(stack, 'stop_owned', stop)
    monkeypatch.setattr(stack, 'preflight', Mock())
    children = list(n.children)
    n.recover_position('limit fault', active_limit_fault=True)
    n.stable_ready.assert_not_called()  # Recovery capture is not a READY check.
    assert n.clock[0] >= 11.
    assert [c.args[0] for c in stop.call_args_list] == children[::-1]
    assert n.controller_fault == -1 and not n.children
    n.spawn.assert_not_called()
    assert all(c.args[0] is stack.ListControllers for c in n.call.call_args_list)


def test_active_fault_never_recovers_on_old_support_tokens_or_without_consent(monkeypatch):
    n = active_fault_node(monkeypatch)
    n.pump = Mock(side_effect=['supported old', 'supported_pose old', 'recover_position stale',
                               '', KeyboardInterrupt])
    stop = Mock()
    monkeypatch.setattr(stack, 'stop_owned', stop)
    with pytest.raises(KeyboardInterrupt):
        n.recover_position('limit fault', active_limit_fault=True)
    stop.assert_not_called()
    n.call.assert_not_called()
    n.spawn.assert_not_called()


@pytest.mark.parametrize('problem', ['stale', 'hard_fault', 'inactive', 'other_owner',
    'pose_drift', 'feedback_gap', 'fault_after_query', 'stale_after_query',
    'expires_after_query', 'unsettled', 'outside_current_limits', 'stale_limits',
    'limits_after_query'])
def test_active_limit_recovery_cannot_bypass_safety_checks(monkeypatch, problem):
    n = active_fault_node(monkeypatch)
    stop = Mock()
    monkeypatch.setattr(stack, 'stop_owned', stop)
    if problem == 'hard_fault': n.controller_fault = 34
    if problem == 'outside_current_limits': n.current_limits_ok = False
    if problem == 'inactive': n.call.return_value.controller[0].state = 'inactive'
    if problem == 'other_owner':
        n.call.return_value.controller.append(NS(name='other', state='active',
                                                 claimed_interfaces=['joint1/effort']))
    pump = n.pump
    def feedback():
        line = pump()
        if problem == 'stale': n.controller_at = 0.
        if problem == 'stale_limits': n.limit_at = 0.
        if problem == 'unsettled': n.pose.v[0] = .06
        return line
    n.pump = feedback
    result = n.call.return_value
    def query(*_):
        if problem == 'pose_drift': n.pose.q[0] = .04
        if problem == 'feedback_gap': n.pose.epoch += 1
        if problem == 'fault_after_query': n.controller_fault = 288
        if problem == 'limits_after_query': n.current_limits_ok = False
        if problem == 'stale_after_query': n.controller_at = 0.
        if problem == 'expires_after_query':
            n.clock[0] += 61.
            n.controller_at = n.limit_at = n.pose.received = n.clock[0]
        return result
    n.call.side_effect = query
    terminal = problem in ('hard_fault', 'inactive', 'other_owner', 'fault_after_query')
    with pytest.raises(RuntimeError if terminal else KeyboardInterrupt):
        n.recover_position('limit fault', active_limit_fault=True)
    if not terminal:
        assert n.phase.call_args.args[0] == 'recovery_pose_blocked'
        assert 'limit fault' in n.phase.call_args.args[1]
        assert 'Recovery check:' in n.phase.call_args.args[1]
        assert 'READY did not settle' not in n.phase.call_args.args[1]
    stop.assert_not_called()
    n.spawn.assert_not_called()


def test_recovery_pose_shift_requires_explicit_recapture_and_new_motion_consent(monkeypatch):
    n = active_fault_node(monkeypatch)
    stop = Mock()
    monkeypatch.setattr(stack, 'stop_owned', stop)
    monkeypatch.setattr(stack, 'preflight', Mock())
    phase_count = {'await_recovery': 0}
    blocked_steps, confirmed_steps = [0], [0]
    def phase(name, *args):
        if name == 'await_recovery': phase_count[name] += 1
    n.phase.side_effect = phase
    def pump():
        n.clock[0] += .1
        n.controller_at = n.limit_at = n.clock[0]
        p = n.phase.call_args.args[0]
        changed = phase_count['await_recovery'] >= 1
        n.pose.update(message(q=.1 if changed else 0., ns=int(n.clock[0]*1e9)),
                      n.clock[0], int(n.clock[0]*1e9))
        if p == 'recovery_pose_blocked':
            blocked_steps[0] += 1
            stop.assert_not_called()
            return 'recover_position old' if blocked_steps[0] == 1 else 'recapture_recovery'
        if p == 'await_recovery' and phase_count['await_recovery'] == 2:
            confirmed_steps[0] += 1
            stop.assert_not_called()
            return 'recover_position old' if confirmed_steps[0] == 1 else 'recover_position new'
        return 'recover_position old'
    n.pump = pump
    n.recover_position('limit fault', active_limit_fault=True)
    assert phase_count['await_recovery'] == 2
    assert blocked_steps[0] == 2 and confirmed_steps[0] == 2
    assert stop.call_count == 2
    messages = [c.args[1] for c in n.phase.call_args_list if c.args[0] == 'await_recovery']
    assert 'joint1=+0.00' in messages[0] and 'joint1=+5.73' in messages[1]


def test_fault_telemetry_validation_and_freshness(monkeypatch):
    n = active_fault_node(monkeypatch)
    n.on_state(NS(data=[3., 32.] + [0.] * 42))
    assert n.limit_fault_fresh()
    n.on_state(NS(data=[3., 32.]))
    assert not n.limit_fault_fresh()
    n.on_state(NS(data=[3., 32.] + [float('nan')] * 42))
    assert not n.limit_fault_fresh()
    n.on_limit(NS(data=[1., 2., 1., .4, -.3, .3, 1.]))
    assert n.recovery_limits_valid()
    n.on_limit(NS(data=[1., 2., 1., .4, -.3, .3, 0.]))
    assert not n.recovery_limits_valid()
    n.on_limit(NS(data=[1., 2., 1., .4, -.3, .3]))
    assert not n.recovery_limits_valid()  # Older/malformed protocol fails closed.


def captured_pose_node(monkeypatch):
    n = support_node(monkeypatch)
    n.children = [NS(poll=lambda: None)]
    clock = [10.]
    monkeypatch.setattr(stack.time, 'monotonic', lambda: clock[0])
    def refresh(q=.4, v=0.):
        n.pose.update(message(q=q, v=v, ns=int(clock[0] * 1e9)), clock[0], int(clock[0] * 1e9))
    def wait(predicate, *_):
        for _ in range(20):
            clock[0] += .1
            refresh()
            if predicate():
                return
        pytest.fail('pose never settled')
    n.wait = wait
    n.pump = lambda: 'supported_pose old'
    return n, clock, refresh


def test_supported_pose_is_actual_post_switch_pose_not_old_ready(monkeypatch):
    n, clock, _ = captured_pose_node(monkeypatch)
    pose, epoch, deadline = n.confirm_supported_pose()
    assert pose == [.4] * 6  # deliberately far from initial READY
    assert epoch == n.pose.epoch and deadline > clock[0]
    assert n.phase.call_args.args[0] == 'await_pose_support'
    assert 'joint1=+22.92' in n.phase.call_args.args[1]
    n.spawn.assert_not_called()


def test_pose_movement_revokes_token_and_requires_explicit_recapture_and_new_consent(monkeypatch):
    n, clock, refresh = captured_pose_node(monkeypatch)
    steps = iter(['move', 'supported_pose old', 'recapture_pose', 'supported_pose old', 'supported_pose new'])
    def pump():
        line = next(steps)
        if line == 'move':
            clock[0] += .01
            refresh(q=.8)
            return 'supported_pose old'
        return line
    n.pump = pump
    assert n.confirm_supported_pose()[0] == [.4] * 6
    phases = [c.args[0] for c in n.phase.call_args_list]
    assert phases == ['capturing_pose', 'await_pose_support', 'pose_blocked',
                      'capturing_pose', 'await_pose_support']
    assert n.phase.call_args.args[2] == 'new'
    n.spawn.assert_not_called()


@pytest.mark.parametrize('failure', ['stale', 'gap', 'moving', 'expired'])
def test_supported_pose_stale_or_expired_consent_never_advances(monkeypatch, failure):
    n, clock, refresh = captured_pose_node(monkeypatch)
    def pump():
        if n.phase.call_args.args[0] == 'pose_blocked':
            raise KeyboardInterrupt
        clock[0] += .3 if failure == 'stale' else .01
        if failure == 'gap':
            n.pose.epoch += 1
        if failure == 'moving':
            refresh(v=.06)
        if failure == 'expired':
            clock[0] += 61.
            refresh()
        return 'supported_pose old'
    n.pump = pump
    with pytest.raises(KeyboardInterrupt):
        n.confirm_supported_pose()
    assert n.phase.call_args.args[0] == 'pose_blocked'


@pytest.mark.parametrize('reject', ['', 'owner', 'parameter', 'gap_during_service', 'pose_during_service'])
def test_confirmed_pose_parameter_gate_is_atomic_and_rechecks_live_state(monkeypatch, reject):
    n = support_node(monkeypatch)
    requests = []
    def call(typ, name, req, *args, **kwargs):
        if kwargs.get('before_send'):
            kwargs['before_send']()
        requests.append((name, req))
        if name.endswith('list_controllers'):
            return NS(controller=[NS(name=stack.CONTROLLER,
                state='active' if reject == 'owner' else 'inactive', claimed_interfaces=[])])
        assert name.endswith('set_parameters_atomically')
        if reject == 'gap_during_service':
            n.pose.epoch += 1
        if reject == 'pose_during_service':
            n.pose.q[0] = .1
        return NS(result=NS(successful=reject != 'parameter', reason='test rejection'))
    n.call = call
    if reject:
        with pytest.raises(RuntimeError):
            n.arm_confirmed_pose([0.] * 6, n.pose.epoch, 70.)
    else:
        n.arm_confirmed_pose([0.] * 6, n.pose.epoch, 70.)
        params = requests[-1][1].parameters
        assert params[0].name == 'activation_pose' and list(params[0].value.double_array_value) == [0.] * 6
        assert params[1].name == 'enable_on_activation' and params[1].value.bool_value
    assert all(not name.endswith('switch_controller') for name, _ in requests)


@pytest.mark.parametrize('version,required,enabled,accepted', [
    (1, True, False, True), (None, True, False, False),
    (1, False, False, False), (1, True, True, False), (2, True, False, False)])
def test_old_plugin_or_open_gate_cannot_enter_supported_pose_protocol(version, required, enabled, accepted):
    n = object.__new__(stack.ImpedanceStack)
    n.auto_force = False
    n.call = Mock(return_value=NS(values=[
        NS(type=0 if version is None else stack.ParameterType.PARAMETER_INTEGER, integer_value=version),
        NS(type=stack.ParameterType.PARAMETER_BOOL, bool_value=required),
        NS(type=stack.ParameterType.PARAMETER_BOOL, bool_value=enabled)]))
    if accepted:
        n.verify_supported_pose_gate()
    else:
        with pytest.raises(RuntimeError, match='version/gate mismatch'):
            n.verify_supported_pose_gate()


def test_service_discovery_delay_cannot_dispatch_expired_activation(monkeypatch):
    n = support_node(monkeypatch)
    client = NS(service_is_ready=lambda: True, call_async=Mock())
    n.create_client = Mock(return_value=client)
    n.destroy_client = Mock()
    def wait(*_):
        monkeypatch.setattr(stack.time, 'monotonic', lambda: 80.)
        n.pose.received = 80.
    n.wait = wait
    with pytest.raises(RuntimeError, match='expired'):
        n.call(object, '/fake_switch', object(),
               before_send=lambda: n.check_confirmed_pose([0.] * 6, n.pose.epoch, 70.))
    client.call_async.assert_not_called()
    n.destroy_client.assert_called_once_with(client)
