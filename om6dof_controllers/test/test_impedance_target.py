"""Offline tests only: no hardware, manager, lifecycle or motor commands."""
import json
from types import SimpleNamespace as NS

import numpy as np
import pytest

from om6dof_controllers import impedance_target as module
from om6dof_controllers.impedance_target import (
    CONTROLLER, JOINTS, ImpedanceTargetNode, TargetSession, exclusive_controller)


def controller(name=CONTROLLER, state='active', claims=None, kind=None):
    return NS(name=name, state=state, claimed_interfaces=claims if claims is not None
              else [f'{j}/effort' for j in JOINTS],
              type=kind or 'om6dof_controllers/PositionImpedanceController')


def test_exclusive_controller_requires_exact_plugin_all_claims_and_no_conflicts():
    assert exclusive_controller([controller()])
    assert not exclusive_controller([])
    assert not exclusive_controller([controller(state='inactive')])
    assert not exclusive_controller([controller(kind='wrong')])
    assert not exclusive_controller([controller(claims=['joint1/effort'])])
    for kind in ('position', 'velocity', 'effort'):
        assert not exclusive_controller([controller(), controller('other', claims=[f'joint3/{kind}'])])
    assert exclusive_controller([controller(), controller('offset', claims=['joint1/position_offset'])])
    assert exclusive_controller([controller(), controller('other', state='inactive')])


def test_lease_keeps_final_target_when_blocked_but_never_resumes_after_expiry():
    s = TargetSession()
    s.begin('a', [0.1] * 6, 10.)
    assert s.tick(10.1, True) == pytest.approx([0.1] * 6)
    s.renew('a', 10.5)
    assert s.tick(11.4, True) is not None
    assert s.tick(11.5, True) is None
    with pytest.raises(ValueError):
        s.renew('a', 11.6)
    assert s.tick(11.7, True) is None


def test_late_renew_cannot_hide_expiry_between_timer_ticks():
    s = TargetSession()
    s.begin('a', [0.] * 6, 10.)
    with pytest.raises(ValueError, match='expired'):
        s.renew('a', 11.1)
    assert not s.active


def test_source_age_consumes_lease_and_replay_future_and_invalid_stamps_rejected():
    s = TargetSession()
    s.check_stamp({'stamp_ns': 800_000_000}, 1_000_000_000)
    for value in (800_000_000, 700_000_000, 1_000_000_001, 1, 'now', None, True):
        with pytest.raises(ValueError):
            s.check_stamp({'stamp_ns': value}, 1_000_000_000)
    s.begin('a', [0.] * 6, 10., source_age=.4)
    assert s.tick(10.61, True) is None


@pytest.mark.parametrize('goal', [[0.] * 5, [float('nan')] * 6, [float('inf')] * 6])
def test_session_rejects_bad_goal(goal):
    with pytest.raises(ValueError):
        TargetSession().begin('a', goal, 10.)


def test_fault_or_stop_revokes_old_session_and_wrong_id_never_extends_it():
    s = TargetSession()
    s.begin('a', [0.] * 6, 10.)
    with pytest.raises(ValueError):
        s.renew('b', 10.1)
    assert s.deadline == 11.
    assert s.tick(10.2, False) is None
    assert s.tick(10.3, True) is None
    s.begin('b', [0.] * 6, 11.)
    s.stop()
    assert s.tick(11.1, True) is None


@pytest.fixture
def node(monkeypatch):
    monkeypatch.setattr(module.time, 'monotonic', lambda: 10.)
    n = object.__new__(ImpedanceTargetNode)
    n.session = TargetSession()
    n.ik = NS(self_collides=lambda *_: False,
              fk_pose=lambda _: (np.array([.15, 0., .3]), np.eye(3)))
    n.model_changed = False
    n.lower, n.upper = np.full(6, -2.), np.full(6, 2.)
    n.q = np.zeros(6)
    n.state_at = n.manager_at = 10.
    n.controller_ok = True
    n.controller_status, n.fault = 0, 0
    n.command_started = 10.
    n.last_command_stamp = 0
    n.origin, n.radius = (.012, 0), .025
    n.mode, n.values, n.target_pose = 'JOINT', None, None
    n.sent, n.statuses = [], []
    n.pub = NS(publish=n.sent.append)
    n.status_pub = NS(publish=n.statuses.append)
    n.get_clock = lambda: NS(now=lambda: NS(nanoseconds=1_000_000_000,
        to_msg=lambda: module.JointState().header.stamp))
    return n


def test_specific_limit_detail_is_reported_without_enabling_target(node):
    node.on_limit(NS(data=[1., 2., 1., -.45, -.3, .3, 1.]))
    node.on_state(NS(data=[3., 32.] + [0.] * 42))
    assert 'joint2 speed -0.4500 rad/s' in node.session.message
    assert 'Latched fault 32' in node.session.message
    assert not node.ready()
    assert not node.session.active
    node.tick()
    assert node.sent == []


@pytest.mark.parametrize('values', [[], [1., 2., 1.5, .4, -.3, .3, 1.],
    [1., 2., 6., .4, -.3, .3, 1.], [2., 2., 1., .4, -.3, .3, 1.],
    [1., 2., 1., float('nan'), -.3, .3, 1.], [1., 2., 1., .4, .3, -.3, 1.],
    [1., 2., 1., .4, -.3, .3, 2.]])
def test_malformed_limit_detail_does_not_hide_latched_fault(node, values):
    node.on_limit(NS(data=values))
    node.on_state(NS(data=[3., 32.] + [0.] * 42))
    assert 'Latched fault 32' in node.session.message
    assert 'First limit trip' not in node.session.message
    assert not node.ready()


def test_stale_limit_detail_not_reused_and_ramp_is_not_reported_as_fault(node):
    node.on_limit(NS(data=[1., 1., 0., 2., -1., 1., 1.]))
    node.limit_at = 0.
    node.on_state(NS(data=[3., 34.] + [0.] * 42))
    assert 'bus health' in node.session.message and 'limit exceeded' in node.session.message
    assert 'First limit trip' not in node.session.message
    node.on_state(NS(data=[4., 0.] + [0.] * 42))
    assert 'Gravity ramp in progress' in node.session.message
    assert not node.ready()


def command(node, **extra):
    request = dict(action='move', mode='JOINT', values=[.01] * 6,
                   request_id='a', stamp_ns=900_000_000)
    request.update(extra)
    node.on_command(NS(data=json.dumps(request)))


def test_move_sends_only_full_jointstate_effort_target_and_stop_never_trajectory(node):
    command(node)
    assert node.session.active
    node.tick()
    assert len(node.sent) == 1
    assert node.sent[0].name == JOINTS
    assert node.sent[0].position == pytest.approx([.01] * 6)
    assert not node.sent[0].velocity and not node.sent[0].effort
    command(node, action='stop')
    node.tick()
    assert not node.session.active and len(node.sent) == 1


@pytest.mark.parametrize('field,value', [('controller_ok', False), ('state_at', 0.),
    ('manager_at', 0.), ('fault', 1), ('controller_status', 4), ('ik', None)])
def test_run_rejects_inactive_stale_fault_ramp_or_missing_model(node, field, value):
    setattr(node, field, value)
    command(node)
    node.tick()
    assert not node.session.active and not node.sent


@pytest.mark.parametrize('values', [[3.] * 6, [float('nan')] * 6, [0.] * 5])
def test_no_out_of_limit_or_nonfinite_target_sent(node, values):
    command(node, values=values)
    assert not node.session.active and not node.sent


def test_no_approximate_cartesian_goal_and_no_collision_goal(node, monkeypatch):
    monkeypatch.setattr(module, 'plan_pose', lambda *_: NS(approximate=True, joints=np.zeros(6)))
    command(node, mode='CARTESIAN')
    assert not node.session.active
    node.session.last_stamp = 0
    node.ik.self_collides = lambda *_: True
    command(node)
    assert not node.session.active


def test_exact_cartesian_goal_uses_ik_joint_output(node, monkeypatch):
    monkeypatch.setattr(module, 'plan_pose', lambda *_: NS(approximate=False, joints=np.full(6, .02)))
    command(node, mode='CARTESIAN', values=[.15, 0., .3, 0., 0., 0.])
    node.tick()
    assert node.sent[0].position == pytest.approx([.02] * 6)


def test_telemetry_gap_or_fault_then_recovery_cannot_resume(node):
    command(node)
    node.state_at = 9.
    node.on_state(NS(data=[1., 0.] + [0.] * 42))
    node.tick()
    assert not node.session.active and not node.sent
    command(node, stamp_ns=950_000_000)
    node.on_state(NS(data=[3., 2.] + [0.] * 42))
    node.on_state(NS(data=[0., 0.] + [0.] * 42))
    node.tick()
    assert not node.session.active and not node.sent


def test_controller_watchdog_does_not_automatically_rearm_active_target(node):
    command(node)
    node.command_started = 9.
    node.on_state(NS(data=[2., 0.] + [0.] * 42))
    node.tick()
    assert not node.session.active and not node.sent


def test_stalled_ros_clock_revokes_stream(node):
    command(node)
    node.tick()
    node.tick()
    assert len(node.sent) == 1 and not node.session.active


def test_real_v2_live_urdf_model_and_current_pose_target_without_xacro_rerender(node, monkeypatch):
    from om6dof_controllers.ik_solver import render_chain_urdf
    from om6dof_controllers import ik_solver
    xml = render_chain_urdf('om6dof_description', 'urdf/om6dof_v2.urdf.xacro')
    monkeypatch.setattr(ik_solver, 'render_chain_urdf', lambda *_: pytest.fail('must use live model'))
    node.urdf_xml = ''
    node.on_model(NS(data=xml))
    assert node.ik is not None
    node.q = np.radians([0, -60, 15, 0, 60, 0])
    goal, pose = node.plan('JOINT', node.q.tolist())
    assert pose is None and goal == pytest.approx(node.q)
    xyz, rotation = node.ik.fk_pose(node.q)
    values = xyz.tolist() + list(module.tip_rotation_to_cartesian_rpy(rotation))
    goal, pose = node.plan('CARTESIAN', values)
    assert goal == pytest.approx(node.q, abs=.01)
    node.on_model(NS(data=xml + '\n'))
    assert node.model_changed and node.ik is None and not node.ready()
    node.on_model(NS(data=xml))
    assert node.ik is None  # no automatic recovery to a potentially mismatched model


def test_holding_keeps_lease_but_current_feedback_is_not_commanded_pose(node):
    command(node, values=[0.] * 6)
    node.publish_status()
    status = json.loads(node.statuses[-1].data)
    assert status['state'] == 'holding' and status['active']
    assert status['current']['joint'] == [0.] * 6


def test_ros_bridge_round_trip_on_isolated_domain(monkeypatch):
    """Fake state/manager only, domain 192; never instantiate a hardware plugin."""
    import time
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, DurabilityPolicy
    from controller_manager_msgs.msg import ControllerState
    from controller_manager_msgs.srv import ListControllers
    from sensor_msgs.msg import JointState
    from std_msgs.msg import String, Float64MultiArray
    from om6dof_controllers.ik_solver import render_chain_urdf

    xml = render_chain_urdf('om6dof_description', 'urdf/om6dof_v2.urdf.xacro')
    monkeypatch.setenv('ROS_LOCALHOST_ONLY', '1')
    rclpy.init(args=[], domain_id=192)
    bridge = ImpedanceTargetNode()
    fixture = Node('impedance_offline_fixture')
    executor = SingleThreadedExecutor()
    executor.add_node(bridge)
    executor.add_node(fixture)
    received = []
    tracked = [False]
    ready_q = np.radians([0., -60., 15., 0., 60., 0.]).tolist()
    qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    model_pub = fixture.create_publisher(String, '/robot_description', qos)
    telemetry = fixture.create_publisher(Float64MultiArray, f'/{CONTROLLER}/state', 1)
    command_pub = fixture.create_publisher(String, module.COMMAND_TOPIC, 1)

    def manager_callback(_request, response):
        response.controller = [ControllerState(name=CONTROLLER, state='active',
            type='om6dof_controllers/PositionImpedanceController',
            claimed_interfaces=[f'{j}/effort' for j in JOINTS])]
        return response

    def on_target(msg):
        received.append(msg)
        tracked[0] = True

    fixture.create_service(ListControllers, '/controller_manager/list_controllers', manager_callback)
    fixture.create_subscription(JointState, f'/{CONTROLLER}/target', on_target, 10)
    fixture.create_timer(.01, lambda: telemetry.publish(Float64MultiArray(
        data=[1. if tracked[0] else 0., 0.] + ready_q + [0.] * 36)))

    def spin_until(predicate, seconds=3.):
        end = time.monotonic() + seconds
        while time.monotonic() < end and not predicate():
            executor.spin_once(timeout_sec=.01)
        assert predicate()

    def send(action):
        command_pub.publish(String(data=json.dumps(dict(action=action, request_id='offline',
            stamp_ns=fixture.get_clock().now().nanoseconds, mode='JOINT', values=ready_q))))

    try:
        model_pub.publish(String(data=xml))
        spin_until(bridge.ready)
        assert not received  # no auto-command after becoming ready
        send('move')
        spin_until(lambda: len(received) >= 3)
        assert received[-1].name == JOINTS
        assert received[-1].position == pytest.approx(ready_q)
        stamps = [m.header.stamp.sec * 1_000_000_000 + m.header.stamp.nanosec for m in received]
        assert all(a < b for a, b in zip(stamps, stamps[1:]))
        send('stop')
        spin_until(lambda: not bridge.session.active)
        # Drain already-published DDS samples, then prove no ongoing stream.
        until = time.monotonic() + .1
        while time.monotonic() < until:
            executor.spin_once(timeout_sec=.01)
        count = len(received)
        until = time.monotonic() + .3
        while time.monotonic() < until:
            executor.spin_once(timeout_sec=.01)
        assert len(received) == count
        send('renew')
        until = time.monotonic() + .2
        while time.monotonic() < until:
            executor.spin_once(timeout_sec=.01)
        assert not bridge.session.active and len(received) == count
    finally:
        executor.shutdown()
        bridge.destroy_node()
        fixture.destroy_node()
        rclpy.shutdown()
