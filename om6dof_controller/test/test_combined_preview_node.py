import json
import time
from types import SimpleNamespace

import numpy as np
from builtin_interfaces.msg import Time
from geometry_msgs.msg import TwistStamped
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from om6dof_controller.combined_preview_node import CombinedPreviewNode
from om6dof_controller.combined_reference import CombinedReference


class IK:
    def fk_pose(self, q):
        return q[:3], np.eye(3)
    def solve_pose_ik(self, _q, pos, rotation, **kwargs):
        return np.r_[pos, np.zeros(3)], True
    def self_collides(self, _q, _r):
        return False


def preview():
    node = object.__new__(CombinedPreviewNode)
    node.ik = IK()
    node.engine = CombinedReference(node.ik.fk_pose)
    node.joint_names = [f'joint{i}' for i in range(1, 7)]
    node.lower, node.upper = np.full(6, -3), np.full(6, 3)
    node.feedback, node.seed = np.zeros(6), np.zeros(6)
    node.feedback_time = time.monotonic()
    node.last_status = 0
    node.nominal_points, node.combined_points = [], []
    node.twist_armed = False
    node.last_twist_receipt = -float('inf')
    node.last_ros_twist_ns = -1
    node.message = ''
    node.samples, node.statuses = [], []
    node.references = SimpleNamespace(publish=node.samples.append)
    node.status = SimpleNamespace(publish=node.statuses.append)
    node.markers = SimpleNamespace(publish=lambda _: None)
    node.get_clock = lambda: SimpleNamespace(now=lambda: SimpleNamespace(
        nanoseconds=200000000000, to_msg=lambda: Time(sec=200)))
    return node


def trajectory():
    msg = JointTrajectory()
    msg.joint_names = [f'joint{i}' for i in range(1, 7)]
    for t, q in ((0, [0.]*6), (10, [.1, 0., 0., 0., 0., 0.])):
        point = JointTrajectoryPoint(positions=q)
        point.time_from_start.sec = t
        msg.points.append(point)
    return msg


def test_composed_reference_emitted_only_to_preview_and_never_executable():
    node = preview()
    node._trajectory(trajectory())
    node.engine.offset[:] = [0, .01, 0]
    node._tick()
    assert node.samples[-1].position[1] == .01
    status = json.loads(node.statuses[-1].data)
    assert status['kinematic_preview_valid']
    assert status['preview_only'] and not status['executable']
    assert not status['environment_collision_validated']


def test_bad_ik_suppresses_joint_reference_but_reports_invalid_pose():
    node = preview()
    node._trajectory(trajectory())
    node.ik.solve_pose_ik = lambda *_args, **_kw: (np.full(6, 99), True)
    node._tick()
    assert not node.samples
    assert not json.loads(node.statuses[-1].data)['kinematic_preview_valid']


def test_stale_feedback_stops_joint_reference_output():
    node = preview()
    node._trajectory(trajectory())
    node.feedback_time -= 2
    node._tick()
    assert not node.samples
    assert 'stale' in json.loads(node.statuses[-1].data)['reason']


def test_rejected_new_trajectory_does_not_keep_playing_old_preview():
    node = preview()
    node._trajectory(trajectory())
    bad = trajectory()
    bad.joint_names.reverse()
    node._trajectory(bad)
    node._tick()
    assert not node.samples
    assert node.engine.started is None


def test_start_requires_neutral_and_duplicate_stamp_disarms():
    node = preview()
    node._trajectory(trajectory())
    msg = TwistStamped()
    msg.header.frame_id = 'world'
    msg.header.stamp = Time(sec=199, nanosec=900000000)
    msg.twist.linear.y = .01
    node._twist(msg)
    assert not node.twist_armed
    assert not np.any(node.engine.velocity)

    msg.header.stamp.nanosec = 910000000
    msg.twist.linear.y = 0.0
    node._twist(msg)
    assert node.twist_armed
    node._twist(msg)
    assert not node.twist_armed
    assert not np.any(node.engine.velocity)


def test_waiting_preview_renders_instructions_without_a_trajectory():
    node = preview()
    captured = []
    node.message = 'waiting_for_new_trajectory_preview'
    node.markers = SimpleNamespace(publish=captured.append)
    node._tick()
    text = [m.text for m in captured[-1].markers if m.type == m.TEXT_VIEW_FACING]
    assert len(text) == 1
    assert 'Set planning target' in text[0]
    assert not node.samples


def test_preview_gui_cannot_call_execute_service():
    import importlib.util
    import threading
    from pathlib import Path
    path = Path(__file__).resolve().parents[2] / 'om6dof_dd_gng/scripts/semantic_target_gui.py'
    spec = importlib.util.spec_from_file_location('preview_target_gui', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    node = object.__new__(module.SemanticTargetGui)
    node.preview_only = True
    node._lock = threading.Lock()
    node.execute_client = None
    node.execute_path()
    assert 'tidak menggerakkan robot' in node.service_notice
