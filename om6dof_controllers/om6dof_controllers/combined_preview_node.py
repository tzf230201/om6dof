"""Isolated preview of concurrent trajectory + world-frame joystick trim.

Deliberately has no motor publisher, action client/server, controller-switch
client or hardware-enable parameter. It cannot execute the computed reference.
"""
import json
import time

import numpy as np
import rclpy
from geometry_msgs.msg import Point, TwistStamped
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory
from visualization_msgs.msg import Marker, MarkerArray

from .combined_reference import CombinedReference
from .control_math import rotation_error
from .ik_solver import IKSolver, joint_limits_from_urdf
from .target_planner import path_is_clear, pose_error


class CombinedPreviewNode(Node):
    def __init__(self):
        super().__init__('om6dof_combined_preview')
        self.declare_parameter('trajectory_topic',
                               '/om6dof_topo_gng_v2/graph_pick/trajectory_preview')
        self.declare_parameter('twist_topic', '/om6dof/combined_preview/teleop_twist')
        self.joint_names = [f'joint{i}' for i in range(1, 7)]
        self.ik = IKSolver(xacro_rel='urdf/om6dof_v2.urdf.xacro', damping=0.05)
        lower, upper = joint_limits_from_urdf(
            self.joint_names, xacro_rel='urdf/om6dof_v2.urdf.xacro')
        self.lower, self.upper = np.array(lower) + 0.02, np.array(upper) - 0.02
        self.engine = CombinedReference(self.ik.fk_pose)
        self.seed = None
        self.feedback = None
        self.feedback_time = -float('inf')
        self.last_status = 0.0
        self.message = 'waiting_for_new_trajectory_preview'
        self.nominal_points, self.combined_points = [], []
        self.twist_armed = False
        self.last_twist_receipt = -float('inf')
        self.last_ros_twist_ns = -1
        transient = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                               durability=DurabilityPolicy.TRANSIENT_LOCAL)
        # Volatile: never replay a frozen preview retained from an old session.
        self.create_subscription(JointTrajectory, self.get_parameter('trajectory_topic').value,
                                 self._trajectory, 1)
        self.create_subscription(TwistStamped, self.get_parameter('twist_topic').value,
                                 self._twist, 1)
        self.create_subscription(JointState, '/joint_states', self._feedback, 5)
        self.status = self.create_publisher(String, '/om6dof/combined_preview/status', transient)
        self.markers = self.create_publisher(MarkerArray, '/om6dof/combined_preview/markers', 1)
        self.references = self.create_publisher(
            JointState, '/om6dof/combined_preview/joint_reference', 1)
        self.create_timer(0.02, self._tick)
        self.get_logger().info('Combined PREVIEW only: no connection to robot commands. '
                               'Green = nominal; magenta = accumulated joystick correction.')

    def _feedback(self, msg):
        if len(msg.name) != len(msg.position) or len(set(msg.name)) != len(msg.name):
            return
        mapping = dict(zip(msg.name, msg.position))
        if all(j in mapping for j in self.joint_names):
            values = np.array([mapping[j] for j in self.joint_names])
            if np.isfinite(values).all():
                self.feedback = values
                self.feedback_time = time.monotonic()

    def _trajectory(self, msg):
        try:
            if msg.joint_names != self.joint_names:
                raise ValueError('trajectory joint order must be joint1..joint6')
            if msg.header.frame_id not in ('', 'world'):
                raise ValueError('trajectory must use world frame')
            if self.feedback is None or time.monotonic() - self.feedback_time > 0.5:
                raise ValueError('fresh joint feedback required for preview IK seed')
            values = np.array([p.positions for p in msg.points])
            if values.ndim != 2 or values.shape[1] != 6:
                raise ValueError('trajectory positions must have six joints')
            if np.any(values < self.lower) or np.any(values > self.upper):
                raise ValueError('nominal trajectory violates effective URDF limits')
            self.engine.start([p.time_from_start.sec + p.time_from_start.nanosec * 1e-9
                               for p in msg.points], values, time.monotonic())
            self.seed = self.feedback.copy()
            self.nominal_points, self.combined_points = [], []
            self.twist_armed = False
            self.message = 'playing_preview_waiting_for_neutral_joystick'
        except (ValueError, TypeError) as error:
            self.engine.times = self.engine.started = None
            self.message = f'trajectory_rejected:{error}'

    def _twist(self, msg):
        try:
            now = time.monotonic()
            ros_now = self.get_clock().now().nanoseconds * 1e-9
            stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            stamp_ns = msg.header.stamp.sec * 1000000000 + msg.header.stamp.nanosec
            if stamp_ns <= self.last_ros_twist_ns:
                raise ValueError('duplicate or out-of-order Twist')
            values = np.array([msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z,
                               msg.twist.angular.x, msg.twist.angular.y, msg.twist.angular.z])
            # ROS timestamp validated first, then mapped to monotonic time.
            age = ros_now - stamp
            if not 0 <= age <= self.engine.timeout or msg.header.frame_id != 'world':
                raise ValueError('Twist must have a fresh stamp and world frame')
            if not np.isfinite(values).all():
                raise ValueError('non-finite Twist')
            if now - self.last_twist_receipt > self.engine.timeout:
                self.twist_armed = False
            self.last_twist_receipt = now
            if not self.twist_armed:
                self.twist_armed = bool(np.all(np.abs(values) <= 1e-9))
                values = np.zeros(6)
            self.engine.twist(values, now - age, now, msg.header.frame_id)
            self.last_ros_twist_ns = stamp_ns
        except (ValueError, TypeError) as error:
            self.engine.velocity[:] = 0
            self.engine.received = -float('inf')
            self.twist_armed = False
            self.message = f'twist_rejected:{error}'

    @staticmethod
    def _point(xyz):
        return Point(x=float(xyz[0]), y=float(xyz[1]), z=float(xyz[2]))

    def _draw(self, ref, reason):
        if ref is not None:
            self.nominal_points.append(self._point(ref.nominal_position))
            self.combined_points.append(self._point(ref.position))
        else:
            self.nominal_points, self.combined_points = [], []
        self.nominal_points = self.nominal_points[-1500:]
        self.combined_points = self.combined_points[-1500:]
        markers = MarkerArray()
        notice = Marker()
        notice.header.frame_id = 'world'
        notice.ns, notice.id = 'combined_preview', 2
        notice.type, notice.action = Marker.TEXT_VIEW_FACING, Marker.ADD
        notice.pose.orientation.w = 1.0
        notice.pose.position.x, notice.pose.position.z = 0.3, 0.45
        notice.scale.z = 0.025
        notice.color.r, notice.color.g, notice.color.b, notice.color.a = 1.0, 0.85, 0.3, 1.0
        if ref is None:
            if self.feedback is None or time.monotonic() - self.feedback_time > 0.5:
                text = 'Menunggu /joint_states nyata yang terbaru.'
            elif reason == 'waiting_for_new_trajectory_preview':
                text = 'Pilih objek > Set planning target > Preview EoE path in RViz.'
            else:
                text = reason
            notice.text = 'PREVIEW GABUNGAN - BELUM ADA TRAJECTORY\n' + text
        else:
            notice.text = ('PREVIEW SAJA - robot tidak bergerak\n'
                           'Hijau: acuan | Magenta: acuan + offset joystick\n'
                           + reason)
        markers.markers.append(notice)
        for index, points in enumerate((self.nominal_points, self.combined_points)):
            line = Marker()
            line.header.frame_id = 'world'
            line.header.stamp = self.get_clock().now().to_msg()
            line.ns, line.id = 'combined_preview', index
            line.type, line.action = Marker.LINE_STRIP, Marker.ADD
            if not points:
                line.action = Marker.DELETE
            line.pose.orientation.w = 1.0
            line.scale.x = 0.003
            line.color.a = 1.0
            line.color.g = 1.0 if index == 0 else 0.0
            line.color.r = line.color.b = 0.0 if index == 0 else 1.0
            line.points = points
            markers.markers.append(line)
        self.markers.publish(markers)

    def _tick(self):
        now = time.monotonic()
        ref, valid, reason = None, False, self.message
        try:
            ref = self.engine.tick(now)
            if ref is not None:
                if self.feedback is None or now - self.feedback_time > 0.5:
                    raise ValueError('joint_feedback_missing_or_stale')
                candidate, _ = self.ik.solve_pose_ik(self.seed, ref.position,
                                                     ref.rotation, max_iter=40)
                candidate = np.asarray(candidate)
                if (candidate.shape != (6,) or not np.isfinite(candidate).all()
                        or np.any(candidate < self.lower) or np.any(candidate > self.upper)):
                    raise ValueError('combined_reference_outside_joint_limits')
                pos_error, rot_error = pose_error(self.ik, candidate, ref.position, ref.rotation)
                if pos_error > 0.001 or rot_error > np.deg2rad(0.5):
                    raise ValueError('combined_reference_IK_residual_too_large')
                if not path_is_clear(self.ik, self.seed, candidate, 0.025):
                    raise ValueError('combined_reference_approximate_self_collision')
                self.seed = candidate
                sample = JointState()
                sample.header.stamp = self.get_clock().now().to_msg()
                sample.name, sample.position = self.joint_names, candidate.tolist()
                self.references.publish(sample)
                valid = True
                reason = 'preview_finished_offset_retained' if ref.finished else 'preview_running'
                # World collision, speed, dynamics and hardware tracking have
                # NOT been validated. Never report this as executable.
        except (ValueError, TypeError, np.linalg.LinAlgError) as error:
            reason = str(error)
        if now - self.last_status >= 0.1:
            self.last_status = now
            self._draw(ref, reason)
            payload = dict(preview_only=True, executable=False,
                           automatic_phase_s=ref.phase if ref else None,
                           offset_world_m=self.engine.offset.tolist(),
                           offset_rotation_rad=float(np.linalg.norm(rotation_error(
                               self.engine.offset_rotation, np.eye(3)))),
                           kinematic_preview_valid=valid, reason=reason,
                           environment_collision_validated=False,
                           waiting_for_neutral=not self.twist_armed)
            self.status.publish(String(data=json.dumps(payload)))


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = CombinedPreviewNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
