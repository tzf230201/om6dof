"""Read-only, timestamp-matched D435 projection using the v2 URDF.

This is a nominal CAD preview, not a calibrated grasp target. The module has no
controller dependency, TF broadcaster, camera driver, or hardware access. Pure
FK and buffering helpers can be tested without importing or starting ROS.
"""

from dataclasses import dataclass
import hashlib
import json
from pathlib import Path
import subprocess
import threading
import xml.etree.ElementTree as ET

import numpy as np


class ProjectionError(ValueError):
    """A rejected observation; ``code`` is suitable for projection_status."""

    def __init__(self, code):
        super().__init__(code)
        self.code = code


def _vector(text, default):
    result = np.asarray([float(value) for value in (text or default).split()])
    if result.shape != (3,) or not np.all(np.isfinite(result)):
        raise ValueError('URDF vector must contain three finite numbers')
    return result


def _axis_rotation(axis, angle):
    x, y, z = axis
    cross = np.array([[0., -z, y], [z, 0., -x], [-y, x, 0.]])
    return np.eye(3) + np.sin(angle) * cross + (1. - np.cos(angle)) * (cross @ cross)


def _origin(element):
    transform = np.eye(4)
    if element is None:
        return transform
    transform[:3, 3] = _vector(element.get('xyz'), '0 0 0')
    roll, pitch, yaw = _vector(element.get('rpy'), '0 0 0')
    # URDF fixed-axis RPY: Rz(yaw) Ry(pitch) Rx(roll).
    transform[:3, :3] = (
        _axis_rotation(np.array([0., 0., 1.]), yaw)
        @ _axis_rotation(np.array([0., 1., 0.]), pitch)
        @ _axis_rotation(np.array([1., 0., 0.]), roll)
    )
    return transform


@dataclass(frozen=True)
class _Joint:
    name: str
    kind: str
    origin: np.ndarray
    axis: np.ndarray


class UrdfKinematics:
    """Forward kinematics for a selected ancestor-to-descendant URDF chain."""

    def __init__(self, xml, base_frame='base_link', optical_frame='d435_color_optical_frame'):
        root = ET.fromstring(xml)
        links = {link.get('name') for link in root.findall('link')}
        if base_frame not in links or optical_frame not in links:
            raise ValueError(f'URDF is missing {base_frame!r} or {optical_frame!r}')
        parents = {}
        for element in root.findall('joint'):
            parent, child = element.find('parent'), element.find('child')
            if parent is None or child is None:
                raise ValueError('URDF joint is missing its parent or child')
            child_name = child.get('link')
            if child_name in parents:
                raise ValueError(f'Duplicate URDF parent for {child_name}')
            parents[child_name] = (parent.get('link'), element)
        chain, visited, frame = [], set(), optical_frame
        while frame != base_frame:
            if frame in visited or frame not in parents:
                raise ValueError(f'No acyclic URDF chain from {base_frame} to {optical_frame}')
            visited.add(frame)
            frame, element = parents[frame]
            kind = element.get('type')
            if kind not in ('fixed', 'revolute', 'continuous', 'prismatic'):
                raise ValueError(f'Unsupported URDF joint type {kind!r}')
            if element.find('mimic') is not None:
                raise ValueError('Mimic joints are not supported in the projection chain')
            axis_element = element.find('axis')
            axis = _vector(None if axis_element is None else axis_element.get('xyz'), '1 0 0')
            norm = np.linalg.norm(axis)
            if not np.isfinite(norm) or norm <= 1e-12:
                raise ValueError('URDF joint axis must have a finite nonzero length')
            chain.append(_Joint(element.get('name'), kind, _origin(element.find('origin')), axis / norm))
        self.chain = tuple(reversed(chain))
        self.joint_names = tuple(joint.name for joint in self.chain if joint.kind != 'fixed')
        if len(set(self.joint_names)) != len(self.joint_names):
            raise ValueError('Duplicate joint names in URDF projection chain')
        self.base_frame = base_frame
        self.optical_frame = optical_frame
        self.urdf_sha256 = hashlib.sha256(xml.encode('utf-8')).hexdigest()

    def transform(self, positions):
        """Return T_base_optical; origins and axes come exclusively from URDF."""
        transform = np.eye(4)
        for joint in self.chain:
            transform = transform @ joint.origin
            if joint.kind == 'fixed':
                continue
            if joint.name not in positions:
                raise ProjectionError('missing_joint_positions')
            value = float(positions[joint.name])
            if not np.isfinite(value):
                raise ProjectionError('nonfinite_joint_position')
            motion = np.eye(4)
            if joint.kind == 'prismatic':
                motion[:3, 3] = joint.axis * value
            else:
                motion[:3, :3] = _axis_rotation(joint.axis, value)
            transform = transform @ motion
        return transform


def load_model_urdf(model_version='v2'):
    """Render the installed v2 model only; never silently substitute v1."""
    if model_version != 'v2':
        raise ValueError('World projection supports model_version=v2 only; v1 calibration remains in the existing picker')
    from ament_index_python.packages import get_package_share_directory

    source = Path(get_package_share_directory('om6dof_description')) / 'urdf' / 'om6dof_v2.urdf.xacro'
    result = subprocess.run(['xacro', str(source)], check=True, capture_output=True,
                            text=True, timeout=20)
    return result.stdout


def _valid_stamp(stamp_ns, now_ns, max_age_ns, label):
    if not isinstance(stamp_ns, (int, np.integer)) or stamp_ns <= 0:
        raise ProjectionError(f'{label}_stamp_invalid')
    if not isinstance(now_ns, (int, np.integer)) or now_ns <= 0:
        raise ProjectionError('clock_invalid')
    if stamp_ns > now_ns:
        raise ProjectionError(f'{label}_stamp_future')
    if now_ns - stamp_ns > max_age_ns:
        raise ProjectionError(f'{label}_stamp_stale')


class WorldProjector:
    """Bounded complete-state buffer and strict capture-time point projection.

    No interpolation or latest-state fallback is performed. A point for which
    no close-enough complete state has arrived is dropped, even if a suitable
    later state eventually arrives. Reordered joint messages are kept sorted.
    """

    def __init__(self, kinematics, *, model_version='v2', max_joint_delta_sec=.05,
                 max_age_sec=.5, buffer_size=200):
        if model_version != 'v2':
            raise ValueError('World projection supports model_version=v2 only')
        for value in (max_joint_delta_sec, max_age_sec):
            if not np.isfinite(value) or value <= 0:
                raise ValueError('Time limits must be finite and positive')
        if not isinstance(buffer_size, int) or isinstance(buffer_size, bool) or buffer_size < 1:
            raise ValueError('buffer_size must be a positive integer')
        self.kinematics = kinematics
        self.max_joint_delta_ns = int(round(max_joint_delta_sec * 1e9))
        self.max_age_ns = int(round(max_age_sec * 1e9))
        if min(self.max_joint_delta_ns, self.max_age_ns) < 1:
            raise ValueError('Time limits must be at least one nanosecond')
        self.buffer_size = buffer_size
        self._samples = []
        self._lock = threading.Lock()
        self._point_sequence = 0
        self._last_point_receipt_ns = None
        self._last_projection_status = None
        self._last_point_error = None
        self.metadata = {
            'model_version': model_version,
            'base_frame': kinematics.base_frame,
            'optical_frame': kinematics.optical_frame,
            'transform_source': 'urdf_nominal',
            'calibration_verified': False,
            'urdf_sha256': kinematics.urdf_sha256,
            'point_timestamp_basis': 'host_receipt_before_processing',
            'hardware_time_synchronized': False,
            'usage': 'CAD_preview_not_automatic_pickup',
            'max_age_sec': self.max_age_ns / 1e9,
            'max_joint_delta_sec': self.max_joint_delta_ns / 1e9,
        }

    def add_joint_state(self, names, positions, stamp_ns, now_ns):
        _valid_stamp(stamp_ns, now_ns, self.max_age_ns, 'joint')
        names = tuple(names)
        if len(names) != len(positions) or len(set(names)) != len(names):
            raise ProjectionError('invalid_joint_state_layout')
        try:
            values = np.asarray(positions, dtype=float)
        except (TypeError, ValueError):
            raise ProjectionError('nonfinite_joint_position') from None
        if values.shape != (len(names),) or not np.all(np.isfinite(values)):
            raise ProjectionError('nonfinite_joint_position')
        mapping = dict(zip(names, values))
        if any(name not in mapping for name in self.kinematics.joint_names):
            raise ProjectionError('missing_joint_positions')
        complete = tuple(float(mapping[name]) for name in self.kinematics.joint_names)
        with self._lock:
            # Same-stamp replacement avoids duplicate entries; never combine
            # partial states sampled at different times into one configuration.
            samples = [sample for sample in self._samples if sample[0] != stamp_ns]
            samples.append((int(stamp_ns), complete))
            samples.sort(key=lambda sample: sample[0])
            self._samples = samples[-self.buffer_size:]

    def project(self, point, frame_id, stamp_ns, now_ns):
        with self._lock:
            self._point_sequence += 1
            sequence = self._point_sequence
            self._last_point_receipt_ns = (
                int(now_ns) if isinstance(now_ns, (int, np.integer)) and now_ns > 0 else None)
            self._last_projection_status = None
            self._last_point_error = None
        try:
            result, status = self._project(point, frame_id, stamp_ns, now_ns)
        except ProjectionError as error:
            with self._lock:
                if self._point_sequence == sequence:
                    self._last_point_error = error.code
            raise
        with self._lock:
            if self._point_sequence == sequence:
                self._last_projection_status = dict(status)
        return result, status

    def _project(self, point, frame_id, stamp_ns, now_ns):
        _valid_stamp(stamp_ns, now_ns, self.max_age_ns, 'point')
        if frame_id != self.kinematics.optical_frame:
            raise ProjectionError('wrong_optical_frame')
        try:
            point = np.asarray(point, dtype=float)
        except (TypeError, ValueError):
            raise ProjectionError('nonfinite_point') from None
        if point.shape != (3,) or not np.all(np.isfinite(point)):
            raise ProjectionError('nonfinite_point')
        if point[2] <= 0:
            raise ProjectionError('nonpositive_optical_depth')
        with self._lock:
            candidates = [sample for sample in self._samples
                          if 0 <= now_ns - sample[0] <= self.max_age_ns]
            if not candidates:
                raise ProjectionError('no_fresh_complete_joint_state')
            # Ties choose the earlier sample, independent of arrival ordering.
            joint_stamp, values = min(candidates, key=lambda sample: (abs(sample[0] - stamp_ns), sample[0]))
        delta_ns = abs(joint_stamp - stamp_ns)
        if delta_ns > self.max_joint_delta_ns:
            raise ProjectionError('joint_point_time_mismatch')
        transform = self.kinematics.transform(dict(zip(self.kinematics.joint_names, values)))
        projected = transform[:3, :3] @ point + transform[:3, 3]
        if not np.all(np.isfinite(projected)):
            raise ProjectionError('nonfinite_projection')
        status = dict(self.metadata, status='projected', accepted=True,
                      point_stamp_ns=int(stamp_ns), joint_stamp_ns=joint_stamp,
                      joint_point_delta_sec=delta_ns / 1e9)
        return projected, status

    def liveness_status(self, now_ns):
        """Report whether the last projection remains fresh, without new FK.

        Called periodically by ROS so a stopped camera/joint stream cannot
        leave a latched ``accepted=true`` status indefinitely.
        """
        common = dict(self.metadata, accepted=False)
        if not isinstance(now_ns, (int, np.integer)) or now_ns <= 0:
            return dict(common, status='waiting_for_clock', reason='clock_invalid')
        common['status_stamp_ns'] = int(now_ns)
        with self._lock:
            receipt = self._last_point_receipt_ns
            projection = self._last_projection_status
            rejection = self._last_point_error
            joints_fresh = any(0 <= now_ns - stamp <= self.max_age_ns
                               for stamp, _ in self._samples)
        if receipt is None:
            return dict(common, status='waiting_for_data', reason='no_point_received')
        common['last_point_receipt_stamp_ns'] = receipt
        if now_ns < receipt:
            return dict(common, status='stale', reason='clock_rollback')
        if now_ns - receipt > self.max_age_ns:
            return dict(common, status='stale', reason='point_stream_stale')
        if not joints_fresh:
            return dict(common, status='stale', reason='joint_state_stream_stale')
        if projection is None:
            return dict(common, status='waiting_for_valid_point', reason=rejection or 'projection_pending')
        if now_ns < projection['point_stamp_ns'] or now_ns < projection['joint_stamp_ns']:
            return dict(common, status='stale', reason='clock_rollback')
        if now_ns - projection['point_stamp_ns'] > self.max_age_ns:
            return dict(common, status='stale', reason='point_stamp_stale')
        if now_ns - projection['joint_stamp_ns'] > self.max_age_ns:
            return dict(common, status='stale', reason='projection_joint_stamp_stale')
        return dict(common, **projection)


def main(args=None):
    """ROS adapter; its only output is observation data and status topics."""
    import rclpy
    from rclpy.node import Node
    from rcl_interfaces.msg import ParameterDescriptor
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
    from geometry_msgs.msg import PointStamped
    from sensor_msgs.msg import JointState
    from std_msgs.msg import String

    class WorldProjectionNode(Node):
        def __init__(self):
            super().__init__('om6dof_world_projection')
            defaults = {
                'model_version': 'v2', 'base_frame': 'base_link',
                'optical_frame': 'd435_color_optical_frame',
                'joint_state_topic': '/joint_states',
                'point_topic': '/om6dof_perception/target_point',
                'output_topic': '/om6dof_perception/target_point_world',
                'status_topic': '/om6dof_perception/projection_status',
                'max_joint_delta_sec': .05, 'max_age_sec': .5, 'buffer_size': 200,
            }
            for name, default in defaults.items():
                self.declare_parameter(name, default, ParameterDescriptor(
                    read_only=True, description='Restart projection to change its model or timing contract'))
            params = {name: self.get_parameter(name).value for name in defaults}
            xml = load_model_urdf(params['model_version'])
            kinematics = UrdfKinematics(xml, params['base_frame'], params['optical_frame'])
            self.projector = WorldProjector(
                kinematics, model_version=params['model_version'],
                max_joint_delta_sec=params['max_joint_delta_sec'],
                max_age_sec=params['max_age_sec'], buffer_size=params['buffer_size'])
            self.output = self.create_publisher(PointStamped, params['output_topic'], 10)
            status_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                   durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self.status = self.create_publisher(String, params['status_topic'], status_qos)
            self.create_subscription(JointState, params['joint_state_topic'],
                                     self.on_joints, qos_profile_sensor_data)
            self.create_subscription(PointStamped, params['point_topic'], self.on_point, 10)
            self.create_timer(.1, self.on_liveness)
            self.publish_status(status='waiting_for_data', accepted=False)
            self.get_logger().warning('V2 projection uses nominal URDF extrinsics, not verified calibration; CAD preview only.')

        def publish_status(self, **values):
            message = String()
            message.data = json.dumps(dict(self.projector.metadata, **values), allow_nan=False)
            self.status.publish(message)

        @staticmethod
        def stamp_ns(header):
            if header.stamp.sec < 0 or not 0 <= header.stamp.nanosec < 1_000_000_000:
                return 0
            return header.stamp.sec * 1_000_000_000 + header.stamp.nanosec

        def on_joints(self, message):
            try:
                self.projector.add_joint_state(message.name, message.position,
                                               self.stamp_ns(message.header),
                                               self.get_clock().now().nanoseconds)
            except ProjectionError as error:
                self.publish_status(status='rejected_joint_state', accepted=False, reason=error.code)

        def on_point(self, message):
            try:
                point, status = self.projector.project(
                    (message.point.x, message.point.y, message.point.z), message.header.frame_id,
                    self.stamp_ns(message.header), self.get_clock().now().nanoseconds)
            except ProjectionError as error:
                self.publish_status(status='rejected_point', accepted=False, reason=error.code)
                return
            output = PointStamped()
            output.header.stamp = message.header.stamp
            output.header.frame_id = self.projector.kinematics.base_frame
            output.point.x, output.point.y, output.point.z = map(float, point)
            self.output.publish(output)
            self.publish_status(**status)

        def on_liveness(self):
            self.publish_status(**self.projector.liveness_status(self.get_clock().now().nanoseconds))

    rclpy.init(args=args)
    node = None
    try:
        node = WorldProjectionNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
