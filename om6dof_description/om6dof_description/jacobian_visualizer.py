"""RViz markers and numeric Jacobian using the viewer's measured motor samples."""

import json
import math

from geometry_msgs.msg import Point, TwistStamped
import numpy as np
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.time import Time
from std_msgs.msg import Float64MultiArray, MultiArrayDimension, String
from tf2_ros import TransformException
from visualization_msgs.msg import Marker, MarkerArray

from om6dof_description.jacobian import GeometricJacobian, ellipsoid_segments
from om6dof_description.vectors import transform_point


COLORS = (
    (1.0, 0.2, 0.2), (0.2, 1.0, 0.2), (0.3, 0.55, 1.0),
    (1.0, 0.25, 1.0), (1.0, 0.9, 0.1), (0.1, 1.0, 1.0),
)


class JacobianVisualizer:
    def __init__(self, node, tf_buffer, frame):
        self.node, self.tf, self.frame = node, tf_buffer, frame
        defaults = {
            'show_jacobian': True,
            'tip_link': 'end_effector_link',
            'jacobian_scale': 0.4,
            'angular_jacobian_scale': 0.07,
            'ellipsoid_scale': 0.4,
            'show_ellipsoid': True,
            'ee_velocity_scale': 0.5,
        }
        for name, value in defaults.items():
            node.declare_parameter(name, value, ParameterDescriptor(read_only=True))
        self.enabled = node.get_parameter('show_jacobian').value
        self.tip = node.get_parameter('tip_link').value
        self.show_ellipsoid = node.get_parameter('show_ellipsoid').value
        self.scales = {}
        for name in ('jacobian_scale', 'angular_jacobian_scale', 'ellipsoid_scale', 'ee_velocity_scale'):
            value = float(node.get_parameter(name).value)
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(name + ' must be finite and positive')
            self.scales[name] = value
        self.markers = node.create_publisher(MarkerArray, '~/jacobian_markers', 1)
        self.matrix = node.create_publisher(Float64MultiArray, '~/jacobian', 1)
        self.info = node.create_publisher(String, '~/jacobian_info', 1)
        self.twist = node.create_publisher(TwistStamped, '~/ee_twist', 1)
        self.model = None

    def load(self, robot):
        if not self.enabled:
            return
        self.model = GeometricJacobian(robot, self.tip)
        self.clear()
        self.node.get_logger().info(
            'Jacobian TCP %s, root %s, columns %s'
            % (self.tip, self.model.root, ', '.join(self.model.names)))

    def clear(self):
        self.markers.publish(MarkerArray(markers=[Marker(action=Marker.DELETEALL)]))

    def _marker(self, namespace, index, kind, stamp, lifetime, color):
        marker = Marker()
        marker.header.frame_id = self.frame
        marker.header.stamp = stamp
        marker.ns, marker.id, marker.type = namespace, index, kind
        marker.pose.orientation.w = 1.0
        marker.color.r, marker.color.g, marker.color.b = map(float, color)
        marker.color.a = 1.0
        marker.lifetime = lifetime
        return marker

    @staticmethod
    def _point(value):
        return Point(x=float(value[0]), y=float(value[1]), z=float(value[2]))

    def _arrow(self, namespace, index, center, vector, stamp, lifetime, color, width=0.003):
        length = np.linalg.norm(vector)
        marker = self._marker(namespace, index, Marker.ARROW, stamp, lifetime, color)
        if length < 1e-10:
            # A previous nonzero column/velocity must disappear immediately.
            marker.action = Marker.DELETE
            return marker
        marker.points = [self._point(center), self._point(center + vector)]
        marker.scale.x, marker.scale.y = width, width * 2.5
        marker.scale.z = min(0.012, length * 0.3)
        return marker

    def publish(self, states, sample_stamp, lifetime):
        if not self.enabled or self.model is None:
            return
        try:
            positions = {name: states.get(name, {}).get('position', float('nan'))
                         for name in self.model.names}
            root_transform = np.eye(4)
            if self.model.root != self.frame:
                tf = self.tf.lookup_transform(
                    self.frame, self.model.root, sample_stamp).transform
                t, q = tf.translation, tf.rotation
                translation, quaternion = (t.x, t.y, t.z), (q.x, q.y, q.z, q.w)
                root_transform[:3, 3] = translation
                for column in range(3):
                    basis = np.eye(3)[:, column]
                    root_transform[:3, column] = transform_point(basis, (0, 0, 0), quaternion)
            pose, jacobian = self.model.evaluate(positions, root_transform)
        except (ValueError, TransformException) as error:
            self.clear()
            self.node.get_logger().warning('Jacobian: ' + str(error), throttle_duration_sec=5.0)
            return
        center = pose[:3, 3]
        stamp = sample_stamp.to_msg()
        array = MarkerArray()
        point = self._marker('tcp', 0, Marker.SPHERE, stamp, lifetime, (1.0, 1.0, 1.0))
        point.pose.position = self._point(center)
        point.scale.x = point.scale.y = point.scale.z = 0.008
        array.markers.append(point)
        for index, name in enumerate(self.model.names):
            color = COLORS[index % len(COLORS)]
            for namespace, vector, scale in (
                    ('linear', jacobian[:3, index], self.scales['jacobian_scale']),
                    ('angular', jacobian[3:, index], self.scales['angular_jacobian_scale'])):
                array.markers.append(self._arrow(
                    namespace, index, center, vector * scale, stamp, lifetime, color))
            endpoint = center + jacobian[:3, index] * self.scales['jacobian_scale']
            if np.linalg.norm(jacobian[:3, index]) > 1e-10:
                label = self._marker('joint_labels', index, Marker.TEXT_VIEW_FACING,
                                     stamp, lifetime, color)
                label.pose.position = self._point(endpoint + np.array([0, 0, 0.012]))
                label.scale.z = 0.012
                label.text = name
            else:
                label = self._marker('joint_labels', index, Marker.TEXT_VIEW_FACING,
                                     stamp, lifetime, color)
                label.action = Marker.DELETE
            array.markers.append(label)
        segments, singular_values = ellipsoid_segments(center, jacobian[:3], self.scales['ellipsoid_scale'])
        if self.show_ellipsoid:
            ellipsoid = self._marker('ellipsoid', 0, Marker.LINE_LIST,
                                     stamp, lifetime, (0.65, 0.85, 1.0))
            ellipsoid.scale.x = 0.0015
            ellipsoid.color.a = 0.7
            ellipsoid.points = [self._point(point) for point in segments]
            array.markers.append(ellipsoid)
        velocities = [states.get(name, {}).get('velocity', float('nan')) for name in self.model.names]
        measured_velocity = all(math.isfinite(value) for value in velocities)
        velocity = jacobian @ np.array(velocities) if measured_velocity else None
        array.markers.append(self._arrow(
            'ee_velocity', 0, center,
            velocity[:3] * self.scales['ee_velocity_scale'] if measured_velocity else np.zeros(3),
            stamp, lifetime, (1.0, 1.0, 1.0), width=0.004))
        label = self._marker('info', 0, Marker.TEXT_VIEW_FACING, stamp, lifetime, (1, 1, 1))
        label.pose.position = self._point(center + np.array([0, 0, 0.055]))
        label.scale.z = 0.012
        label.text = 'TCP:%s\nJv_sigma:%s' % (
            self.tip, ','.join('%.3f' % value for value in singular_values))
        array.markers.append(label)
        self.markers.publish(array)
        message = Float64MultiArray()
        count = len(self.model.names)
        message.layout.dim = [
            MultiArrayDimension(label='vx,vy,vz,wx,wy,wz@' + self.frame, size=6, stride=6 * count),
            MultiArrayDimension(label=','.join(self.model.names), size=count, stride=count),
        ]
        message.data = jacobian.ravel().tolist()
        self.matrix.publish(message)
        self.info.publish(String(data=json.dumps({
            'frame': self.frame, 'tip_link': self.tip, 'joints': self.model.names,
            'stamp_sec': sample_stamp.nanoseconds / 1e9,
            'singular_values_linear': singular_values.tolist(),
            'linear_rank': int(np.linalg.matrix_rank(jacobian[:3])),
            'linear_zero_columns': [name for index, name in enumerate(self.model.names)
                                    if np.linalg.norm(jacobian[:3, index]) < 1e-10],
            'measured_joint_velocity_available': measured_velocity,
        })))
        if measured_velocity:
            twist = TwistStamped()
            twist.header.frame_id, twist.header.stamp = self.frame, stamp
            twist.twist.linear.x, twist.twist.linear.y, twist.twist.linear.z = velocity[:3].tolist()
            twist.twist.angular.x, twist.twist.angular.y, twist.twist.angular.z = velocity[3:].tolist()
            self.twist.publish(twist)
