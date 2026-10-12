"""Draw each link's weight at its CoM using the description's inertial values."""

import math

from control_msgs.msg import DynamicJointState
from dynamixel_interfaces.msg import DynamixelState
from geometry_msgs.msg import Point
from rcl_interfaces.msg import ParameterDescriptor
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rclpy.time import Time
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener
from urdf_parser_py.urdf import URDF
from visualization_msgs.msg import Marker, MarkerArray

from om6dof_description.vectors import link_masses, vector_endpoints
from om6dof_description.feedback import decode_dynamic_state, motors_from_description, motor_text
from om6dof_description.jacobian_visualizer import JacobianVisualizer


class CogVectors(Node):
    def __init__(self):
        super().__init__('om6dof_dynamic')
        defaults = {
            'robot_description': '',
            'reference_frame': 'world',
            'vector_mode': 'gravity',
            'gravity': [0.0, 0.0, -9.81],
            'force_scale': 0.05,
            'publish_rate': 10.0,
            'shaft_diameter': 0.003,
            'head_diameter': 0.008,
            'head_length': 0.012,
            'color': [1.0, 0.55, 0.05, 1.0],
            'description_topic': '/robot_description',
            'dynamic_state_topic': '/dynamic_joint_states',
            'dxl_state_topic': '/dynamixel_hardware_interface/dxl_state',
            'enable_motor_feedback': False,
            'require_motor_feedback': False,
            'feedback_timeout': 0.5,
            'show_mass_labels': True,
            'feedback_label_scale': 0.012,
            'feedback_origin': [0.30, -0.20, 0.50],
            'feedback_row_spacing': 0.065,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value, ParameterDescriptor(read_only=True))

        description = self.get_parameter('robot_description').value
        self._description = ''
        self._links = []
        self._motors = []
        self._states = {}
        self._state_stamp = None
        self._comm_state = None
        self._frame = self.get_parameter('reference_frame').value
        if not self._frame or self._frame.startswith('/'):
            raise ValueError('reference_frame must be a nonempty TF frame without a leading /')
        self._mode = self.get_parameter('vector_mode').value
        if self._mode not in ('gravity', 'com_offset'):
            raise ValueError('vector_mode must be gravity or com_offset')
        self._gravity = self._array_parameter('gravity', 3)
        self._color = self._array_parameter('color', 4)
        if not all(0.0 <= value <= 1.0 for value in self._color):
            raise ValueError('color values must be between zero and one')
        self._force_scale = self._positive_parameter('force_scale')
        rate = self._positive_parameter('publish_rate')
        self._shaft = self._positive_parameter('shaft_diameter')
        self._head = self._positive_parameter('head_diameter')
        self._head_length = self._positive_parameter('head_length')
        self._feedback_timeout = self._positive_parameter('feedback_timeout')
        self._feedback_scale = self._positive_parameter('feedback_label_scale')
        self._feedback_origin = self._array_parameter('feedback_origin', 3)
        self._feedback_spacing = self._positive_parameter('feedback_row_spacing')
        self._require_feedback = self.get_parameter('require_motor_feedback').value
        self._motor_feedback = self.get_parameter('enable_motor_feedback').value
        self._mass_labels = self.get_parameter('show_mass_labels').value
        self._lifetime = Duration(seconds=max(0.5, 3.0 / rate)).to_msg()

        self._tf = Buffer()
        self._listener = TransformListener(self._tf, self)
        self._publisher = self.create_publisher(MarkerArray, '~/cog_vectors', 1)
        self._mass_publisher = self.create_publisher(MarkerArray, '~/mass_labels', 1)
        self._feedback_publisher = self.create_publisher(MarkerArray, '~/motor_feedback', 1)
        self._jacobian = JacobianVisualizer(self, self._tf, self._frame)
        if self._motor_feedback:
            self.create_subscription(
                DynamicJointState, self.get_parameter('dynamic_state_topic').value,
                self._on_feedback, qos_profile_sensor_data)
            self.create_subscription(
                DynamixelState, self.get_parameter('dxl_state_topic').value,
                self._on_dxl_state, qos_profile_sensor_data)
        if description:
            self._load_description(description)
        else:
            self.create_subscription(
                String, self.get_parameter('description_topic').value,
                lambda message: self._load_description(message.data),
                QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
            self.get_logger().info('Waiting for the running hardware robot_description and TF.')
        self._timer = self.create_timer(1.0 / rate, self._publish)

    def _load_description(self, description):
        if description == self._description:
            return
        robot = URDF.from_xml_string(description)
        links = link_masses(robot)
        if not links:
            raise ValueError('robot_description has no links with positive mass')
        self._links = links
        self._motors = motors_from_description(description, robot)
        self._description = description
        self._jacobian.load(robot)
        # Description updates may change marker count or order.
        for publisher in (self._publisher, self._mass_publisher, self._feedback_publisher):
            publisher.publish(MarkerArray(markers=[Marker(action=Marker.DELETEALL)]))
        self.get_logger().info(
            '%s vectors for %d inertial links (%.4f kg), %d motors, reference frame %s'
            % (self._mode, len(self._links), sum(link.mass for link in self._links),
               len(self._motors), self._frame))

    def _on_feedback(self, message):
        try:
            self._states = decode_dynamic_state(message)
        except ValueError as error:
            self.get_logger().warning(str(error), throttle_duration_sec=5.0)
            return
        self._state_stamp = Time.from_msg(message.header.stamp)

    def _on_dxl_state(self, message):
        self._comm_state = message.comm_state

    def _feedback_fresh(self):
        if self._state_stamp is None:
            return False
        age = (self.get_clock().now() - self._state_stamp).nanoseconds / 1e9
        return 0.0 <= age <= self._feedback_timeout

    def _text_marker(self, frame, namespace, index, xyz, text, scale, color):
        marker = Marker()
        marker.header.frame_id = frame
        marker.frame_locked = True
        marker.ns = namespace
        marker.id = index
        marker.type = Marker.TEXT_VIEW_FACING
        marker.pose.orientation.w = 1.0
        marker.pose.position = Point(x=float(xyz[0]), y=float(xyz[1]), z=float(xyz[2]))
        marker.scale.z = scale
        marker.text = text
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = color
        marker.lifetime = self._lifetime
        return marker

    def _publish_labels(self, fresh):
        masses = MarkerArray()
        if self._mass_labels:
            for index, link in enumerate(self._links):
                masses.markers.append(self._text_marker(
                    link.name, 'mass', index, (link.com[0] + 0.03, link.com[1], link.com[2]),
                    '%.4f' % link.mass, 0.01, (1.0, 1.0, 1.0, 1.0)))
        self._mass_publisher.publish(masses)
        feedback = MarkerArray()
        if self._motor_feedback:
            for index, motor in enumerate(self._motors):
                text = motor_text(motor, self._states) if fresh else motor.joint + '\nNO FRESH MOTOR DATA'
                if self._comm_state is not None and self._comm_state != 0:
                    text += '\nCOMM ERROR %d' % self._comm_state
                good = fresh and self._comm_state in (None, 0)
                feedback.markers.append(self._text_marker(
                    self._frame, 'motor', index,
                    (self._feedback_origin[0], self._feedback_origin[1],
                     self._feedback_origin[2] - index * self._feedback_spacing), text,
                    self._feedback_scale,
                    (0.25, 1.0, 1.0, 1.0) if good else (1.0, 0.25, 0.15, 1.0)))
        self._feedback_publisher.publish(feedback)

    def _array_parameter(self, name, length):
        values = list(self.get_parameter(name).value)
        if len(values) != length or not all(math.isfinite(value) for value in values):
            raise ValueError('%s must contain %d finite values' % (name, length))
        return values

    def _positive_parameter(self, name):
        value = float(self.get_parameter(name).value)
        if not math.isfinite(value) or value <= 0.0:
            raise ValueError('%s must be finite and positive' % name)
        return value

    def _publish(self):
        if not self._links:
            self.get_logger().warning('Waiting for robot_description.', throttle_duration_sec=5.0)
            return
        fresh = self._feedback_fresh()
        self._publish_labels(fresh)
        if self._require_feedback and (
                not fresh or not self._motors or self._comm_state not in (None, 0)):
            # Old arrows expire instead of looking like live motor data.
            self._publisher.publish(MarkerArray())
            self._jacobian.clear()
            self.get_logger().warning(
                'Waiting for fresh dynamic_joint_states motor feedback.', throttle_duration_sec=5.0)
            return
        if fresh:
            self._jacobian.publish(self._states, self._state_stamp, self._lifetime)
        array = MarkerArray()
        stamp = self.get_clock().now().to_msg()
        missing = []
        for index, link in enumerate(self._links):
            try:
                transform = self._tf.lookup_transform(self._frame, link.name, Time()).transform
            except TransformException:
                missing.append(link.name)
                continue
            translation = transform.translation
            rotation = transform.rotation
            start, end = vector_endpoints(
                link, (translation.x, translation.y, translation.z),
                (rotation.x, rotation.y, rotation.z, rotation.w),
                self._gravity, self._force_scale, self._mode)
            length = math.sqrt(sum((end[i] - start[i]) ** 2 for i in range(3)))
            if length < 1e-9:
                continue
            marker = Marker()
            marker.header.frame_id = self._frame
            marker.header.stamp = stamp
            marker.ns = self._mode
            marker.id = index
            marker.type = Marker.ARROW
            marker.action = Marker.ADD
            marker.pose.orientation.w = 1.0
            marker.points = [Point(x=float(p[0]), y=float(p[1]), z=float(p[2]))
                             for p in (start, end)]
            # ARROW with two points uses shaft diameter, head diameter and
            # head length (not xyz dimensions of the full arrow).
            marker.scale.x = self._shaft
            marker.scale.y = self._head
            marker.scale.z = min(self._head_length, length * 0.3)
            marker.color.r, marker.color.g, marker.color.b, marker.color.a = self._color
            marker.lifetime = self._lifetime
            array.markers.append(marker)
        self._publisher.publish(array)
        if missing:
            self.get_logger().warning(
                'Waiting for TF from %s to: %s. Check /joint_states and robot_state_publisher.'
                % (self._frame, ', '.join(missing)), throttle_duration_sec=5.0)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = CogVectors()
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    except RuntimeError:
        # Humble can interrupt take_message while the signal handler closes
        # its context. Preserve errors that occur while ROS is still running.
        if rclpy.ok():
            raise
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
