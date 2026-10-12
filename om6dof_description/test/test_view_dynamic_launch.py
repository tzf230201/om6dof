"""Exercise the real launch, joint sources, TF, URDF and marker topic headlessly."""

import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time

from ament_index_python.packages import get_package_share_directory
from control_msgs.msg import DynamicJointState, InterfaceValue
from geometry_msgs.msg import TwistStamped
import numpy as np
import pytest
import rclpy
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from urdf_parser_py.urdf import URDF
from visualization_msgs.msg import Marker, MarkerArray
import yaml


@pytest.mark.parametrize('launch_name', ['view_dynamic.launch.py', 'view_jacobian.launch.py'])
def test_live_launch_motor_feedback_motion_and_stale_data(monkeypatch, launch_name):
    joint_source = 'external'
    # Keep test TF and joint states isolated from other running robots.
    domain = 100 + os.getpid() % 80
    env = dict(os.environ, ROS_DOMAIN_ID=str(domain), ROS_LOCALHOST_ONLY='1')
    monkeypatch.setenv('ROS_LOCALHOST_ONLY', '1')
    description_share = get_package_share_directory('om6dof_description')
    xml = subprocess.check_output([
        'xacro', str(Path(description_share) / 'urdf' / 'om6dof.urdf.xacro')], env=env, text=True)
    robot = URDF.from_xml_string(xml)
    links = [link for link in robot.links if link.inertial and link.inertial.mass > 0.0]
    movable = [joint.name for joint in robot.joints
               if joint.type in ('revolute', 'continuous', 'prismatic')]
    if joint_source == 'external':
        # Model metadata only: this fixture never loads a hardware plugin or
        # opens a bus. RSP consumes the same URDF as the visualizer.
        count = len(movable)
        matrix = ','.join(str(int(row == column)) for row in range(count)
                          for column in range(count))
        control = ('<ros2_control name="fixture" type="system"><hardware>'
                   '<plugin>dynamixel_hardware_interface/DynamixelHardware</plugin>'
                   '<param name="transmission_to_joint_matrix">' + matrix + '</param>'
                   '</hardware>')
        control += ''.join('<joint name="%s"/>' % name for name in movable)
        control += ''.join('<gpio name="dxl%d"><param name="type">dxl</param>'
                           '<param name="ID">%d</param></gpio>' % (i + 1, i + 31)
                           for i in range(count))
        xml = xml.replace('</robot>', control + '</ros2_control></robot>')
    context = Context()
    rclpy.init(context=context, domain_id=domain)
    node = Node('dynamic_launch_test', context=context)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(node)
    arrays = []
    labels = []
    feedback = []
    jacobian_markers = []
    matrices = []
    twists = []
    node.create_subscription(
        MarkerArray, '/om6dof_dynamic/cog_vectors', arrays.append, 10)
    node.create_subscription(
        MarkerArray, '/om6dof_dynamic/mass_labels', labels.append, 10)
    node.create_subscription(
        MarkerArray, '/om6dof_dynamic/motor_feedback', feedback.append, 10)
    node.create_subscription(
        MarkerArray, '/om6dof_dynamic/jacobian_markers', jacobian_markers.append, 10)
    node.create_subscription(
        Float64MultiArray, '/om6dof_dynamic/jacobian', matrices.append, 10)
    node.create_subscription(
        TwistStamped, '/om6dof_dynamic/ee_twist', twists.append, 10)
    publisher = node.create_publisher(JointState, '/joint_states', 10)
    dynamic_publisher = node.create_publisher(DynamicJointState, '/dynamic_joint_states', 10)

    with tempfile.TemporaryFile(mode='w+') as output, tempfile.TemporaryDirectory() as directory:
        hardware_rsp = None
        if joint_source == 'external':
            params = Path(directory) / 'rsp.yaml'
            params.write_text(yaml.safe_dump({
                'robot_state_publisher': {'ros__parameters': {'robot_description': xml}}}))
            hardware_rsp = subprocess.Popen([
                'ros2', 'run', 'robot_state_publisher', 'robot_state_publisher',
                '--ros-args', '--params-file', str(params)],
                env=env, stdout=output, stderr=subprocess.STDOUT, start_new_session=True)
        process = subprocess.Popen([
            'ros2', 'launch', 'om6dof_description', launch_name,
            'joints:=' + joint_source, 'rviz:=false'],
            env=env, stdout=output, stderr=subprocess.STDOUT, start_new_session=True)
        try:
            def wait_for(predicate, positions, publish=True):
                deadline = time.monotonic() + 20.0
                while time.monotonic() < deadline:
                    if process.poll() is not None:
                        output.seek(0)
                        pytest.fail('launch exited:\n' + output.read())
                    if joint_source == 'external' and publish:
                        message = JointState()
                        message.header.stamp = node.get_clock().now().to_msg()
                        message.name = movable
                        message.position = positions
                        publisher.publish(message)
                        dynamic = DynamicJointState()
                        dynamic.header = message.header
                        dynamic.joint_names = movable + ['dxl%d' % (i + 1) for i in range(len(movable))]
                        dynamic.interface_values = [
                            InterfaceValue(interface_names=['velocity', 'position'],
                                           values=[0.2, position]) for position in positions]
                        dynamic.interface_values.extend([
                            InterfaceValue(
                                interface_names=['Present Current', 'Present Input Voltage',
                                                 'Torque Enable', 'Hardware Error Status'],
                                values=[-3.0, 12.0, 0.0, 0.0]) for _ in movable])
                        dynamic_publisher.publish(dynamic)
                    executor.spin_once(timeout_sec=0.05)
                    if predicate():
                        return
                output.seek(0)
                pytest.fail('timed out waiting for complete markers:\n' + output.read())

            wait_for(lambda: arrays and len(arrays[-1].markers) == len(links)
                     and labels and len(labels[-1].markers) == len(links),
                     [0.0] * len(movable))
            first = arrays[-1]
            assert len(labels[-1].markers) == len(links)
            for marker, link in zip(first.markers, links):
                assert marker.header.frame_id == 'world'
                assert marker.type == Marker.ARROW
                start, end = marker.points
                assert end.x == pytest.approx(start.x)
                assert end.y == pytest.approx(start.y)
                assert end.z - start.z == pytest.approx(-0.05 * link.inertial.mass * 9.81)
                assert marker.lifetime.sec > 0 or marker.lifetime.nanosec > 0

            if joint_source == 'external':
                wait_for(lambda: feedback and len(feedback[-1].markers) == len(movable),
                         [0.0] * len(movable))
                assert 'I=-3.000raw' in feedback[-1].markers[0].text
                assert 'dq=0.200rad/s' in feedback[-1].markers[0].text
                assert 'torque=OFF' in feedback[-1].markers[0].text
                wait_for(lambda: matrices and twists and jacobian_markers
                         and any(marker.ns == 'tcp' for marker in jacobian_markers[-1].markers),
                         [0.0] * len(movable))
                matrix_message = matrices[-1]
                assert [dim.size for dim in matrix_message.layout.dim] == [6, 6]
                assert matrix_message.layout.dim[1].label == ','.join('joint%d' % i for i in range(1, 7))
                matrix = np.array(matrix_message.data).reshape(6, 6)
                expected_twist = matrix @ np.full(6, 0.2)
                linear, angular = twists[-1].twist.linear, twists[-1].twist.angular
                np.testing.assert_allclose([linear.x, linear.y, linear.z], expected_twist[:3], atol=1e-10)
                np.testing.assert_allclose([angular.x, angular.y, angular.z], expected_twist[3:], atol=1e-10)
                tcp = next(marker.pose.position for marker in jacobian_markers[-1].markers if marker.ns == 'tcp')
                for marker in jacobian_markers[-1].markers:
                    if marker.ns in ('linear', 'angular') and marker.action == Marker.ADD:
                        np.testing.assert_allclose(
                            [marker.points[0].x, marker.points[0].y, marker.points[0].z],
                            [tcp.x, tcp.y, tcp.z])
                        assert marker.header.frame_id == 'world'
                first_points = {marker.id: marker.points[0] for marker in first.markers}

                def moved():
                    if not arrays or len(arrays[-1].markers) != len(links):
                        return False
                    return any(
                        abs(marker.points[0].x - first_points[marker.id].x) > 0.001
                        or abs(marker.points[0].y - first_points[marker.id].y) > 0.001
                        or abs(marker.points[0].z - first_points[marker.id].z) > 0.001
                        for marker in arrays[-1].markers)

                pose = [0.0] * len(movable)
                pose[1] = 0.6
                wait_for(moved, pose)
                for marker, link in zip(arrays[-1].markers, links):
                    start, end = marker.points
                    assert end.x == pytest.approx(start.x)
                    assert end.y == pytest.approx(start.y)
                    assert end.z - start.z == pytest.approx(-0.05 * link.inertial.mass * 9.81)
                wait_for(lambda: arrays and not arrays[-1].markers and feedback
                         and 'NO FRESH MOTOR DATA' in feedback[-1].markers[0].text,
                         pose, publish=False)
                wait_for(lambda: jacobian_markers and jacobian_markers[-1].markers
                         and all(marker.action == Marker.DELETEALL
                                 for marker in jacobian_markers[-1].markers),
                         pose, publish=False)
        finally:
            for child in (process, hardware_rsp):
                if child is not None and child.poll() is None:
                    os.killpg(child.pid, signal.SIGINT)
                    try:
                        child.wait(timeout=10)
                    except subprocess.TimeoutExpired:
                        os.killpg(child.pid, signal.SIGKILL)
                        child.wait(timeout=5)
            executor.shutdown()
            node.destroy_node()
            context.shutdown()
