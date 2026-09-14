"""Exercise the installed mapper through real DDS in an isolated ROS domain.

No camera or robot is used: measured pixels and a known right-handed optical
transform check message decoding, exact frame/time attribution and TF gating.
"""
import os
from pathlib import Path
import struct
import subprocess
import time

import numpy as np
import pytest
import rclpy
from geometry_msgs.msg import PointStamped, TransformStamped
from rclpy.parameter import Parameter
from rclpy.executors import SingleThreadedExecutor
from rcl_interfaces.srv import GetParameters, SetParameters
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, PointCloud2
from std_srvs.srv import Trigger
from tf2_ros import StaticTransformBroadcaster
from visualization_msgs.msg import MarkerArray


@pytest.mark.parametrize("algorithm", ["fgng", "ddgng", "dblgng"])
def test_world_cloud_requires_tf_and_preserves_right_handed_geometry(tmp_path, monkeypatch, algorithm):
    executable = os.environ.get('F_GNG_NODE')
    if not executable:
        pytest.skip('F_GNG_NODE must point to the compiled ROS mapper')
    assert Path(executable).is_file(), 'F_GNG_NODE is not an existing executable'
    env = dict(os.environ, ROS_DOMAIN_ID=str(120 + os.getpid() % 70),
               ROS_LOCALHOST_ONLY='1')
    monkeypatch.setenv('ROS_LOCALHOST_ONLY', '1')
    context = rclpy.context.Context()
    rclpy.init(context=context, domain_id=int(env['ROS_DOMAIN_ID']))
    node = rclpy.create_node('fgng_pipeline_test', context=context,
                            use_global_arguments=False)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(node)
    log = (tmp_path / 'mapper.log').open('w+')
    process = subprocess.Popen([
        executable, '--ros-args', '-r', '__ns:=/fgng_test',
        '-p', 'algorithm:=' + algorithm,
        '-p', 'world_frame:=test_world', '-p', 'camera_frame:=test_optical',
        '-p', 'require_joint_states:=false', '-p', 'max_nodes:=64',
        '-p', 'memory_capacity:=256', '-p', 'replay_budget:=128',
        '-p', 'input_budget:=128', '-p', 'max_pose_age:=0.5',
    ], env=env, stdout=log, stderr=subprocess.STDOUT)
    try:
        cloud_messages, graph_messages, focus_messages = [], [], []
        node.create_subscription(PointCloud2, '/fgng_test/points',
                                 cloud_messages.append, qos_profile_sensor_data)
        node.create_subscription(MarkerArray, '/fgng_test/graph',
                                 graph_messages.append, 10)
        node.create_subscription(MarkerArray, '/fgng_test/focus',
                                 focus_messages.append, 10)
        images = node.create_publisher(Image, '/fgng_test/depth/image_raw',
                                       qos_profile_sensor_data)
        infos = node.create_publisher(CameraInfo, '/fgng_test/depth/camera_info',
                                      qos_profile_sensor_data)
        clicked_points = node.create_publisher(PointStamped, '/clicked_point', 10)
        submitted_stamps = set()

        def spin_for(duration, emit=None):
            deadline = time.monotonic() + duration
            next_emit = 0.0
            while time.monotonic() < deadline:
                assert process.poll() is None, 'Mapper exited: ' + read_log()
                if emit and time.monotonic() >= next_emit:
                    emit()
                    next_emit = time.monotonic() + .075
                executor.spin_once(timeout_sec=.015)

        def read_log():
            log.flush()
            return (tmp_path / 'mapper.log').read_text()

        def send(encoding='32FC1', stale=False, bigendian=False, empty=False):
            stamp = node.get_clock().now().to_msg()
            if stale:
                stamp.sec -= 2
            submitted_stamps.add((stamp.sec, stamp.nanosec))
            info = CameraInfo()
            info.header.frame_id = 'test_optical'
            info.header.stamp = stamp
            info.width = info.height = 3
            info.k = [2., 0., 1., 0., 2., 1., 0., 0., 1.]
            info.p = [2., 0., 1., 0., 0., 2., 1., 0., 0., 0., 1., 0.]
            info.distortion_model = 'plumb_bob'
            info.d = [0.] * 5
            msg = Image()
            msg.header = info.header
            msg.width = msg.height = 3
            msg.encoding = encoding
            msg.is_bigendian = bigendian
            # Nontrivial row stride verifies that padding is never read as depth.
            byte_order = '>' if bigendian else '<'
            if encoding == '32FC1':
                values = (0., float('nan'), float('inf')) if empty else (1., 1., 1.)
                row = struct.pack(byte_order + '3f', *values)
            else:
                values = (0, 65535, 0) if empty else (1000, 1000, 1000)
                row = struct.pack(byte_order + '3H', *values)
            row += b'\x00\x00\x00\x00'
            msg.step = len(row)
            msg.data = row * 3
            infos.publish(info)
            images.publish(msg)

        spin_for(1.6, send)
        assert not cloud_messages, 'Mapping must wait for the camera TF'

        broadcaster = StaticTransformBroadcaster(node)
        transform = TransformStamped()
        transform.header.frame_id = 'test_world'
        transform.child_frame_id = 'test_optical'
        transform.header.stamp = node.get_clock().now().to_msg()
        transform.transform.translation.x = .25
        transform.transform.translation.y = .10
        transform.transform.translation.z = .20
        # Optical right/down/forward -> world forward/left/up; determinant +1.
        transform.transform.rotation.x = -.5
        transform.transform.rotation.y = .5
        transform.transform.rotation.z = -.5
        transform.transform.rotation.w = .5
        broadcaster.sendTransform(transform)
        spin_for(2., send)
        assert cloud_messages, 'No transformed cloud: ' + read_log()
        assert graph_messages, 'No graph markers: ' + read_log()

        def check_cloud(cloud):
            assert cloud.header.frame_id == 'test_world'
            assert (cloud.header.stamp.sec, cloud.header.stamp.nanosec) in submitted_stamps
            offsets = {field.name: field.offset for field in cloud.fields}
            xyz = np.array([
                [struct.unpack_from('<f', cloud.data, i * cloud.point_step + offsets[key])[0]
                 for key in ('x', 'y', 'z')]
                for i in range(cloud.width * cloud.height)
            ])
            expected = np.array([[1.25, .1 - (u - 1) / 2, .2 - (v - 1) / 2]
                                 for v in range(3) for u in range(3)])
            assert xyz.shape == (9, 3)
            # Compare sets: message point order is not an API requirement.
            distances = np.linalg.norm(xyz[:, None] - expected[None, :], axis=-1)
            assert np.max(np.min(distances, axis=0)) < 1e-6

        check_cloud(cloud_messages[-1])
        before = len(cloud_messages)
        spin_for(.6, lambda: send('16UC1'))
        assert len(cloud_messages) > before
        check_cloud(cloud_messages[-1])

        for encoding in ('32FC1', '16UC1'):
            before = len(cloud_messages)
            spin_for(.45, lambda: send(encoding, bigendian=True))
            assert len(cloud_messages) > before, f'No {encoding} big-endian cloud'
            check_cloud(cloud_messages[-1])

        # RViz's Publish Point must select a literal world location; selecting a
        # target must not mirror Y or reinterpret it as an optical coordinate.
        target = (1.25, .10, .20)

        def send_with_target():
            send()
            clicked = PointStamped()
            clicked.header.frame_id = 'test_world'
            clicked.header.stamp = node.get_clock().now().to_msg()
            clicked.point.x, clicked.point.y, clicked.point.z = target
            clicked_points.publish(clicked)

        spin_for(.65, send_with_target)
        if algorithm == "fgng":
            selected = [marker for message in focus_messages[-5:] for marker in message.markers
                        if marker.ns == 'world_target' and marker.action == marker.ADD]
            assert selected, 'Publish Point did not create the selected world target marker'
            selected_position = selected[-1].pose.position
            assert selected[-1].header.frame_id == 'test_world'
            assert np.allclose([selected_position.x, selected_position.y, selected_position.z],
                               target, atol=1e-7, rtol=0)
            get_parameters = node.create_client(GetParameters, '/fgng_test/f_gng/get_parameters')
            assert get_parameters.wait_for_service(timeout_sec=2.)
            get_request = GetParameters.Request()
            get_request.names = ['focus_mode', 'world_target_x', 'world_target_y', 'world_target_z']
            future = get_parameters.call_async(get_request)
            executor.spin_until_future_complete(future, timeout_sec=2.)
            assert future.done() and future.result() is not None
            values = future.result().values
            assert values[0].string_value == 'world_lock'
            assert np.allclose([value.double_value for value in values[1:]], target,
                               atol=1e-7, rtol=0)
        else:
            assert focus_messages, 'Baseline camera frustum is missing'
            assert any(marker.ns == 'camera' and marker.action == marker.ADD
                       for marker in focus_messages[-1].markers)
            assert not any(marker.ns in ('gaze', 'world_target', 'accommodation')
                           and marker.action == marker.ADD
                           for message in focus_messages for marker in message.markers), \
                'Baseline must not pretend to apply foveation'

        def graph_geometry(message):
            return {marker.ns: [(point.x, point.y, point.z) for point in marker.points]
                    for marker in message.markers if marker.action == marker.ADD}

        # Missing depth must emit an empty measured cloud and retain learned
        # world geometry. This also covers the empty PointCloud2 iterator path.
        spin_for(.6)  # Drain valid frames before taking the retained snapshot.
        retained = graph_geometry(graph_messages[-1])
        for encoding in ('32FC1', '16UC1'):
            before = len(cloud_messages)
            spin_for(.45, lambda: send(encoding, empty=True))
            assert len(cloud_messages) > before, 'Missing depth must still produce a frame snapshot'
            assert cloud_messages[-1].width * cloud_messages[-1].height == 0
            assert not cloud_messages[-1].data
            assert graph_geometry(graph_messages[-1]) == retained

        parameters = node.create_client(SetParameters, '/fgng_test/f_gng/set_parameters')
        assert parameters.wait_for_service(timeout_sec=2.)
        request = SetParameters.Request()
        request.parameters = [Parameter('focus_distance', value=-1.).to_parameter_msg()]
        future = parameters.call_async(request)
        executor.spin_until_future_complete(future, timeout_sec=2.)
        assert future.done() and not future.result().results[0].successful

        spin_for(.8)  # Drain pending valid measurements before testing staleness.
        before = len(cloud_messages)
        spin_for(.7, lambda: send(stale=True))
        assert len(cloud_messages) == before, 'Stale images must not enter world memory'

        reset = node.create_client(Trigger, '/fgng_test/reset')
        assert reset.wait_for_service(timeout_sec=2.)
        future = reset.call_async(Trigger.Request())
        executor.spin_until_future_complete(future, timeout_sec=2.)
        assert future.done() and future.result().success
        spin_for(.2)
        assert any(marker.action == marker.DELETEALL
                   for message in graph_messages[-3:] for marker in message.markers)
    finally:
        process.terminate()
        try:
            process.wait(timeout=5.)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait(timeout=2.)
        log.close()
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown(context=context)
