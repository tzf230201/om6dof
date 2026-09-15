#!/usr/bin/env python3
"""Offline ROS integration: actual BoxerPickNode preview, no motor controllers.

Run after sourcing Humble and the workspace:
  python3 test/boxer_pick_preview_smoke.py

Uses a reserved local ROS domain and rejecting fake arm/gripper action servers.
The joint/target fixture is from the 2026-09-15 pregrasp diagnosis. It checks
planning and request wiring, not physical grasp success or camera inference.
"""

import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import threading
import time
import xml.etree.ElementTree as ET

import yaml


CAPTURED_JOINTS = {
    'joint1': -0.04141748127311917, 'joint2': -2.0018449281909687,
    'joint3': 1.4127963056424688, 'joint4': -0.07516505860660327,
    'joint5': 0.645805911699648, 'joint6': 0.07209709703041822,
    'gripper_left_joint': 0.01896624547196,
}
CAPTURED_CENTER = [0.2911944091320038, -0.03090064819843974, 0.11449165642261505]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--domain-id', type=int, default=221)
    parser.add_argument('--output-dir')
    parser.add_argument('--start-joints-json', help='Optional ROS JointState JSON fixture')
    args = parser.parse_args()
    if not 200 <= args.domain_id <= 232:
        raise ValueError('Use a reserved test domain 200..232, never the live robot domain')
    os.environ.update(ROS_DOMAIN_ID=str(args.domain_id), ROS_LOCALHOST_ONLY='1',
                      RMW_IMPLEMENTATION='rmw_fastrtps_cpp')
    from ament_index_python.packages import get_package_share_directory
    from moveit_configs_utils import MoveItConfigsBuilder
    import rclpy
    from rclpy.action import ActionServer, GoalResponse
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
    from control_msgs.action import FollowJointTrajectory, GripperCommand
    from sensor_msgs.msg import JointState
    from std_msgs.msg import String
    from std_srvs.srv import Trigger
    from om6dof_pick_and_place.boxer_pick_node import BoxerPickNode

    output = Path(args.output_dir or tempfile.mkdtemp(prefix='boxer_pick_preview_smoke_'))
    output.mkdir(parents=True, exist_ok=True)
    description = Path(get_package_share_directory('om6dof_description'))
    config = (MoveItConfigsBuilder('om6dof_v2', package_name='om6dof_moveit_config')
              .robot_description(file_path=str(description / 'urdf/om6dof_v2.urdf.xacro'))
              .robot_description_semantic(file_path='config/om6dof.srdf')
              .robot_description_kinematics(file_path='config/kinematics.yaml')
              .joint_limits(file_path='config/joint_limits.yaml')
              .planning_pipelines(default_planning_pipeline='ompl', pipelines=['ompl'])
              .to_moveit_configs())
    urdf = config.robot_description['robot_description']
    model = ET.fromstring(urdf)
    semantic = ET.fromstring(config.robot_description_semantic['robot_description_semantic'])
    semantic.set('name', model.attrib['name'])
    links = {link.attrib['name'] for link in model.findall('link')}
    for pair in list(semantic.findall('disable_collisions')):
        if pair.get('link1') not in links or pair.get('link2') not in links or pair.get('reason') == 'Never':
            semantic.remove(pair)
    if not any({p.get('link1'), p.get('link2')} == {'link2', 'link6'}
               for p in semantic.findall('disable_collisions')):
        ET.SubElement(semantic, 'disable_collisions', link1='link2', link2='link6', reason='User')
    config.robot_description_semantic['robot_description_semantic'] = ET.tostring(semantic, encoding='unicode')
    moveit_parameters = config.to_dict()
    moveit_parameters.update(allow_trajectory_execution=False, publish_robot_description_semantic=True,
                             moveit_manage_controllers=False,
                             disable_capabilities='move_group/MoveGroupExecuteTrajectoryAction move_group/MoveGroupMoveAction')
    moveit_yaml = output / 'move_group.yaml'
    moveit_yaml.write_text(yaml.safe_dump({'/boxer_pick/move_group': {'ros__parameters': moveit_parameters}}))
    rsp_yaml = output / 'rsp.yaml'
    rsp_yaml.write_text(yaml.safe_dump({'/robot_state_publisher': {'ros__parameters': {'robot_description': urdf}}}))
    node_yaml = output / 'coordinator.yaml'
    node_yaml.write_text(yaml.safe_dump({'/boxer_center_pick': {'ros__parameters': {
        'robot_description': urdf, 'camera_serial': 'smoke_camera',
        'camera_calibration_sha256': 'smoke_verified_calibration', 'execution_enabled': False,
        'arm_action_name': '/boxer_pick_smoke/arm', 'gripper_action_name': '/boxer_pick_smoke/gripper',
    }}}))
    children, handles, executor, nodes, spin = [], [], None, [], None
    result = {'valid': False, 'physical_execution': False, 'domain_id': args.domain_id,
              'output_directory': str(output)}
    counts = {'arm_goals': 0, 'gripper_goals': 0}
    try:
        for name, command in [
            ('rsp', ['/opt/ros/humble/lib/robot_state_publisher/robot_state_publisher', '--ros-args',
                     '--params-file', str(rsp_yaml), '-r', '/tf:=/boxer/tf', '-r', '/tf_static:=/boxer/tf_static']),
            ('move_group', ['/opt/ros/humble/lib/moveit_ros_move_group/move_group', '--ros-args',
                            '-r', '__ns:=/boxer_pick', '--params-file', str(moveit_yaml),
                            '-r', '/tf:=/boxer/tf', '-r', '/tf_static:=/boxer/tf_static',
                            '-r', 'joint_states:=/joint_states']),
        ]:
            log = open(output / f'{name}.log', 'w')
            handles.append(log)
            children.append((name, subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT)))
        rclpy.init(args=['--ros-args', '--params-file', str(node_yaml)])
        inputs = Node('boxer_pick_smoke_inputs')
        coordinator = BoxerPickNode()
        nodes.extend([inputs, coordinator])
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        joint_pub = inputs.create_publisher(JointState, '/joint_states', 10)
        box_pub = inputs.create_publisher(String, '/boxer3d/detections', latched)
        statuses = []
        status_subscription = inputs.create_subscription(
            String, '/boxer_pick/status', lambda msg: statuses.append(json.loads(msg.data)), latched)
        def reject_arm(_goal):
            counts['arm_goals'] += 1
            return GoalResponse.REJECT
        def reject_gripper(_goal):
            counts['gripper_goals'] += 1
            return GoalResponse.REJECT
        arm_server = ActionServer(inputs, FollowJointTrajectory, '/boxer_pick_smoke/arm',
                                  execute_callback=lambda _: FollowJointTrajectory.Result(), goal_callback=reject_arm)
        grip_server = ActionServer(inputs, GripperCommand, '/boxer_pick_smoke/gripper',
                                   execute_callback=lambda _: GripperCommand.Result(), goal_callback=reject_gripper)
        positions = CAPTURED_JOINTS
        if args.start_joints_json:
            data = json.loads(Path(args.start_joints_json).read_text())
            positions = dict(zip(data['name'], data['position']))
        measured = JointState(name=list(positions), position=[float(v) for v in positions.values()])
        payload = {'schema': 'om6dof.boxer3d_detections.v1', 'frame_id': 'world',
                   'calibration_verified': True, 'calibration_sha256': 'smoke_verified_calibration',
                   'camera_serial': 'smoke_camera', 'boxes': [
                       {'id': 'target_1', 'label': 'bottle', 'score': .95, 'inside_point_count': 1500,
                        'center': CAPTURED_CENTER, 'size': [.06, .06, .15],
                        'quaternion_xyzw': [0., 0., 0., 1.]}]}
        def publish():
            now = inputs.get_clock().now()
            measured.header.stamp = now.to_msg()
            joint_pub.publish(measured)
            payload.update(source_stamp_ns=now.nanoseconds, result_stamp_ns=now.nanoseconds)
            box_pub.publish(String(data=json.dumps(payload)))
        timer = inputs.create_timer(.05, publish)
        executor = MultiThreadedExecutor(num_threads=4)
        for node in nodes:
            executor.add_node(node)
        spin = threading.Thread(target=executor.spin, daemon=True)
        spin.start()
        time.sleep(7.0)
        preview_client = inputs.create_client(Trigger, '/boxer_pick/preview')
        execute_client = inputs.create_client(Trigger, '/boxer_pick/execute')
        assert preview_client.wait_for_service(timeout_sec=5.0), 'preview service missing'
        def await_future(future, seconds=5.0):
            deadline = time.monotonic() + seconds
            while not future.done() and time.monotonic() < deadline:
                time.sleep(.02)
            assert future.done(), 'service response timed out'
            return future.result()
        started = time.monotonic()
        preview = await_future(preview_client.call_async(Trigger.Request()))
        assert preview.success, preview.message
        deadline = time.monotonic() + 90.0
        while time.monotonic() < deadline:
            if (statuses and not statuses[-1]['busy']
                    and statuses[-1]['state'] in ('preview_ready', 'planning_failed')):
                break
            time.sleep(.05)
        assert statuses and statuses[-1]['state'] == 'preview_ready', statuses[-1] if statuses else 'no status'
        assert coordinator._frozen is not None
        motion = coordinator._frozen.motion
        assert len(motion.pregrasp.points) >= 2 and len(motion.insertion.points) >= 2
        assert motion.position_error <= .003
        assert len(motion.display.trajectory) == 2
        assert execute_client.wait_for_service(timeout_sec=2.0)
        execute = await_future(execute_client.call_async(Trigger.Request()))
        assert not execute.success, 'execution must remain disabled'
        time.sleep(.3)
        assert counts == {'arm_goals': 0, 'gripper_goals': 0}, counts
        result.update(valid=True, elapsed=time.monotonic() - started,
                      preview_service_success=preview.success, preview_ready=True,
                      execute_denied=True, pregrasp_points=len(motion.pregrasp.points),
                      insertion_points=len(motion.insertion.points), center_error_m=motion.position_error,
                      status=next(s for s in reversed(statuses) if s['state'] == 'preview_ready'), **counts)
        arm_server.destroy()
        grip_server.destroy()
    except Exception as exc:
        result.update(error=str(exc), **counts)
    finally:
        if executor is not None:
            executor.shutdown(timeout_sec=3.0)
        for node in reversed(nodes):
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        for _, child in children:
            if child.poll() is None:
                child.send_signal(signal.SIGINT)
        for _, child in children:
            try:
                child.wait(timeout=8.0)
            except subprocess.TimeoutExpired:
                child.kill()
                child.wait()
        result['child_exit_codes'] = {name: child.returncode for name, child in children}
        for handle in handles:
            handle.close()
        (output / 'result.json').write_text(json.dumps(result, indent=2))
        print(json.dumps(result, indent=2), flush=True)
    return 0 if result['valid'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
