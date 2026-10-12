#!/usr/bin/env python3
"""Read-only DDS regression: late pickup subscribers receive the frozen scene.

Runs synthetic geometry in a separate ROS domain; opens no camera and creates
no controller clients. Source ROS setup before invoking this standalone smoke.
"""
import argparse
import json
import os
from pathlib import Path
import sys
import time


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--camera-calibration-file', required=True)
    parser.add_argument('--camera-serial', default='243222076197')
    parser.add_argument('--domain-id', type=int, default=230)
    args = parser.parse_args()
    if not 1 <= args.domain_id <= 232:
        parser.error('Use an isolated ROS domain between 1 and 232')
    os.environ['ROS_DOMAIN_ID'] = str(args.domain_id)
    os.environ['ROS_LOCALHOST_ONLY'] = '1'
    os.environ['RMW_IMPLEMENTATION'] = 'rmw_fastrtps_cpp'
    os.environ.setdefault('ROS_LOG_DIR', '/tmp/boxer_pick_dds_smoke_ros_logs')
    sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
    import numpy as np
    import rclpy
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from std_msgs.msg import String
    from visualization_msgs.msg import MarkerArray
    from boxer_pick_geometry import load_pick_calibration, capture_world_camera
    from barath_boxer3d_preview import AutomaticOutputs

    calibration = load_pick_calibration(args.camera_calibration_file, args.camera_serial)
    output = AutomaticOutputs('/boxer3d/rgb/image/compressed', '/boxer3d/yolox/detections',
                              camera_serial=args.camera_serial, calibration=calibration)
    qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)
    try:
        assert output.detection_sub.qos_profile.durability == DurabilityPolicy.VOLATILE
        stamp = output.node.get_clock().now().nanoseconds
        snapshot = {'receipt_ns': stamp, 'R_local_camera': np.eye(3),
                    'T_world_camera': capture_world_camera(np.eye(4), calibration)}
        result = {'boxes': [{'accepted': True, 'label': 'bottle', 'score': .9,
                            'yolo_score': .95, 'translation_local_object_m': [.1, .2, .3],
                            'rotation_local_object': np.eye(3), 'size_m': [.05, .05, .15],
                            'inside_point_count': 400}]}
        # Publish before creating either subscriber: receiving this requires
        # compatible transient-local durability, not merely live delivery.
        output.publish_pick(result, snapshot)

        def late_join(name, expected_count, expected_reason=''):
            received = {}
            reader = rclpy.create_node(name)
            reader.create_subscription(String, '/boxer3d/detections',
                                       lambda m: received.update(data=json.loads(m.data)), qos)
            reader.create_subscription(MarkerArray, '/boxer3d/world_boxes',
                                       lambda m: received.update(markers=m), qos)
            try:
                deadline = time.monotonic()+8.
                while len(received) < 2 and time.monotonic() < deadline:
                    rclpy.spin_once(output.node, timeout_sec=.01)
                    rclpy.spin_once(reader, timeout_sec=.04)
                assert len(received) == 2, f'Late subscriber did not receive both topics: {list(received)}'
                data = received['data']
                assert data['schema'] == 'om6dof.boxer3d_detections.v1'
                assert data['frame_id'] == 'world' and data['calibration_verified'] is True
                assert data['calibration_sha256'] == calibration['camera_calibration_sha256']
                assert data['source_stamp_ns'] == stamp
                assert len(data['boxes']) == expected_count
                assert data['reason'] == expected_reason
                markers = received['markers'].markers
                assert len(markers) == expected_count+1
                assert markers[0].action == 3  # DELETEALL
                if expected_count:
                    assert data['boxes'][0]['label'] == 'bottle'
                    assert markers[1].header.frame_id == 'world'
                    source_stamp = markers[1].header.stamp
                    assert source_stamp.sec*1_000_000_000+source_stamp.nanosec == stamp
            finally:
                reader.destroy_node()

        late_join('boxer_pick_late_scene_reader', 1)
        output.invalidate_pick('Synthetic observation cleared', stamp)
        late_join('boxer_pick_late_empty_reader', 0, 'Synthetic observation cleared')
        print(json.dumps({'result': 'PASS', 'domain_id': args.domain_id,
                          'late_join_scene': True, 'late_join_empty_clears': True,
                          'yolo_subscription_volatile': True,
                          'hardware_actions': False}))
    finally:
        output.close()


if __name__ == '__main__':
    main()
