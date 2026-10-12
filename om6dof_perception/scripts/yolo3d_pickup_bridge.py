#!/usr/bin/env python3
"""Bridge yolo_3d_box_node's fast camera-frame detections into boxer_pick_node.

yolo_3d_box_node (om6dof_dd_gng) already detects objects and their 3D centre
at ~30 fps -- axis-aligned depth segmentation, no neural 3D-box regressor, no
yaw estimate. boxer_pick_node.py's PickupPlanner only ever needs a target
CENTRE point (it searches its own yaw/roll grasp candidates via IK); a box's
orientation is used solely for collision-avoidance shape, so the missing yaw
estimate costs nothing for picking, only a slightly looser collision box.

This node does exactly one thing: read /om6dof_yolo3d/objects (camera-optical
frame, metres), transform each detection's centre into the world frame using
the robot's own TF tree -- the D435 is a link in the URDF, so TF already
carries world <- d435_color_optical_frame -- and republish it on
/boxer3d/detections in the schema boxer_pick_node.py already validates
(parse_boxes()). boxer_pick_node itself is untouched -- it does not know or
care which detector fed it.
"""
import json
import math
import time

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

from boxer_pick_geometry import (
    load_pick_calibration, rigid_transform, rotation_quaternion,
)


class Yolo3DPickupBridge(Node):
    def __init__(self):
        super().__init__('yolo3d_pickup_bridge')
        defaults = {
            'objects_topic': '/om6dof_yolo3d/objects',
            'detections_topic': '/boxer3d/detections',
            'world_boxes_topic': '/boxer3d/world_boxes',
            'world_frame': 'world',
            'camera_frame': 'd435_color_optical_frame',
            'camera_serial': '243222076197',
            'camera_calibration_file': '',
            'min_score': 0.3,
            'min_point_count': 30,
            'max_source_age_sec': 1.0,
            'tf_timeout_sec': 0.4,
            'process_rate_hz': 10.0,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.calibration = load_pick_calibration(
            self.get_parameter('camera_calibration_file').value,
            self.get_parameter('camera_serial').value)
        self.camera_serial = str(self.get_parameter('camera_serial').value)
        self.world_frame = self.get_parameter('world_frame').value
        self.camera_frame = self.get_parameter('camera_frame').value
        self.min_score = float(self.get_parameter('min_score').value)
        self.min_point_count = int(self.get_parameter('min_point_count').value)
        self.max_source_age_sec = float(self.get_parameter('max_source_age_sec').value)
        self.tf_timeout_sec = float(self.get_parameter('tf_timeout_sec').value)
        process_rate_hz = float(self.get_parameter('process_rate_hz').value)
        self.min_process_interval_sec = 1.0 / process_rate_hz if process_rate_hz > 0 else 0.0
        self.last_processed_monotonic = 0.0

        callback_group = ReentrantCallbackGroup()
        # A plain rclpy TransformListener on the robot's own /tf, /tf_static --
        # unlike the Boxer3D pipeline's private /boxer/tf remap, this node
        # does not run its own robot_state_publisher; it reads the real
        # controller stack's TF directly.
        # spin_thread=True is required here: _on_objects blocks its executor
        # thread for up to tf_timeout_sec on every ~30 fps message. With the
        # default spin_thread=False, TF's own subscription would share the
        # same 2-thread executor and starve behind those blocked callbacks,
        # so the TF buffer falls further and further behind wall-clock time
        # (observed as a growing "extrapolation into the future" gap). A
        # dedicated spin thread keeps /tf processing independent of that.
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=True)

        detections_qos = QoSProfile(
            depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE)
        self.detections_pub = self.create_publisher(
            String, self.get_parameter('detections_topic').value, detections_qos)
        self.world_box_pub = self.create_publisher(
            MarkerArray, self.get_parameter('world_boxes_topic').value, detections_qos)
        # Matches yolo_3d_box_node's own publisher QoS (create_publisher<String>(topic, 2),
        # i.e. reliable/volatile/depth 2) so this subscription never silently drops frames.
        objects_qos = QoSProfile(
            depth=2, durability=DurabilityPolicy.VOLATILE,
            reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(
            String, self.get_parameter('objects_topic').value, self._on_objects,
            objects_qos, callback_group=callback_group)
        self.get_logger().info(
            f"yolo3d_pickup_bridge ready: {self.get_parameter('objects_topic').value} "
            f"-> {self.get_parameter('detections_topic').value} (world frame)")

    def _world_camera_transform(self, stamp_ns):
        """world <- colour-optical, straight from the robot's own TF tree.

        The camera is a link in the URDF, so robot_state_publisher already
        produces this transform from CAD-derived mounting geometry. The older
        route -- world <- end_effector_link composed with the
        d435_hand_eye.yaml calibration -- disagrees with it by 31 mm and
        7.6 deg on this robot, far worse than the 3.2 deg that calibration
        reports for itself (9 samples), which moves a target at 0.26 m by
        ~19 mm and one at 0.6 m by ~45 mm. The URDF mount is deterministic,
        so it is the one trusted here; the calibration file is still loaded,
        but only for the serial/sha256 handshake boxer_pick_node validates.

        Looked up at the detection's own capture stamp, not "now": if the arm
        is moving, the latest TF would attribute the object to wherever the
        camera is NOW rather than where it was when it saw the object -- the
        same reasoning AutomaticOutputs.freeze_pick_transform uses.
        """
        stamp = Time(nanoseconds=int(stamp_ns))
        try:
            # A single call with tf2's own timeout, not a manual sleep-retry
            # loop: with 30 fps source messages, a Python-level busy-retry
            # here (even one sleeping 10ms at a time) monopolises the GIL
            # against the TF listener's own spin thread and every other
            # subscription callback, so the TF buffer falls further and
            # further behind wall-clock time the longer this runs -- observed
            # as a growing "extrapolation into the future" gap that never
            # recovers. tf2's built-in timeout waits on a condition variable
            # instead of busy-polling, so it does not have this effect.
            transform = self.tf_buffer.lookup_transform(
                self.world_frame, self.camera_frame, stamp,
                timeout=Duration(seconds=self.tf_timeout_sec))
        except Exception as error:
            raise ValueError(
                f'{self.world_frame} -> {self.camera_frame} TF unavailable '
                f'at capture time: {error}') from error
        p = transform.transform.translation
        q = transform.transform.rotation
        return rigid_transform([p.x, p.y, p.z], [q.x, q.y, q.z, q.w])

    def _on_objects(self, message):
        # yolo_3d_box_node publishes at ~30 fps; a pick target does not need
        # updating that often, and processing every single frame is what
        # created the GIL-contention/backlog bug above. Throttling here
        # caps this callback's worst-case load independent of TF latency.
        now_monotonic = time.monotonic()
        if now_monotonic - self.last_processed_monotonic < self.min_process_interval_sec:
            return
        self.last_processed_monotonic = now_monotonic
        try:
            payload = json.loads(message.data)
        except (TypeError, ValueError) as error:
            self.get_logger().warn(f'Invalid objects JSON: {error}', throttle_duration_sec=2.0)
            return
        stamp_ns = payload.get('stamp_ns')
        if not isinstance(stamp_ns, int):
            self.get_logger().warn(
                'objects message has no stamp_ns; rebuild yolo_3d_box_node',
                throttle_duration_sec=5.0)
            return
        now_ns = self.get_clock().now().nanoseconds
        age = (now_ns - stamp_ns) * 1e-9
        if not math.isfinite(age) or age < -0.1 or age > self.max_source_age_sec:
            self.get_logger().warn(
                f'Stale detection source ({age:.2f}s old); skipping', throttle_duration_sec=2.0)
            return
        try:
            world_camera = self._world_camera_transform(stamp_ns)
        except Exception as error:
            self.get_logger().warn(str(error), throttle_duration_sec=2.0)
            return
        rotation = world_camera[:3, :3]
        translation = world_camera[:3, 3]
        # The box's own orientation is the camera's own orientation at capture
        # time: this node's boxes are axis-aligned in the camera frame (no
        # per-object yaw estimate), so their world orientation is exactly
        # however the camera itself was tilted, not an arbitrary identity.
        orientation = rotation_quaternion(rotation)

        boxes = []
        for index, item in enumerate(payload.get('objects', [])):
            center = item.get('center')
            size = item.get('size')
            score = item.get('confidence')
            point_count = item.get('point_count', 0)
            if center is None or size is None:
                continue  # box3d: null -- yolo_3d_box_node could not segment depth here
            if not (isinstance(score, (int, float)) and score >= self.min_score):
                continue
            if not (isinstance(point_count, (int, float)) and point_count >= self.min_point_count):
                continue
            center = [float(v) for v in center]
            size = [float(v) for v in size]
            if (len(center) != 3 or len(size) != 3 or
                    not all(math.isfinite(v) for v in center + size) or any(v <= 0 for v in size)):
                continue
            world_center = (rotation @ center + translation).tolist()
            boxes.append({
                'id': str(index), 'label': str(item.get('label', '')), 'score': float(score),
                'yolo_score': float(score), 'center': world_center, 'size': size,
                'quaternion_xyzw': orientation, 'inside_point_count': int(point_count),
            })

        self._publish(boxes, stamp_ns, now_ns)

    def _publish(self, boxes, source_stamp_ns, result_stamp_ns):
        detections = String(data=json.dumps({
            'schema': 'om6dof.boxer3d_detections.v1', 'frame_id': self.world_frame,
            'source_stamp_ns': int(source_stamp_ns), 'result_stamp_ns': int(result_stamp_ns),
            'camera_serial': self.camera_serial, 'calibration_verified': True,
            'calibration_sha256': self.calibration['camera_calibration_sha256'],
            'boxes': boxes, 'reason': '' if boxes else 'no detections above threshold',
        }, allow_nan=False))
        self.detections_pub.publish(detections)

        markers = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        markers.markers.append(clear)
        stamp = Time(nanoseconds=int(source_stamp_ns)).to_msg()
        for box in boxes:
            marker = Marker()
            marker.header.frame_id = self.world_frame
            marker.header.stamp = stamp
            marker.ns, marker.id = 'yolo3d_world_boxes', int(box['id'])
            marker.type, marker.action = Marker.CUBE, Marker.ADD
            (marker.pose.position.x, marker.pose.position.y,
             marker.pose.position.z) = box['center']
            (marker.pose.orientation.x, marker.pose.orientation.y,
             marker.pose.orientation.z, marker.pose.orientation.w) = box['quaternion_xyzw']
            marker.scale.x, marker.scale.y, marker.scale.z = box['size']
            marker.color.r, marker.color.g, marker.color.b, marker.color.a = .2, .8, 1., .3
            marker.lifetime.sec = 1
            markers.markers.append(marker)
        self.world_box_pub.publish(markers)


def main(args=None):
    rclpy.init(args=args)
    node = Yolo3DPickupBridge()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
