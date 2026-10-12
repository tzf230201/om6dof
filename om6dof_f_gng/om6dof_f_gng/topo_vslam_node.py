"""Depth-to-world point cloud for om6dof_topo_vslam.

Stage 1 of the topological VSLAM front end: no GNG, no map. It only turns the
RealSense depth image into world-frame XYZ points using the robot TF tree, so
RViz can show the arm and what its wrist camera currently sees.
"""
from __future__ import annotations

import numpy as np
import rclpy
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image, PointCloud2, PointField
from tf2_ros import Buffer, TransformListener, TransformException


def depth_metres(image: Image) -> np.ndarray:
    """Depth as optical Z in metres; 16UC1 is millimetres by ROS convention."""
    if image.encoding == '32FC1':
        depth = np.frombuffer(image.data, dtype='<f4')
    elif image.encoding == '16UC1':
        depth = np.frombuffer(image.data, dtype='<u2').astype(np.float32) * 1e-3
    else:
        raise ValueError('Unsupported depth encoding ' + image.encoding)
    if depth.size != image.height * image.width:
        raise ValueError('Depth payload does not match the declared image size')
    return depth.reshape(image.height, image.width)


def rotation_matrix(q) -> np.ndarray:
    """Rotation of a geometry_msgs quaternion, without tf_transformations."""
    x, y, z, w = q.x, q.y, q.z, q.w
    norm = np.sqrt(x * x + y * y + z * z + w * w)
    if norm <= 0.:
        raise ValueError('A transform carried a zero-length quaternion')
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ], dtype=np.float64)


def height_colours(z: np.ndarray, low: float, high: float) -> np.ndarray:
    """Blue (low) through green to red (high), packed as one float32 RGB word."""
    span = max(high - low, 1e-6)
    t = np.clip((z - low) / span, 0., 1.)
    red = np.clip(2. * t - 1., 0., 1.)
    blue = np.clip(1. - 2. * t, 0., 1.)
    green = 1. - red - blue
    packed = ((np.uint32(255) * red).astype(np.uint32) << np.uint32(16)
              | (np.uint32(255) * green).astype(np.uint32) << np.uint32(8)
              | (np.uint32(255) * blue).astype(np.uint32))
    return packed.view(np.float32) if packed.size else np.empty(0, dtype=np.float32)


class TopoVslamNode(Node):
    def __init__(self):
        super().__init__('om6dof_topo_vslam')
        defaults = {
            'world_frame': 'world',
            'camera_frame': 'd435_depth_optical_frame',
            'min_depth': .15,
            'max_depth': 3.,
            'stride': 2,
            'tf_timeout': .5,
            'max_pose_age': .5,
        }
        for key, value in defaults.items():
            self.declare_parameter(key, value, ParameterDescriptor(read_only=True))
        self.world_frame = self.get_parameter('world_frame').value
        self.camera_frame = self.get_parameter('camera_frame').value
        self.min_depth = float(self.get_parameter('min_depth').value)
        self.max_depth = float(self.get_parameter('max_depth').value)
        self.stride = max(1, int(self.get_parameter('stride').value))
        self.tf_timeout = float(self.get_parameter('tf_timeout').value)
        self.max_pose_age = float(self.get_parameter('max_pose_age').value)
        if not (0. < self.min_depth < self.max_depth):
            raise ValueError('Require 0 < min_depth < max_depth')

        self._buffer = Buffer()
        self._listener = TransformListener(self._buffer, self)
        self._info = None
        self._grid = None
        self._dropped = 0
        sensor_qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=2,
                                reliability=ReliabilityPolicy.BEST_EFFORT,
                                durability=DurabilityPolicy.VOLATILE)
        self._points = self.create_publisher(PointCloud2, 'points', sensor_qos)
        self.create_subscription(CameraInfo, 'depth/camera_info', self._on_info, sensor_qos)
        self.create_subscription(Image, 'depth/image_raw', self._on_depth, sensor_qos)
        self._report = self.create_timer(5., self._report_drops)

    def _on_info(self, info: CameraInfo):
        self._info = info
        self._grid = None

    def _rays(self, info: CameraInfo):
        """Per-pixel unit-Z rays for the strided grid, cached per calibration."""
        if self._grid is None:
            fx, fy, cx, cy = info.k[0], info.k[4], info.k[2], info.k[5]
            if fx <= 0. or fy <= 0.:
                raise ValueError('CameraInfo carries a non-positive focal length')
            u = np.arange(0, info.width, self.stride, dtype=np.float32)
            v = np.arange(0, info.height, self.stride, dtype=np.float32)
            uu, vv = np.meshgrid(u, v)
            self._grid = ((uu - cx) / fx, (vv - cy) / fy)
        return self._grid

    def _on_depth(self, image: Image):
        info = self._info
        if info is None:
            self._dropped += 1
            return
        if (info.height, info.width) != (image.height, image.width):
            self.get_logger().warn('CameraInfo size does not match the depth image; skipping frame',
                                   throttle_duration_sec=5.)
            return
        frame = image.header.frame_id or self.camera_frame
        stamp = Time.from_msg(image.header.stamp)
        try:
            # Look the pose up at the exact capture stamp: a wrist camera on a
            # moving arm projects into the wrong place under a latest-pose read.
            transform = self._buffer.lookup_transform(
                self.world_frame, frame, stamp,
                timeout=rclpy.duration.Duration(seconds=self.tf_timeout))
        except TransformException as exc:
            self._dropped += 1
            self.get_logger().warn(f'No {self.world_frame} <- {frame} pose: {exc}',
                                   throttle_duration_sec=5.)
            return
        age = (self.get_clock().now() - Time.from_msg(transform.header.stamp)).nanoseconds * 1e-9
        if abs(age) > self.max_pose_age:
            self._dropped += 1
            return

        depth = depth_metres(image)[::self.stride, ::self.stride]
        x_ray, y_ray = self._rays(info)
        valid = np.isfinite(depth) & (depth >= self.min_depth) & (depth <= self.max_depth)
        z = depth[valid]
        camera_points = np.stack((x_ray[valid] * z, y_ray[valid] * z, z), axis=1)
        rotation = rotation_matrix(transform.transform.rotation)
        translation = np.array([transform.transform.translation.x,
                                transform.transform.translation.y,
                                transform.transform.translation.z])
        world = (camera_points.astype(np.float64) @ rotation.T + translation).astype(np.float32)
        self._points.publish(self._cloud(world, image.header.stamp))

    def _cloud(self, world: np.ndarray, stamp) -> PointCloud2:
        cloud = PointCloud2()
        cloud.header.stamp = stamp
        cloud.header.frame_id = self.world_frame
        cloud.height = 1
        cloud.width = world.shape[0]
        cloud.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='rgb', offset=12, datatype=PointField.FLOAT32, count=1),
        ]
        cloud.is_bigendian = False
        cloud.point_step = 16
        cloud.row_step = cloud.point_step * cloud.width
        cloud.is_dense = True
        if cloud.width:
            packed = np.empty((cloud.width, 4), dtype=np.float32)
            packed[:, :3] = world
            packed[:, 3] = height_colours(world[:, 2], float(world[:, 2].min()),
                                          float(world[:, 2].max()))
            cloud.data = packed.tobytes()
        return cloud

    def _report_drops(self):
        if self._dropped:
            self.get_logger().info(f'Dropped {self._dropped} frames without a usable pose')
            self._dropped = 0


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = TopoVslamNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        return 0
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0
