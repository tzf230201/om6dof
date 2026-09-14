"""ROS depth publisher with one SDK worker and a bounded latest-frame mailbox."""
from __future__ import annotations

from copy import deepcopy
import threading
import time

import numpy as np
import rclpy
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image

from .realsense_source import CaptureCancelled, RealSenseSource


def frame_messages(frame, camera_frame):
    """Build an exact-stamp calibrated pair; depth is optical Z in metres."""
    if not camera_frame or camera_frame.startswith('/'):
        raise ValueError('camera_frame must be a nonempty TF frame without a leading slash')
    depth = np.ascontiguousarray(frame.depth, dtype='<f4')
    if depth.ndim != 2 or min(depth.shape) < 1:
        raise ValueError('Depth must be a nonempty two-dimensional image')
    image = Image()
    ns = round(frame.timestamp * 1_000_000_000)
    if ns <= 0:
        raise ValueError('A positive acquisition timestamp is required')
    image.header.stamp.sec, image.header.stamp.nanosec = divmod(ns, 1_000_000_000)
    image.header.frame_id = camera_frame
    image.height, image.width = depth.shape
    image.encoding = '32FC1'
    image.is_bigendian = 0
    image.step = image.width * 4
    image.data = depth.tobytes()
    fx, fy, cx, cy = (frame.intrinsics[key] for key in ('fx', 'fy', 'cx', 'cy'))
    info = CameraInfo()
    info.header = image.header
    info.height, info.width = image.height, image.width
    info.distortion_model = 'plumb_bob'
    info.d = [0.] * 5
    info.k = [fx, 0., cx, 0., fy, cy, 0., 0., 1.]
    info.r = [1., 0., 0., 0., 1., 0., 0., 0., 1.]
    info.p = [fx, 0., cx, 0., 0., fy, cy, 0., 0., 0., 1., 0.]
    return image, info


class RealSenseDepthNode(Node):
    def __init__(self):
        super().__init__('realsense_depth')
        if self.get_parameter('use_sim_time').value:
            raise ValueError('RealSense capture requires use_sim_time=false and the ROS wall clock')
        # Humble's set_descriptor validates the supplied type, unlike
        # declare_parameter: retain the existing BOOL type while freezing it.
        clock_descriptor = deepcopy(self.describe_parameter('use_sim_time'))
        clock_descriptor.read_only = True
        self.set_descriptor('use_sim_time', clock_descriptor)
        defaults = {
            'fps': 15, 'stride': 2, 'serial': '', 'timeout_ms': 3000,
            'min_depth': .15, 'max_depth': 3., 'warmup_frames': 15,
            'depth_filter': 'supported', 'max_frame_age': .5,
            'camera_frame': 'd435_depth_optical_frame',
        }
        for key, value in defaults.items():
            self.declare_parameter(key, value, ParameterDescriptor(read_only=True))
        options = {key: self.get_parameter(key).value for key in defaults}
        self.camera_frame = options.pop('camera_frame')
        if not self.camera_frame or self.camera_frame.startswith('/'):
            raise ValueError('camera_frame must be a nonempty TF frame without a leading slash')
        options['z_min'] = options.pop('min_depth')
        options['z_max'] = options.pop('max_depth')
        self._max_frame_age = options['max_frame_age']
        self._stop = threading.Event()
        self._source = RealSenseSource(**options, stop_event=self._stop)
        self._lock = threading.Lock()
        self._pending = None
        self._failure = None
        self._announced = False
        self._overwritten_frames = 0
        self._stale_frames = 0
        self._published_frames = 0
        qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=2,
                         reliability=ReliabilityPolicy.BEST_EFFORT,
                         durability=DurabilityPolicy.VOLATILE)
        self._image_pub = self.create_publisher(Image, 'depth/image_raw', qos)
        self._info_pub = self.create_publisher(CameraInfo, 'depth/camera_info', qos)
        self._timer = self.create_timer(1. / 60., self._publish_pending)
        self._worker = threading.Thread(target=self._capture, name='realsense-capture', daemon=True)
        self._worker.start()

    def _capture(self):
        try:
            with self._source as source:
                while not self._stop.is_set():
                    frame = source.capture()
                    with self._lock:
                        if self._pending is not None:
                            self._overwritten_frames += 1
                        self._pending = (frame, source.metadata.copy())
        except CaptureCancelled:
            pass
        except Exception as exc:
            with self._lock:
                self._failure = str(exc)

    def _publish_pending(self):
        with self._lock:
            pending, self._pending = self._pending, None
            failure = self._failure
        if failure is not None:
            raise RuntimeError('RealSense capture stopped: ' + failure)
        if pending is None:
            return
        frame, metadata = pending
        # A busy executor must discard its delayed mailbox entry, rather than
        # publishing an old image under a fresh, misleading ROS timestamp.
        if time.time() - frame.timestamp > self._max_frame_age:
            self._stale_frames += 1
            return
        image, info = frame_messages(frame, self.camera_frame)
        self._info_pub.publish(info)
        self._image_pub.publish(image)
        self._published_frames += 1
        if not self._announced:
            self._announced = True
            self.get_logger().info(
                f"{metadata['sensor_name']}: {image.width}x{image.height} depth, "
                f"{metadata['fps']} FPS, metres, {metadata['timestamp_domain']}, "
                f"frame={self.camera_frame}, filter={metadata['depth_filter']}")

    def close(self):
        self._stop.set()
        self._timer.cancel()
        self._worker.join(timeout=6.)
        alive = self._worker.is_alive()
        if alive:
            self.get_logger().error('SDK worker did not finish within 6 seconds; exiting process')
        self.get_logger().info(
            f'Depth publisher stopped: {self._published_frames} published, '
            f'{self._overwritten_frames} superseded, {self._stale_frames} stale frames discarded')
        return not alive


def main(args=None):
    rclpy.init(args=args)
    node = None
    code = 0
    try:
        node = RealSenseDepthNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception as exc:
        code = 1
        logger = node.get_logger() if node is not None else rclpy.logging.get_logger('realsense_depth')
        logger.fatal(str(exc))
    finally:
        if node is not None:
            if not node.close():
                code = 1
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return code
