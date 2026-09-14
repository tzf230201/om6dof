"""Read-only RGB-centre depth and nominal-URDF world-coordinate verifier.

The node owns one RealSense camera, aligns depth to the unmodified RGB image,
deprojects a robust median depth around the image centre, and transforms that
3-D optical point with the currently published URDF TF. It never commands a
controller or changes robot state.
"""

from __future__ import annotations

import json
import math
import threading
import time
from typing import Iterable, Optional

import cv2
import numpy as np
import pyrealsense2 as rs
import rclpy
import tf2_ros
from cv_bridge import CvBridge
from geometry_msgs.msg import PointStamped
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import Image
from std_msgs.msg import String
from tf2_ros import TransformException
from visualization_msgs.msg import Marker, MarkerArray


def robust_depth(values: Iterable[float], minimum: float, maximum: float,
                 minimum_samples: int) -> Optional[float]:
    """Return the median of finite, in-range depth samples."""
    valid = [float(value) for value in values
             if math.isfinite(float(value)) and minimum <= float(value) <= maximum]
    if len(valid) < minimum_samples:
        return None
    return float(np.median(np.asarray(valid, dtype=np.float64)))


def quaternion_matrix(x: float, y: float, z: float, w: float) -> np.ndarray:
    """Convert a normalized ROS quaternion to a 3x3 rotation matrix."""
    quaternion = np.asarray([x, y, z, w], dtype=np.float64)
    norm = float(np.linalg.norm(quaternion))
    if not math.isfinite(norm) or norm < 1.0e-12:
        raise ValueError("invalid zero/non-finite quaternion")
    x, y, z, w = quaternion / norm
    return np.array([
        [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w),
         2.0 * (x * z + y * w)],
        [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z),
         2.0 * (y * z - x * w)],
        [2.0 * (x * z - y * w), 2.0 * (y * z + x * w),
         1.0 - 2.0 * (x * x + y * y)],
    ], dtype=np.float64)


def transform_point(point: Iterable[float], translation: Iterable[float],
                    quaternion: Iterable[float]) -> np.ndarray:
    """Apply parent<-child transform to a point expressed in child."""
    qx, qy, qz, qw = [float(value) for value in quaternion]
    return (quaternion_matrix(qx, qy, qz, qw)
            @ np.asarray(point, dtype=np.float64)
            + np.asarray(translation, dtype=np.float64))


def apply_world_z_offset(point: Iterable[float], offset: float) -> np.ndarray:
    """Apply an explicit diagnostic Z correction without changing the input."""
    corrected = np.asarray(point, dtype=np.float64).copy()
    corrected[2] += float(offset)
    return corrected


class CenterDepthUrdfCheck(Node):
    def __init__(self) -> None:
        super().__init__("center_depth_urdf_check")
        dynamic = ParameterDescriptor(dynamic_typing=True)
        self.declare_parameter("camera_serial", "", dynamic)
        self.declare_parameter("width", 640)
        self.declare_parameter("height", 480)
        self.declare_parameter("fps", 30)
        self.declare_parameter("camera_frame", "d435_color_optical_frame")
        self.declare_parameter("world_frame", "world")
        self.declare_parameter("patch_radius", 4)
        self.declare_parameter("minimum_depth", 0.08)
        self.declare_parameter("maximum_depth", 3.0)
        self.declare_parameter("minimum_depth_samples", 12)
        self.declare_parameter("maximum_tf_age", 0.5)
        self.declare_parameter("world_z_offset", 0.0)
        self.declare_parameter("show_window", True)
        self.declare_parameter("publish_debug", True)
        self.declare_parameter("window_name", "D435i Center Depth - URDF Check")

        self.camera_frame = str(self.get_parameter("camera_frame").value)
        self.world_frame = str(self.get_parameter("world_frame").value)
        self.patch_radius = int(self.get_parameter("patch_radius").value)
        self.minimum_depth = float(self.get_parameter("minimum_depth").value)
        self.maximum_depth = float(self.get_parameter("maximum_depth").value)
        self.minimum_depth_samples = int(
            self.get_parameter("minimum_depth_samples").value)
        self.maximum_tf_age = float(self.get_parameter("maximum_tf_age").value)
        self.world_z_offset = float(self.get_parameter("world_z_offset").value)
        self.show_window = bool(self.get_parameter("show_window").value)
        self.publish_debug = bool(self.get_parameter("publish_debug").value)
        self.window_name = str(self.get_parameter("window_name").value)
        if self.patch_radius < 0:
            raise ValueError("patch_radius must be non-negative")
        patch_size = (2 * self.patch_radius + 1) ** 2
        if not 1 <= self.minimum_depth_samples <= patch_size:
            raise ValueError("minimum_depth_samples exceeds centre patch capacity")

        self.tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=5.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.camera_pub = self.create_publisher(
            PointStamped, "/om6dof/center_depth/camera_point", 5)
        self.world_pub = self.create_publisher(
            PointStamped, "/om6dof/center_depth/world_point", 5)
        self.raw_world_pub = self.create_publisher(
            PointStamped, "/om6dof/center_depth/world_point_raw", 5)
        self.marker_pub = self.create_publisher(
            MarkerArray, "/om6dof/center_depth/markers", 5)
        self.status_pub = self.create_publisher(
            String, "/om6dof/center_depth/status", 5)
        self.debug_pub = (self.create_publisher(
            Image, "/om6dof/center_depth/debug_image", 2)
            if self.publish_debug else None)
        self.bridge = CvBridge()

        self.pipeline = rs.pipeline()
        config = rs.config()
        serial = str(self.get_parameter("camera_serial").value)
        if serial:
            config.enable_device(serial)
        width = int(self.get_parameter("width").value)
        height = int(self.get_parameter("height").value)
        fps = int(self.get_parameter("fps").value)
        config.enable_stream(rs.stream.color, width, height, rs.format.bgr8, fps)
        config.enable_stream(rs.stream.depth, width, height, rs.format.z16, fps)
        try:
            profile = self.pipeline.start(config)
        except RuntimeError as exc:
            raise RuntimeError(
                "RealSense tidak dapat dibuka. Hentikan launch DD-GNG/AprilTag "
                "lain yang sedang mengambil kamera, lalu coba lagi: " + str(exc)) from exc
        device = profile.get_device()
        actual_serial = device.get_info(rs.camera_info.serial_number)
        actual_name = device.get_info(rs.camera_info.name)
        color_profile = profile.get_stream(rs.stream.color).as_video_stream_profile()
        self.intrinsics = color_profile.get_intrinsics()
        self.align = rs.align(rs.stream.color)
        self.get_logger().info(
            f"read-only verifier using {actual_name} serial={actual_serial}, "
            f"RGB {width}x{height}@{fps}; transform source=nominal URDF TF")

        if self.show_window:
            cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
            cv2.resizeWindow(self.window_name, 960, 720)

        self.stop_event = threading.Event()
        self.window_ever_visible = False
        self.last_terminal_log = 0.0
        self.thread = threading.Thread(target=self._capture_loop, daemon=True)
        self.thread.start()
        self.create_timer(0.2, self._stop_when_window_closes)

    def _stop_when_window_closes(self) -> None:
        if self.stop_event.is_set() and rclpy.ok():
            rclpy.shutdown()

    def _capture_loop(self) -> None:
        frame_count = 0
        fps_value = 0.0
        fps_start = time.monotonic()
        try:
            while not self.stop_event.is_set() and rclpy.ok():
                try:
                    frames = self.pipeline.wait_for_frames(timeout_ms=2000)
                    aligned = self.align.process(frames)
                except RuntimeError as exc:
                    self.get_logger().warn(f"RealSense frame gagal: {exc}")
                    continue
                color = aligned.get_color_frame()
                depth = aligned.get_depth_frame()
                if not color or not depth:
                    continue
                image = np.asanyarray(color.get_data()).copy()
                height, width = image.shape[:2]
                u, v = width // 2, height // 2
                values = []
                for vv in range(max(0, v - self.patch_radius),
                                min(height, v + self.patch_radius + 1)):
                    for uu in range(max(0, u - self.patch_radius),
                                    min(width, u + self.patch_radius + 1)):
                        values.append(depth.get_distance(uu, vv))
                distance = robust_depth(
                    values, self.minimum_depth, self.maximum_depth,
                    self.minimum_depth_samples)

                frame_count += 1
                now_mono = time.monotonic()
                elapsed = now_mono - fps_start
                if elapsed >= 1.0:
                    fps_value = frame_count / elapsed
                    frame_count = 0
                    fps_start = now_mono

                camera_point = None
                raw_world_point = None
                world_point = None
                tf_age = None
                status = "depth pusat tidak valid"
                if distance is not None:
                    camera_point = np.asarray(
                        rs.rs2_deproject_pixel_to_point(
                            self.intrinsics, [float(u), float(v)], distance),
                        dtype=np.float64)
                    stamp = self.get_clock().now()
                    self._publish_point(self.camera_pub, self.camera_frame,
                                        camera_point, stamp)
                    try:
                        tf = self.tf_buffer.lookup_transform(
                            self.world_frame, self.camera_frame, Time())
                        tf_stamp = Time.from_msg(tf.header.stamp)
                        if tf_stamp.nanoseconds != 0:
                            tf_age = max(
                                0.0, (stamp - tf_stamp).nanoseconds * 1.0e-9)
                        if tf_age is not None and tf_age > self.maximum_tf_age:
                            status = f"TF URDF stale {tf_age:.2f}s"
                        else:
                            t = tf.transform.translation
                            q = tf.transform.rotation
                            raw_world_point = transform_point(
                                camera_point, [t.x, t.y, t.z],
                                [q.x, q.y, q.z, q.w])
                            world_point = apply_world_z_offset(
                                raw_world_point, self.world_z_offset)
                            self._publish_point(
                                self.raw_world_pub, self.world_frame,
                                raw_world_point, stamp)
                            self._publish_point(self.world_pub, self.world_frame,
                                                world_point, stamp)
                            self._publish_markers(world_point, stamp)
                            status = (
                                "OK: URDF + offset Z uji "
                                f"{self.world_z_offset:+.4f} m")
                    except (TransformException, ValueError) as exc:
                        status = "TF world<-camera tidak tersedia: " + str(exc)

                self._publish_status(distance, camera_point, raw_world_point,
                                     world_point,
                                     tf_age, status, u, v, len(values))
                self._draw_overlay(image, u, v, distance, camera_point,
                                   raw_world_point, world_point, tf_age,
                                   status, fps_value)
                if self.debug_pub is not None:
                    msg = self.bridge.cv2_to_imgmsg(image, encoding="bgr8")
                    msg.header.stamp = self.get_clock().now().to_msg()
                    msg.header.frame_id = self.camera_frame
                    self.debug_pub.publish(msg)
                if self.show_window:
                    cv2.imshow(self.window_name, image)
                    key = cv2.waitKey(1) & 0xFF
                    if key in (27, ord("q")):
                        self.stop_event.set()
                        break
                    try:
                        visible = cv2.getWindowProperty(
                            self.window_name, cv2.WND_PROP_VISIBLE)
                        if visible >= 1:
                            self.window_ever_visible = True
                        elif self.window_ever_visible:
                            self.stop_event.set()
                            break
                    except cv2.error:
                        self.stop_event.set()
                        break
        except Exception as exc:
            self.get_logger().error(
                f"thread kamera berhenti karena error: {type(exc).__name__}: {exc}")
            self.stop_event.set()
        finally:
            if self.show_window:
                cv2.destroyAllWindows()

    def _publish_point(self, publisher, frame: str, point: np.ndarray,
                       stamp: Time) -> None:
        message = PointStamped()
        message.header.frame_id = frame
        message.header.stamp = stamp.to_msg()
        message.point.x, message.point.y, message.point.z = map(float, point)
        publisher.publish(message)

    def _publish_markers(self, point: np.ndarray, stamp: Time) -> None:
        sphere = Marker()
        sphere.header.frame_id = self.world_frame
        sphere.header.stamp = stamp.to_msg()
        sphere.ns = "center_depth_urdf"
        sphere.id = 0
        sphere.type = Marker.SPHERE
        sphere.action = Marker.ADD
        sphere.pose.position.x, sphere.pose.position.y, sphere.pose.position.z = \
            map(float, point)
        sphere.pose.orientation.w = 1.0
        sphere.scale.x = sphere.scale.y = sphere.scale.z = 0.025
        sphere.color.r, sphere.color.g, sphere.color.b, sphere.color.a = \
            1.0, 0.15, 0.05, 1.0
        sphere.lifetime = Duration(seconds=0.25).to_msg()

        text_marker = Marker()
        text_marker.header = sphere.header
        text_marker.ns = sphere.ns
        text_marker.id = 1
        text_marker.type = Marker.TEXT_VIEW_FACING
        text_marker.action = Marker.ADD
        text_marker.pose.position.x = float(point[0])
        text_marker.pose.position.y = float(point[1])
        text_marker.pose.position.z = float(point[2] + 0.04)
        text_marker.pose.orientation.w = 1.0
        text_marker.scale.z = 0.025
        text_marker.color.r = text_marker.color.g = text_marker.color.b = 1.0
        text_marker.color.a = 1.0
        text_marker.text = (f"URDF center: {point[0]:+.3f}, "
                            f"{point[1]:+.3f}, {point[2]:+.3f} m "
                            f"(Z offset {self.world_z_offset:+.4f})")
        text_marker.lifetime = sphere.lifetime
        self.marker_pub.publish(MarkerArray(markers=[sphere, text_marker]))

    def _publish_status(self, distance, camera_point, raw_world_point,
                        world_point, tf_age, status, u, v, sample_count) -> None:
        payload = {
            "status": status,
            "pixel": [u, v],
            "patch_radius": self.patch_radius,
            "patch_sample_capacity": sample_count,
            "depth_m": distance,
            "camera_frame": self.camera_frame,
            "camera_xyz_m": None if camera_point is None else camera_point.tolist(),
            "world_frame": self.world_frame,
            "raw_world_xyz_m": (
                None if raw_world_point is None else raw_world_point.tolist()),
            "world_xyz_m": None if world_point is None else world_point.tolist(),
            "world_z_offset_m": self.world_z_offset,
            "tf_age_s": tf_age,
            "transform_source": "nominal_urdf_tf_plus_explicit_test_z_offset",
        }
        message = String()
        message.data = json.dumps(payload, separators=(",", ":"), allow_nan=False)
        self.status_pub.publish(message)
        now = time.monotonic()
        if now - self.last_terminal_log >= 2.0:
            self.last_terminal_log = now
            self.get_logger().info(message.data)

    def _draw_overlay(self, image, u, v, distance, camera_point,
                      raw_world_point, world_point, tf_age, status,
                      fps_value) -> None:
        radius = max(8, self.patch_radius + 3)
        cv2.line(image, (u - 25, v), (u + 25, v), (0, 255, 255), 2)
        cv2.line(image, (u, v - 25), (u, v + 25), (0, 255, 255), 2)
        cv2.rectangle(image, (u - radius, v - radius),
                      (u + radius, v + radius), (255, 120, 0), 1)
        overlay = image.copy()
        cv2.rectangle(overlay, (0, 0), (image.shape[1], 174), (0, 0, 0), -1)
        cv2.addWeighted(overlay, 0.62, image, 0.38, 0.0, image)
        depth_text = "invalid" if distance is None else f"{distance:.4f} m"
        cam_text = ("-" if camera_point is None else
                    ", ".join(f"{value:+.4f}" for value in camera_point))
        raw_world_text = ("-" if raw_world_point is None else
                          ", ".join(f"{value:+.4f}"
                                    for value in raw_world_point))
        world_text = ("-" if world_point is None else
                      ", ".join(f"{value:+.4f}" for value in world_point))
        tf_text = "-" if tf_age is None else f"{tf_age:.3f}s"
        lines = [
            f"RGB center ({u},{v}) | depth median: {depth_text} | {fps_value:.1f} FPS",
            f"camera optical xyz [m]: {cam_text}",
            f"world raw URDF xyz [m]: {raw_world_text}",
            f"world corrected xyz [m]: {world_text} | TF age {tf_text}",
            status,
        ]
        color = (40, 230, 40) if world_point is not None else (40, 170, 255)
        for index, line in enumerate(lines):
            cv2.putText(image, line, (14, 30 + index * 32),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.62,
                        (255, 255, 255) if index < 4 else color,
                        2, cv2.LINE_AA)

    def destroy_node(self):
        self.stop_event.set()
        if hasattr(self, "thread"):
            self.thread.join(timeout=3.0)
        try:
            self.pipeline.stop()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = CenterDepthUrdfCheck()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
