"""Vision-only RGB-D viewer for the wrist RealSense.

The viewer owns no arm or gripper action client.  It renders aligned RGB and
depth, then reports the raw depth at the centre pixel and that pixel's 3-D
position in both the camera optical frame and ``world``.  The world transform
is intentionally looked up at the frame timestamp, never at an arbitrary
latest time.
"""

from __future__ import annotations

import math
from typing import Optional, Sequence, Tuple

import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.time import Time

from .rgbd_source import RGBDFrame, make_source
from .transforms import deproject, quat_to_matrix


def centre_pixel(frame: RGBDFrame) -> Tuple[int, int]:
    """Return the integer centre pixel as ``(u, v)``."""
    width, height = frame.size
    return width // 2, height // 2


def depth_at_pixel(frame: RGBDFrame, u: int, v: int) -> Optional[float]:
    """Return metric depth, or ``None`` for an invalid/out-of-image sample."""
    if not (0 <= int(v) < frame.depth.shape[0]
            and 0 <= int(u) < frame.depth.shape[1]):
        return None
    depth_m = float(frame.depth[int(v), int(u)]) * float(frame.depth_scale)
    return depth_m if math.isfinite(depth_m) and depth_m > 0.0 else None


def point_in_world(depth_m: float, u: int, v: int,
                   intrinsics: Sequence[float], translation: Sequence[float],
                   rotation: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    """Deproject a pixel and express it in world coordinates.

    ``rotation`` has camera optical axes as columns in the world frame.
    Returns ``(point_camera, point_world)``.
    """
    point_camera = deproject(u, v, depth_m, intrinsics)
    point_world = (np.asarray(rotation, dtype=float) @ point_camera
                   + np.asarray(translation, dtype=float))
    return point_camera, point_world


class RGBDViewer(Node):
    """Display live RGB-D and a capture-time world-coordinate measurement."""

    def __init__(self) -> None:
        super().__init__("rgbd_viewer")
        self._declare_parameters()
        self._cv2 = self._import_cv2()
        self._window = str(self.get_parameter("window_name").value)
        self._cv2.namedWindow(self._window, self._cv2.WINDOW_NORMAL)

        from tf2_ros import Buffer, TransformListener

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._source = self._make_source()
        self._first_capture = True
        self._last_error = ""
        self._timer = self.create_timer(
            1.0 / float(self.get_parameter("display_fps").value),
            self._update)
        self.get_logger().info(
            "RGB-D viewer is vision-only: it never sends arm or gripper goals")

    def _declare_parameters(self) -> None:
        self.declare_parameter("camera_source", "realsense")
        self.declare_parameter("camera_serial", "427622271962")
        self.declare_parameter("camera_width", 640)
        self.declare_parameter("camera_height", 480)
        self.declare_parameter("camera_fps", 15)
        # The first D405 frame after pipeline start can take longer than one
        # frame period while USB/auto-exposure settles.
        self.declare_parameter("camera_timeout_ms", 3000)
        self.declare_parameter("camera_optical_frame",
                               "d405_depth_optical_frame")
        self.declare_parameter("base_frame", "world")
        self.declare_parameter("color_topic", "/camera/color/image_raw")
        self.declare_parameter(
            "depth_topic", "/camera/aligned_depth_to_color/image_raw")
        self.declare_parameter("camera_info_topic", "/camera/color/camera_info")
        self.declare_parameter("topic_depth_scale", 0.0)
        self.declare_parameter("topic_sync_tolerance_s", 0.05)
        self.declare_parameter("topic_sync_queue_size", 10)
        self.declare_parameter("display_fps", 10.0)
        self.declare_parameter("max_display_depth_m", 1.0)
        self.declare_parameter("window_name", "OM6DOF RGB-D viewer")

    @staticmethod
    def _import_cv2():
        try:
            import cv2
        except ImportError as exc:
            raise RuntimeError("OpenCV is required: install python3-opencv") from exc
        return cv2

    def _make_source(self):
        source = str(self.get_parameter("camera_source").value).lower()
        if source == "realsense":
            return make_source(
                source,
                width=int(self.get_parameter("camera_width").value),
                height=int(self.get_parameter("camera_height").value),
                fps=int(self.get_parameter("camera_fps").value),
                serial=str(self.get_parameter("camera_serial").value),
                optical_frame_id=str(
                    self.get_parameter("camera_optical_frame").value),
                logger=self.get_logger(), clock=self.get_clock())
        depth_scale = float(self.get_parameter("topic_depth_scale").value)
        return make_source(
            source, node=self,
            color_topic=str(self.get_parameter("color_topic").value),
            depth_topic=str(self.get_parameter("depth_topic").value),
            info_topic=str(self.get_parameter("camera_info_topic").value),
            depth_scale=depth_scale if depth_scale > 0.0 else None,
            sync_tolerance_s=float(
                self.get_parameter("topic_sync_tolerance_s").value),
            sync_queue_size=int(
                self.get_parameter("topic_sync_queue_size").value))

    @staticmethod
    def _format_point(name: str, point: Optional[np.ndarray]) -> str:
        if point is None:
            return f"{name}: N/A"
        return (f"{name}: X={point[0]:+.3f}  Y={point[1]:+.3f}  "
                f"Z={point[2]:+.3f} m")

    def _world_measurement(self, frame: RGBDFrame, u: int, v: int,
                           depth_m: Optional[float]
                           ) -> Tuple[Optional[np.ndarray], Optional[np.ndarray], str]:
        if depth_m is None:
            return None, None, "depth invalid (zero/out of range)"
        camera_point = deproject(u, v, depth_m, frame.intrinsics)
        optical_frame = frame.frame_id or str(
            self.get_parameter("camera_optical_frame").value)
        try:
            stamp = Time(nanoseconds=int(round(frame.stamp * 1e9)))
            transform = self._tf_buffer.lookup_transform(
                str(self.get_parameter("base_frame").value), optical_frame,
                stamp, timeout=Duration(seconds=0.05))
            translation = transform.transform.translation
            rotation = transform.transform.rotation
            world_point = (quat_to_matrix(rotation.x, rotation.y, rotation.z,
                                          rotation.w) @ camera_point
                           + np.array([translation.x, translation.y, translation.z]))
        except Exception as exc:  # noqa: BLE001 - tf2 has several exception types
            return camera_point, None, f"TF unavailable: {exc}"
        return camera_point, world_point, ""

    def _draw_text(self, image: np.ndarray, lines: Sequence[str]) -> None:
        for index, line in enumerate(lines):
            origin = (12, 28 + 27 * index)
            self._cv2.putText(image, line, origin,
                              self._cv2.FONT_HERSHEY_SIMPLEX, 0.55,
                              (0, 0, 0), 3, self._cv2.LINE_AA)
            self._cv2.putText(image, line, origin,
                              self._cv2.FONT_HERSHEY_SIMPLEX, 0.55,
                              (255, 255, 255), 1, self._cv2.LINE_AA)

    def _render(self, frame: RGBDFrame) -> None:
        u, v = centre_pixel(frame)
        depth_m = depth_at_pixel(frame, u, v)
        camera_point, world_point, note = self._world_measurement(
            frame, u, v, depth_m)
        rgb = np.asarray(frame.color).copy()
        max_depth = max(0.05, float(
            self.get_parameter("max_display_depth_m").value))
        metric_depth = np.asarray(frame.depth, dtype=np.float32) * frame.depth_scale
        normalized = np.uint8(np.clip(metric_depth / max_depth, 0.0, 1.0) * 255.0)
        depth_image = self._cv2.applyColorMap(normalized, self._cv2.COLORMAP_TURBO)
        depth_image[metric_depth <= 0.0] = 0

        for image in (rgb, depth_image):
            self._cv2.drawMarker(image, (u, v), (255, 255, 255),
                                 self._cv2.MARKER_CROSS, 18, 1,
                                 self._cv2.LINE_AA)
            self._cv2.circle(image, (u, v), 5, (0, 0, 0), 1,
                             self._cv2.LINE_AA)
        depth_text = "Depth: N/A" if depth_m is None else f"Depth: {depth_m:.3f} m"
        self._draw_text(rgb, [
            f"Centre pixel: u={u}, v={v}", depth_text,
            self._format_point("Camera", camera_point),
            self._format_point("World", world_point), note])
        self._draw_text(depth_image, ["Depth colour map", f"0 to {max_depth:.2f} m"])
        display = np.hstack((rgb, depth_image))
        self._cv2.imshow(self._window, display)

    def _update(self) -> None:
        try:
            frame = self._source.capture(
                warmup=10 if self._first_capture else 1,
                timeout_ms=int(self.get_parameter("camera_timeout_ms").value))
            self._first_capture = False
        except Exception as exc:  # noqa: BLE001 - camera SDK errors are reported
            message = f"camera capture failed: {exc}"
            if message != self._last_error:
                self.get_logger().error(message)
                self._last_error = message
            return
        if frame is None:
            message = "camera returned no RGB-D frame"
            detail = getattr(self._source, "last_error", "")
            if detail:
                message += f": {detail}"
            if message != self._last_error:
                self.get_logger().warn(message)
                self._last_error = message
            return
        self._last_error = ""
        self._render(frame)
        key = self._cv2.waitKey(1) & 0xFF
        if key in (27, ord("q")):
            self.get_logger().info("viewer closed by user")
            rclpy.shutdown()

    def destroy_node(self) -> None:
        try:
            self._source.stop()
        finally:
            self._cv2.destroyAllWindows()
            super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    executor = None
    try:
        node = RGBDViewer()
        executor = MultiThreadedExecutor(num_threads=2)
        executor.add_node(node)
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        if executor is not None:
            executor.shutdown(timeout_sec=1.0)
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
