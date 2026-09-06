"""RGB-D capture and point-cloud construction.

Two sources, same ``RGBDFrame`` out:

``realsense``
    Opens the wrist D405 directly through ``pyrealsense2``, the way
    ``om6dof_perception`` and ``apriltag_detector`` do. A RealSense can only be
    opened by one process, so the perception node (and its systemd unit) has to
    be stopped first.

``topic``
    Subscribes to ``sensor_msgs/Image`` colour + aligned depth +
    ``CameraInfo``, for when a driver node already owns the camera.

The two images in a frame are always registered to the same projection. Direct
RealSense capture aligns colour to depth so the resulting cloud remains in the
robot's existing depth optical TF; the topic source follows the projection and
frame advertised by the aligned image topics.
"""

from __future__ import annotations

import threading
import time
from collections import deque
from dataclasses import dataclass
from typing import Deque, Optional, Sequence, Tuple

import numpy as np


@dataclass
class RGBDFrame:
    """One aligned capture in ``frame_id`` optical coordinates.

    ``color`` is BGR uint8. ``depth`` retains the source representation (most
    commonly uint16 millimetres or float32 metres); ``depth_scale`` converts it
    to metres.
    """
    color: np.ndarray
    depth: np.ndarray
    intrinsics: Tuple[float, float, float, float]   # fx, fy, cx, cy
    depth_scale: float                              # raw unit -> metres
    stamp: float
    frame_id: str = ""

    @property
    def size(self) -> Tuple[int, int]:
        return int(self.color.shape[1]), int(self.color.shape[0])


def point_cloud(depth: np.ndarray, intrinsics: Sequence[float],
                depth_scale: float, *, stride: int = 2,
                z_min: float = 0.05, z_max: float = 1.0,
                color: Optional[np.ndarray] = None):
    """Deproject a depth image into an optical-frame (N, 3) cloud.

    Returns ``(points, colors, pixels)``. ``colors`` is ``None`` unless a
    colour image is given; ``pixels`` is the (N, 2) source pixel of each point,
    which is what lets a Gemini pixel hit be matched back to a grasp candidate.
    """
    fx, fy, cx, cy = [float(v) for v in intrinsics]
    depth = np.asarray(depth)
    sub = depth[::stride, ::stride].astype(np.float32) * float(depth_scale)
    rows = np.arange(0, depth.shape[0], stride, dtype=np.float32)
    cols = np.arange(0, depth.shape[1], stride, dtype=np.float32)
    grid_u, grid_v = np.meshgrid(cols, rows)

    valid = (sub > float(z_min)) & (sub < float(z_max))
    z = sub[valid]
    u = grid_u[valid]
    v = grid_v[valid]
    points = np.stack([(u - cx) * z / fx, (v - cy) * z / fy, z], axis=1)
    pixels = np.stack([u, v], axis=1)

    colors = None
    if color is not None:
        colors = np.asarray(color)[::stride, ::stride][valid]
    return points, colors, pixels


class RealSenseSource:
    """Exclusive RealSense owner, with colour aligned into the depth frame."""

    def __init__(self, *, width: int = 640, height: int = 480, fps: int = 15,
                 serial: str = "", logger=None,
                 optical_frame_id: str = "d405_depth_optical_frame",
                 clock=None) -> None:
        self.width = int(width)
        self.height = int(height)
        self.fps = int(fps)
        self.serial = str(serial)
        self._log = logger
        self._clock = clock
        self._pipe = None
        self._align = None
        self._depth_scale: Optional[float] = None
        self._intrinsics: Optional[Tuple[float, float, float, float]] = None
        # rs.align(depth) below reprojects colour into the depth viewport. A
        # cloud deprojected from the returned depth image therefore really is
        # in this existing TF frame; no synthetic colour TF is assumed.
        self.optical_frame_id = str(optical_frame_id)

    def start(self) -> None:
        import pyrealsense2 as rs

        pipe = rs.pipeline()
        cfg = rs.config()
        if self.serial:
            cfg.enable_device(self.serial)
        cfg.enable_stream(rs.stream.color, self.width, self.height,
                          rs.format.bgr8, self.fps)
        cfg.enable_stream(rs.stream.depth, self.width, self.height,
                          rs.format.z16, self.fps)
        profile = pipe.start(cfg)
        depth_scale = float(
            profile.get_device().first_depth_sensor().get_depth_scale())
        if not np.isfinite(depth_scale) or depth_scale <= 0.0:
            raise RuntimeError("RealSense reported an invalid depth scale")
        self._depth_scale = depth_scale
        intr = (profile.get_stream(rs.stream.depth)
                .as_video_stream_profile().get_intrinsics())
        self._intrinsics = (float(intr.fx), float(intr.fy),
                            float(intr.ppx), float(intr.ppy))
        self._pipe = pipe
        self._align = rs.align(rs.stream.depth)
        if self._log:
            self._log.info(
                f"RealSense open {self.width}x{self.height}@{self.fps} "
                f"fx={intr.fx:.1f} cx={intr.ppx:.1f} "
                f"scale={self._depth_scale}")

    def stop(self) -> None:
        if self._pipe is not None:
            try:
                self._pipe.stop()
            except Exception:   # noqa: BLE001 - shutdown is best effort
                pass
            self._pipe = None

    def capture(self, *, warmup: int = 5,
                timeout_ms: int = 3000) -> Optional[RGBDFrame]:
        """Grab a frame, discarding ``warmup`` frames of auto-exposure."""
        if self._pipe is None:
            self.start()
        frames = None
        for _ in range(max(1, int(warmup))):
            frames = self._pipe.wait_for_frames(timeout_ms)
        aligned = self._align.process(frames)
        color_frame = aligned.get_color_frame()
        depth_frame = aligned.get_depth_frame()
        if not color_frame or not depth_frame:
            return None

        # Use metadata from the actual aligned frames when librealsense makes
        # it available. rs.align(depth) returns both images in the depth
        # viewport, so its depth profile is the correct projection model.
        try:
            profile = depth_frame.profile.as_video_stream_profile()
            intr = profile.get_intrinsics()
            intrinsics = (float(intr.fx), float(intr.fy),
                          float(intr.ppx), float(intr.ppy))
        except (AttributeError, RuntimeError):
            intrinsics = self._intrinsics

        depth_scale = self._depth_scale
        try:
            frame_scale = float(depth_frame.get_units())
            if np.isfinite(frame_scale) and frame_scale > 0.0:
                depth_scale = frame_scale
        except (AttributeError, RuntimeError, TypeError, ValueError):
            pass
        if intrinsics is None or depth_scale is None:
            raise RuntimeError("RealSense frame has no usable calibration")

        if self._clock is None:
            stamp = time.time()
        else:
            now = self._clock.now()
            nanoseconds = getattr(now, "nanoseconds", None)
            if nanoseconds is None:
                seconds, nanos = now.seconds_nanoseconds()
                nanoseconds = int(seconds) * 1_000_000_000 + int(nanos)
            stamp = float(nanoseconds) * 1e-9

        return RGBDFrame(
            color=np.asanyarray(color_frame.get_data()).copy(),
            depth=np.asanyarray(depth_frame.get_data()).copy(),
            intrinsics=intrinsics,
            depth_scale=depth_scale,
            stamp=stamp,
            frame_id=self.optical_frame_id,
        )


@dataclass(frozen=True)
class _ImageSample:
    array: np.ndarray
    stamp: float
    frame_id: str
    received_at: float
    depth_scale: Optional[float] = None


class TopicSource:
    """Timestamp-paired RGB-D frames from a driver-owned ROS camera."""

    def __init__(self, node, *, color_topic: str, depth_topic: str,
                 info_topic: str, depth_scale: Optional[float] = None,
                 sync_tolerance_s: float = 0.05,
                 sync_queue_size: int = 10) -> None:
        from sensor_msgs.msg import CameraInfo, Image
        from rclpy.qos import qos_profile_sensor_data

        if not np.isfinite(sync_tolerance_s) or sync_tolerance_s < 0.0:
            raise ValueError(
                "sync_tolerance_s must be finite and non-negative")
        if sync_queue_size < 1:
            raise ValueError("sync_queue_size must be at least one")
        if depth_scale is not None:
            depth_scale = float(depth_scale)
            if not np.isfinite(depth_scale) or depth_scale <= 0.0:
                raise ValueError("depth_scale must be a positive finite value")

        self._node = node
        self._lock = threading.Lock()
        self._condition = threading.Condition(self._lock)
        self._capture_lock = threading.Lock()
        self._colors: Deque[_ImageSample] = deque(maxlen=sync_queue_size)
        self._depths: Deque[_ImageSample] = deque(maxlen=sync_queue_size)
        self._pair: Optional[Tuple[_ImageSample, _ImageSample]] = None
        self._pair_sequence = 0
        self._capture_generation = 0
        self._accept_after = 0.0
        self._intrinsics: Optional[Tuple[float, float, float, float]] = None
        self._info_frame_id = ""
        self._info_size: Optional[Tuple[int, int]] = None
        self._last_error = ""
        self._depth_scale_override = depth_scale
        self._sync_tolerance_s = float(sync_tolerance_s)
        self._subscriptions = (
            node.create_subscription(
                Image, color_topic, self._on_color, qos_profile_sensor_data),
            node.create_subscription(
                Image, depth_topic, self._on_depth, qos_profile_sensor_data),
            node.create_subscription(
                CameraInfo, info_topic, self._on_info,
                qos_profile_sensor_data),
        )

    @staticmethod
    def _image_to_array(msg) -> np.ndarray:
        # Avoid cv_bridge so this imports without an OpenCV/ROS ABI mix.
        scalar_type, channels = {
            "mono8": (np.uint8, 1), "8UC1": (np.uint8, 1),
            "bgr8": (np.uint8, 3), "rgb8": (np.uint8, 3),
            "16UC1": (np.uint16, 1), "mono16": (np.uint16, 1),
            "32FC1": (np.float32, 1),
        }[msg.encoding]
        native_dtype = np.dtype(scalar_type)
        byte_order = ">" if bool(getattr(msg, "is_bigendian", False)) else "<"
        wire_dtype = (native_dtype.newbyteorder(byte_order)
                      if native_dtype.itemsize > 1 else native_dtype)
        height, width = int(msg.height), int(msg.width)
        row_bytes = width * channels * native_dtype.itemsize
        step = int(getattr(msg, "step", 0)) or row_bytes
        if height < 1 or width < 1:
            raise ValueError("image dimensions must be positive")
        if step < row_bytes:
            raise ValueError(
                f"image step {step} is smaller than packed row {row_bytes}")

        raw = np.frombuffer(msg.data, dtype=np.uint8)
        expected_bytes = height * step
        if raw.size < expected_bytes:
            raise ValueError(
                f"image has {raw.size} bytes, expected at least "
                f"{expected_bytes}")
        rows = raw[:expected_bytes].reshape(height, step)
        packed = np.ascontiguousarray(rows[:, :row_bytes])
        arr = np.frombuffer(packed, dtype=wire_dtype).reshape(
            height, width, channels)
        if native_dtype.itemsize > 1 and not wire_dtype.isnative:
            arr = arr.astype(native_dtype)
        if msg.encoding == "rgb8":
            arr = arr[:, :, ::-1]
        return arr if channels > 1 else arr[:, :, 0]

    @staticmethod
    def _stamp_and_frame(msg) -> Tuple[float, str]:
        header = getattr(msg, "header", None)
        stamp = getattr(header, "stamp", None)
        seconds = (float(getattr(stamp, "sec", 0.0))
                   + float(getattr(stamp, "nanosec", 0.0)) * 1e-9)
        return seconds, str(getattr(header, "frame_id", ""))

    def _depth_scale(self, msg) -> float:
        if self._depth_scale_override is not None:
            return self._depth_scale_override

        # sensor_msgs/Image itself has no scale field, but a wrapped/custom
        # message may supply one. Prefer that metadata when present.
        metadata = getattr(msg, "metadata", None)
        candidates = [getattr(msg, "depth_scale", None)]
        if isinstance(metadata, dict):
            candidates.append(metadata.get("depth_scale"))
        elif metadata is not None:
            candidates.append(getattr(metadata, "depth_scale", None))
        for value in candidates:
            if value is None:
                continue
            try:
                scale = float(value)
            except (TypeError, ValueError):
                continue
            if np.isfinite(scale) and scale > 0.0:
                return scale

        # REP 118 defines 16UC1 depth in millimetres and 32FC1 in metres.
        encoding_scales = {
            "16UC1": 0.001,
            "mono16": 0.001,
            "32FC1": 1.0,
        }
        try:
            return encoding_scales[msg.encoding]
        except KeyError as exc:
            raise ValueError(
                f"cannot infer depth scale for encoding '{msg.encoding}'"
            ) from exc

    @property
    def last_error(self) -> str:
        """Most recent fail-closed rejection or capture timeout."""
        with self._lock:
            return self._last_error

    @staticmethod
    def _pair_error(color: _ImageSample, depth: _ImageSample) -> str:
        if color.array.shape[:2] != depth.array.shape[:2]:
            return (f"color/depth dimensions differ: "
                    f"{color.array.shape[:2]} != {depth.array.shape[:2]}")
        if not color.frame_id or not depth.frame_id:
            return "color and aligned depth must both name an optical frame"
        if color.frame_id != depth.frame_id:
            return ("color/depth frames differ, so depth is not proven "
                    f"aligned: '{color.frame_id}' != '{depth.frame_id}'")
        return ""

    def _calibration_error_locked(self, color: _ImageSample) -> str:
        if self._intrinsics is None or self._info_size is None:
            return "no valid CameraInfo received"
        if not self._info_frame_id:
            return "CameraInfo does not name an optical frame"
        if self._info_frame_id != color.frame_id:
            return (f"CameraInfo frame '{self._info_frame_id}' does not match "
                    f"image frame '{color.frame_id}'")
        image_size = (int(color.array.shape[1]), int(color.array.shape[0]))
        if self._info_size != image_size:
            return (f"CameraInfo size {self._info_size} does not match image "
                    f"size {image_size}")
        return ""

    def _match_locked(self) -> None:
        """Pair FIFO samples whose ROS timestamps are close enough."""
        while self._colors and self._depths:
            color = self._colors[0]
            depth = self._depths[0]
            delta = color.stamp - depth.stamp
            if abs(delta) <= self._sync_tolerance_s:
                self._colors.popleft()
                self._depths.popleft()
                error = self._pair_error(color, depth)
                if error:
                    self._last_error = error
                else:
                    self._pair = (color, depth)
                    self._pair_sequence += 1
                    self._last_error = ""
                continue
            # With timestamp-ordered camera topics, the older sample can never
            # match the queue head on the other stream or any newer sample.
            if delta < 0.0:
                self._colors.popleft()
            else:
                self._depths.popleft()

    def _on_color(self, msg) -> None:
        received_at = time.monotonic()
        stamp, frame_id = self._stamp_and_frame(msg)
        sample = _ImageSample(self._image_to_array(msg).copy(), stamp,
                              frame_id, received_at)
        with self._condition:
            if received_at < self._accept_after:
                return
            self._colors.append(sample)
            self._match_locked()
            self._condition.notify_all()

    def _on_depth(self, msg) -> None:
        received_at = time.monotonic()
        stamp, frame_id = self._stamp_and_frame(msg)
        sample = _ImageSample(self._image_to_array(msg).copy(), stamp,
                              frame_id, received_at, self._depth_scale(msg))
        with self._condition:
            if received_at < self._accept_after:
                return
            self._depths.append(sample)
            self._match_locked()
            self._condition.notify_all()

    def _on_info(self, msg) -> None:
        _, frame_id = self._stamp_and_frame(msg)
        try:
            intrinsics = (float(msg.k[0]), float(msg.k[4]),
                          float(msg.k[2]), float(msg.k[5]))
            size = (int(msg.width), int(msg.height))
            if (not np.all(np.isfinite(intrinsics))
                    or intrinsics[0] <= 0.0 or intrinsics[1] <= 0.0):
                raise ValueError("CameraInfo has invalid focal lengths")
            if size[0] < 1 or size[1] < 1:
                raise ValueError("CameraInfo dimensions must be positive")
        except (AttributeError, IndexError, TypeError, ValueError) as exc:
            with self._condition:
                self._intrinsics = None
                self._info_size = None
                self._info_frame_id = ""
                self._last_error = f"invalid CameraInfo: {exc}"
                self._condition.notify_all()
            return
        with self._condition:
            self._intrinsics = intrinsics
            self._info_size = size
            self._info_frame_id = frame_id
            self._condition.notify_all()

    def start(self) -> None:
        """Present for interface parity with :class:`RealSenseSource`."""

    def stop(self) -> None:
        """Present for interface parity with :class:`RealSenseSource`."""

    def capture(self, *, warmup: int = 0,
                timeout_ms: int = 3000) -> Optional[RGBDFrame]:
        frames_needed = max(1, int(warmup))
        with self._capture_lock:
            requested_at = time.monotonic()
            deadline = (requested_at
                        + max(0, int(timeout_ms)) / 1000.0)
            with self._condition:
                # A capture is an acquisition request, not a read of a cache.
                # Clear both complete and half-complete old samples. The
                # received_at gate also rejects callbacks which started before
                # this request but finished decoding after the queues cleared.
                self._capture_generation += 1
                self._accept_after = requested_at
                self._colors.clear()
                self._depths.clear()
                self._pair = None
                first_sequence = self._pair_sequence
                self._last_error = ""

                while True:
                    enough_frames = (
                        self._pair is not None
                        and self._pair_sequence - first_sequence
                        >= frames_needed)
                    if enough_frames:
                        color, depth = self._pair
                        error = self._calibration_error_locked(color)
                        if not error:
                            self._pair = None
                            return RGBDFrame(
                                color=color.array.copy(),
                                depth=depth.array.copy(),
                                intrinsics=self._intrinsics,
                                depth_scale=depth.depth_scale,
                                stamp=color.stamp,
                                frame_id=color.frame_id)
                        self._last_error = error

                    remaining = deadline - time.monotonic()
                    if remaining <= 0.0:
                        if not self._last_error:
                            self._last_error = (
                                "timed out waiting for fresh synchronized "
                                "RGB-D data")
                        return None
                    self._condition.wait(timeout=remaining)


def make_source(kind: str, node=None, **kwargs):
    """Build the configured source. ``kind`` is ``realsense`` or ``topic``."""
    kind = str(kind).lower()
    if kind == "realsense":
        return RealSenseSource(**kwargs)
    if kind == "topic":
        if node is None:
            raise ValueError("the topic source needs a node to subscribe with")
        return TopicSource(node, **kwargs)
    raise ValueError(f"unknown camera_source '{kind}' (realsense|topic)")
