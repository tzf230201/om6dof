"""Calibrated optical depth capture, independent of ROS and lazy SDK loading.

Adapted from TopoVLA/native_fgng_bio/tools/realsense_source.py. The ROS mapper
owns the timestamped optical-to-world transform; this source only measures depth.
"""
from __future__ import annotations

from dataclasses import dataclass
import math
import re
import time

import numpy as np


@dataclass(frozen=True)
class DepthFrame:
    timestamp: float
    intrinsics: dict
    depth: np.ndarray


class CaptureCancelled(Exception):
    """The owning worker was asked to stop."""


def _supported_depth_mask(depth):
    """Retain measurements supported by two of eight original neighbours.

    Apply once at full resolution, before stride sampling. Holes remain empty;
    no depth value is interpolated or propagated into another pixel.
    """
    valid = depth > 0
    tolerance = np.maximum(np.float32(.02), np.float32(.02) * depth)
    support = np.zeros(depth.shape, dtype=np.uint8)
    height, width = depth.shape
    for dy in (-1, 0, 1):
        for dx in (-1, 0, 1):
            if dx == 0 and dy == 0:
                continue
            ys = slice(max(0, -dy), min(height, height - dy))
            xs = slice(max(0, -dx), min(width, width - dx))
            yn = slice(max(0, dy), min(height, height + dy))
            xn = slice(max(0, dx), min(width, width + dx))
            support[ys, xs] += (valid[ys, xs] & valid[yn, xn]
                & (np.abs(depth[ys, xs] - depth[yn, xn]) <= tolerance[ys, xs]))
    return valid & (support >= 2)


class RealSenseSource:
    """One thread must own start, capture, and close for an entire session."""

    def __init__(self, fps=15, stride=2, serial='', timeout_ms=3000,
                 z_min=.15, z_max=3., warmup_frames=15,
                 depth_filter='supported', max_frame_age=.5, stop_event=None):
        if fps not in (15, 30):
            raise ValueError('RealSense fps must be 15 or 30')
        if not isinstance(stride, int) or not 1 <= stride <= 480:
            raise ValueError('Depth stride must be an integer between 1 and 480')
        if not 100 <= timeout_ms <= 10000:
            raise ValueError('Capture timeout must be between 100 and 10000 ms')
        if not (math.isfinite(z_min) and math.isfinite(z_max) and 0 < z_min < z_max):
            raise ValueError('Depth limits require finite 0 < min_depth < max_depth')
        if not isinstance(warmup_frames, int) or not 0 <= warmup_frames <= 120:
            raise ValueError('Warmup must be between 0 and 120 frames')
        if depth_filter not in ('supported', 'none'):
            raise ValueError('Depth filter must be supported or none')
        if not math.isfinite(max_frame_age) or not 0 < max_frame_age <= 10:
            raise ValueError('max_frame_age must be in (0, 10] seconds')
        self.requested_fps, self.fps, self.stride = fps, fps, stride
        self.serial, self.timeout_ms = serial, int(timeout_ms)
        self.z_min, self.z_max = float(z_min), float(z_max)
        self.warmup_frames, self.depth_filter = warmup_frames, depth_filter
        self.max_frame_age, self.stop_event = max_frame_age, stop_event
        self.depth_scale, self.depth_intrinsics = None, None
        self.metadata = {}
        self._pipeline = None
        self._last_timestamp = -math.inf
        self._timestamp_domain = None
        self._capture_count = self._duplicate_frames = 0

    def _check_cancelled(self):
        if self.stop_event is not None and self.stop_event.is_set():
            raise CaptureCancelled()

    def _safe_error(self, error):
        message = str(error)
        if self.serial:
            message = message.replace(str(self.serial), '[selected device]')
        return re.sub(r'\b\d{10,}\b', '[device ID]', message)

    def start(self):
        if self._pipeline is not None:
            return self
        self._check_cancelled()
        try:
            import pyrealsense2 as rs
        except ImportError as exc:
            raise RuntimeError('pyrealsense2 is required for RealSense capture') from exc
        pipeline, config = rs.pipeline(), rs.config()
        if self.serial:
            config.enable_device(str(self.serial))
        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, self.requested_fps)
        started = False
        try:
            profile = pipeline.start(config)
            started = True
            self._check_cancelled()
            depth_profile = profile.get_stream(rs.stream.depth).as_video_stream_profile()
            intrinsics = depth_profile.get_intrinsics()
            coefficients = [float(value) for value in intrinsics.coeffs]
            pinhole_models = (rs.distortion.none, rs.distortion.brown_conrady,
                              rs.distortion.inverse_brown_conrady,
                              rs.distortion.modified_brown_conrady)
            if intrinsics.model not in pinhole_models or any(
                    not math.isfinite(value) or abs(value) > 1e-12 for value in coefficients):
                raise ValueError('This depth profile needs distortion rectification before pinhole mapping')
            device = profile.get_device()
            sensor = device.first_depth_sensor()
            # Global time preserves acquisition time while sharing the ROS wall
            # clock epoch. Never substitute arrival time for hardware timestamps.
            option = getattr(getattr(rs, 'option', None), 'global_time_enabled', None)
            if option is not None and sensor.supports(option):
                sensor.set_option(option, 1.)
            scale = float(sensor.get_depth_scale())
            if not math.isfinite(scale) or scale <= 0:
                raise ValueError('Camera returned an invalid depth scale')
            original = {'fx': float(intrinsics.fx), 'fy': float(intrinsics.fy),
                        'cx': float(intrinsics.ppx), 'cy': float(intrinsics.ppy)}
            if not all(math.isfinite(v) for v in original.values()) or min(original['fx'], original['fy']) <= 0:
                raise ValueError('Camera returned invalid pinhole intrinsics')
            if min(intrinsics.width, intrinsics.height) <= 0:
                raise ValueError('Camera returned invalid image dimensions')
            self.fps = int(depth_profile.fps())
            self.depth_scale = scale
            self.depth_intrinsics = {key: value / self.stride for key, value in original.items()}
            name = device.get_info(rs.camera_info.name) if device.supports(rs.camera_info.name) else 'Intel RealSense'
            self.metadata = {
                'sensor_name': name, 'fps': self.fps, 'depth_scale_m_per_unit': scale,
                'source_image_size': [int(intrinsics.width), int(intrinsics.height)],
                'image_size': [(int(intrinsics.width) + self.stride - 1) // self.stride,
                               (int(intrinsics.height) + self.stride - 1) // self.stride],
                'original_intrinsics': original, 'intrinsics': self.depth_intrinsics.copy(),
                'stride': self.stride, 'depth_filter': self.depth_filter,
                'timestamp_domain': 'not yet captured',
            }
            self._pipeline = pipeline
            self._last_timestamp = -math.inf
            self._timestamp_domain = None
            self._capture_count = self._duplicate_frames = 0
            deadline = time.monotonic() + max(5., self.warmup_frames / self.fps * 3)
            for _ in range(self.warmup_frames):
                self._wait_depth(deadline)
            return self
        except Exception as exc:
            self._pipeline = None
            if started:
                try:
                    pipeline.stop()
                except RuntimeError:
                    pass
            if isinstance(exc, (ValueError, TimeoutError, CaptureCancelled)):
                raise
            raise RuntimeError('Could not start RealSense. Check camera access, connection, '
                               'and other camera applications. ' + self._safe_error(exc)) from exc

    def _wait_depth(self, deadline):
        last_error = ''
        while True:
            self._check_cancelled()
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise TimeoutError('Timed out waiting for RealSense depth. ' + last_error)
            try:
                frames = self._pipeline.wait_for_frames(max(1, min(100, int(remaining * 1000))))
                depth = frames.get_depth_frame()
            except RuntimeError as exc:
                last_error = self._safe_error(exc)
                continue
            if not depth:
                raise RuntimeError('RealSense frameset did not contain depth')
            return depth

    def capture(self):
        if self._pipeline is None:
            raise RuntimeError('RealSense source is not started')
        deadline = time.monotonic() + self.timeout_ms / 1000
        while True:
            depth_frame = self._wait_depth(deadline)
            domain = str(depth_frame.get_frame_timestamp_domain())
            if self._timestamp_domain is not None and domain != self._timestamp_domain:
                raise RuntimeError('Camera timestamp domain changed; restart capture')
            if domain.rsplit('.', 1)[-1] not in ('global_time', 'system_time'):
                raise RuntimeError('Camera requires global_time or system_time timestamps '
                                   'for timestamped ROS TF; received ' + domain)
            timestamp = float(depth_frame.get_timestamp()) / 1000
            if not math.isfinite(timestamp):
                raise RuntimeError('Camera returned a nonfinite timestamp')
            if timestamp < self._last_timestamp:
                raise RuntimeError('Camera timestamp moved backward; restart capture')
            if timestamp == self._last_timestamp:
                self._duplicate_frames += 1
                continue
            age = time.time() - timestamp
            if age > self.max_frame_age or age < -.05:
                raise RuntimeError(f'Camera acquisition clock does not match ROS wall time '
                                   f'or frame is stale (age {age:.3f} s)')
            raw = np.asanyarray(depth_frame.get_data())
            if raw.shape != tuple(reversed(self.metadata['source_image_size'])) or raw.dtype != np.uint16:
                raise RuntimeError('Live depth dimensions or encoding changed')
            full_depth = raw.astype(np.float32) * self.depth_scale
            range_valid = ((raw != 0) & (raw != 65535) & np.isfinite(full_depth)
                           & (full_depth >= self.z_min) & (full_depth <= self.z_max))
            full_depth[~range_valid] = 0
            filtered_valid = _supported_depth_mask(full_depth) if self.depth_filter == 'supported' else range_valid
            full_depth[~filtered_valid] = 0
            depth = full_depth[::self.stride, ::self.stride].copy()
            input_count, filtered_count = int(np.count_nonzero(range_valid)), int(np.count_nonzero(filtered_valid))
            self._last_timestamp, self._timestamp_domain = timestamp, domain
            self._capture_count += 1
            self.metadata.update({
                'captured_frames': self._capture_count,
                'duplicate_frames_skipped': self._duplicate_frames,
                'timestamp_domain': domain,
                'input_range_valid_fraction': input_count / raw.size,
                'filtered_valid_fraction': filtered_count / raw.size,
                'removed_isolated_pixels': input_count - filtered_count,
                'sampled_valid_depth_fraction': float(np.count_nonzero(depth) / depth.size),
            })
            return DepthFrame(timestamp, self.depth_intrinsics.copy(), depth)

    def close(self):
        pipeline, self._pipeline = self._pipeline, None
        if pipeline is not None:
            try:
                pipeline.stop()
            except RuntimeError:
                pass

    def __enter__(self):
        return self.start()

    def __exit__(self, exc_type, exc_value, traceback):
        self.close()
