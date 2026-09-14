"""Acquisition, calibration and ROS wire contracts using an explicit fake SDK."""
import os
import sys
import threading
from types import SimpleNamespace as NS
import unittest
from unittest.mock import patch

import numpy as np
import rclpy
from rclpy.parameter import Parameter

from om6dof_f_gng.realsense_source import (
    CaptureCancelled, DepthFrame, RealSenseSource, _supported_depth_mask,
)
from om6dof_f_gng.realsense_depth_node import RealSenseDepthNode, frame_messages


class Depth:
    def __init__(self, timestamp, domain='global_time', data=None):
        self.timestamp, self.domain, self.data = timestamp, domain, data

    def get_timestamp(self):
        return self.timestamp * 1000

    def get_frame_timestamp_domain(self):
        return self.domain

    def get_data(self):
        if self.data is not None:
            return self.data
        data = np.full((6, 8), 1500, np.uint16)
        data[0, 0], data[0, 2], data[0, 4], data[0, 6] = 65535, 7000, 100, 1000
        return data


class FakeSDK:
    def __init__(self, distorted=False):
        self.distortion = NS(none=0, brown_conrady=1, inverse_brown_conrady=2, modified_brown_conrady=3)
        self.stream, self.format = NS(depth=1), NS(z16=1)
        self.option = NS(global_time_enabled=4)
        self.camera_info = NS(name=1)
        self.depths = [Depth(t) for t in (1000., 1000.1, 1000.2, 1000.2, 1000.3, 1000.1)]
        self.stopped = False
        self.options = []
        self.stream_options = None
        intr = NS(width=8, height=6, fx=400., fy=420., ppx=3.5, ppy=2.5,
                  model=1, coeffs=[.1 if distorted else 0, 0, 0, 0, 0])
        video = NS(get_intrinsics=lambda: intr, fps=lambda: 15)
        sensor = NS(get_depth_scale=lambda: .001, supports=lambda option: True,
                    set_option=lambda *args: self.options.append(args))
        device = NS(first_depth_sensor=lambda: sensor, supports=lambda info: False)
        self.profile = NS(get_stream=lambda stream: NS(as_video_stream_profile=lambda: video),
                          get_device=lambda: device)

    def pipeline(self):
        return self

    def config(self):
        return NS(enable_stream=self.set_stream, enable_device=lambda *args: None)

    def set_stream(self, *args):
        self.stream_options = args

    def start(self, config):
        return self.profile

    def stop(self):
        self.stopped = True

    def wait_for_frames(self, timeout_ms):
        if not self.depths:
            raise RuntimeError('no frames')
        return NS(get_depth_frame=lambda depth=self.depths.pop(0): depth)


class RealSenseTests(unittest.TestCase):
    def setUp(self):
        self.clock = patch('om6dof_f_gng.realsense_source.time.time', return_value=1000.4)
        self.clock.start()
        self.addCleanup(self.clock.stop)

    def test_calibration_stride_units_invalid_depth_and_chronology(self):
        sdk = FakeSDK()
        with patch.dict(sys.modules, {'pyrealsense2': sdk}):
            with RealSenseSource(stride=2, warmup_frames=2, depth_filter='none') as source:
                frame = source.capture()
                self.assertEqual(frame.timestamp, 1000.2)
                self.assertEqual(frame.depth.shape, (3, 4))
                np.testing.assert_allclose(frame.depth[0], [0, 0, 0, 1])
                np.testing.assert_allclose(frame.depth[1], [1.5] * 4)
                self.assertEqual(frame.intrinsics, {'fx': 200., 'fy': 210., 'cx': 1.75, 'cy': 1.25})
                self.assertEqual(source.capture().timestamp, 1000.3)
                self.assertEqual(source.metadata['duplicate_frames_skipped'], 1)
                self.assertEqual(sdk.stream_options, (1, 640, 480, 1, 15))
                self.assertEqual(sdk.options, [(4, 1.)])
                with self.assertRaisesRegex(RuntimeError, 'backward'):
                    source.capture()
                self.assertEqual(source.metadata['captured_frames'], 2)
        self.assertTrue(sdk.stopped)

    def test_supported_filter_retains_discontinuities_and_holes(self):
        data = np.full((6, 8), 1000, np.uint16)
        data[:, 4:] = 2000
        data[2, 2] = 0
        sdk = FakeSDK()
        sdk.depths = [Depth(1000.2, data=data)]
        with patch.dict(sys.modules, {'pyrealsense2': sdk}):
            with RealSenseSource(stride=1, warmup_frames=0) as source:
                np.testing.assert_allclose(source.capture().depth, data.astype(np.float32) * .001)
                self.assertEqual(source.metadata['removed_isolated_pixels'], 0)

    def test_supported_filter_uses_neighbours_before_stride(self):
        data = np.zeros((6, 8), np.uint16)
        data[2:4, 2:4] = 1000
        sdk = FakeSDK()
        sdk.depths = [Depth(1000.2, data=data)]
        with patch.dict(sys.modules, {'pyrealsense2': sdk}):
            with RealSenseSource(stride=2, warmup_frames=0) as source:
                frame = source.capture()
                self.assertEqual(frame.depth[1, 1], 1.)
                self.assertEqual(np.count_nonzero(frame.depth), 1)

    def test_supported_filter_removes_spike_without_changing_values(self):
        data = np.full((6, 8), 1000, np.uint16)
        data[2, 2], data[0, 0] = 2000, 0
        sdk = FakeSDK()
        sdk.depths = [Depth(1000.2, data=data)]
        with patch.dict(sys.modules, {'pyrealsense2': sdk}):
            with RealSenseSource(stride=2, warmup_frames=0) as source:
                frame = source.capture()
                expected = data[::2, ::2].astype(np.float32) * .001
                expected[1, 1] = 0
                np.testing.assert_allclose(frame.depth, expected)
                self.assertEqual(source.metadata['removed_isolated_pixels'], 1)
                self.assertAlmostEqual(source.metadata['input_range_valid_fraction'], 47 / 48)
                self.assertAlmostEqual(source.metadata['filtered_valid_fraction'], 46 / 48)

    def test_no_recursive_filter_erosion_or_hole_fill(self):
        depth = np.zeros((4, 4), np.float32)
        depth[1, 1:3] = 1.
        depth[2, 1] = 1.
        np.testing.assert_array_equal(_supported_depth_mask(depth), depth > 0)

    def test_hardware_clock_rejected_without_committing_frame(self):
        sdk = FakeSDK()
        sdk.depths = [Depth(1000.2, 'hardware_clock')]
        with patch.dict(sys.modules, {'pyrealsense2': sdk}):
            with RealSenseSource(warmup_frames=0) as source:
                with self.assertRaisesRegex(RuntimeError, 'requires global_time'):
                    source.capture()
                self.assertEqual(source._capture_count, 0)

    def test_timestamp_domain_change_rejected(self):
        sdk = FakeSDK()
        sdk.depths = [Depth(1000.2), Depth(1000.3, 'system_time')]
        with patch.dict(sys.modules, {'pyrealsense2': sdk}):
            with RealSenseSource(warmup_frames=0) as source:
                source.capture()
                with self.assertRaisesRegex(RuntimeError, 'timestamp domain changed'):
                    source.capture()
                self.assertEqual(source._capture_count, 1)
                self.assertEqual(source._last_timestamp, 1000.2)

    def test_stale_future_and_nonfinite_timestamps_rejected(self):
        for timestamp in (999., 1001., float('nan'), float('inf')):
            with self.subTest(timestamp=timestamp):
                sdk = FakeSDK()
                sdk.depths = [Depth(timestamp)]
                with patch.dict(sys.modules, {'pyrealsense2': sdk}):
                    with RealSenseSource(warmup_frames=0) as source:
                        with self.assertRaises(RuntimeError):
                            source.capture()
                        self.assertEqual(source._capture_count, 0)

    def test_distortion_rejected_and_pipeline_released(self):
        sdk = FakeSDK(distorted=True)
        with patch.dict(sys.modules, {'pyrealsense2': sdk}):
            with self.assertRaisesRegex(ValueError, 'rectification'):
                RealSenseSource(warmup_frames=0).start()
        self.assertTrue(sdk.stopped)

    def test_shutdown_cancels_wait_and_releases_camera(self):
        sdk, stop = FakeSDK(), threading.Event()

        def stalled_capture(timeout_ms):
            self.assertLessEqual(timeout_ms, 100)
            stop.set()
            raise RuntimeError('no frame')

        sdk.wait_for_frames = stalled_capture
        with patch.dict(sys.modules, {'pyrealsense2': sdk}):
            with RealSenseSource(warmup_frames=0, stop_event=stop) as source:
                with self.assertRaises(CaptureCancelled):
                    source.capture()
        self.assertTrue(sdk.stopped)

    def test_invalid_options_fail_before_sdk_import(self):
        for kwargs in ({'fps': 60}, {'stride': 0}, {'z_min': 3., 'z_max': 1.},
                       {'depth_filter': 'fill'}, {'max_frame_age': 0.}):
            with self.subTest(kwargs=kwargs), self.assertRaises(ValueError):
                RealSenseSource(**kwargs)


class ROSWireTests(unittest.TestCase):
    def test_actual_humble_constructor_keeps_typed_readonly_parameters(self):
        # Real rclpy parameter declaration catches descriptor/type errors that
        # mocked Node methods miss. Only DDS is real; no SDK/device is opened.
        class FakeSource:
            def __init__(self, **options):
                self.stop = options['stop_event']
                self.metadata = {}
                self.closed = False

            def __enter__(self):
                return self

            def capture(self):
                self.stop.wait(3.)
                raise CaptureCancelled()

            def __exit__(self, *args):
                self.closed = True

        node = None
        with patch.dict(os.environ, {'ROS_LOCALHOST_ONLY': '1'}), patch(
                'om6dof_f_gng.realsense_depth_node.RealSenseSource', FakeSource):
            rclpy.init(args=[], domain_id=83)
            try:
                node = RealSenseDepthNode()
                self.assertIs(node.get_parameter('use_sim_time').value, False)
                for name in ('use_sim_time', 'fps', 'stride', 'serial', 'timeout_ms',
                             'min_depth', 'max_depth', 'warmup_frames', 'depth_filter',
                             'max_frame_age', 'camera_frame'):
                    descriptor = node.describe_parameter(name)
                    self.assertTrue(descriptor.read_only, name)
                    self.assertEqual(descriptor.type, node.get_parameter(name).type_.value, name)
                result = node.set_parameters([Parameter('use_sim_time', value=True)])[0]
                self.assertFalse(result.successful)
            finally:
                if node is not None:
                    stopped = node.close()
                    closed = node._source.closed
                    node.destroy_node()
                rclpy.shutdown()
            self.assertTrue(stopped)
            self.assertTrue(closed)

    def test_message_pair_preserves_calibration_and_acquisition_stamp(self):
        depth = np.array([[0., 1.25], [2., 3.]], dtype=np.float32)
        frame = DepthFrame(1000.123456789,
                           {'fx': 200., 'fy': 210., 'cx': .7, 'cy': .8}, depth)
        image, info = frame_messages(frame, 'd435_depth_optical_frame')
        self.assertEqual(image.header, info.header)
        self.assertEqual(image.header.stamp.sec, 1000)
        self.assertEqual(image.header.stamp.nanosec, 123456789)
        self.assertEqual(image.header.frame_id, 'd435_depth_optical_frame')
        self.assertEqual((image.height, image.width, image.step, image.is_bigendian), (2, 2, 8, 0))
        self.assertEqual(image.encoding, '32FC1')
        np.testing.assert_array_equal(np.frombuffer(image.data, dtype='<f4').reshape(2, 2), depth)
        np.testing.assert_allclose(info.k, [200., 0., .7, 0., 210., .8, 0., 0., 1.])
        np.testing.assert_allclose(info.p, [200., 0., .7, 0., 0., 210., .8, 0., 0., 0., 1., 0.])
        self.assertEqual(info.distortion_model, 'plumb_bob')
        self.assertEqual(list(info.d), [0.] * 5)

    def test_publisher_never_restamps_or_publishes_delayed_frame(self):
        frame = DepthFrame(1000., {}, np.ones((2, 2), np.float32))
        node = NS(_lock=threading.Lock(), _pending=(frame, {}), _failure=None,
                  _max_frame_age=.5, _stale_frames=0)
        with patch('om6dof_f_gng.realsense_depth_node.time.time', return_value=1001.):
            RealSenseDepthNode._publish_pending(node)
        self.assertEqual(node._stale_frames, 1)
        self.assertIsNone(node._pending)

    def test_worker_failure_propagates_to_executor(self):
        node = NS(_lock=threading.Lock(), _pending=None, _failure='camera disconnected')
        with self.assertRaisesRegex(RuntimeError, 'camera disconnected'):
            RealSenseDepthNode._publish_pending(node)


if __name__ == '__main__':
    unittest.main()
