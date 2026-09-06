"""Deprojection of a depth image into an optical-frame cloud."""

import sys
import threading
import time
from types import SimpleNamespace

import numpy as np
import pytest
from rclpy.qos import ReliabilityPolicy, qos_profile_sensor_data

from om6dof_pick_and_place_gemini.rgbd_source import (
    RealSenseSource,
    TopicSource,
    make_source,
    point_cloud,
)

INTR = (600.0, 600.0, 320.0, 240.0)


class _FakeNode:
    def __init__(self):
        self.subscriptions = []

    def create_subscription(self, msg_type, topic, callback, qos):
        subscription = SimpleNamespace(
            msg_type=msg_type, topic=topic, callback=callback, qos=qos)
        self.subscriptions.append(subscription)
        return subscription


def _header(stamp, frame_id="camera_color_optical_frame"):
    sec = int(stamp)
    nanosec = int(round((float(stamp) - sec) * 1e9))
    return SimpleNamespace(
        stamp=SimpleNamespace(sec=sec, nanosec=nanosec),
        frame_id=frame_id,
    )


def _image(array, encoding, stamp, frame_id="camera_color_optical_frame",
           **metadata):
    array = np.asarray(array)
    step = metadata.pop("step", array.strides[0])
    data = metadata.pop("data", array.tobytes())
    is_bigendian = metadata.pop("is_bigendian", False)
    message = SimpleNamespace(
        header=_header(stamp, frame_id),
        height=array.shape[0],
        width=array.shape[1],
        encoding=encoding,
        is_bigendian=is_bigendian,
        step=step,
        data=data,
    )
    for name, value in metadata.items():
        setattr(message, name, value)
    return message


def _camera_info(frame_id="camera_color_optical_frame", width=2, height=2):
    return SimpleNamespace(
        header=_header(0.0, frame_id),
        width=width,
        height=height,
        k=[600.0, 0.0, 1.0, 0.0, 600.0, 1.0, 0.0, 0.0, 1.0],
    )


def _topic_source(**kwargs):
    node = _FakeNode()
    source = TopicSource(
        node, color_topic="/color", depth_topic="/depth",
        info_topic="/info", **kwargs)
    source._on_info(_camera_info())
    return source, node


def _start_capture(source, *, warmup=0, timeout_ms=250):
    """Start capture and wait until its freshness gate is armed."""
    with source._lock:
        generation = source._capture_generation
    result = {}

    def run():
        try:
            result["frame"] = source.capture(
                warmup=warmup, timeout_ms=timeout_ms)
        except BaseException as exc:  # propagate worker failures in the test
            result["error"] = exc

    thread = threading.Thread(target=run, daemon=True)
    thread.start()
    deadline = time.monotonic() + 0.2
    while time.monotonic() < deadline:
        with source._lock:
            if source._capture_generation > generation:
                return thread, result
        time.sleep(0.001)
    pytest.fail("capture did not arm its freshness gate")


def _finish_capture(thread, result):
    thread.join(timeout=1.0)
    assert not thread.is_alive(), "capture did not finish"
    if "error" in result:
        raise result["error"]
    return result["frame"]


def test_zero_and_out_of_range_depth_is_dropped():
    depth = np.zeros((10, 10), np.uint16)
    depth[5, 5] = 500        # 0.5 m at a 0.001 scale
    depth[6, 6] = 60000      # 60 m — past z_max
    points, _, _ = point_cloud(depth, INTR, 0.001, stride=1,
                               z_min=0.05, z_max=1.0)
    assert points.shape == (1, 3)
    assert points[0, 2] == pytest.approx(0.5)


def test_the_principal_point_deprojects_onto_the_optical_axis():
    depth = np.zeros((480, 640), np.uint16)
    depth[240, 320] = 400
    points, _, pixels = point_cloud(depth, INTR, 0.001, stride=1)
    assert np.allclose(points[0], [0.0, 0.0, 0.4], atol=1e-9)
    assert np.allclose(pixels[0], [320.0, 240.0])


def test_stride_subsamples_without_shifting_the_grid():
    depth = np.full((8, 8), 500, np.uint16)
    full, _, _ = point_cloud(depth, INTR, 0.001, stride=1)
    strided, _, pixels = point_cloud(depth, INTR, 0.001, stride=2)
    assert full.shape[0] == 64 and strided.shape[0] == 16
    assert pixels[0, 0] == 0.0 and pixels[0, 1] == 0.0


def test_colours_come_back_aligned_with_the_points():
    depth = np.zeros((4, 4), np.uint16)
    depth[1, 2] = 500
    color = np.zeros((4, 4, 3), np.uint8)
    color[1, 2] = (7, 8, 9)
    points, colors, _ = point_cloud(depth, INTR, 0.001, stride=1, color=color)
    assert points.shape[0] == 1
    assert list(colors[0]) == [7, 8, 9]


def test_an_empty_depth_image_yields_an_empty_cloud():
    points, _, _ = point_cloud(np.zeros((16, 16), np.uint16), INTR, 0.001)
    assert points.shape == (0, 3)


def test_make_source_rejects_an_unknown_kind():
    with pytest.raises(ValueError, match="realsense|topic"):
        make_source("webcam")


def test_the_topic_source_needs_a_node():
    with pytest.raises(ValueError, match="node"):
        make_source("topic")


def test_topic_subscriptions_use_sensor_data_qos():
    _, node = _topic_source()

    assert len(node.subscriptions) == 3
    for subscription in node.subscriptions:
        assert subscription.qos.reliability == ReliabilityPolicy.BEST_EFFORT
        assert (subscription.qos.reliability
                == qos_profile_sensor_data.reliability)


def test_topic_source_only_returns_a_timestamp_bounded_pair():
    source, _ = _topic_source(sync_tolerance_s=0.01, sync_queue_size=2)
    old_color = np.full((2, 2, 3), 1, np.uint8)
    new_color = np.full((2, 2, 3), 9, np.uint8)
    depth = np.full((2, 2), 500, np.uint16)

    thread, result = _start_capture(source)
    source._on_color(_image(old_color, "bgr8", 1.0))
    source._on_depth(_image(depth, "16UC1", 1.2))
    assert thread.is_alive()

    source._on_color(_image(new_color, "bgr8", 1.205))
    frame = _finish_capture(thread, result)

    assert frame is not None
    assert np.array_equal(frame.color, new_color)
    assert np.array_equal(frame.depth, depth)
    assert frame.stamp == pytest.approx(1.205)
    assert frame.frame_id == "camera_color_optical_frame"


def test_topic_capture_requires_a_pair_received_after_the_request():
    source, _ = _topic_source()
    old_color = np.zeros((2, 2, 3), np.uint8)
    new_color = np.full((2, 2, 3), 7, np.uint8)
    depth = np.full((2, 2), 500, np.uint16)

    source._on_color(_image(old_color, "bgr8", 2.0))
    source._on_depth(_image(depth, "16UC1", 2.0))
    thread, result = _start_capture(source)
    assert thread.is_alive()

    source._on_color(_image(new_color, "bgr8", 3.0))
    source._on_depth(_image(depth, "16UC1", 3.0))
    frame = _finish_capture(thread, result)

    assert frame is not None
    assert np.array_equal(frame.color, new_color)


def test_topic_capture_waits_for_the_requested_warmup_pairs():
    source, _ = _topic_source()
    depth = np.full((2, 2), 500, np.uint16)
    thread, result = _start_capture(source, warmup=3)

    for index in range(2):
        color = np.full((2, 2, 3), index, np.uint8)
        source._on_color(_image(color, "bgr8", 10.0 + index))
        source._on_depth(_image(depth, "16UC1", 10.0 + index))
        assert thread.is_alive()

    expected = np.full((2, 2, 3), 2, np.uint8)
    source._on_color(_image(expected, "bgr8", 12.0))
    source._on_depth(_image(depth, "16UC1", 12.0))
    frame = _finish_capture(thread, result)

    assert frame is not None
    assert np.array_equal(frame.color, expected)


def test_topic_sync_queue_is_bounded():
    source, _ = _topic_source(sync_queue_size=2)
    color = np.zeros((2, 2, 3), np.uint8)

    for stamp in range(5):
        source._on_color(_image(color, "bgr8", float(stamp)))

    assert len(source._colors) == 2


@pytest.mark.parametrize(
    ("encoding", "depth", "expected_scale"),
    [
        ("16UC1", np.full((2, 2), 500, np.uint16), 0.001),
        ("32FC1", np.full((2, 2), 0.5, np.float32), 1.0),
    ],
)
def test_topic_depth_scale_is_inferred_from_ros_encoding(
        encoding, depth, expected_scale):
    source, _ = _topic_source()
    color = np.zeros((2, 2, 3), np.uint8)
    thread, result = _start_capture(source)
    source._on_color(_image(color, "bgr8", 3.0))
    source._on_depth(_image(depth, encoding, 3.0))

    frame = _finish_capture(thread, result)

    assert frame is not None
    assert frame.depth.dtype == depth.dtype
    assert frame.depth_scale == pytest.approx(expected_scale)


def test_topic_depth_metadata_takes_precedence_over_encoding():
    source, _ = _topic_source()
    color = np.zeros((2, 2, 3), np.uint8)
    depth = np.full((2, 2), 2000, np.uint16)
    thread, result = _start_capture(source)
    source._on_color(_image(color, "bgr8", 4.0))
    source._on_depth(_image(
        depth, "16UC1", 4.0, metadata={"depth_scale": 0.00025}))

    frame = _finish_capture(thread, result)

    assert frame is not None
    assert frame.depth_scale == pytest.approx(0.00025)


def test_topic_source_rejects_depth_in_a_different_projection_frame():
    source, _ = _topic_source()
    color = np.zeros((2, 2, 3), np.uint8)
    depth = np.full((2, 2), 500, np.uint16)
    thread, result = _start_capture(source, timeout_ms=40)

    source._on_color(_image(color, "bgr8", 5.0))
    source._on_depth(_image(
        depth, "16UC1", 5.0, frame_id="camera_depth_optical_frame"))
    frame = _finish_capture(thread, result)

    assert frame is None
    assert "frames differ" in source.last_error


@pytest.mark.parametrize(
    "camera_info",
    [
        _camera_info(frame_id="camera_depth_optical_frame"),
        _camera_info(width=4, height=2),
    ],
)
def test_topic_source_rejects_mismatched_camera_info(camera_info):
    source, _ = _topic_source()
    source._on_info(camera_info)
    color = np.zeros((2, 2, 3), np.uint8)
    depth = np.full((2, 2), 500, np.uint16)
    thread, result = _start_capture(source, timeout_ms=40)

    source._on_color(_image(color, "bgr8", 6.0))
    source._on_depth(_image(depth, "16UC1", 6.0))
    frame = _finish_capture(thread, result)

    assert frame is None
    assert "CameraInfo" in source.last_error


def test_image_decoder_skips_ros_row_padding():
    # Two BGR pixels (six bytes) plus two padding bytes in each row.
    wire = bytes([
        1, 2, 3, 4, 5, 6, 99, 99,
        7, 8, 9, 10, 11, 12, 99, 99,
    ])
    template = np.zeros((2, 2, 3), np.uint8)
    message = _image(
        template, "bgr8", 1.0, step=8, data=wire)

    decoded = TopicSource._image_to_array(message)

    assert decoded.tolist() == [
        [[1, 2, 3], [4, 5, 6]],
        [[7, 8, 9], [10, 11, 12]],
    ]


def test_image_decoder_converts_big_endian_depth_to_native_values():
    expected = np.array([[1, 500], [1000, 65535]], np.uint16)
    wire = expected.astype(">u2").tobytes()
    message = _image(
        expected, "16UC1", 1.0, is_bigendian=True, data=wire)

    decoded = TopicSource._image_to_array(message)

    assert decoded.dtype.isnative
    assert np.array_equal(decoded, expected)


def test_direct_realsense_keeps_points_in_depth_optical_frame_and_metadata():
    aligned_intrinsics = SimpleNamespace(
        fx=610.0, fy=611.0, ppx=319.5, ppy=239.5)
    profile = SimpleNamespace(
        as_video_stream_profile=lambda: SimpleNamespace(
            get_intrinsics=lambda: aligned_intrinsics))

    class _Frame:
        def __init__(self, array, *, units=None):
            self._array = array
            self.profile = profile
            self._units = units

        def get_data(self):
            return self._array

        def get_units(self):
            if self._units is None:
                raise AttributeError
            return self._units

    color_frame = _Frame(np.zeros((2, 2, 3), np.uint8))
    depth_frame = _Frame(np.full((2, 2), 500, np.uint16), units=0.00025)
    aligned = SimpleNamespace(
        get_color_frame=lambda: color_frame,
        get_depth_frame=lambda: depth_frame,
    )
    clock = SimpleNamespace(
        now=lambda: SimpleNamespace(nanoseconds=12_345_678_900))
    source = RealSenseSource(
        optical_frame_id="d405_depth_optical_frame", clock=clock)
    source._pipe = SimpleNamespace(wait_for_frames=lambda timeout: object())
    source._align = SimpleNamespace(process=lambda frames: aligned)
    source._intrinsics = INTR
    source._depth_scale = 0.001

    frame = source.capture(warmup=1)

    assert frame is not None
    assert frame.frame_id == "d405_depth_optical_frame"
    assert frame.intrinsics == (610.0, 611.0, 319.5, 239.5)
    assert frame.depth_scale == pytest.approx(0.00025)
    assert frame.stamp == pytest.approx(12.3456789)


def test_direct_realsense_aligns_color_into_the_depth_viewport(monkeypatch):
    calls = {"streams": []}
    color_stream = object()
    depth_stream = object()
    intrinsics = SimpleNamespace(fx=600.0, fy=601.0, ppx=320.0, ppy=240.0)

    class _Config:
        def enable_device(self, serial):
            calls["serial"] = serial

        def enable_stream(self, *args):
            calls["streams"].append(args)

    class _Profile:
        def get_device(self):
            sensor = SimpleNamespace(get_depth_scale=lambda: 0.001)
            return SimpleNamespace(first_depth_sensor=lambda: sensor)

        def get_stream(self, stream):
            calls["intrinsics_stream"] = stream
            return SimpleNamespace(
                as_video_stream_profile=lambda: SimpleNamespace(
                    get_intrinsics=lambda: intrinsics))

    pipeline = SimpleNamespace(start=lambda config: _Profile())
    fake_rs = SimpleNamespace(
        pipeline=lambda: pipeline,
        config=_Config,
        stream=SimpleNamespace(color=color_stream, depth=depth_stream),
        format=SimpleNamespace(bgr8=object(), z16=object()),
        align=lambda stream: calls.setdefault("align_stream", stream),
    )
    monkeypatch.setitem(sys.modules, "pyrealsense2", fake_rs)

    source = RealSenseSource(serial="test-camera")
    source.start()

    assert calls["align_stream"] is depth_stream
    assert calls["intrinsics_stream"] is depth_stream
