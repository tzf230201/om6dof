from types import SimpleNamespace
import json
import threading

import numpy as np
import pytest

from om6dof_perception.camera_model import (
    camera_model_for, color_point_map, resolve_ee_mode, select_camera, select_stream_fps,
)
from om6dof_perception.perception_node import Entity, PerceptionNode, estimate_bbox3d


def test_versions_have_distinct_camera_and_frame_contracts():
    v1, v2 = camera_model_for("v1"), camera_model_for("v2")
    assert (v1.camera_model, v1.optical_frame) == ("D405", "camera_color_optical_frame")
    assert (v2.camera_model, v2.optical_frame) == ("D435", "d435_color_optical_frame")
    assert resolve_ee_mode(v1, "auto") == "fixed_jaw_pixels"
    assert resolve_ee_mode(v2, "auto") == "disabled"
    with pytest.raises(ValueError):
        camera_model_for("v3")


def test_v2_refuses_inherited_jaw_pixels_without_new_calibration():
    with pytest.raises(ValueError, match="require new calibration"):
        resolve_ee_mode(camera_model_for("v2"), "fixed_jaw_pixels")
    assert resolve_ee_mode(camera_model_for("v2"), "fixed_jaw_pixels", True) == "fixed_jaw_pixels"


def test_explicit_d435i_uses_shared_rgb_geometry_but_distinct_identity():
    profile = camera_model_for("v2", "D435i")
    assert profile.camera_model == "D435i"
    assert profile.optical_frame == "d435_color_optical_frame"
    assert resolve_ee_mode(profile, "auto") == "disabled"
    devices = [{"name": "Intel RealSense D435I", "serial": "001"}]
    assert select_camera(devices, profile.camera_model)["serial"] == "001"
    with pytest.raises(ValueError):
        camera_model_for("v1", "D435i")
    with pytest.raises(ValueError):
        camera_model_for("v2", "D405")
    with pytest.raises(ValueError):
        camera_model_for("v2", "D435f")


def test_device_selection_matches_model_and_serial_without_fallback():
    devices = [{"name": "Intel RealSense D405", "serial": "01"},
               {"name": "Intel RealSense D435", "serial": "000123"}]
    assert select_camera(devices, "D435")["serial"] == "000123"
    with pytest.raises(ValueError, match="No D435"):
        select_camera(devices, "D435", "01")
    with pytest.raises(ValueError, match="No D435"):
        select_camera([{"name": "Intel RealSense D435I", "serial": "2"}], "D435")


def test_multiple_devices_require_serial_and_reconnect_stays_on_same_unit():
    devices = [{"name": "Intel RealSense D435", "serial": serial}
               for serial in ["1", "2"]]
    with pytest.raises(ValueError, match="Multiple"):
        select_camera(devices, "D435")
    selected = select_camera(devices, "D435", "1")
    with pytest.raises(ValueError):
        select_camera(devices[1:], "D435", selected["serial"])


def test_color_points_use_column_major_rotation_and_translation():
    rotation = np.array([[0., -1., 0.], [1., 0., 0.], [0., 0., 1.]])
    points = color_point_map([[.2, .3, .5]], [[.5, .5]],
                             rotation.flatten(order="F"), [.01, .02, .03], 4, 4)
    assert np.allclose(points[2, 2], [-.29, .22, .53])
    assert np.isnan(points[0, 0]).all()


def test_low_light_fps_comes_from_device_capabilities_not_d405_default():
    assert select_stream_fps([6, 15, 30], [6, 30, 60], 30, True) == 6
    assert select_stream_fps([5, 30], [5, 30], 30, True) == 5
    assert select_stream_fps([6, 30], [6, 30], 30) == 30
    with pytest.raises(ValueError, match="unsupported"):
        select_stream_fps([6, 30], [6, 30], 5)
    with pytest.raises(ValueError, match="No common"):
        select_stream_fps([30], [60], 30)


def test_color_registration_z_buffer_and_invalid_depth():
    points = color_point_map(
        [[0., 0., 2.], [.01, .02, .5], [0., 0., 0.], [0., 0., 1.], [np.nan, 0., 1.]],
        [[.5, .5], [.5, .5], [0., 0.], [2., 2.], [.25, .25]],
        np.eye(3).flatten(), [0., 0., 0.], 4, 4)
    assert np.allclose(points[2, 2], [.01, .02, .5])
    assert np.isfinite(points[:, :, 2]).sum() == 1


def test_v2_bbox_uses_registered_xyz_not_an_unrelated_pinhole():
    intr = SimpleNamespace(fx=1., fy=1., ppx=0., ppy=0.)
    rows, cols = np.indices((10, 10))
    xyz = np.stack([.1 + cols * .001, .2 + rows * .001,
                    np.full((10, 10), .5)], axis=2)
    box = estimate_bbox3d(xyz[:, :, 2], intr, 1., (0, 0, 10, 10), points_xyz=xyz)
    assert .1 < box["center"][0] < .11
    assert .2 < box["center"][1] < .21
    assert box["front_z"] == .5
    xyz[:] = np.nan
    assert estimate_bbox3d(np.full((10, 10), .5), intr, 1., (0, 0, 10, 10),
                           points_xyz=xyz) is None


def _bare_node():
    node = object.__new__(PerceptionNode)
    node.frame_lock = threading.Lock()
    node.stop_event = threading.Event()
    node.points_xyz = None
    node.depth = np.full((8, 12), .5)
    node.intr = SimpleNamespace(fx=1., fy=1., ppx=0., ppy=0.)
    node.depth_scale = 1.
    return node


def test_pixel_coordinates_use_actual_resolution_and_reject_outside_pixels():
    node = _bare_node()
    node.points_xyz = np.full((8, 12, 3), np.nan)
    node.points_xyz[7, 11] = [.1, .2, .3]
    assert np.allclose(node._pixel_to_point(11, 7, 0), [.1, .2, .3])
    assert node._pixel_to_point(12, 7, 0) is None
    assert node._pixel_to_point(-1, 0, 0) is None
    assert node._pixel_to_point(0, 0, 0) is None


def test_disconnect_invalidates_camera_epoch_and_observations():
    node = _bare_node()
    node.rgb = np.ones((8, 12, 3))
    node.target, node.ee = Entity("target", "bottle"), Entity("ee", "gripper")
    node.target.state, node.target.point = "tracking", [.1, .2, .3]
    node.target.bbox3d = {"center": [.1, .2, .3]}
    node.ee_mode = "disabled"
    node.camera_epoch = 0
    node._publish = lambda stamp: None
    node.get_clock = lambda: SimpleNamespace(now=lambda: SimpleNamespace(to_msg=lambda: None))
    node._camera_disconnected("unplugged")
    assert node.target.point is node.target.bbox3d is node.rgb is None
    assert node.camera_epoch == 1
    assert node.ee.state == "disabled"
    assert node.camera_metadata["connected"] is False


def test_late_detector_cannot_restore_disconnected_camera_data():
    node = _bare_node()
    node.detection_lock = threading.Lock()
    node.camera_epoch = 2
    node.stop_event = threading.Event()
    node.yolo = SimpleNamespace(detect=lambda _image: [])
    ent = Entity("target", "bottle", "bottle")
    node._acquire(ent, np.zeros((8, 12, 3), dtype=np.uint8), 1)
    assert ent.state == "acquiring"
    assert ent.tracker is None


def test_v2_publishes_distinct_frame_and_model_metadata():
    from builtin_interfaces.msg import Time
    node = _bare_node()
    node.camera_profile = camera_model_for("v2")
    node.camera_frame = node.camera_profile.optical_frame
    node.camera_metadata = {"connected": True}
    node.target, node.ee = Entity("target", "bottle"), Entity("ee", "gripper")
    node.target.point, node.target.state = [.1, .2, .5], "tracking"
    node.ee.state, node.ee_mode = "disabled", "disabled"
    outputs = {}
    for name in ("target", "ee", "target_bbox3d", "rel", "distance", "status"):
        outputs[name] = []
        setattr(node, "pub_" + name, SimpleNamespace(publish=outputs[name].append))
    stamp = Time(sec=123, nanosec=456)
    node._publish(stamp)
    target = outputs["target"][0]
    assert target.header.frame_id == "d435_color_optical_frame"
    assert target.header.stamp == stamp
    status = json.loads(outputs["status"][0].data)
    assert status["model_version"] == "v2"
    assert status["camera_model"] == "D435"
    assert status["camera_frame"] == target.header.frame_id
    assert status["ee"]["mode"] == "disabled"
    assert outputs["ee"] == outputs["rel"] == []
    compressed = node._compressed(np.array([1, 2], dtype=np.uint8), stamp)
    assert compressed.header.frame_id == target.header.frame_id
