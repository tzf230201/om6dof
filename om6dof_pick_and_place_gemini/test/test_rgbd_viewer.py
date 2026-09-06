"""Pure measurement helpers used by the vision-only RGB-D viewer."""

import numpy as np
import pytest

from om6dof_pick_and_place_gemini.rgbd_source import RGBDFrame
from om6dof_pick_and_place_gemini.rgbd_viewer import (
    centre_pixel, depth_at_pixel, point_in_world)


def frame(depth):
    return RGBDFrame(
        color=np.zeros((4, 6, 3), np.uint8), depth=np.asarray(depth),
        intrinsics=(100.0, 100.0, 3.0, 2.0), depth_scale=0.001,
        stamp=1.0, frame_id="camera")


def test_centre_pixel_uses_image_dimensions():
    assert centre_pixel(frame(np.zeros((4, 6), np.uint16))) == (3, 2)


def test_depth_at_pixel_converts_raw_millimetres_to_metres():
    image = np.zeros((4, 6), np.uint16)
    image[2, 3] = 425
    assert depth_at_pixel(frame(image), 3, 2) == pytest.approx(0.425)


def test_invalid_or_outside_depth_is_reported_as_none():
    image = np.zeros((4, 6), np.uint16)
    sample = frame(image)
    assert depth_at_pixel(sample, 3, 2) is None
    assert depth_at_pixel(sample, -1, 2) is None


def test_world_measurement_applies_camera_rotation_and_translation():
    camera, world = point_in_world(
        0.5, 3, 2, (100.0, 100.0, 3.0, 2.0),
        (1.0, 2.0, 3.0), np.eye(3))
    assert camera == pytest.approx([0.0, 0.0, 0.5])
    assert world == pytest.approx([1.0, 2.0, 3.5])
