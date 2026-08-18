import cv2
import numpy as np

from om6dof_perception.realsense_low_light import enhance_low_light_bgr


def config(enabled=True, threshold=45.0):
    return {
        "enabled": enabled,
        "laser_power": 150.0,
        "brightness_threshold": threshold,
    }


def test_dark_rgb_frame_is_lifted_for_yolo():
    gradient = np.tile(np.arange(0, 16, dtype=np.uint8), (80, 5))
    image = cv2.merge((gradient, gradient, gradient))

    enhanced = enhance_low_light_bgr(image, config())

    assert enhanced.mean() > image.mean() * 3.0
    assert enhanced.shape == image.shape
    assert enhanced.dtype == np.uint8


def test_bright_frame_is_not_modified():
    image = np.full((40, 60, 3), 120, dtype=np.uint8)

    enhanced = enhance_low_light_bgr(image, config())

    assert enhanced is image


def test_disabled_mode_is_not_modified():
    image = np.full((40, 60, 3), 4, dtype=np.uint8)

    enhanced = enhance_low_light_bgr(image, config(enabled=False))

    assert enhanced is image
