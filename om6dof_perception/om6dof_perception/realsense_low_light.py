"""Shared RealSense low-light configuration for OM6DOF camera workloads."""

from __future__ import annotations

import json
import os
from typing import Any, Dict

import cv2
import numpy as np


CONFIG_PATH = os.path.expanduser("~/.config/om6dof-realsense/low_light.json")
DEFAULT_CONFIG: Dict[str, Any] = {
    "enabled": False,
    "laser_power": 150.0,
    "brightness_threshold": 45.0,
}


def load_low_light_config(path: str = CONFIG_PATH) -> Dict[str, Any]:
    """Return safe defaults if the user configuration is absent or invalid."""
    config = dict(DEFAULT_CONFIG)
    try:
        with open(path, "r", encoding="utf-8") as handle:
            supplied = json.load(handle)
        if isinstance(supplied, dict):
            config["enabled"] = bool(supplied.get("enabled", config["enabled"]))
            config["laser_power"] = float(supplied.get("laser_power", config["laser_power"]))
            config["brightness_threshold"] = float(supplied.get("brightness_threshold", config["brightness_threshold"]))
    except (OSError, ValueError, TypeError, json.JSONDecodeError):
        pass
    config["laser_power"] = max(0.0, min(360.0, config["laser_power"]))
    config["brightness_threshold"] = max(0.0, min(255.0, config["brightness_threshold"]))
    return config


def configure_depth_sensor(profile, rs_module, config=None) -> str:
    """Apply the saved IR-emitter setting and return a human-readable result."""
    try:
        config = config or load_low_light_config()
        sensor = profile.get_device().first_depth_sensor()
        if not config["enabled"]:
            if sensor.supports(rs_module.option.emitter_enabled):
                sensor.set_option(rs_module.option.emitter_enabled, 0.0)
            return "low-light mode off (IR emitter disabled)"

        if sensor.supports(rs_module.option.emitter_enabled):
            sensor.set_option(rs_module.option.emitter_enabled, 1.0)
        if sensor.supports(rs_module.option.laser_power):
            limits = sensor.get_option_range(rs_module.option.laser_power)
            power = max(limits.min, min(limits.max, float(config["laser_power"])))
            sensor.set_option(rs_module.option.laser_power, power)
            return f"low-light mode on (IR emitter, laser power {power:.0f})"
        if sensor.supports(rs_module.option.enable_auto_exposure):
            sensor.set_option(rs_module.option.enable_auto_exposure, 1.0)
            if sensor.supports(rs_module.option.auto_exposure_limit):
                limit = sensor.get_option_range(rs_module.option.auto_exposure_limit)
                sensor.set_option(rs_module.option.auto_exposure_limit, limit.max)
            return "low-light mode on (auto exposure; no controllable IR emitter)"
        return "low-light mode on (device exposes no controllable low-light option)"
    except Exception as exc:
        # Camera streaming must remain available even with an unfamiliar device.
        return f"low-light configuration unavailable: {exc}"


def configure_color_sensor(profile, rs_module, config=None) -> str:
    """Give the RGB sensor the longest automatic exposure it supports."""
    try:
        config = config or load_low_light_config()
        if not config["enabled"]:
            return "RGB low-light mode off"

        sensors = profile.get_device().query_sensors()
        color_sensor = None
        for sensor in sensors:
            name = sensor.get_info(rs_module.camera_info.name).lower()
            has_color_stream = any(
                stream.stream_type() == rs_module.stream.color
                for stream in sensor.get_stream_profiles()
            )
            if "rgb" in name or "color" in name or has_color_stream:
                color_sensor = sensor
                break
        if color_sensor is None:
            return "RGB sensor not found"
        if color_sensor.supports(rs_module.option.enable_auto_exposure):
            color_sensor.set_option(rs_module.option.enable_auto_exposure, 1.0)
        if color_sensor.supports(rs_module.option.auto_exposure_priority):
            color_sensor.set_option(rs_module.option.auto_exposure_priority, 1.0)
        if color_sensor.supports(rs_module.option.auto_exposure_limit_toggle):
            color_sensor.set_option(
                rs_module.option.auto_exposure_limit_toggle, 1.0
            )
        if color_sensor.supports(rs_module.option.auto_exposure_limit):
            exposure = color_sensor.get_option_range(
                rs_module.option.auto_exposure_limit
            )
            color_sensor.set_option(
                rs_module.option.auto_exposure_limit, exposure.max
            )
        gain_value = None
        if color_sensor.supports(rs_module.option.auto_gain_limit_toggle):
            color_sensor.set_option(rs_module.option.auto_gain_limit_toggle, 1.0)
        if color_sensor.supports(rs_module.option.auto_gain_limit):
            gain = color_sensor.get_option_range(rs_module.option.auto_gain_limit)
            gain_value = gain.max
            color_sensor.set_option(rs_module.option.auto_gain_limit, gain_value)
        suffix = f"; gain limit {gain_value:.0f}" if gain_value is not None else ""
        return "RGB auto exposure enabled" + suffix
    except Exception as exc:
        return f"RGB low-light configuration unavailable: {exc}"


def enhance_low_light_bgr(image, config=None):
    """Lift a dark RGB scene for YOLO while retaining bright-scene colours.

    RealSense depth illumination is infrared and therefore invisible to the
    RGB stream.  This bounded gain plus local luminance equalisation makes the
    best use of the visible signal that is present without changing geometry.
    """
    config = config or load_low_light_config()
    if not config["enabled"] or image.size == 0:
        return image

    # High analog gain on the D405 produces strong chroma speckle.  A small
    # edge-preserving filter keeps object boundaries while preventing the
    # following luminance lift from magnifying individual noisy pixels.
    denoised = cv2.bilateralFilter(image, 5, 25, 25)
    lab = cv2.cvtColor(denoised, cv2.COLOR_BGR2LAB)
    luminance = lab[:, :, 0]
    scene_level = float(np.percentile(luminance, 60.0))
    threshold = float(config["brightness_threshold"])
    if scene_level >= threshold:
        return image

    gain = min(6.0, threshold / max(scene_level, 4.0))
    lifted = np.clip(luminance.astype(np.float32) * gain, 0, 255).astype(
        np.uint8
    )
    clahe = cv2.createCLAHE(clipLimit=1.5, tileGridSize=(8, 8))
    lab[:, :, 0] = clahe.apply(lifted)
    return cv2.cvtColor(lab, cv2.COLOR_LAB2BGR)
