"""Shared RealSense low-light configuration for OM6DOF camera workloads.

The setting is deliberately stored outside either ROS package so it survives
rebuilds and is shared by perception and DD-GNG.  It only configures the depth
sensor: YOLO object names still need visible light on the RGB camera.
"""

from __future__ import annotations

import json
import os
from typing import Any, Dict


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
