"""Versioned camera contracts and SDK point-cloud registration (no device I/O)."""

from dataclasses import dataclass
import re

import numpy as np


@dataclass(frozen=True)
class CameraModel:
    version: str
    camera_model: str
    optical_frame: str
    default_ee_mode: str


def camera_model_for(version, model="auto"):
    profiles = {
        "v1": CameraModel("v1", "D405", "camera_color_optical_frame",
                          "fixed_jaw_pixels"),
        "v2": CameraModel("v2", "D435", "d435_color_optical_frame", "disabled"),
    }
    if version not in profiles:
        raise ValueError("model_version must be v1 or v2")
    profile = profiles[version]
    if model == "auto":
        return profile
    allowed = {"v1": ("D405",), "v2": ("D435", "D435i")}
    canonical = next((name for name in allowed[version]
                      if name.lower() == model.lower()), None)
    if canonical is None:
        raise ValueError(f"camera_model={model!r} is not supported for {version}")
    # Official D435i description reuses D435 RGB/depth geometry and adds IMU.
    # Keep the existing RGB optical frame; this node does not stream the IMU.
    return CameraModel(version, canonical, profile.optical_frame, profile.default_ee_mode)


def select_camera(cameras, expected_model, requested_serial=""):
    """Select exactly one matching SDK identity; never fall back to another model.

    ``cameras`` contains dictionaries with ``name`` and ``serial``. A caller
    should keep the selected serial on reconnect, so calibration cannot jump
    silently to another unit. D435i requires explicit selection; D435f is not
    treated as a D435. Sensor-to-arm calibration remains a separate concern.
    """
    matching = [camera for camera in cameras
                if re.search(r"\b" + re.escape(expected_model) + r"\b",
                             camera["name"], re.IGNORECASE)
                and (not requested_serial or
                     camera["serial"] == requested_serial)]
    if not matching:
        raise ValueError(f"No {expected_model} camera matches camera_serial; "
                         "check model_version and the connected device")
    if len(matching) != 1:
        raise ValueError(f"Multiple {expected_model} cameras: set camera_serial")
    return dict(matching[0])


def resolve_ee_mode(profile, mode, pixels_calibrated=False):
    mode = profile.default_ee_mode if mode == "auto" else mode
    if mode not in ("fixed_jaw_pixels", "yolo", "disabled"):
        raise ValueError("ee_mode must be auto, fixed_jaw_pixels, yolo or disabled")
    if profile.version == "v2" and mode == "fixed_jaw_pixels" and not pixels_calibrated:
        raise ValueError("D435 jaw pixels require new calibration: set "
                         "ee_left_pixel/ee_right_pixel and ee_pixels_calibrated:=true")
    return mode


def select_stream_fps(color_rates, depth_rates, requested, low_light=False):
    """Use only rates advertised for both requested stream profiles.

    Low-light selects the slowest common device rate, not a D405-specific 5 Hz
    assumption. Normal mode fails explicitly if the requested rate is absent.
    """
    common = set(color_rates) & set(depth_rates)
    if not common:
        raise ValueError("No common color/depth FPS for the requested image size/formats")
    if low_light:
        return min(common)
    if requested not in common:
        raise ValueError(f"camera_fps={requested} unsupported; common FPS: {sorted(common)}")
    return requested


def color_point_map(vertices, texture_uv, rotation, translation, height, width):
    """Organize native depth vertices in the *color optical* coordinate system.

    UVs come from SDK pointcloud.map_to(color), which uses the device's actual
    color distortion and depth-to-color calibration. Vertices are in the native
    depth optical frame, so they MUST also be transformed, not just relabelled.
    The SDK stores rs2_extrinsics.rotation column-major. Nearest color-Z wins
    when several depth samples project onto one color pixel; holes stay NaN.
    """
    vertices = np.asarray(vertices, dtype=np.float64).reshape(-1, 3)
    uv = np.asarray(texture_uv, dtype=np.float64).reshape(-1, 2)
    rotation = np.asarray(rotation, dtype=np.float64).reshape(3, 3, order="F")
    translation = np.asarray(translation, dtype=np.float64).reshape(3)
    if len(vertices) != len(uv) or height <= 0 or width <= 0:
        raise ValueError("Invalid point-cloud/image dimensions")
    if not np.all(np.isfinite(rotation)) or not np.all(np.isfinite(translation)):
        raise ValueError("Non-finite depth-to-color extrinsics")
    valid = (np.isfinite(vertices).all(axis=1) & (vertices[:, 2] > 0) &
             np.isfinite(uv).all(axis=1) & (uv >= 0).all(axis=1) &
             (uv < 1).all(axis=1))
    points = vertices[valid] @ rotation.T + translation
    pixels = np.floor(uv[valid] * [width, height] + 0.5).astype(np.int64)
    valid = ((pixels[:, 0] < width) & (pixels[:, 1] < height) &
             np.isfinite(points).all(axis=1) & (points[:, 2] > 0))
    points, pixels = points[valid], pixels[valid]
    indices = pixels[:, 1] * width + pixels[:, 0]
    nearest = np.full(height * width, np.inf)
    np.minimum.at(nearest, indices, points[:, 2])
    visible = points[:, 2] == nearest[indices]
    result = np.full((height * width, 3), np.nan, dtype=np.float32)
    result[indices[visible]] = points[visible]
    return result.reshape(height, width, 3)
