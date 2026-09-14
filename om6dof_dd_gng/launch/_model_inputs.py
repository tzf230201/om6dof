"""Offline model inputs shared by the two DD-GNG launch descriptions.

V1 bytes and provenance remain unchanged. V2 always uses a separate TF tree;
its state publisher consumes measured joint states but never controls hardware.
"""

import hashlib
import os
import shutil
import subprocess
import xml.etree.ElementTree as ET
import math
from pathlib import Path

from launch.substitutions import LaunchConfiguration
import yaml


# These remaps also protect custom RViz/parameter files that still contain
# the old V1 default topic/service names when used with a V2 launch.
GRAPH_ENDPOINTS = (
    "environment_graph", "environment_graph_data", "robot_graph", "labels",
    "status", "reachability_query", "reachability_graph_data",
    "reachability_graph", "reachability_plan", "reachability_path",
    "reachability_training_samples", "reachability_training_samples_cloud",
    "rebuild_reachability", "plan_reachability", "validate_reachability_scene",
)


def _sha256(payload):
    return hashlib.sha256(payload).hexdigest()


def load_hand_eye_calibration(path):
    """Validate one measured eye-in-hand artifact and return ROS parameters.

    The artifact describes ``end_effector_link <- color optical``.  The native
    depth pose is derived at runtime from the connected RealSense device's
    factory depth-to-colour extrinsic, so RGB inference stays on the raw colour
    image while DD-GNG remains in the native depth geometry.
    """
    if not path:
        return {}
    calibration_path = Path(path).expanduser().resolve(strict=True)
    raw = calibration_path.read_bytes()
    document = yaml.safe_load(raw)
    if not isinstance(document, dict) or document.get("schema") != "om6dof.hand_eye.v1":
        raise RuntimeError("camera calibration schema must be om6dof.hand_eye.v1")
    camera = document.get("camera")
    transform = document.get("transform")
    validation = document.get("validation")
    if not all(isinstance(value, dict) for value in (camera, transform, validation)):
        raise RuntimeError("camera calibration requires camera, transform, and validation mappings")
    serial = str(camera.get("serial", "")).strip()
    model = str(camera.get("model", "")).strip().upper()
    parent = str(transform.get("parent_frame", "")).strip()
    child = str(transform.get("child_frame", "")).strip()
    xyz = transform.get("xyz")
    xyzw = transform.get("quaternion_xyzw")
    method = str(validation.get("method", "")).strip()
    try:
        samples = int(validation.get("samples", 0))
        translation_rmse = float(validation.get("translation_rmse_m"))
        rotation_rmse = float(validation.get("rotation_rmse_rad"))
        xyz = [float(value) for value in xyz]
        xyzw = [float(value) for value in xyzw]
    except (TypeError, ValueError):
        raise RuntimeError("camera calibration has invalid numeric fields") from None
    if not serial or model not in ("D435", "D435I"):
        raise RuntimeError("camera calibration requires a D435/D435i model and exact serial")
    if parent != "end_effector_link" or child != "d435_color_optical_frame":
        raise RuntimeError(
            "camera calibration transform must be end_effector_link -> d435_color_optical_frame")
    if len(xyz) != 3 or len(xyzw) != 4 or not all(map(math.isfinite, xyz + xyzw)):
        raise RuntimeError("camera calibration transform must contain finite xyz[3] and quaternion[4]")
    quaternion_norm = math.sqrt(sum(value * value for value in xyzw))
    if abs(quaternion_norm - 1.0) > 1.0e-3:
        raise RuntimeError("camera calibration quaternion must be normalized")
    if method != "opencv_calibrateHandEye_eye_in_hand":
        raise RuntimeError("camera calibration method is not the supported eye-in-hand solver")
    if samples < 8:
        raise RuntimeError("camera calibration needs at least 8 distinct robot poses")
    if not math.isfinite(translation_rmse) or translation_rmse < 0.0 or translation_rmse > 0.015:
        raise RuntimeError("camera calibration translation RMSE exceeds 0.015 m")
    if not math.isfinite(rotation_rmse) or rotation_rmse < 0.0 or rotation_rmse > 0.08:
        raise RuntimeError("camera calibration rotation RMSE exceeds 0.08 rad")
    return {
        "camera_calibration_file": os.fspath(calibration_path),
        "camera_calibration_sha256": _sha256(raw),
        "camera_calibration_schema": document["schema"],
        "camera_calibration_model": model,
        "camera_calibration_serial": serial,
        "camera_calibration_parent_frame": parent,
        "camera_calibration_color_xyz": xyz,
        "camera_calibration_color_quaternion_xyzw": xyzw,
        "camera_calibration_method": method,
        "camera_calibration_samples": samples,
        "camera_calibration_translation_rmse_m": translation_rmse,
        "camera_calibration_rotation_rmse_rad": rotation_rmse,
    }


def resolve_model_inputs(context, *, moveit_share, description_share, dd_gng_share):
    version = LaunchConfiguration("model_version").perform(context)
    if version not in ("v1", "v2"):
        raise RuntimeError("model_version must be v1 or v2")
    configured_params = LaunchConfiguration("params_file").perform(context)
    params_name = "topo_gng_v2.yaml" if version == "v2" else "topo_gng.yaml"
    params_path = Path(configured_params or Path(dd_gng_share) / "config" / params_name)
    params_path = params_path.expanduser().resolve(strict=True)
    model_name = "om6dof_v2.urdf.xacro" if version == "v2" else "om6dof.urdf.xacro"
    xacro_path = Path(description_share) / "urdf" / model_name
    executable = shutil.which("xacro")
    if not executable:
        raise RuntimeError("could not resolve the xacro executable")
    expanded = subprocess.run(
        [executable, os.fspath(xacro_path)], stdout=subprocess.PIPE,
        stderr=subprocess.PIPE, check=False,
    )
    if expanded.returncode:
        detail = expanded.stderr.decode("utf-8", errors="replace").strip()
        raise RuntimeError(f"xacro expansion failed: {detail}")
    urdf_bytes = expanded.stdout
    srdf_bytes = (Path(moveit_share) / "config" / "om6dof.srdf").read_bytes()
    if version == "v2":
        urdf = ET.fromstring(urdf_bytes)
        srdf = ET.fromstring(srdf_bytes)
        srdf.set("name", urdf.attrib["name"])
        links = {link.attrib["name"] for link in urdf.findall("link")}
        # Dropping an exemption can only make collision checking stricter.
        # Old "Never" pairs were sampled for V1, not the changed V2 geometry.
        # Never map a removed D405 frame onto a different physical V2 link.
        for pair in list(srdf.findall("disable_collisions")):
            if (pair.get("link1") not in links or pair.get("link2") not in links
                    or pair.get("reason") == "Never"):
                srdf.remove(pair)
        srdf_bytes = ET.tostring(srdf, encoding="utf-8", xml_declaration=True)
    params_bytes = params_path.read_bytes()
    calibration_parameters = load_hand_eye_calibration(
        LaunchConfiguration("camera_calibration_file").perform(context))
    return {
        "version": version,
        "params_path": os.fspath(params_path),
        "model_parameters": {
            "robot_description": urdf_bytes.decode("utf-8"),
            "robot_description_semantic": srdf_bytes.decode("utf-8"),
            "expanded_urdf_sha256": _sha256(urdf_bytes),
            "srdf_sha256": _sha256(srdf_bytes),
            "reachability_parameters_sha256": _sha256(params_bytes),
        },
        "calibration_parameters": calibration_parameters,
        "remappings": ([
            ("/tf", "/om6dof_dd_gng/v2/tf"),
            ("/tf_static", "/om6dof_dd_gng/v2/tf_static"),
            ("/robot_description", "/om6dof_dd_gng/v2/robot_description"),
        ] + [(f"/om6dof_topo_gng/{name}", f"/om6dof_topo_gng_v2/{name}")
             for name in GRAPH_ENDPOINTS] if version == "v2" else []),
    }
