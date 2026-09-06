"""Frame algebra for the Gemini grasp pipeline.

The camera is on the wrist, so nothing in this chain is static:

    world <- FK(joint_states) <- end_effector_link <- camera body <- optical

`om6dof_pick_and_place` carries the same convention (``R_BODY_OPTICAL``,
``rpy_to_matrix``), and the calibration GUI on port 8081 tunes the same
``camera_xyz`` / ``camera_rpy`` numbers. They are re-derived here rather than
imported so this module — and its tests — stay usable without rclpy, KDL or a
camera attached.

Tool convention (from ``om6dof.urdf.xacro``): ``end_effector_link`` inherits
link7's orientation, the fingers translate along its **Y** axis, and the tool
reaches along its **Z** axis. So a grasp is a position plus two unit vectors,
``approach`` (tool +Z) and ``closing`` (tool +Y).
"""

from __future__ import annotations

import math
from typing import Iterable, Sequence, Tuple

import numpy as np

# Camera BODY (x forward, y left, z up) -> OPTICAL (x right, y down, z forward).
# Columns are the optical axes written in body coordinates.
R_BODY_OPTICAL = np.array([
    [0.0, 0.0, 1.0],
    [-1.0, 0.0, 0.0],
    [0.0, -1.0, 0.0],
])


def rpy_to_matrix(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """Fixed-axis XYZ roll/pitch/yaw as a rotation matrix (Rz @ Ry @ Rx)."""
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    rx = np.array([[1.0, 0.0, 0.0], [0.0, cr, -sr], [0.0, sr, cr]])
    ry = np.array([[cp, 0.0, sp], [0.0, 1.0, 0.0], [-sp, 0.0, cp]])
    rz = np.array([[cy, -sy, 0.0], [sy, cy, 0.0], [0.0, 0.0, 1.0]])
    return rz @ ry @ rx


def quat_to_matrix(x: float, y: float, z: float, w: float) -> np.ndarray:
    q = np.array([x, y, z, w], dtype=float)
    n = float(np.linalg.norm(q))
    if n < 1e-9:
        return np.eye(3)
    x, y, z, w = q / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def matrix_to_quat(R: np.ndarray) -> Tuple[float, float, float, float]:
    """Rotation matrix -> (x, y, z, w), branch-selected for numeric stability."""
    R = np.asarray(R, dtype=float)
    trace = float(np.trace(R))
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        return ((R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s,
                (R[1, 0] - R[0, 1]) / s, 0.25 * s)
    idx = int(np.argmax([R[0, 0], R[1, 1], R[2, 2]]))
    if idx == 0:
        s = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2.0
        return (0.25 * s, (R[0, 1] + R[1, 0]) / s, (R[0, 2] + R[2, 0]) / s,
                (R[2, 1] - R[1, 2]) / s)
    if idx == 1:
        s = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2.0
        return ((R[0, 1] + R[1, 0]) / s, 0.25 * s, (R[1, 2] + R[2, 1]) / s,
                (R[0, 2] - R[2, 0]) / s)
    s = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2.0
    return ((R[0, 2] + R[2, 0]) / s, (R[1, 2] + R[2, 1]) / s, 0.25 * s,
            (R[1, 0] - R[0, 1]) / s)


def rotation_distance(actual: np.ndarray, target: np.ndarray) -> float:
    """Principal SO(3) angle between two rotation matrices, in radians."""
    relative = np.asarray(actual, dtype=float).T @ np.asarray(target, dtype=float)
    cosine = float(np.clip((np.trace(relative) - 1.0) * 0.5, -1.0, 1.0))
    return math.acos(cosine)


def optical_from_parent(camera_rpy: Sequence[float]) -> np.ndarray:
    """Rotation ``parent -> optical`` for a camera body mounted at ``camera_rpy``."""
    return rpy_to_matrix(*[float(v) for v in camera_rpy]) @ R_BODY_OPTICAL


def camera_pose_in_base(p_we: np.ndarray, R_we: np.ndarray,
                        t_ec: Sequence[float],
                        R_eo: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    """Place the camera optical frame in the base frame.

    ``p_we`` / ``R_we`` come from FK of the current joint state, ``t_ec`` is the
    parent->camera translation and ``R_eo`` the parent->optical rotation.
    """
    R_we = np.asarray(R_we, dtype=float)
    p_wc = R_we @ np.asarray(t_ec, dtype=float) + np.asarray(p_we, dtype=float)
    return p_wc, R_we @ np.asarray(R_eo, dtype=float)


def points_to_base(points_optical: np.ndarray, p_wc: np.ndarray,
                   R_wc: np.ndarray) -> np.ndarray:
    """Transform an (N, 3) optical-frame cloud into the base frame."""
    pts = np.asarray(points_optical, dtype=float).reshape(-1, 3)
    return pts @ np.asarray(R_wc, dtype=float).T + np.asarray(p_wc, dtype=float)


def directions_to_base(dirs_optical: np.ndarray, R_wc: np.ndarray) -> np.ndarray:
    """Rotate (N, 3) optical-frame direction vectors into the base frame."""
    d = np.asarray(dirs_optical, dtype=float).reshape(-1, 3)
    return d @ np.asarray(R_wc, dtype=float).T


def deproject(u: float, v: float, depth_m: float,
              intrinsics: Sequence[float]) -> np.ndarray:
    """Pixel + metric depth -> optical-frame XYZ. ``intrinsics`` = (fx, fy, cx, cy)."""
    fx, fy, cx, cy = [float(value) for value in intrinsics]
    return np.array([(float(u) - cx) * depth_m / fx,
                     (float(v) - cy) * depth_m / fy,
                     float(depth_m)])


def project(points_optical: np.ndarray,
            intrinsics: Sequence[float]) -> np.ndarray:
    """Optical-frame (N, 3) -> (N, 2) pixels. Points at or behind z=0 give NaN."""
    fx, fy, cx, cy = [float(value) for value in intrinsics]
    pts = np.asarray(points_optical, dtype=float).reshape(-1, 3)
    z = pts[:, 2]
    with np.errstate(divide="ignore", invalid="ignore"):
        u = np.where(z > 1e-6, pts[:, 0] * fx / z + cx, np.nan)
        v = np.where(z > 1e-6, pts[:, 1] * fy / z + cy, np.nan)
    return np.stack([u, v], axis=1)


def _unit(v: Iterable[float]) -> np.ndarray:
    v = np.asarray(v, dtype=float)
    n = float(np.linalg.norm(v))
    if n < 1e-9:
        raise ValueError("cannot normalise a zero-length vector")
    return v / n


def tool_rotation(approach: Iterable[float],
                  closing: Iterable[float]) -> np.ndarray:
    """Tool rotation matrix from a grasp's approach and closing directions.

    ``end_effector_link`` reaches along +Z and its fingers move along +Y, so the
    columns are (Y x Z, Y, Z). ``closing`` is re-orthogonalised against
    ``approach``, which lets callers pass a roughly-perpendicular axis such as a
    cluster's principal direction.
    """
    z_axis = _unit(approach)
    y_raw = np.asarray(closing, dtype=float)
    y_axis = y_raw - np.dot(y_raw, z_axis) * z_axis
    if float(np.linalg.norm(y_axis)) < 1e-6:
        # Degenerate: closing was parallel to approach. Any perpendicular will do.
        fallback = np.array([0.0, 0.0, 1.0])
        if abs(float(np.dot(fallback, z_axis))) > 0.9:
            fallback = np.array([1.0, 0.0, 0.0])
        y_axis = fallback - np.dot(fallback, z_axis) * z_axis
    y_axis = _unit(y_axis)
    x_axis = np.cross(y_axis, z_axis)
    return np.column_stack([x_axis, y_axis, z_axis])


def approach_tilt_from_vertical(approach: Iterable[float]) -> float:
    """Angle in radians between a grasp approach and straight down (-Z world)."""
    a = _unit(approach)
    return math.acos(float(np.clip(np.dot(a, np.array([0.0, 0.0, -1.0])), -1.0, 1.0)))
