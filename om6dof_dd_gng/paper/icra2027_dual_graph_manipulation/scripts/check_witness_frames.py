#!/usr/bin/env python3
"""Recompute saved green witness FK from frozen URDF, without ROS or hardware.

Uses NumPy homogeneous transforms and XML parsing, independent of the original
KDL scan implementation. Reports all comparisons to the recorded residuals.
No IK, collision queries, robot connection or raw-data mutations occur.
"""
from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np

PAPER = Path(__file__).resolve().parents[1]
REPO = Path(__file__).resolve().parents[4]
DEFAULT = REPO / "experiments/cartesian_workspace/results/comparison_v1_v2_25mm_20260909/v2"
EXPECTED_MODEL_SHA256 = "02e81b41d6c6643c4af1ddd75f444fc1cb5ebed99c56f35657cd480d28857339"
EXPECTED_POINTS_SHA256 = "ded51632edcd1b041cd3e3d07c9d9aa04dace55ef1624fdf80f6fb1ad636fa1b"


def sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def rotation(axis, angle):
    axis = np.asarray(axis, dtype=float)
    axis = axis / np.linalg.norm(axis)
    x, y, z = axis
    skew = np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]])
    return np.eye(3) + math.sin(angle) * skew + (1 - math.cos(angle)) * (skew @ skew)


def rpy_rotation(rpy):
    r, p, y = rpy
    return rotation([0, 0, 1], y) @ rotation([0, 1, 0], p) @ rotation([1, 0, 0], r)


def rotation_error_deg(desired, actual):
    delta = desired @ actual.T
    skew_vector = np.array([delta[2, 1] - delta[1, 2],
                            delta[0, 2] - delta[2, 0],
                            delta[1, 0] - delta[0, 1]]) / 2
    return math.degrees(math.atan2(np.linalg.norm(skew_vector), (np.trace(delta) - 1) / 2))


def vector_angle_deg(left, right):
    return math.degrees(math.atan2(np.linalg.norm(np.cross(left, right)), np.dot(left, right)))


def describe(values):
    a = np.asarray(values)
    assert a.size and np.isfinite(a).all()
    return {"n": int(a.size), "min": float(a.min()), "median": float(np.median(a)),
            "mean": float(a.mean()), "p95": float(np.quantile(a, .95)), "max": float(a.max())}


def load_chain(model):
    root = ET.parse(model).getroot()
    by_child = {joint.find("child").attrib["link"]: joint for joint in root.findall("joint")}
    tip = "end_effector_link"
    nodes = []
    while tip != "world":
        joint = by_child[tip]
        nodes.append(joint)
        tip = joint.find("parent").attrib["link"]
    chain = []
    for joint in reversed(nodes):
        origin = joint.find("origin")
        xyz = [float(v) for v in origin.attrib.get("xyz", "0 0 0").split()] if origin is not None else [0, 0, 0]
        rpy = [float(v) for v in origin.attrib.get("rpy", "0 0 0").split()] if origin is not None else [0, 0, 0]
        transform = np.eye(4)
        transform[:3, :3] = rpy_rotation(rpy)
        transform[:3, 3] = xyz
        axis = joint.find("axis")
        axis = [float(v) for v in axis.attrib.get("xyz", "1 0 0").split()] if axis is not None else [1, 0, 0]
        kind = joint.attrib["type"]
        assert kind in {"fixed", "revolute", "continuous"}
        assert joint.find("mimic") is None
        chain.append((joint.attrib["name"], kind, transform, axis))
    return chain


def fk(chain, joints):
    transform = np.eye(4)
    for name, kind, origin, axis in chain:
        transform = transform @ origin
        if kind != "fixed":
            motion = np.eye(4)
            motion[:3, :3] = rotation(axis, joints[name])
            transform = transform @ motion
    return transform


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dataset", type=Path, default=DEFAULT)
    parser.add_argument("--output", type=Path, default=PAPER / "data/frame_verification.json")
    args = parser.parse_args()
    model = args.dataset / "model.urdf"
    points = args.dataset / "points.csv"
    assert sha256(model) == EXPECTED_MODEL_SHA256, "Not the audited frozen model"
    assert sha256(points) == EXPECTED_POINTS_SHA256, "Not the audited original point file"
    chain = load_chain(model)
    with points.open(newline="") as stream:
        rows = [row for row in csv.DictReader(stream) if row["status"] == "pose_found"
                and row["pose_found"] == "1" and row["position_found"] == "1"]
    assert len(rows) == 3198
    assert [name for name, kind, _, _ in chain if kind != "fixed"] == [f"joint{i}" for i in range(1, 7)]
    original_tip_from_gripper = np.array([[0., 0., -1.], [0., 1., 0.], [1., 0., 0.]])
    measures = {key: [] for key in ["local_x_tilt_from_world_down_deg",
        "local_z_tilt_from_horizontal_deg", "local_z_absolute_vertical_component",
        "local_z_angle_from_requested_radial_direction_deg", "recomputed_position_error_mm",
        "recomputed_orientation_error_deg", "absolute_difference_vs_recorded_position_error_mm",
        "absolute_difference_vs_recorded_orientation_error_deg", "rotation_orthogonality_error_frobenius"]}
    worst_index = 0
    for index, row in enumerate(rows):
        joints = {f"joint{i}": float(row[f"q{i}_rad"]) for i in range(1, 7)}
        transform = fk(chain, joints)
        actual_r = transform[:3, :3]
        target_p = np.array([float(row[f"{key}_mm"]) / 1000 for key in "xyz"])
        target_rpy = np.radians([float(row[f"target_{key}_deg"]) for key in ["roll", "pitch", "yaw"]])
        target_r = rpy_rotation(target_rpy) @ original_tip_from_gripper.T
        radial_direction = np.array([math.cos(target_rpy[2]), math.sin(target_rpy[2]), 0.])
        x_tilt = vector_angle_deg(actual_r[:, 0], np.array([0., 0., -1.]))
        pos_error = float(np.linalg.norm(transform[:3, 3] - target_p) * 1000)
        rot_error = rotation_error_deg(target_r, actual_r)
        values = {
            "local_x_tilt_from_world_down_deg": x_tilt,
            "local_z_tilt_from_horizontal_deg": math.degrees(math.asin(min(1., abs(actual_r[2, 2])))),
            "local_z_absolute_vertical_component": abs(float(actual_r[2, 2])),
            "local_z_angle_from_requested_radial_direction_deg": vector_angle_deg(actual_r[:, 2], radial_direction),
            "recomputed_position_error_mm": pos_error,
            "recomputed_orientation_error_deg": rot_error,
            "absolute_difference_vs_recorded_position_error_mm": abs(pos_error - float(row["position_error_mm"])),
            "absolute_difference_vs_recorded_orientation_error_deg": abs(rot_error - float(row["orientation_error_deg"])),
            "rotation_orthogonality_error_frobenius": float(np.linalg.norm(actual_r.T @ actual_r - np.eye(3))),
        }
        for key, value in values.items():
            measures[key].append(value)
        if x_tilt >= measures["local_x_tilt_from_world_down_deg"][worst_index]:
            worst_index = index
        assert x_tilt <= rot_error + 1e-9
    checks = {
        "position_residual_matches_saved_KDL_mm_tolerance_1e_minus_7": max(measures["absolute_difference_vs_recorded_position_error_mm"]) < 1e-7,
        "orientation_residual_matches_saved_KDL_deg_tolerance_1e_minus_7": max(measures["absolute_difference_vs_recorded_orientation_error_deg"]) < 1e-7,
        "all_local_x_axes_within_half_degree_of_world_down": max(measures["local_x_tilt_from_world_down_deg"]) < .5,
        "all_local_z_axes_within_half_degree_of_horizontal": max(measures["local_z_tilt_from_horizontal_deg"]) < .5,
        "rotation_orthogonality_error_below_1e_minus_12": max(measures["rotation_orthogonality_error_frobenius"]) < 1e-12,
    }
    assert all(checks.values()), checks
    result = {
        "scope": "Independent offline NumPy/XML forward kinematics of all selected stored joints; no ROS, IK, collision validation or hardware",
        "witness_count": len(rows), "base_frame": "world", "tip_frame": "end_effector_link",
        "chain": [{"name": name, "type": kind} for name, kind, _, _ in chain],
        "provenance": {"model_sha256": sha256(model), "points_sha256": sha256(points),
                       "verification_script_sha256": sha256(__file__), "numpy_version": np.__version__},
        "checks": checks, "statistics": {key: describe(values) for key, values in measures.items()},
        "largest_local_x_tilt_witness": {key: rows[worst_index][key] for key in ["x_mm", "y_mm", "z_mm", "q1_rad", "q2_rad", "q3_rad", "q4_rad", "q5_rad", "q6_rad", "orientation_error_deg"]},
        "conclusion": "Every selected green witness actually has end_effector_link local +X nearly world-down; this bank does not provide horizontal local-+X approaches",
        "limitations": ["Saved-model FK is not physical pose measurement", "Only the original 3198 green witnesses are tested", "Collision geometry and current runtime model differences are not evaluated"],
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2, sort_keys=True) + "\n")
    print(json.dumps({"checks": checks, "max_local_x_tilt_deg": max(measures["local_x_tilt_from_world_down_deg"]),
                      "max_local_z_horizontal_tilt_deg": max(measures["local_z_tilt_from_horizontal_deg"]),
                      "max_local_z_absolute_vertical_component": max(measures["local_z_absolute_vertical_component"]),
                      "max_position_residual_difference_mm": max(measures["absolute_difference_vs_recorded_position_error_mm"]),
                      "max_orientation_residual_difference_deg": max(measures["absolute_difference_vs_recorded_orientation_error_deg"])}, indent=2))


if __name__ == "__main__":
    main()
