"""Measured eye-in-hand calibration for the DD-GNG D435i pipeline.

This tool never commands the robot. Keep one AprilTag fixed in the world,
move the wrist to at least eight distinct poses with the normal operator
controls, and press Enter once the arm and image are steady at each pose.
It combines ``world <- end_effector_link`` with
``d435_color_optical_frame <- AprilTag`` and writes an artifact accepted by
the fail-closed DD-GNG loader only when the validation residual passes.
"""

from __future__ import annotations

import math
from collections import deque
from pathlib import Path
import threading
import time

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.duration import Duration
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.time import Time
import tf2_ros
import yaml


def quaternion_matrix(x, y, z, w):
    quaternion = np.asarray([x, y, z, w], dtype=float)
    norm = np.linalg.norm(quaternion)
    if not math.isfinite(norm) or norm < 1.0e-12:
        raise ValueError("invalid quaternion")
    x, y, z, w = quaternion / norm
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def matrix_quaternion(rotation):
    matrix = np.asarray(rotation, dtype=float)
    K = np.array([
        [matrix[0, 0] - matrix[1, 1] - matrix[2, 2],
         matrix[0, 1] + matrix[1, 0], matrix[0, 2] + matrix[2, 0],
         matrix[2, 1] - matrix[1, 2]],
        [matrix[0, 1] + matrix[1, 0],
         matrix[1, 1] - matrix[0, 0] - matrix[2, 2],
         matrix[1, 2] + matrix[2, 1], matrix[0, 2] - matrix[2, 0]],
        [matrix[0, 2] + matrix[2, 0], matrix[1, 2] + matrix[2, 1],
         matrix[2, 2] - matrix[0, 0] - matrix[1, 1],
         matrix[1, 0] - matrix[0, 1]],
        [matrix[2, 1] - matrix[1, 2], matrix[0, 2] - matrix[2, 0],
         matrix[1, 0] - matrix[0, 1], matrix.trace()],
    ]) / 3.0
    values, vectors = np.linalg.eigh(K)
    quaternion = vectors[:, np.argmax(values)]
    if quaternion[3] < 0.0:
        quaternion = -quaternion
    return quaternion / np.linalg.norm(quaternion)


def transform_matrix(translation, quaternion):
    result = np.eye(4)
    result[:3, :3] = quaternion_matrix(*quaternion)
    result[:3, 3] = np.asarray(translation, dtype=float)
    return result


def pose_matrix(pose):
    return transform_matrix(
        [pose.position.x, pose.position.y, pose.position.z],
        [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w],
    )


def transform_message_matrix(transform):
    return transform_matrix(
        [transform.translation.x, transform.translation.y, transform.translation.z],
        [transform.rotation.x, transform.rotation.y, transform.rotation.z, transform.rotation.w],
    )


def rotation_angle(rotation):
    cosine = np.clip((np.trace(rotation) - 1.0) * 0.5, -1.0, 1.0)
    return float(math.acos(cosine))


def average_transform(transforms):
    """Robust stationary-pose aggregate for timestamp-coherent samples."""
    if not transforms:
        raise ValueError("at least one transform is required")
    translations = np.stack([item[:3, 3] for item in transforms])
    quaternions = np.stack([matrix_quaternion(item[:3, :3]) for item in transforms])
    reference = quaternions[0]
    quaternions = np.stack([
        -value if np.dot(value, reference) < 0.0 else value
        for value in quaternions
    ])
    quaternion = quaternions.mean(axis=0)
    quaternion /= np.linalg.norm(quaternion)
    result = np.eye(4)
    result[:3, :3] = quaternion_matrix(*quaternion)
    # A median prevents a single bad AprilTag depth/PnP sample from moving the
    # complete captured pose while preserving the real metric scale.
    result[:3, 3] = np.median(translations, axis=0)
    return result


def transform_dispersion(transforms, center):
    translations = np.asarray([
        np.linalg.norm(item[:3, 3] - center[:3, 3]) for item in transforms])
    rotations = np.asarray([
        rotation_angle(center[:3, :3].T @ item[:3, :3]) for item in transforms])
    return (
        float(np.sqrt(np.mean(np.square(translations)))), float(translations.max()),
        float(np.sqrt(np.mean(np.square(rotations)))), float(rotations.max()),
    )


def solve_hand_eye(base_from_gripper, camera_from_target):
    if len(base_from_gripper) != len(camera_from_target) or len(base_from_gripper) < 3:
        raise ValueError("paired hand-eye observations are required")
    rotation, translation = cv2.calibrateHandEye(
        [item[:3, :3] for item in base_from_gripper],
        [item[:3, 3] for item in base_from_gripper],
        [item[:3, :3] for item in camera_from_target],
        [item[:3, 3] for item in camera_from_target],
        method=cv2.CALIB_HAND_EYE_TSAI,
    )
    gripper_from_camera = np.eye(4)
    gripper_from_camera[:3, :3] = rotation
    gripper_from_camera[:3, 3] = np.asarray(translation).reshape(3)
    base_from_target = [
        base_gripper @ gripper_from_camera @ camera_target
        for base_gripper, camera_target in zip(base_from_gripper, camera_from_target)
    ]
    translations = np.stack([item[:3, 3] for item in base_from_target])
    center = translations.mean(axis=0)
    translation_errors = np.linalg.norm(translations - center, axis=1)
    translation_rmse = float(np.sqrt(np.mean(np.square(translation_errors))))
    rotation_center = average_transform(base_from_target)[:3, :3]
    rotation_errors = np.asarray([
        rotation_angle(rotation_center.T @ item[:3, :3]) for item in base_from_target])
    rotation_rmse = float(np.sqrt(np.mean(np.square(rotation_errors))))
    return (
        gripper_from_camera, translation_rmse, rotation_rmse,
        translation_errors, rotation_errors,
    )


class HandEyeCollector(Node):
    def __init__(self):
        super().__init__("ddgng_hand_eye_calibrate")
        self.declare_parameter("tag_pose_topic", "/apriltag/pose")
        self.declare_parameter("world_frame", "world")
        self.declare_parameter("gripper_frame", "end_effector_link")
        # ROS CLI parses an all-decimal RealSense serial as an integer unless
        # users add nested YAML quotes. Accept either representation and bind
        # the artifact to its canonical decimal string below.
        self.declare_parameter(
            "camera_serial", "", ParameterDescriptor(dynamic_typing=True))
        self.declare_parameter("capture_seconds", 1.5)
        self.declare_parameter("output_file", "~/.config/om6dof/d435_hand_eye.yaml")
        self.world_frame = str(self.get_parameter("world_frame").value)
        self.gripper_frame = str(self.get_parameter("gripper_frame").value)
        self.camera_serial = str(self.get_parameter("camera_serial").value).strip()
        if not self.camera_serial:
            raise RuntimeError("camera_serial is required; calibration must bind one physical D435i")
        self.buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.buffer, self)
        self._lock = threading.Lock()
        self._pending_tags = deque(maxlen=90)
        self._resolved_pairs = deque(maxlen=180)
        self.create_subscription(
            PoseStamped, str(self.get_parameter("tag_pose_topic").value), self._on_tag, 20)
        self.create_timer(0.02, self._resolve_pending_tags)
        self.last_capture_quality = ""

    def _on_tag(self, message):
        if message.header.frame_id != "d435_color_optical_frame":
            self.get_logger().error(
                f"tag frame must be d435_color_optical_frame, got {message.header.frame_id}",
                throttle_duration_sec=2.0,
            )
            return
        with self._lock:
            self._pending_tags.append(message)

    def _resolve_pending_tags(self):
        # A tag message can be delivered just before the /tf sample carrying
        # the same timestamp. Keep it pending rather than falling back to a
        # latest transform or blocking the single-threaded executor that must
        # receive that TF sample.
        with self._lock:
            pending = list(self._pending_tags)
            self._pending_tags.clear()
        unresolved = []
        resolved = []
        expired = 0
        now = self.get_clock().now()
        last_error = None
        for message in pending:
            stamp = Time.from_msg(message.header.stamp)
            try:
                base_gripper = self.buffer.lookup_transform(
                    self.world_frame, self.gripper_frame, stamp,
                    timeout=Duration(seconds=0.0),
                )
                resolved.append((
                    time.monotonic(),
                    transform_message_matrix(base_gripper.transform),
                    pose_matrix(message.pose),
                ))
            except Exception as error:
                last_error = error
                age = (now - stamp).nanoseconds / 1.0e9
                if -0.1 <= age <= 2.0:
                    unresolved.append(message)
                else:
                    expired += 1
        with self._lock:
            for message in unresolved:
                self._pending_tags.append(message)
            self._resolved_pairs.extend(resolved)
        if expired and last_error is not None:
            self.get_logger().warn(
                f"{expired} pasangan AprilTag/TF kedaluwarsa: {last_error}",
                throttle_duration_sec=2.0,
            )

    def capture(self):
        started = time.monotonic()
        with self._lock:
            self._resolved_pairs.clear()
        deadline = time.monotonic() + float(self.get_parameter("capture_seconds").value)
        while time.monotonic() < deadline:
            time.sleep(0.05)
        with self._lock:
            pairs = [(base, tag) for received, base, tag in self._resolved_pairs
                     if received >= started]
        if len(pairs) < 5:
            self.last_capture_quality = f"hanya {len(pairs)} pasangan timestamp tersedia"
            return None
        base_samples = [base for base, _tag in pairs]
        tag_samples = [tag for _base, tag in pairs]
        base_center = average_transform(base_samples)
        tag_center = average_transform(tag_samples)
        base_t_rms, base_t_max, base_r_rms, base_r_max = transform_dispersion(
            base_samples, base_center)
        tag_t_rms, tag_t_max, tag_r_rms, tag_r_max = transform_dispersion(
            tag_samples, tag_center)
        self.last_capture_quality = (
            f"robot rms={base_t_rms * 1000:.1f}mm/{math.degrees(base_r_rms):.2f}deg, "
            f"tag rms={tag_t_rms * 1000:.1f}mm/{math.degrees(tag_r_rms):.2f}deg")
        if base_t_max > 0.003 or base_r_max > 0.025:
            self.last_capture_quality += " (robot bergerak selama capture)"
            return None
        if tag_t_max > 0.010 or tag_r_max > 0.10:
            self.last_capture_quality += " (deteksi tag terlalu berisik)"
            return None
        return base_center, tag_center

    def save(self, gripper_from_camera, samples, translation_rmse, rotation_rmse):
        output = Path(str(self.get_parameter("output_file").value)).expanduser()
        output.parent.mkdir(parents=True, exist_ok=True)
        quaternion = matrix_quaternion(gripper_from_camera[:3, :3])
        document = {
            "schema": "om6dof.hand_eye.v1",
            "camera": {"model": "D435i", "serial": self.camera_serial},
            "transform": {
                "parent_frame": self.gripper_frame,
                "child_frame": "d435_color_optical_frame",
                "xyz": [float(value) for value in gripper_from_camera[:3, 3]],
                "quaternion_xyzw": [float(value) for value in quaternion],
            },
            "validation": {
                "method": "opencv_calibrateHandEye_eye_in_hand",
                "samples": int(samples),
                "translation_rmse_m": float(translation_rmse),
                "rotation_rmse_rad": float(rotation_rmse),
            },
        }
        output.write_text(yaml.safe_dump(document, sort_keys=False), encoding="utf-8")
        return output


def main():
    rclpy.init()
    node = HandEyeCollector()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    base_gripper = []
    camera_target = []
    print(__doc__)
    try:
        while True:
            command = input(
                f"\n{len(base_gripper)} pose tersimpan. "
                "Diamkan robot lalu Enter untuk capture; ketik selesai untuk solve: "
            ).strip().lower()
            if command in ("selesai", "done", "solve"):
                if len(base_gripper) >= 8:
                    break
                print("Minimal 8 pose berbeda diperlukan.")
                continue
            pair = node.capture()
            if pair is None:
                print("Gagal: " + (node.last_capture_quality or
                                   "AprilTag/TF tidak cukup stabil atau tidak tersedia."))
                continue
            if base_gripper:
                movement = np.linalg.inv(base_gripper[-1]) @ pair[0]
                if np.linalg.norm(movement[:3, 3]) < 0.02 and rotation_angle(movement[:3, :3]) < 0.12:
                    print("Pose terlalu mirip; geser atau rotasikan wrist lebih jauh.")
                    continue
            base_gripper.append(pair[0])
            camera_target.append(pair[1])
            print("Pose diterima: " + node.last_capture_quality)
            print("Pertahankan AprilTag tetap diam selama seluruh kalibrasi.")
        transform, translation_rmse, rotation_rmse, translation_errors, rotation_errors = solve_hand_eye(
            base_gripper, camera_target)
        print(f"RMSE translasi: {translation_rmse:.4f} m")
        print(f"RMSE rotasi:    {rotation_rmse:.4f} rad")
        print("Residual per pose:")
        for index, (translation_error, rotation_error) in enumerate(
                zip(translation_errors, rotation_errors), start=1):
            print(
                f"  pose {index:02d}: {translation_error * 1000:5.1f} mm, "
                f"{math.degrees(rotation_error):5.2f} deg")
        if translation_rmse > 0.015 or rotation_rmse > 0.08:
            print("DITOLAK: residual melewati batas 0.015 m / 0.08 rad; file tidak ditulis.")
            return
        output = node.save(transform, len(base_gripper), translation_rmse, rotation_rmse)
        print(f"DITERIMA: artefak disimpan di {output}")
    finally:
        executor.shutdown(timeout_sec=2.0)
        spin_thread.join(timeout=2.0)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
