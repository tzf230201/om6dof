"""Vision-only target/Gemini/learned-grasp inspection viewer.

Call ``~/run`` to capture the latest RGB-D frame, ask Gemini to locate the
requested object, run the selected backend on the complete scene, associate candidates
with its RGB-D component, and publish the selected grasp to RViz. This is
deliberately separate from
``rgbd_viewer`` and never creates MoveIt or controller clients.
"""

from __future__ import annotations

import math
import threading
from dataclasses import dataclass
from typing import List, Optional, Tuple

import numpy as np
import rclpy
from geometry_msgs.msg import Point
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import String
from std_srvs.srv import Trigger
from visualization_msgs.msg import Marker, MarkerArray

from .gemini_client import GeminiClient, GeminiError, Localization
from .grasp_backends import (GraspCandidate, GraspScene, crop_to_workspace,
                             make_backend, segment_target_component,
                             self_exclusion_mask, target_region_mask)
from .grasp_filter import (FilterConfig, Rejection,
                           conservative_gripper_collision, filter_and_rank,
                           rejection_summary)
from .rgbd_source import RGBDFrame, make_source, point_cloud
from .target_selection import (candidate_diagnostic_summary,
                               select_target_candidate)
from .transforms import points_to_base, quat_to_matrix


@dataclass
class TargetGraspResult:
    """One explicit user-requested inference result for the current target."""
    target: str
    localization: Localization
    scene: GraspScene
    target_scene: Optional[GraspScene]
    raw_candidates: List[GraspCandidate]
    candidates: List[GraspCandidate]
    rejected: List[Tuple[GraspCandidate, Rejection]]
    selected: Optional[GraspCandidate]
    message: str


def parallel_gripper_points(candidate: GraspCandidate, *,
                            palm_depth: float = 0.020,
                            tail_length: float = 0.040) -> np.ndarray:
    """Build a GraspNet-style parallel-jaw line glyph in the world frame.

    The returned eight vertices form four LINE_LIST segments: left finger,
    right finger, palm, and the rear approach stem.  This follows the same
    convention as graspnetAPI's ``plot_gripper_pro_max``: translation is the
    grasp centre, rotation column 0 is approach, column 1 is jaw closing,
    ``width`` is the inner opening, and ``depth`` extends the fingers forward.
    """
    centre = np.asarray(candidate.position, dtype=float).reshape(3)
    # These arrays belong to the candidate and may later be consumed by IK or
    # motion planning.  Normalising a NumPy view in-place would let an RViz
    # debug drawing silently alter the candidate itself.
    approach = np.asarray(candidate.approach, dtype=float).reshape(3).copy()
    approach /= max(float(np.linalg.norm(approach)), 1e-9)
    closing = np.asarray(candidate.closing, dtype=float).reshape(3).copy()
    closing = closing - float(np.dot(closing, approach)) * approach
    closing /= max(float(np.linalg.norm(closing)), 1e-9)
    half_width = 0.5 * max(0.0, float(candidate.width))
    finger_depth = max(
        0.005, float(candidate.extras.get("graspnet_depth", 0.020)))

    palm_centre = centre - approach * max(0.0, float(palm_depth))
    tip_centre = centre + approach * finger_depth
    left_palm = palm_centre - closing * half_width
    right_palm = palm_centre + closing * half_width
    left_tip = tip_centre - closing * half_width
    right_tip = tip_centre + closing * half_width
    tail = palm_centre - approach * max(0.0, float(tail_length))
    return np.vstack([
        left_palm, left_tip,
        right_palm, right_tip,
        left_palm, right_palm,
        tail, palm_centre,
    ])


def cloud_message(points: np.ndarray, colors: Optional[np.ndarray], *,
                  frame_id: str, stamp: float) -> PointCloud2:
    """Create a compact coloured XYZ PointCloud2 in the requested frame."""
    pts = np.asarray(points, dtype=np.float32).reshape(-1, 3)
    bgr = (np.zeros((pts.shape[0], 3), dtype=np.uint8) if colors is None
           else np.asarray(colors, dtype=np.uint8).reshape(-1, 3))
    if bgr.shape[0] != pts.shape[0]:
        raise ValueError("point and colour counts differ")
    records = np.empty(pts.shape[0], dtype=np.dtype([
        ("x", "<f4"), ("y", "<f4"), ("z", "<f4"), ("rgb", "<u4"),
    ]))
    records["x"], records["y"], records["z"] = pts.T
    records["rgb"] = (bgr[:, 2].astype(np.uint32) << 16
                      | bgr[:, 1].astype(np.uint32) << 8
                      | bgr[:, 0].astype(np.uint32))
    message = PointCloud2()
    message.header.frame_id = str(frame_id)
    message.header.stamp = Time(nanoseconds=int(round(stamp * 1e9))).to_msg()
    message.height = 1
    message.width = int(records.shape[0])
    message.is_bigendian = False
    message.is_dense = False
    message.point_step = records.dtype.itemsize
    message.row_step = message.point_step * message.width
    message.fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        PointField(name="rgb", offset=12, datatype=PointField.UINT32, count=1),
    ]
    message.data = records.tobytes()
    return message


class TargetGraspViewer(Node):
    """Publish a Gemini target box and learned grasp to RViz only."""

    def __init__(self) -> None:
        super().__init__("target_grasp_viewer")
        self._declare_parameters()

        from tf2_ros import Buffer, TransformListener

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._source = self._make_source()
        self._gemini = GeminiClient(
            api_key=str(self.get_parameter("gemini_api_key").value),
            model=str(self.get_parameter("gemini_model").value),
            key_env=str(self.get_parameter("gemini_key_env").value),
            key_file=str(self.get_parameter("gemini_key_file").value),
            timeout_sec=float(self.get_parameter("gemini_timeout_sec").value),
            max_retries=int(self.get_parameter("gemini_max_retries").value),
            logger=self.get_logger())
        self._backend = make_backend(
            str(self.get_parameter("grasp_backend").value),
            logger=self.get_logger(), graspnet={
                "repo_path": str(self.get_parameter("graspnet_repo_path").value),
                "checkpoint_path": str(
                    self.get_parameter("graspnet_checkpoint").value),
                "device": str(self.get_parameter("graspnet_device").value),
                "top_k": int(self.get_parameter("top_k").value),
                "max_width": float(self.get_parameter(
                    "gripper_max_width_m").value),
                "sampling_seed": int(self.get_parameter(
                    "graspnet_sampling_seed").value),
                "collision_thresh": float(
                    self.get_parameter("graspnet_collision_thresh").value),
                "voxel_size": float(
                    self.get_parameter("graspnet_collision_voxel").value),
                "empty_thresh": float(
                    self.get_parameter("graspnet_empty_thresh").value),
            }, anygrasp={
                "runtime_dir": str(self.get_parameter(
                    "anygrasp_runtime_dir").value),
                "checkpoint_path": str(self.get_parameter(
                    "anygrasp_checkpoint").value),
                "license_dir": str(self.get_parameter(
                    "anygrasp_license_dir").value),
                "max_width": float(self.get_parameter(
                    "anygrasp_max_width").value),
                "gripper_height": float(self.get_parameter(
                    "anygrasp_gripper_height").value),
                "top_k": int(self.get_parameter("top_k").value),
                "dense_grasp": bool(self.get_parameter(
                    "anygrasp_dense_grasp").value),
                "collision_detection": bool(self.get_parameter(
                    "anygrasp_collision_detection").value),
            })
        self._target = str(self.get_parameter("target").value).strip()
        self._lock = threading.Lock()
        self._latest_frame: Optional[RGBDFrame] = None
        self._result: Optional[TargetGraspResult] = None
        self._result_generation = 0
        self._published_result_generation = -1
        self._status = (
            f"Call ~/run to execute Gemini + {self._backend.name} on the "
            "latest frame")
        self._inference_thread: Optional[threading.Thread] = None
        self._first_capture = True
        self._last_capture_error = ""
        self._last_cloud_publish = 0.0
        self._last_cloud_error = ""
        self._world_cloud_pub = self.create_publisher(
            PointCloud2, "~/world_cloud", 1)
        # Target result is produced only when ~/run is called.  Make it
        # latched so RViz also receives it if it subscribes after inference.
        result_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._target_cloud_pub = self.create_publisher(
            PointCloud2, "~/target_cloud", result_qos)
        self._marker_pub = self.create_publisher(
            MarkerArray, "~/markers", result_qos)
        self.create_subscription(String, "~/set_target", self._on_set_target, 10)
        self.create_service(Trigger, "~/run", self._srv_run)
        self.create_service(Trigger, "~/status", self._srv_status)
        self._timer = self.create_timer(
            1.0 / float(self.get_parameter("display_fps").value),
            self._update)
        self.get_logger().info(
            f"Target/{self._backend.name} RViz viewer is vision-only: "
            "no OpenCV window, "
            "MoveIt, arm, or gripper client is created. " + self._gemini.describe())

    def _declare_parameters(self) -> None:
        self.declare_parameter("target", "the object on the table")
        self.declare_parameter("camera_source", "realsense")
        self.declare_parameter("camera_serial", "427622271962")
        self.declare_parameter("camera_width", 640)
        self.declare_parameter("camera_height", 480)
        self.declare_parameter("camera_fps", 15)
        # Startup frames occasionally arrive after more than one second on
        # the wrist D405, even though the steady-state stream is 15 FPS.
        self.declare_parameter("camera_timeout_ms", 3000)
        self.declare_parameter("camera_optical_frame",
                               "d405_depth_optical_frame")
        self.declare_parameter("base_frame", "world")
        self.declare_parameter("color_topic", "/camera/color/image_raw")
        self.declare_parameter(
            "depth_topic", "/camera/aligned_depth_to_color/image_raw")
        self.declare_parameter("camera_info_topic", "/camera/color/camera_info")
        self.declare_parameter("topic_depth_scale", 0.0)
        self.declare_parameter("topic_sync_tolerance_s", 0.05)
        self.declare_parameter("topic_sync_queue_size", 10)
        self.declare_parameter("display_fps", 10.0)
        self.declare_parameter("cloud_publish_hz", 5.0)
        self.declare_parameter("cloud_stride", 2)
        self.declare_parameter("cloud_z_min", 0.05)
        self.declare_parameter("cloud_z_max", 0.80)
        self.declare_parameter("self_exclusion_radius_m", 0.09)
        self.declare_parameter("target_crop_pad_px", 4.0)
        self.declare_parameter("target_seed_radius_px", 14.0)
        self.declare_parameter("target_depth_tolerance_m", 0.05)
        self.declare_parameter("target_component_voxel_m", 0.008)
        self.declare_parameter("target_component_min_points", 30)
        self.declare_parameter("table_z", 0.0)
        self.declare_parameter("target_table_margin_m", 0.006)
        self.declare_parameter("target_bounds_margin_m", 0.020)
        self.declare_parameter("gripper_min_width_m", 0.010)
        self.declare_parameter("gripper_max_width_m", 0.065)
        # Optional measured clear aperture at the fully-open joint endpoint.
        # A negative value keeps this vision-only tool in an explicitly
        # assumed preview mode; it is never evidence for physical execution.
        self.declare_parameter("gripper_width_at_open_pos", -1.0)
        self.declare_parameter("grasp_min_clearance_m", 0.005)
        self.declare_parameter("grasp_max_tilt_rad", 1.50)
        self.declare_parameter("workspace_min", [0.08, -0.35, -0.05])
        self.declare_parameter("workspace_max", [0.50, 0.35, 0.45])
        self.declare_parameter("marker_pregrasp_standoff", 0.08)
        self.declare_parameter("gripper_scene_collision_enabled", True)
        self.declare_parameter("gripper_scene_collision_min_points", 3)
        self.declare_parameter("gripper_collision_finger_back_m", 0.070)
        self.declare_parameter("gripper_collision_finger_front_m", 0.021)
        self.declare_parameter(
            "gripper_collision_finger_thickness_m", 0.040)
        self.declare_parameter("gripper_collision_height_m", 0.058)
        self.declare_parameter("gripper_collision_margin_m", 0.002)
        self.declare_parameter("selection_score_slack", 0.15)
        self.declare_parameter(
            "selection_tilt_slack_rad", math.radians(10.0))
        self.declare_parameter("grasp_backend", "graspnet")
        self.declare_parameter("graspnet_repo_path", (
            "/mnt/agx_nvme/om6dof-graspnet-jp622/src/graspnet-baseline"))
        self.declare_parameter("graspnet_checkpoint", (
            "/mnt/agx_nvme/om6dof-graspnet-jp622/checkpoint-rs.tar"))
        self.declare_parameter("graspnet_device", "cuda")
        self.declare_parameter("top_k", 25)
        self.declare_parameter("graspnet_collision_thresh", 0.01)
        self.declare_parameter("graspnet_collision_voxel", 0.01)
        self.declare_parameter("graspnet_empty_thresh", 0.01)
        self.declare_parameter("graspnet_sampling_seed", 0)
        self.declare_parameter(
            "anygrasp_runtime_dir",
            "/home/kublab/ros2_ws/src/anygrasp_sdk/grasp_detection")
        self.declare_parameter(
            "anygrasp_checkpoint",
            "/home/kublab/ros2_ws/src/anygrasp_sdk/grasp_detection/"
            "checkpoint_detection.tar")
        self.declare_parameter("anygrasp_license_dir", "")
        self.declare_parameter("anygrasp_max_width", 0.065)
        self.declare_parameter("anygrasp_gripper_height", 0.058)
        self.declare_parameter("anygrasp_dense_grasp", False)
        self.declare_parameter("anygrasp_collision_detection", True)
        self.declare_parameter("gemini_api_key", "")
        self.declare_parameter("gemini_model", "gemini-3.5-flash-lite")
        self.declare_parameter("gemini_key_env", "GEMINI_API_KEY")
        self.declare_parameter("gemini_key_file",
                               "~/.config/om6dof/gemini_api_key")
        self.declare_parameter("gemini_timeout_sec", 20.0)
        self.declare_parameter("gemini_max_retries", 2)

    def _make_source(self):
        source = str(self.get_parameter("camera_source").value).lower()
        if source == "realsense":
            return make_source(
                source,
                width=int(self.get_parameter("camera_width").value),
                height=int(self.get_parameter("camera_height").value),
                fps=int(self.get_parameter("camera_fps").value),
                serial=str(self.get_parameter("camera_serial").value),
                optical_frame_id=str(
                    self.get_parameter("camera_optical_frame").value),
                logger=self.get_logger(), clock=self.get_clock())
        scale = float(self.get_parameter("topic_depth_scale").value)
        return make_source(
            source, node=self,
            color_topic=str(self.get_parameter("color_topic").value),
            depth_topic=str(self.get_parameter("depth_topic").value),
            info_topic=str(self.get_parameter("camera_info_topic").value),
            depth_scale=scale if scale > 0.0 else None,
            sync_tolerance_s=float(
                self.get_parameter("topic_sync_tolerance_s").value),
            sync_queue_size=int(
                self.get_parameter("topic_sync_queue_size").value))

    def _on_set_target(self, msg: String) -> None:
        target = msg.data.strip()
        if not target:
            self.get_logger().warn("Ignoring an empty target description")
            return
        with self._lock:
            self._target = target
            self._result = None
            self._result_generation += 1
            self._status = f"Target changed to '{target}'. Call ~/run."

    @staticmethod
    def _copy_frame(frame: RGBDFrame) -> RGBDFrame:
        return RGBDFrame(
            color=frame.color.copy(), depth=frame.depth.copy(),
            intrinsics=frame.intrinsics, depth_scale=frame.depth_scale,
            stamp=frame.stamp, frame_id=frame.frame_id)

    def _lookup_pose(self, source_frame: str, stamp: float) -> Tuple[np.ndarray, np.ndarray]:
        transform = self._tf_buffer.lookup_transform(
            str(self.get_parameter("base_frame").value), source_frame,
            Time(nanoseconds=int(round(stamp * 1e9))),
            timeout=Duration(seconds=0.1))
        translation = transform.transform.translation
        rotation = transform.transform.rotation
        return (np.array([translation.x, translation.y, translation.z]),
                quat_to_matrix(rotation.x, rotation.y, rotation.z, rotation.w))

    def _scene_from_frame(self, frame: RGBDFrame) -> GraspScene:
        optical_frame = frame.frame_id or str(
            self.get_parameter("camera_optical_frame").value)
        camera_position, camera_rotation = self._lookup_pose(
            optical_frame, frame.stamp)
        points_optical, colors, pixels = point_cloud(
            frame.depth, frame.intrinsics, frame.depth_scale,
            stride=int(self.get_parameter("cloud_stride").value),
            z_min=float(self.get_parameter("cloud_z_min").value),
            z_max=float(self.get_parameter("cloud_z_max").value),
            color=frame.color)
        points_base = points_to_base(points_optical, camera_position,
                                     camera_rotation)
        # Prevent the wrist camera from proposing its own fingers as the target.
        tool_rotation = None
        try:
            tool_position, tool_rotation = self._lookup_pose(
                "end_effector_link", frame.stamp)
            keep = self_exclusion_mask(
                points_base, tool_position,
                float(self.get_parameter("self_exclusion_radius_m").value))
            points_optical = points_optical[keep]
            points_base = points_base[keep]
            pixels = pixels[keep]
            colors = colors[keep] if colors is not None else None
        except Exception as exc:  # noqa: BLE001 - still show target evidence
            self.get_logger().warn(
                f"could not exclude gripper points: {exc}")
        return GraspScene(
            points_optical=points_optical, points_base=points_base,
            pixels=pixels, colors=colors, p_wc=camera_position,
            R_wc=camera_rotation, intrinsics=frame.intrinsics,
            color_image=frame.color, base_frame=str(
                self.get_parameter("base_frame").value),
            tool_rotation_base=tool_rotation,
            source_indices=np.arange(
                points_base.shape[0], dtype=np.int64))

    def _run_inference(self, frame: RGBDFrame, target: str) -> None:
        try:
            if not self._gemini.enabled:
                raise RuntimeError(self._gemini.describe())
            scene = self._scene_from_frame(frame)
            located = self._gemini.locate(scene.color_image, target)
            if not located.found:
                message = f"Gemini did not find '{target}': {located.reason}"
                result = TargetGraspResult(
                    target, located, scene, None, [], [], [], None, message)
            elif located.box is None:
                message = ("Gemini returned only a point, not a bounding box; "
                           f"{self._backend.name} target steering is skipped")
                result = TargetGraspResult(
                    target, located, scene, None, [], [], [], None, message)
            else:
                segmented = segment_target_component(
                    scene, located.box, located.pixel,
                    pad_px=float(self.get_parameter("target_crop_pad_px").value),
                    seed_radius_px=float(self.get_parameter(
                        "target_seed_radius_px").value),
                    depth_tolerance=float(self.get_parameter(
                        "target_depth_tolerance_m").value),
                    voxel_size=float(self.get_parameter(
                        "target_component_voxel_m").value),
                    min_points=int(self.get_parameter(
                        "target_component_min_points").value),
                    table_z=float(self.get_parameter("table_z").value),
                    table_margin=float(self.get_parameter(
                        "target_table_margin_m").value))
                if segmented is None:
                    message = ("Gemini box did not yield one usable target "
                               "component above the table")
                    result = TargetGraspResult(
                        target, located, scene, None, [], [], [], None, message)
                else:
                    # Compute this once from stable capture-row IDs and reuse
                    # it for AnyGrasp steering and collision exemptions. Never
                    # reconstruct target membership from approximate XYZ.
                    exact_target_mask = target_region_mask(scene, segmented)
                    # GraspNet clips each returned group before its external
                    # collision check. AnyGrasp's max width is immutable once
                    # its native detector is constructed and remains governed
                    # by the explicit anygrasp_max_width parameter.
                    if self._backend.name == "graspnet":
                        self._backend.max_width = float(self.get_parameter(
                            "gripper_max_width_m").value)
                    if getattr(
                            self._backend, "supports_region_steering", False):
                        # Full scene for collision, exact target rows for
                        # AnyGrasp proposal steering.
                        raw_candidates = self._backend.detect(
                            scene, collision_scene=scene,
                            region_mask=exact_target_mask)
                        network_point_count = scene.points_base.shape[0]
                    else:
                        # GraspNet-baseline has no region steering, so infer on
                        # the complete reachable scene and associate afterward.
                        network_scene = crop_to_workspace(
                            scene,
                            self.get_parameter("workspace_min").value,
                            self.get_parameter("workspace_max").value)
                        if network_scene.points_base.shape[0] < int(
                                self.get_parameter(
                                    "target_component_min_points").value):
                            raise RuntimeError(
                                "too few scene points inside robot workspace")
                        raw_candidates = self._backend.detect(
                            network_scene, collision_scene=scene)
                        network_point_count = network_scene.points_base.shape[0]
                    target_low, target_high = np.percentile(
                        segmented.points_base, [2, 98], axis=0)
                    filter_cfg = FilterConfig(
                        min_width=float(self.get_parameter(
                            "gripper_min_width_m").value),
                        max_width=float(self.get_parameter(
                            "gripper_max_width_m").value),
                        table_z=float(self.get_parameter("table_z").value),
                        min_clearance=float(self.get_parameter(
                            "grasp_min_clearance_m").value),
                        max_tilt=float(self.get_parameter(
                            "grasp_max_tilt_rad").value),
                        workspace_min=list(self.get_parameter(
                            "workspace_min").value),
                        workspace_max=list(self.get_parameter(
                            "workspace_max").value),
                        pregrasp_standoff=float(self.get_parameter(
                            "marker_pregrasp_standoff").value),
                        target_min=target_low,
                        target_max=target_high,
                        target_margin=float(self.get_parameter(
                            "target_bounds_margin_m").value))
                    scene_collision_check = None
                    collision_note = "disabled"
                    if bool(self.get_parameter(
                            "gripper_scene_collision_enabled").value):
                        try:
                            measured_open = float(self.get_parameter(
                                "gripper_width_at_open_pos").value)
                        except (TypeError, ValueError):
                            measured_open = math.nan
                        if math.isfinite(measured_open) and measured_open > 0.0:
                            open_aperture = measured_open
                            collision_note = (
                                f"target-aware open="
                                f"{open_aperture * 1000.0:.1f}mm "
                                "(UNVALIDATED PREVIEW ONLY)")
                        else:
                            open_aperture = float(filter_cfg.max_width)
                            collision_note = (
                                f"target-aware ASSUMED PREVIEW open="
                                f"{open_aperture * 1000.0:.1f}mm; NOT "
                                "EXECUTABLE")
                            self.get_logger().warn(collision_note)
                        scene_collision_check = lambda candidate: (
                            conservative_gripper_collision(
                                candidate, scene.points_base,
                                target_mask=exact_target_mask,
                                open_aperture=open_aperture,
                                pregrasp_standoff=float(
                                    filter_cfg.pregrasp_standoff),
                                finger_back=float(self.get_parameter(
                                    "gripper_collision_finger_back_m").value),
                                finger_front=float(self.get_parameter(
                                    "gripper_collision_finger_front_m").value),
                                finger_thickness=float(self.get_parameter(
                                    "gripper_collision_finger_thickness_m"
                                ).value),
                                gripper_height=float(self.get_parameter(
                                    "gripper_collision_height_m").value),
                                margin=float(self.get_parameter(
                                    "gripper_collision_margin_m").value),
                                min_points=int(self.get_parameter(
                                    "gripper_scene_collision_min_points"
                                ).value)))
                    candidates, rejected = filter_and_rank(
                        raw_candidates, filter_cfg,
                        scene_collision_check=scene_collision_check)
                    candidate_details = candidate_diagnostic_summary(
                        raw_candidates, rejected,
                        reference_rotation=scene.tool_rotation_base)
                    self.get_logger().info(
                        f"{self._backend.name} candidate details: "
                        f"{candidate_details}")
                    backend_stats = getattr(self._backend, "last_stats", {})
                    inference_counts = (
                        f"decoded={backend_stats.get('decoded', '?')}, "
                        f"width-clipped={backend_stats.get('width_clipped', 0)}, "
                        f"collision-kept="
                        f"{backend_stats.get('collision_kept', '?')}, "
                        f"after-nms={backend_stats.get('after_nms', '?')}, "
                        f"returned={len(raw_candidates)}")
                    selected = select_target_candidate(
                        candidates, segmented, located.pixel,
                        score_slack=float(self.get_parameter(
                            "selection_score_slack").value),
                        tilt_slack_rad=float(self.get_parameter(
                            "selection_tilt_slack_rad").value),
                        reference_rotation=scene.tool_rotation_base)
                    box = tuple(int(round(value)) for value in located.box)
                    selected_text = "none"
                    if selected is not None:
                        approach = np.asarray(selected.approach, dtype=float)
                        approach /= max(float(np.linalg.norm(approach)), 1e-9)
                        tilt_deg = math.degrees(math.acos(float(np.clip(
                            -approach[2], -1.0, 1.0))))
                        selected_text = (
                            f"score={selected.score:.3f}, "
                            f"tilt={tilt_deg:.1f}deg, "
                            f"width={selected.width * 1000:.0f}mm, "
                            f"xyz={np.round(selected.position, 3).tolist()}")
                        orientation_delta = selected.extras.get(
                            "orientation_delta_rad")
                        if orientation_delta is not None:
                            jaw_flip = bool(selected.extras.get(
                                "closing_axis_flipped", 0.0))
                            selected_text += (
                                f", dEE={math.degrees(orientation_delta):.1f}deg, "
                                f"jaw-flip={'yes' if jaw_flip else 'no'}")
                    message = (
                        f"Gemini conf={located.confidence:.2f} box={box}; "
                        f"target={segmented.points_base.shape[0]} points; "
                        f"network-scene={network_point_count} points; "
                        f"{self._backend.name} {inference_counts}, "
                        f"geometric-valid={len(candidates)}, "
                        f"rejected={len(rejected)} "
                        f"({rejection_summary(rejected)}); "
                        f"collision={collision_note}; "
                        f"selected={selected_text}; IK not checked; "
                        f"candidates=[{candidate_details}]")
                    result = TargetGraspResult(
                        target, located, scene, segmented, raw_candidates,
                        candidates, rejected, selected, message)
        except (GeminiError, RuntimeError, ValueError, TypeError) as exc:
            result = None
            message = f"Inference failed: {type(exc).__name__}: {exc}"
        with self._lock:
            self._result = result
            self._result_generation += 1
            self._status = message
            self._inference_thread = None
        if result is None:
            self.get_logger().error(message)
        else:
            self.get_logger().info(message)

    def _publish_world_cloud(self, frame: RGBDFrame) -> None:
        """Publish an RGB point cloud at a bounded rate for RViz."""
        now = self.get_clock().now().nanoseconds * 1e-9
        rate = max(0.1, float(self.get_parameter("cloud_publish_hz").value))
        if now - self._last_cloud_publish < 1.0 / rate:
            return
        self._last_cloud_publish = now
        try:
            scene = self._scene_from_frame(frame)
            self._world_cloud_pub.publish(cloud_message(
                scene.points_base, scene.colors, frame_id=scene.base_frame,
                stamp=frame.stamp))
            self._last_cloud_error = ""
        except Exception as exc:  # noqa: BLE001 - viewer remains useful in 2-D
            message = f"cannot publish world cloud: {exc}"
            if message != self._last_cloud_error:
                self.get_logger().warn(message)
                self._last_cloud_error = message

    def _publish_result_visuals(self, result: Optional[TargetGraspResult]) -> None:
        """Publish target cloud and parallel-jaw glyphs once per inference."""
        markers = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        markers.markers.append(clear)
        if result is None or result.target_scene is None:
            self._target_cloud_pub.publish(cloud_message(
                np.zeros((0, 3)), None,
                frame_id=str(self.get_parameter("base_frame").value),
                stamp=self.get_clock().now().nanoseconds * 1e-9))
            self._marker_pub.publish(markers)
            return

        scene = result.target_scene
        yellow = np.tile(np.array([[0, 215, 255]], dtype=np.uint8),
                         (scene.points_base.shape[0], 1))
        self._target_cloud_pub.publish(cloud_message(
            scene.points_base, yellow, frame_id=scene.base_frame,
            stamp=self.get_clock().now().nanoseconds * 1e-9))

        def append_selected_grasp(candidate: GraspCandidate) -> None:
            gripper = Marker()
            gripper.header.frame_id = scene.base_frame
            gripper.header.stamp = self.get_clock().now().to_msg()
            gripper.ns = "parallel_gripper_selected"
            gripper.id = 0
            gripper.type, gripper.action = Marker.LINE_LIST, Marker.ADD
            # Points are already expressed in world coordinates; keep the
            # marker pose explicitly identity (a zero quaternion is invalid).
            gripper.pose.orientation.w = 1.0
            gripper.points = []
            for xyz in parallel_gripper_points(candidate):
                point = Point()
                point.x, point.y, point.z = [float(value) for value in xyz]
                gripper.points.append(point)
            # One bold, fully opaque red marker. No rejected/debug candidates
            # are published by this target viewer.
            gripper.scale.x = 0.010
            gripper.color.r, gripper.color.g = 1.0, 0.02
            gripper.color.b, gripper.color.a = 0.02, 1.0
            markers.markers.append(gripper)

        if result.selected is not None:
            append_selected_grasp(result.selected)
        self._marker_pub.publish(markers)

    def _start_inference(self) -> Tuple[bool, str]:
        with self._lock:
            if self._inference_thread is not None:
                self._status = "Inference is already running"
                return False, self._status
            if self._latest_frame is None:
                self._status = "Waiting for a valid RGB-D frame"
                return False, self._status
            target = self._target
            if not target:
                self._status = "Set a non-empty target first"
                return False, self._status
            frame = self._copy_frame(self._latest_frame)
            self._status = (
                f"Running Gemini + {self._backend.name} for '{target}'...")
            self._inference_thread = threading.Thread(
                target=self._run_inference, args=(frame, target),
                name="target_grasp_inference", daemon=True)
            self._inference_thread.start()
            return True, self._status

    def _srv_run(self, _request, response):
        response.success, response.message = self._start_inference()
        return response

    def _srv_status(self, _request, response):
        with self._lock:
            response.success = self._latest_frame is not None
            response.message = self._status
        return response

    def _update(self) -> None:
        try:
            frame = self._source.capture(
                warmup=10 if self._first_capture else 1,
                timeout_ms=int(self.get_parameter("camera_timeout_ms").value))
            self._first_capture = False
        except Exception as exc:  # noqa: BLE001 - camera SDK errors are reported
            message = f"camera capture failed: {exc}"
            if message != self._last_capture_error:
                self.get_logger().error(message)
                self._last_capture_error = message
            return
        if frame is None:
            message = "camera returned no RGB-D frame"
            detail = getattr(self._source, "last_error", "")
            if detail:
                message += f": {detail}"
            if message != self._last_capture_error:
                self.get_logger().warn(message)
                self._last_capture_error = message
            return
        self._last_capture_error = ""
        with self._lock:
            self._latest_frame = self._copy_frame(frame)
            result = self._result
            result_generation = self._result_generation
        self._publish_world_cloud(frame)
        if result_generation != self._published_result_generation:
            self._publish_result_visuals(result)
            self._published_result_generation = result_generation

    def destroy_node(self) -> None:
        try:
            self._source.stop()
        finally:
            super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    executor = None
    try:
        node = TargetGraspViewer()
        executor = MultiThreadedExecutor(num_threads=3)
        executor.add_node(node)
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        if executor is not None:
            executor.shutdown(timeout_sec=1.0)
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
