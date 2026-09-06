"""Grasp-from-point-cloud pick-and-place with Gemini as the reasoning step.

This is the OM6DOF port of the ROBOTIS OMY "GraspNet pick and place" story. The
pipeline is the same six stages; only the bottom two layers differ, because OMY
publishes ``MoveL`` commands over Zenoh and this arm plans through MoveIt::

    capture RGB-D (wrist D405)
        -> point cloud in the camera optical frame
        -> grasp candidates            (analytic, or graspnet-baseline)
        -> filter + rank               (width, clearance, tilt, workspace, IK)
        -> Gemini                      (locate a described target, or name the pick)
        -> pregrasp / grasp / lift / place-approach / place / home
             through MoveGroup + GripperCommand

The camera rides on the wrist, so every capture is transformed through FK of the
joint state at capture time — the same chain ``om6dof_pick_and_place`` uses, with
the same calibrated ``camera_xyz`` / ``camera_rpy`` (calib GUI, port 8081).

Services
    ``~/run``       std_srvs/Trigger  run the whole sequence
    ``~/perceive``  std_srvs/Trigger  capture + detect + publish markers, no motion
    ``~/stop``      std_srvs/Trigger  ask a running sequence to stop at the next step
    ``~/status``    std_srvs/Trigger  report stage, last pick and last Gemini answer

Topics
    ``~/set_target``  std_msgs/String        target description for ``mode: target``
    ``~/status``      std_msgs/String        one line per stage transition
    ``~/grasp_markers`` visualization_msgs/MarkerArray  selected safe grasp
    ``~/debug_grasp_markers`` visualization_msgs/MarkerArray  non-selected grasps
    ``~/near_miss_markers`` visualization_msgs/MarkerArray  one rejected diagnostic

Only one process can hold the RealSense: stop ``om6dof_perception`` (and its
systemd unit) before running with ``camera_source: realsense``.
"""

from __future__ import annotations

import copy
import json
import math
import os
import signal
import threading
import time
from collections import deque
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np
import rclpy
import yaml
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import Point, Pose
from rcl_interfaces.msg import (ParameterDescriptor, ParameterType,
                                SetParametersResult)
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)
from rclpy.signals import SignalHandlerOptions
from rclpy.time import Time
from sensor_msgs.msg import JointState, PointCloud2, PointField
from std_msgs.msg import Bool, ColorRGBA, String
from std_srvs.srv import Trigger
from visualization_msgs.msg import Marker, MarkerArray

from .bus_health import BusHealthTracker
from .gemini_client import (Classification, GeminiClient, GeminiError,
                            GeminiUnavailable, Localization, crop_around)
from .grasp_backends import (GraspCandidate, GraspScene, crop_to_workspace,
                             make_backend, segment_target_component,
                             self_exclusion_mask, target_region_mask)
from .grasp_filter import (FilterConfig, NearMiss, Rejection, best_near_miss,
                           conservative_gripper_collision, filter_and_rank,
                           nearest_to_pixel, rejection_summary)
from .rgbd_source import make_source, point_cloud
from .target_selection import (align_parallel_jaw_orientation,
                               candidate_diagnostic_summary,
                               select_target_candidate)
from .target_grasp_viewer import parallel_gripper_points
from .transforms import (camera_pose_in_base, matrix_to_quat, points_to_base,
                         project, quat_to_matrix, rotation_distance,
                         rpy_to_matrix, tool_rotation)

ARM_JOINTS = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
MAX_PREVALIDATION_CANDIDATES = 20


def _cloud_message(points: np.ndarray, colors: Optional[np.ndarray], *,
                   frame_id: str, stamp: Time) -> PointCloud2:
    """Pack an RGB XYZ cloud for RViz without opening another camera."""
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
    message.header.stamp = stamp.to_msg()
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


def gripper_calibration_error(*, open_pos: float, close_pos: float,
                              width_at_open_pos: float,
                              width_at_close_pos: float,
                              min_width: Optional[float] = None,
                              max_width: Optional[float] = None
                              ) -> Optional[str]:
    """Return why an aperture-to-joint calibration is unsafe, if anything.

    Widths are the *measured clear jaw apertures* at the two configured joint
    positions.  They are deliberately independent from GraspNet's accepted
    width range: the latter may be narrower, but it must fit inside the
    physically calibrated aperture interval before real execution is allowed.
    """
    values = {
        "gripper_open_pos": open_pos,
        "gripper_close_pos": close_pos,
        "gripper_width_at_open_pos": width_at_open_pos,
        "gripper_width_at_close_pos": width_at_close_pos,
    }
    if min_width is not None:
        values["min_width"] = min_width
    if max_width is not None:
        values["max_width"] = max_width
    try:
        values = {name: float(value) for name, value in values.items()}
    except (TypeError, ValueError):
        return "gripper calibration values must be numeric"
    nonfinite = [name for name, value in values.items()
                 if not math.isfinite(value)]
    if nonfinite:
        return f"non-finite gripper calibration value(s): {', '.join(nonfinite)}"

    open_pos = values["gripper_open_pos"]
    close_pos = values["gripper_close_pos"]
    open_width = values["gripper_width_at_open_pos"]
    close_width = values["gripper_width_at_close_pos"]
    if abs(open_pos - close_pos) <= 1e-9:
        return "gripper_open_pos and gripper_close_pos must be different"
    if close_width < 0.0 or open_width < 0.0:
        return ("gripper aperture endpoints are unmeasured; enter non-negative "
                "measured widths in metres")
    if open_width <= close_width:
        return ("gripper_width_at_open_pos must be greater than "
                "gripper_width_at_close_pos")

    tolerance = 1e-9
    if min_width is not None and max_width is not None:
        accepted_min = values["min_width"]
        accepted_max = values["max_width"]
        if accepted_min < 0.0 or accepted_max <= accepted_min:
            return "GraspNet min_width/max_width must define a positive interval"
        if accepted_min + tolerance < close_width:
            return (f"min_width {accepted_min:.6f} m is below the calibrated "
                    f"closed aperture {close_width:.6f} m")
        if accepted_max > open_width + tolerance:
            return (f"max_width {accepted_max:.6f} m exceeds the calibrated "
                    f"open aperture {open_width:.6f} m")
    return None


def gripper_position_for_width(
        width: float, *, open_pos: float, close_pos: float, bias: float,
        width_at_open_pos: Optional[float] = None,
        width_at_close_pos: Optional[float] = None,
        max_width: Optional[float] = None) -> float:
    """Jaw command for an object of ``width`` metres.

    The commanded prismatic position is interpolated from the measured object
    width using two measured aperture endpoints, then pulled ``bias`` of the
    way further closed so the fingers load the object instead of stopping
    tangent to it — the same idea as the OMY story's
    ``gripper_close_bias``. ``bias`` 0 means "stop at the measured width", 1
    means "close fully".

    ``max_width`` remains as a legacy fallback for plan-only callers: it means
    an aperture interval of ``[0, max_width]``.  Physical execution never uses
    that assumption; its preflight requires both explicit measured endpoints.
    """
    if width_at_open_pos is None and max_width is not None:
        width_at_open_pos = max_width
        width_at_close_pos = 0.0
    if width_at_open_pos is None or width_at_close_pos is None:
        raise ValueError("both measured gripper aperture endpoints are required")
    error = gripper_calibration_error(
        open_pos=open_pos, close_pos=close_pos,
        width_at_open_pos=width_at_open_pos,
        width_at_close_pos=width_at_close_pos)
    if error is not None:
        raise ValueError(error)

    width = float(width)
    bias = float(bias)
    open_pos = float(open_pos)
    close_pos = float(close_pos)
    if not math.isfinite(width) or not math.isfinite(bias):
        raise ValueError("grasp width and gripper_close_bias must be finite")
    span = float(width_at_open_pos) - float(width_at_close_pos)
    ratio = float(np.clip(
        (width - float(width_at_close_pos)) / span, 0.0, 1.0))
    at_width = close_pos + ratio * (open_pos - close_pos)
    bias = float(np.clip(bias, 0.0, 1.0))
    command = at_width + bias * (close_pos - at_width)
    # Never hand the controller a position outside the configured jaw range,
    # not even by a float ulp.
    return float(np.clip(command, min(close_pos, open_pos),
                         max(close_pos, open_pos)))


def ee_position_for_tcp(grasp_position: Sequence[float],
                        tool_R: np.ndarray,
                        tcp_offset_xyz: Sequence[float]) -> np.ndarray:
    """Convert a desired fingertip/TCP point into an EE-link position.

    ``tcp_offset_xyz`` is the TCP position expressed in ``end_effector_link``.
    The current URDF places the EE origin between the fingers, so its nominal
    value is zero; keeping this explicit makes physical TCP calibration usable
    without changing grasp geometry code.
    """
    return (np.asarray(grasp_position, dtype=float)
            - np.asarray(tool_R, dtype=float)
            @ np.asarray(tcp_offset_xyz, dtype=float))


class GeminiPickNode(Node):
    """Sequencer for the whole pipeline."""

    def __init__(self) -> None:
        super().__init__("gemini_pick")
        self._cb = ReentrantCallbackGroup()
        self._declare_parameters()
        self._active_parameters = None
        self._read_parameters()

        self._lock = threading.Lock()
        self._joint_state: Optional[np.ndarray] = None
        self._joint_stamp = 0.0
        self._joint_history = deque(maxlen=500)
        self._last_bad_joint_state_warning = 0.0
        self._operation_mode: Optional[str] = None
        self._remote_enabled: Optional[bool] = None
        self._bus_health = BusHealthTracker(
            str(self._param("dynamixel_health_status_name")))
        self._last_bus_health_log_reason = ""
        self._last_bus_health_log_stamp = 0.0
        self._worker: Optional[threading.Thread] = None
        self._worker_lock = threading.Lock()
        # RealSenseSource owns one librealsense pipeline and wait_for_frames is
        # not safe to enter concurrently from the RViz preview timer and a
        # perception worker.
        self._camera_lock = threading.Lock()
        self._motion_sequence_active = False
        self._preflight_active = False
        self._stop_requested = False
        self._shutdown_requested = False
        self._auto_run_timer = None
        self._stage = "idle"
        self._last_pick: Dict[str, object] = {}
        self._last_gemini = ""
        self._last_perception = "not run yet"
        self._target_description = self._param("target_description")
        # Motion gates and calibration values must remain immutable for the
        # lifetime of one sequence. This also closes the race where a run could
        # pass preflight in plan-only mode and be switched to execution later.
        self.add_on_set_parameters_callback(self._on_parameter_update)

        from om6dof_controller.ik_solver import IKSolver
        from om6dof_pick_and_place.moveit_client import MoveItClient

        self.ik = IKSolver(base_link=self._param("ik_base_link"),
                           tip_link=self._param("ik_tip_link"),
                           urdf_pkg=self._param("ik_urdf_pkg"),
                           xacro_rel="urdf/om6dof.urdf.xacro")
        self.moveit = MoveItClient(
            self, ee_link=self._param("ik_tip_link"),
            reference_frame=self.base_frame, arm_joint_names=self.arm_joints,
            planning_time=float(self._param("planning_time")),
            num_planning_attempts=int(self._param("planning_attempts")),
            max_velocity_scaling=float(self._param("vel_scale")),
            max_acceleration_scaling=float(self._param("acc_scale")),
            position_tolerance=float(self._param("position_tolerance")),
            orientation_tolerance=float(self._param("orientation_tolerance")))

        self.gemini = GeminiClient(
            api_key=self._param("gemini_api_key"),
            model=self._param("gemini_model"),
            key_env=self._param("gemini_key_env"),
            key_file=self._param("gemini_key_file"),
            timeout_sec=float(self._param("gemini_timeout_sec")),
            max_retries=int(self._param("gemini_max_retries")),
            logger=self.get_logger())

        from tf2_ros import Buffer, TransformListener

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._warned_tf_fallback = False

        self.backend = self._make_backend()
        self.camera = self._make_camera()

        self.create_subscription(JointState, self._param("joint_state_topic"),
                                 self._on_joint_state, 10,
                                 callback_group=self._cb)
        state_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
        )
        self.create_subscription(
            String, self._param("operation_mode_state_topic"),
            self._on_operation_mode_state, state_qos,
            callback_group=self._cb)
        self.create_subscription(
            Bool, self._param("remote_enabled_state_topic"),
            self._on_remote_enabled_state, state_qos,
            callback_group=self._cb)
        self.create_subscription(
            DiagnosticArray, self._param("dynamixel_health_topic"),
            self._on_dynamixel_health, state_qos,
            callback_group=self._cb)
        self.create_subscription(String, "~/set_target", self._on_set_target, 10,
                                 callback_group=self._cb)
        self.pub_status = self.create_publisher(String, "~/status", 10)
        # Perception is request-driven rather than continuously streaming.
        # Latch the latest cloud and grasp result so RViz can subscribe after a
        # capture and still display exactly the scene used for planning.
        result_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
        )
        self.pub_world_cloud = self.create_publisher(
            PointCloud2, "~/world_cloud", result_qos)
        self.pub_target_cloud = self.create_publisher(
            PointCloud2, "~/target_cloud", result_qos)
        self.pub_markers = self.create_publisher(
            MarkerArray, "~/grasp_markers", result_qos)
        self.pub_debug_markers = self.create_publisher(
            MarkerArray, "~/debug_grasp_markers", result_qos)
        self.pub_near_miss_markers = self.create_publisher(
            MarkerArray, "~/near_miss_markers", result_qos)

        for name, handler in (("~/run", self._srv_run),
                              ("~/perceive", self._srv_perceive),
                              ("~/preflight", self._srv_preflight),
                              ("~/stop", self._srv_stop),
                              ("~/status", self._srv_status)):
            self.create_service(Trigger, name, handler, callback_group=self._cb)
        self._interlock_timer = self.create_timer(
            0.2, self._monitor_execution_interlocks,
            callback_group=self._cb)
        preview_hz = float(self._param("cloud_preview_hz"))
        if bool(self._param("cloud_preview_enabled")) and preview_hz > 0.0:
            self._cloud_preview_timer = self.create_timer(
                1.0 / preview_hz, self._preview_world_cloud,
                callback_group=self._cb)

        self.get_logger().info(
            f"mode={self.mode} backend={self.backend.name} "
            f"camera={self._param('camera_source')} | {self.gemini.describe()}")
        if not self.backend.available():
            self.get_logger().warn(
                f"grasp backend '{self.backend.name}' is not usable on this "
                f"machine; ~/run will fail until it is installed "
                f"(set grasp_backend: analytic to run without it)")
        if float(self._param("auto_run_delay")) > 0.0 and self._param("auto_run"):
            self._auto_run_timer = self.create_timer(
                float(self._param("auto_run_delay")), self._auto_run_once,
                callback_group=self._cb)

    # ---------------- parameters ----------------
    def _declare_parameters(self) -> None:
        self.declare_parameter("mode", "classify")          # classify | target
        self.declare_parameter("target_description", "the object on the table")

        self.declare_parameter("camera_source", "realsense")  # realsense | topic
        self.declare_parameter("camera_serial", "")
        self.declare_parameter("camera_width", 640)
        self.declare_parameter("camera_height", 480)
        self.declare_parameter("camera_fps", 15)
        self.declare_parameter("camera_warmup_frames", 10)
        self.declare_parameter("color_topic", "/camera/color/image_raw")
        self.declare_parameter("depth_topic",
                               "/camera/aligned_depth_to_color/image_raw")
        self.declare_parameter("camera_info_topic", "/camera/color/camera_info")

        # Live TF (published by robot_state_publisher from om6dof.urdf.xacro,
        # which encodes this mount's true, mesh-measured extrinsic) is the
        # primary source for the camera pose — see _camera_pose(). camera_xyz /
        # camera_rpy below are the FALLBACK only, used if that frame is not in
        # the TF tree (e.g. robot_state_publisher not running). Their default
        # is the same URDF chain resolved once by hand, not a borrowed number.
        self.declare_parameter("camera_optical_frame", "d405_depth_optical_frame")
        self.declare_parameter("realsense_aligned_frame",
                               "d405_depth_optical_frame")
        self.declare_parameter("topic_depth_scale", 0.0)
        self.declare_parameter("topic_sync_tolerance_s", 0.05)
        self.declare_parameter("topic_sync_queue_size", 10)
        self.declare_parameter("camera_xyz", [-0.087, 0.0, -0.074])
        self.declare_parameter("camera_rpy", [-0.436, 0.0, -1.571])
        self.declare_parameter("cloud_stride", 2)
        self.declare_parameter("cloud_z_min", 0.05)
        self.declare_parameter("cloud_z_max", 0.80)
        self.declare_parameter("cloud_preview_enabled", True)
        self.declare_parameter("cloud_preview_hz", 2.0)
        # Debug-only RViz layer. These markers never feed selection, IK,
        # planning, or execution; they only explain why raw grasps vanished.
        self.declare_parameter("debug_grasp_markers_enabled", False)
        self.declare_parameter("near_miss_markers_enabled", True)
        # Radius around end_effector_link excluded from the cloud before
        # detection — the arm's own gripper, not the scene. See
        # grasp_backends.self_exclusion_mask for how this was measured.
        self.declare_parameter("self_exclusion_radius_m", 0.09)

        self.declare_parameter(
            "grasp_backend", "analytic")  # analytic | graspnet | anygrasp
        self.declare_parameter("voxel", 0.006)
        self.declare_parameter("min_cluster_points", 40)
        self.declare_parameter("table_z", 0.0)
        self.declare_parameter("table_margin", 0.010)
        self.declare_parameter("grasp_depth", 0.020)
        self.declare_parameter("finger_clearance", 0.008)
        self.declare_parameter("approach_tilts", [0.0, 0.35, 0.70, 0.87, 1.05, 1.22, 1.40])
        self.declare_parameter("max_clusters", 8)

        self.declare_parameter("graspnet_repo_path", "")
        self.declare_parameter("graspnet_checkpoint", "")
        self.declare_parameter("graspnet_device", "cuda")
        self.declare_parameter("top_k", 50)
        self.declare_parameter("graspnet_collision_thresh", 0.01)
        self.declare_parameter("graspnet_collision_voxel", 0.01)
        self.declare_parameter("graspnet_empty_thresh", 0.01)
        self.declare_parameter("graspnet_sampling_seed", 0)
        self.declare_parameter("graspnet_sampling_attempts", 3)
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

        self.declare_parameter("min_width", 0.010)
        self.declare_parameter("max_width", 0.065)
        self.declare_parameter("min_clearance", 0.005)
        self.declare_parameter("max_tilt", 1.50)
        self.declare_parameter("workspace_min", [0.08, -0.35, -0.05])
        self.declare_parameter("workspace_max", [0.50, 0.35, 0.45])
        self.declare_parameter("min_score", 0.0)
        self.declare_parameter("check_reachability", True)
        self.declare_parameter("max_grasp_attempts", 1)
        self.declare_parameter("max_prevalidation_candidates", 5)

        self.declare_parameter("pregrasp_standoff", 0.10)
        self.declare_parameter("lift_height", 0.08)
        self.declare_parameter("place_approach_height", 0.10)

        self.declare_parameter("observe_pose", [0.0, -0.6806, 1.3613, 0.0, 0.8901, 0.0])
        self.declare_parameter("home_pose", [0.0, -0.6806, 1.3613, 0.0, 0.8901, 0.0])
        self.declare_parameter("default_place_pose", [1.2, -0.5, 1.2, 0.0, 0.9, 0.0])
        # Place bins live in their own YAML (see config/places.yaml), the way
        # the OMY story keeps omy_ai_graspnet_places.yaml separate. File order
        # matters: the LAST category is what an unsure classification falls to.
        self.declare_parameter("place_poses_file", "")
        self.declare_parameter(
            "place_categories", [],
            ParameterDescriptor(type=ParameterType.PARAMETER_STRING_ARRAY))
        self.declare_parameter("place_enabled", False)
        self.declare_parameter("place_poses_validated", False)

        self.declare_parameter("gripper_open_pos", 0.019)
        self.declare_parameter("gripper_close_pos", -0.010)
        # Explicit aperture measurements at the two joint endpoints.  Negative
        # defaults mean "not measured"; plan-only retains the legacy mapping,
        # while physical execution fails closed until these are calibrated.
        self.declare_parameter("gripper_width_at_open_pos", -1.0)
        self.declare_parameter("gripper_width_at_close_pos", -1.0)
        self.declare_parameter("gripper_calibration_validated", False)
        self.declare_parameter("gripper_close_bias", 0.6)
        self.declare_parameter("gripper_max_effort", 5.0)
        self.declare_parameter("gripper_settle_s", 1.2)
        self.declare_parameter(
            "gripper_joint_names",
            ["gripper_left_joint", "gripper_right_joint"])

        self.declare_parameter("gemini_api_key", "")
        self.declare_parameter("gemini_model", "gemini-3.5-flash-lite")
        self.declare_parameter("gemini_key_env", "GEMINI_API_KEY")
        self.declare_parameter("gemini_key_file", "~/.config/om6dof/gemini_api_key")
        self.declare_parameter("gemini_timeout_sec", 20.0)
        self.declare_parameter("gemini_max_retries", 2)
        self.declare_parameter("gemini_crop_half_px", 90)
        # Target mode uses the same tight connected-component segmentation as
        # target_grasp_viewer, so the grasp shown there is the grasp this node
        # evaluates for motion.
        self.declare_parameter("target_crop_pad_px", 4.0)
        self.declare_parameter("target_seed_radius_px", 14.0)
        self.declare_parameter("target_depth_tolerance_m", 0.05)
        self.declare_parameter("target_component_voxel_m", 0.008)
        self.declare_parameter("target_component_min_points", 30)
        # Depth (camera-Z) band, from the crop's nearest surface, treated as
        # the target — cuts out background/supporting surfaces a bounding box
        # often includes (measured on hardware: a box for "the pen on the
        # box" also caught the box's own surface behind it).
        self.declare_parameter("target_foreground_band_m", 0.03)
        # Much smaller than table_margin (which exists for whole-scene noise
        # rejection): a thin flat target can sit within ~1 cm of table_z, so
        # the general margin excludes it entirely. Measured on hardware: a
        # pen lying flat clustered correctly at 0.003, not at the 0.010
        # scene-wide default.
        self.declare_parameter("target_table_margin_m", 0.006)
        self.declare_parameter("target_bounds_margin_m", 0.020)
        self.declare_parameter("selection_score_slack", 0.15)
        self.declare_parameter("selection_tilt_slack_rad", math.radians(10.0))
        self.declare_parameter("gripper_scene_collision_enabled", True)
        self.declare_parameter("gripper_scene_collision_min_points", 3)
        self.declare_parameter("gripper_collision_finger_back_m", 0.070)
        self.declare_parameter("gripper_collision_finger_front_m", 0.021)
        self.declare_parameter("gripper_collision_finger_thickness_m", 0.040)
        self.declare_parameter("gripper_collision_height_m", 0.058)
        self.declare_parameter("gripper_collision_margin_m", 0.002)
        # Fallback only, for a model reply with a point but no box (see
        # _detect_in_target): how close a whole-scene candidate must land to
        # that point to count.
        self.declare_parameter("target_match_radius_px", 90.0)

        self.declare_parameter("base_frame", "world")
        self.declare_parameter("ik_base_link", "world")
        self.declare_parameter("ik_tip_link", "end_effector_link")
        self.declare_parameter("ik_urdf_pkg", "om6dof_description")
        self.declare_parameter("joint_state_topic", "/joint_states")
        self.declare_parameter(
            "operation_mode_state_topic", "/om6dof/operation_mode/state")
        self.declare_parameter(
            "remote_enabled_state_topic", "/om6dof/remote_enabled/state")
        self.declare_parameter(
            "dynamixel_health_topic", "/dynamixel_hardware_interface/health")
        self.declare_parameter(
            "dynamixel_health_status_name",
            "dynamixel_hardware_interface/BusHealth")
        self.declare_parameter("dynamixel_health_timeout_s", 0.30)
        self.declare_parameter("dynamixel_health_clean_window_s", 60.0)
        self.declare_parameter("arm_joint_names", ARM_JOINTS)
        self.declare_parameter("joint_state_timeout_s", 5.0)
        self.declare_parameter("joint_capture_tolerance_s", 0.15)

        self.declare_parameter("vel_scale", 0.2)
        self.declare_parameter("acc_scale", 0.2)
        self.declare_parameter("planning_time", 4.0)
        self.declare_parameter("planning_attempts", 20)
        self.declare_parameter("position_tolerance", 0.02)
        self.declare_parameter("orientation_tolerance", 0.35)
        # This gate validates the IK solution by running FK again.  The old
        # 20 mm / 0.20 rad defaults admitted solutions that MoveIt could not
        # follow during the final LIN approach.  Keep this substantially
        # tighter than the general OMPL goal tolerances.
        self.declare_parameter("ik_position_tolerance", 0.003)
        self.declare_parameter("ik_orientation_tolerance", 0.05)
        self.declare_parameter("grasp_position_tolerance", 0.008)
        self.declare_parameter("linear_orientation_tolerance", 0.10)
        self.declare_parameter("tcp_offset_xyz", [0.0, 0.0, 0.0])
        self.declare_parameter("calibration_validated", False)
        self.declare_parameter("moveit_server_timeout_s", 10.0)
        self.declare_parameter("table_collision_enabled", True)
        self.declare_parameter("table_collision_size", [0.60, 0.80, 0.05])
        self.declare_parameter("table_collision_center_xy", [0.35, 0.0])
        # Optional axis-aligned pedestal/support under a target. Its dimensions
        # deliberately default to zero and are rejected when enabled: collision
        # geometry must be measured, never inferred from a partial depth view.
        self.declare_parameter("target_support_enabled", False)
        self.declare_parameter("target_support_z", 0.0)
        self.declare_parameter("target_support_collision_size_x", 0.0)
        self.declare_parameter("target_support_collision_size_y", 0.0)
        self.declare_parameter("target_support_collision_size_z", 0.0)
        self.declare_parameter("target_support_collision_center_x", 0.0)
        self.declare_parameter("target_support_collision_center_y", 0.0)

        self.declare_parameter("execute_motion", False)
        self.declare_parameter("auto_run", False)
        self.declare_parameter("auto_run_delay", 8.0)

    def _param(self, name: str):
        snapshot = self._active_parameters
        if snapshot is not None and name in snapshot:
            return snapshot[name]
        return self.get_parameter(name).value

    def _snapshot_parameters(self) -> Dict[str, object]:
        """Deep-copy one parameter view for the full lifetime of a run."""
        # Humble's rclpy Node has no ``list_parameters`` method.  An empty
        # prefix is the documented way to retrieve all locally declared
        # parameter values on this release.
        parameters = self.get_parameters_by_prefix("")
        return {
            name: copy.deepcopy(
                parameter.value if hasattr(parameter, "value") else parameter)
            for name, parameter in parameters.items()
        }

    def _read_parameters(self) -> None:
        self.mode = str(self._param("mode")).lower()
        self.base_frame = str(self._param("base_frame"))
        self.arm_joints = [str(j) for j in self._param("arm_joint_names")]
        # Fallback only — _camera_pose() prefers live TF. See the parameter
        # comment on camera_xyz for why.
        self._fallback_t_ec = np.array(
            [float(v) for v in self._param("camera_xyz")])
        # camera_rpy is already the resolved end_effector_link -> D405
        # optical-frame orientation from the URDF chain.  Do not append the
        # generic body -> optical rotation a second time.
        self._fallback_R_eo = rpy_to_matrix(
            *[float(v) for v in self._param("camera_rpy")])
        self.filter_cfg = FilterConfig(
            min_width=float(self._param("min_width")),
            max_width=float(self._param("max_width")),
            table_z=float(self._param("table_z")),
            min_clearance=float(self._param("min_clearance")),
            max_tilt=float(self._param("max_tilt")),
            workspace_min=[float(v) for v in self._param("workspace_min")],
            workspace_max=[float(v) for v in self._param("workspace_max")],
            pregrasp_standoff=float(self._param("pregrasp_standoff")),
            min_score=float(self._param("min_score")))
        self.place_poses = self._load_place_poses(
            str(self._param("place_poses_file")))
        categories = [str(c) for c in self._param("place_categories")]
        self.place_categories = categories or list(self.place_poses.keys()) \
            or ["unknown"]

    def _load_place_poses(self, path: str) -> Dict[str, List[float]]:
        """Load category -> 6 joint values.

        An unreadable file is a warning rather than a startup crash, but it
        never enables a fallback motion: place execution later fails closed
        unless a named or explicit ``unknown`` bin exists.
        """
        if not path:
            from ament_index_python.packages import PackageNotFoundError, \
                get_package_share_directory
            try:
                path = os.path.join(
                    get_package_share_directory("om6dof_pick_and_place_gemini"),
                    "config", "places.yaml")
            except PackageNotFoundError:
                return {}
        if not os.path.isfile(path):
            self.get_logger().warn(f"no place-pose file at {path}")
            return {}
        try:
            with open(path, "r", encoding="utf-8") as handle:
                raw = yaml.safe_load(handle) or {}
            entries = raw.get("places", raw)
            poses = {str(k).lower(): [float(v) for v in values]
                     for k, values in entries.items()}
        except (OSError, ValueError, TypeError, AttributeError,
                yaml.YAMLError) as exc:
            self.get_logger().error(
                f"cannot read place poses from {path}: {exc}")
            return {}
        bad = {k: v for k, v in poses.items() if len(v) != len(self.arm_joints)}
        for name, values in bad.items():
            self.get_logger().error(
                f"place pose '{name}' has {len(values)} joint values, "
                f"expected {len(self.arm_joints)} — ignoring it")
            poses.pop(name)
        self.get_logger().info(
            f"place bins from {path}: {', '.join(poses) or '(none)'}")
        return poses

    def _make_backend(self):
        return make_backend(
            str(self._param("grasp_backend")), logger=self.get_logger(),
            analytic=dict(
                voxel=float(self._param("voxel")),
                min_points=int(self._param("min_cluster_points")),
                table_z=float(self._param("table_z")),
                table_margin=float(self._param("table_margin")),
                grasp_depth=float(self._param("grasp_depth")),
                finger_clearance=float(self._param("finger_clearance")),
                min_width=float(self._param("min_width")),
                max_width=float(self._param("max_width")),
                approach_tilts=[float(v) for v in self._param("approach_tilts")],
                max_clusters=int(self._param("max_clusters"))),
            graspnet=dict(
                repo_path=str(self._param("graspnet_repo_path")),
                checkpoint_path=str(self._param("graspnet_checkpoint")),
                device=str(self._param("graspnet_device")),
                top_k=int(self._param("top_k")),
                max_width=float(self._param("max_width")),
                sampling_seed=int(self._param("graspnet_sampling_seed")),
                collision_thresh=float(
                    self._param("graspnet_collision_thresh")),
                voxel_size=float(self._param("graspnet_collision_voxel")),
                empty_thresh=float(self._param("graspnet_empty_thresh"))),
            anygrasp=dict(
                runtime_dir=str(self._param("anygrasp_runtime_dir")),
                checkpoint_path=str(self._param("anygrasp_checkpoint")),
                license_dir=str(self._param("anygrasp_license_dir")),
                max_width=float(self._param("anygrasp_max_width")),
                gripper_height=float(
                    self._param("anygrasp_gripper_height")),
                top_k=int(self._param("top_k")),
                dense_grasp=bool(self._param("anygrasp_dense_grasp")),
                collision_detection=bool(
                    self._param("anygrasp_collision_detection"))))

    def _make_camera(self):
        source = str(self._param("camera_source")).lower()
        if source == "realsense":
            return make_source("realsense",
                               width=int(self._param("camera_width")),
                               height=int(self._param("camera_height")),
                               fps=int(self._param("camera_fps")),
                               serial=str(self._param("camera_serial")),
                               logger=self.get_logger(),
                               optical_frame_id=str(
                                   self._param("realsense_aligned_frame")),
                               clock=self.get_clock())
        depth_scale = float(self._param("topic_depth_scale"))
        return make_source("topic", node=self,
                           color_topic=str(self._param("color_topic")),
                           depth_topic=str(self._param("depth_topic")),
                           info_topic=str(self._param("camera_info_topic")),
                           depth_scale=(depth_scale
                                        if depth_scale > 0.0 else None),
                           sync_tolerance_s=float(
                               self._param("topic_sync_tolerance_s")),
                           sync_queue_size=int(
                               self._param("topic_sync_queue_size")))

    # ---------------- subscriptions ----------------
    def _on_joint_state(self, msg: JointState) -> None:
        index = {name: i for i, name in enumerate(msg.name)}
        if not all(j in index for j in self.arm_joints):
            return
        required_indices = [index[j] for j in self.arm_joints]
        if (not required_indices
                or max(required_indices) >= len(msg.position)):
            now = time.monotonic()
            if (now - getattr(self, "_last_bad_joint_state_warning", 0.0)
                    >= 5.0):
                self.get_logger().warn(
                    "Ignoring malformed JointState: joint names and position "
                    "array lengths are inconsistent")
                self._last_bad_joint_state_warning = now
            return
        joints = np.asarray(
            [float(msg.position[i]) for i in required_indices], dtype=float)
        if not np.all(np.isfinite(joints)):
            now = time.monotonic()
            if (now - getattr(self, "_last_bad_joint_state_warning", 0.0)
                    >= 5.0):
                self.get_logger().warn(
                    "Ignoring malformed JointState with non-finite positions")
                self._last_bad_joint_state_warning = now
            return
        header = getattr(msg, "header", None)
        stamp_msg = getattr(header, "stamp", None)
        stamp = (float(getattr(stamp_msg, "sec", 0.0))
                 + float(getattr(stamp_msg, "nanosec", 0.0)) * 1e-9)
        if not math.isfinite(stamp) or stamp <= 0.0:
            stamp = float(self.get_clock().now().nanoseconds) * 1e-9
        with self._lock:
            self._joint_state = joints
            self._joint_stamp = time.monotonic()
            self._joint_history.append((stamp, joints.copy()))

    def _on_operation_mode_state(self, msg: String) -> None:
        mode = str(msg.data).strip().upper()
        with self._lock:
            self._operation_mode = mode
        if mode != "AUTONOMOUS":
            self._trip_execution_interlock(
                f"operation mode changed to {mode!r}")

    def _on_remote_enabled_state(self, msg: Bool) -> None:
        enabled = bool(msg.data)
        with self._lock:
            self._remote_enabled = enabled
        if enabled:
            self._trip_execution_interlock("remote control became enabled")

    def _bus_health_assessment(self):
        return self._bus_health.assess(
            timeout_s=float(self._param("dynamixel_health_timeout_s")),
            clean_window_s=float(
                self._param("dynamixel_health_clean_window_s")))

    def _on_dynamixel_health(self, msg: DiagnosticArray) -> None:
        self._bus_health.update(msg)
        assessment = self._bus_health_assessment()
        with self._worker_lock:
            active = (self._motion_sequence_active
                      and not self._stop_requested)
        if (active and bool(self._param("execute_motion"))
                and not assessment.ready):
            self._trip_execution_interlock(
                f"Dynamixel bus health became unsafe: {assessment.reason}")

    def _dynamixel_bus_ready(self) -> bool:
        topic = str(self._param("dynamixel_health_topic"))
        publisher_count = self.count_publishers(topic)
        if publisher_count != 1:
            reason = (f"expected exactly one publisher on {topic}, "
                      f"found {publisher_count}")
            ready = False
        else:
            assessment = self._bus_health_assessment()
            reason = assessment.reason
            ready = assessment.ready
        if not ready:
            now = time.monotonic()
            if (reason != self._last_bus_health_log_reason
                    or now - self._last_bus_health_log_stamp >= 5.0):
                self.get_logger().error(
                    f"execution refused: Dynamixel bus is not healthy: {reason}")
                self._last_bus_health_log_reason = reason
                self._last_bus_health_log_stamp = now
        return ready

    def _trip_execution_interlock(self, reason: str) -> None:
        """Fail-stop an executing worker when controller ownership changes."""
        if not bool(self._param("execute_motion")):
            return
        with self._worker_lock:
            if (not self._busy_unlocked()
                    or not self._motion_sequence_active):
                return
            self._stop_requested = True
            cancelled = self.moveit.cancel_current_goal()
        self.get_logger().error(
            f"execution interlock tripped: {reason}"
            + ("; active action cancellation requested" if cancelled else ""))

    def _monitor_execution_interlocks(self) -> None:
        with self._worker_lock:
            motion_active = self._motion_sequence_active
            stop_requested = self._stop_requested
        # Keep reissuing direct controller cancel-all rounds while an owned
        # physical action remains in doubt.  One early round can legitimately
        # acknowledge zero goals while MoveGroup is still planning and only
        # creates the controller goal later.
        if ((stop_requested or self.moveit.motion_faulted)
                and self.moveit.physical_action_in_flight):
            self.moveit.cancel_controller_goals()
        if (motion_active and bool(self._param("execute_motion"))
                and not self._execution_interlocks_ready()):
            self._trip_execution_interlock(
                "controller state became unavailable or unsafe")

    def _on_set_target(self, msg: String) -> None:
        self._target_description = msg.data.strip()
        self._publish_status(f"target set to '{self._target_description}'")

    def _current_joints(self) -> Optional[np.ndarray]:
        timeout = float(self._param("joint_state_timeout_s"))
        with self._lock:
            fresh = (self._joint_state is not None
                     and time.monotonic() - self._joint_stamp < timeout)
            return self._joint_state.copy() if fresh else None

    def _joints_for_capture(self, stamp: float) -> Optional[np.ndarray]:
        """Nearest fresh joint sample in the same ROS-time domain as a frame."""
        tolerance = float(self._param("joint_capture_tolerance_s"))
        if not math.isfinite(stamp) or stamp <= 0.0 or tolerance < 0.0:
            return None
        with self._lock:
            fresh = (self._joint_state is not None
                     and time.monotonic() - self._joint_stamp
                     < float(self._param("joint_state_timeout_s")))
            history = list(self._joint_history)
        if not fresh or not history:
            return None
        sample_stamp, joints = min(
            history, key=lambda sample: abs(sample[0] - stamp))
        if abs(sample_stamp - stamp) > tolerance:
            return None
        return joints.copy()

    # ---------------- services ----------------
    def _busy_unlocked(self) -> bool:
        return (self._preflight_active
                or (self._worker is not None and self._worker.is_alive()))

    def _busy(self) -> bool:
        with self._worker_lock:
            return self._busy_unlocked()

    def _on_parameter_update(self, parameters) -> SetParametersResult:
        if self._busy():
            names = ", ".join(parameter.name for parameter in parameters)
            return SetParametersResult(
                successful=False,
                reason=("parameters are locked while a pick sequence is "
                        f"running ({names})"),
            )
        return SetParametersResult(successful=True)

    def _start(self, target, name: str, response: Trigger.Response,
               *, motion_sequence: bool = False):
        with self._worker_lock:
            if self._shutdown_requested:
                response.success = False
                response.message = "node is shutting down"
                return response
            if self._busy_unlocked():
                response.success = False
                response.message = f"busy: {self._stage}"
                return response
            if not self.moveit.begin_sequence():
                response.success = False
                response.message = (
                    "motion commands remain locked after an uncertain action; "
                    "verify robot state and restart this node")
                return response
            self._active_parameters = self._snapshot_parameters()
            self._motion_sequence_active = bool(motion_sequence)
            self._stop_requested = False
            self._worker = threading.Thread(
                target=self._worker_entry, args=(target,),
                name=name, daemon=True)
            self._worker.start()
        response.success = True
        response.message = f"{name} started"
        return response

    def _worker_entry(self, target) -> None:
        try:
            target()
        finally:
            with self._worker_lock:
                self._active_parameters = None
                self._motion_sequence_active = False

    def _srv_run(self, _request, response):
        return self._start(self._run_sequence, "gemini_pick_run", response,
                           motion_sequence=True)

    def _srv_perceive(self, _request, response):
        return self._start(self._run_perceive_only, "gemini_pick_perceive",
                           response)

    def _srv_preflight(self, _request, response):
        with self._worker_lock:
            if self._shutdown_requested:
                response.success = False
                response.message = "node is shutting down"
                return response
            if self._busy_unlocked():
                response.success = False
                response.message = f"busy: {self._stage}"
                return response
            self._preflight_active = True
        try:
            response.success = self._motion_preflight()
        finally:
            with self._worker_lock:
                self._preflight_active = False
        response.message = (
            "motion preflight passed" if response.success
            else "motion preflight failed; inspect node log")
        return response

    def _srv_stop(self, _request, response):
        # Linearize stop against worker creation. Both paths take worker_lock
        # before MoveItClient's goal lock, so a completed stop cannot be undone
        # by a concurrently-starting sequence.
        with self._worker_lock:
            self._stop_requested = True
            cancelled = self.moveit.cancel_current_goal()
        response.success = True
        response.message = (
            f"stop requested at stage '{self._stage}'"
            + ("; active action cancellation requested" if cancelled else ""))
        return response

    def _srv_status(self, _request, response):
        with self._lock:
            operation_mode = self._operation_mode
            remote_enabled = self._remote_enabled
        bus_health = self._bus_health_assessment()
        response.success = True
        response.message = json.dumps({
            "stage": self._stage,
            "busy": self._busy(),
            "mode": self.mode,
            "backend": self.backend.name,
            "execute_motion": bool(self._param("execute_motion")),
            "motion_faulted": self.moveit.motion_faulted,
            "operation_mode": operation_mode,
            "remote_enabled": remote_enabled,
            "dynamixel_bus_health": bus_health.as_dict(),
            "calibration_validated": bool(
                self._param("calibration_validated")),
            "gripper_calibration_validated": bool(
                self._param("gripper_calibration_validated")),
            "place_enabled": bool(self._param("place_enabled")),
            "place_poses_validated": bool(
                self._param("place_poses_validated")),
            "target": self._target_description,
            "gemini": self._last_gemini,
            "last_perception": self._last_perception,
            "last_pick": self._last_pick,
        }, default=str)
        return response

    def _auto_run_once(self) -> None:
        if self._auto_run_timer is not None:
            self._auto_run_timer.cancel()
            self._auto_run_timer = None
        if not self._busy():
            self._start(self._run_sequence, "gemini_pick_run",
                        Trigger.Response(), motion_sequence=True)

    # ---------------- stage helpers ----------------
    def _publish_status(self, text: str) -> None:
        self.get_logger().info(text)
        message = String()
        message.data = text
        self.pub_status.publish(message)

    def _set_stage(self, stage: str) -> None:
        self._stage = stage
        self._publish_status(f"[{stage}]")

    def _should_stop(self) -> bool:
        if self._stop_requested:
            self._publish_status("stop requested — sequence aborted")
            self._set_stage("stopped")
            return True
        return False

    # ---------------- perception ----------------
    def _camera_pose(self, joints: np.ndarray, frame_id: str = "",
                     stamp: Optional[float] = None
                     ) -> Tuple[np.ndarray, np.ndarray, str]:
        """Camera optical frame in ``base_frame``: (position, rotation, source).

        Primary source is live TF — ``camera_optical_frame`` as published by
        robot_state_publisher from ``om6dof.urdf.xacro``, which encodes this
        mount's true extrinsic (measured off the fused bracket+D405 mesh, not a
        borrowed convention). Falls back to FK of ``joints`` composed with the
        ``camera_xyz``/``camera_rpy`` parameters only if that frame is not in
        the TF tree; the fallback is logged once, not every capture, so a
        permanently-missing frame does not spam the log every run.
        """
        configured_frame = str(self._param("camera_optical_frame"))
        optical_frame = str(frame_id).strip() or configured_frame
        if optical_frame:
            try:
                if stamp is None:
                    capture_time = Time()
                else:
                    if not math.isfinite(stamp) or stamp <= 0.0:
                        raise RuntimeError("camera frame has an invalid timestamp")
                    capture_time = Time(
                        nanoseconds=int(round(stamp * 1e9)),
                        clock_type=self.get_clock().clock_type)
                transform = self._tf_buffer.lookup_transform(
                    self.base_frame, optical_frame, capture_time,
                    timeout=Duration(seconds=1.0))
                t = transform.transform.translation
                q = transform.transform.rotation
                position = np.array([t.x, t.y, t.z])
                rotation = quat_to_matrix(q.x, q.y, q.z, q.w)
                return position, rotation, f"TF({optical_frame})"
            except Exception as exc:    # noqa: BLE001 - tf2 raises several types
                if not self._warned_tf_fallback:
                    self.get_logger().warn(
                        f"no TF from {self.base_frame} to {optical_frame} "
                        f"({exc}); falling back to the camera_xyz/camera_rpy "
                        f"parameters for every capture until this frame "
                        f"appears (check robot_state_publisher is running "
                        f"the om6dof_description URDF with the D405 mount)")
                    self._warned_tf_fallback = True
                if optical_frame != configured_frame:
                    raise RuntimeError(
                        f"capture is in '{optical_frame}', but no TF exists and "
                        f"fallback extrinsics describe '{configured_frame}'") from exc
        p_we, R_we = self.ik.fk_pose(joints)
        position, rotation = camera_pose_in_base(
            p_we, R_we, self._fallback_t_ec, self._fallback_R_eo)
        return position, rotation, "fallback params"

    def _preview_world_cloud(self) -> None:
        """Refresh RViz without running Gemini, GraspNet, or any motion."""
        if self._busy():
            return
        try:
            self.capture_scene(publish_status=False, warmup_frames=1)
        except Exception as exc:  # noqa: BLE001 - preview must not kill node
            self.get_logger().warn(
                f"RGB-D cloud preview failed: {type(exc).__name__}: {exc}",
                throttle_duration_sec=5.0)

    def capture_scene(self, *, publish_status: bool = True,
                      warmup_frames: Optional[int] = None
                      ) -> Optional[GraspScene]:
        """One RGB-D frame, deprojected and carried into the base frame."""
        joints = self._current_joints()
        if joints is None:
            self.get_logger().error(
                f"no fresh {self._param('joint_state_topic')} — is the bringup "
                f"running?")
            return None
        warmup = (int(self._param("camera_warmup_frames"))
                  if warmup_frames is None else int(warmup_frames))
        with self._camera_lock:
            frame = self.camera.capture(warmup=warmup)
        if frame is None:
            detail = getattr(self.camera, "last_error", "")
            self.get_logger().error(
                "camera returned no frame" + (f": {detail}" if detail else ""))
            return None
        joints = self._joints_for_capture(frame.stamp)
        if joints is None:
            self.get_logger().error(
                "no fresh joint sample close enough to the RGB-D timestamp "
                f"(tolerance={self._param('joint_capture_tolerance_s')} s)")
            return None

        points_opt, colors, pixels = point_cloud(
            frame.depth, frame.intrinsics, frame.depth_scale,
            stride=int(self._param("cloud_stride")),
            z_min=float(self._param("cloud_z_min")),
            z_max=float(self._param("cloud_z_max")),
            color=frame.color)
        try:
            p_wc, R_wc, source = self._camera_pose(
                joints, frame.frame_id, frame.stamp)
        except RuntimeError as exc:
            self.get_logger().error(str(exc))
            return None
        points_base = points_to_base(points_opt, p_wc, R_wc)

        # This FK uses the joint sample nearest the RGB-D timestamp. Keep its
        # orientation with the scene so target selection and IK use the same
        # capture-time gripper reference as the vision-only viewer.
        p_we, R_we = self.ik.fk_pose(joints)
        radius = float(self._param("self_exclusion_radius_m"))
        keep = self_exclusion_mask(points_base, p_we, radius)
        removed = int((~keep).sum())
        points_opt, points_base, pixels = \
            points_opt[keep], points_base[keep], pixels[keep]
        colors = colors[keep] if colors is not None else None

        if publish_status:
            self._publish_status(
                f"captured {points_base.shape[0]} points ({removed} within "
                f"{radius * 100:.0f} cm of the gripper excluded), camera at "
                f"{np.round(p_wc, 3).tolist()} in {self.base_frame} "
                f"(extrinsic: {source})")
        self.pub_world_cloud.publish(_cloud_message(
            points_base, colors, frame_id=self.base_frame,
            stamp=self.get_clock().now()))
        return GraspScene(points_optical=points_opt, points_base=points_base,
                          pixels=pixels, colors=colors, p_wc=p_wc, R_wc=R_wc,
                          intrinsics=frame.intrinsics, color_image=frame.color,
                          base_frame=self.base_frame,
                          tool_rotation_base=R_we,
                          source_indices=np.arange(
                              points_base.shape[0], dtype=np.int64))

    def _tcp_offset(self) -> np.ndarray:
        offset = np.asarray(self._param("tcp_offset_xyz"), dtype=float)
        if offset.shape != (3,) or not np.all(np.isfinite(offset)):
            raise ValueError("tcp_offset_xyz must contain three finite values")
        return offset

    def _ee_target(self, tcp_position: np.ndarray,
                   rotation: np.ndarray) -> np.ndarray:
        return ee_position_for_tcp(tcp_position, rotation, self._tcp_offset())

    def _solve_ik_checked(self, seed: np.ndarray, position: np.ndarray,
                          rotation: np.ndarray
                          ) -> Tuple[np.ndarray, bool, float, float]:
        """Solve IK and judge the FK residual at hardware-scale tolerances.

        The numerical solver's internal convergence threshold is deliberately
        much tighter than a physical arm can achieve, so its boolean alone is
        not a useful reachability verdict.
        """
        solution, _converged = self.ik.solve_pose_ik(
            np.asarray(seed, dtype=float), np.asarray(position, dtype=float),
            np.asarray(rotation, dtype=float))
        solution = np.asarray(solution, dtype=float)
        if not np.all(np.isfinite(solution)):
            return solution, False, math.inf, math.inf
        actual_position, actual_rotation = self.ik.fk_pose(solution)
        position_error = float(np.linalg.norm(
            np.asarray(actual_position) - np.asarray(position)))
        orientation_error = rotation_distance(actual_rotation, rotation)
        usable = (
            position_error <= float(self._param("ik_position_tolerance"))
            and orientation_error
            <= float(self._param("ik_orientation_tolerance"))
        )
        return solution, usable, position_error, orientation_error

    def _reachable_chain(self, pregrasp: np.ndarray, grasp: np.ndarray,
                         candidate: GraspCandidate) -> Tuple[bool, str]:
        """Check the complete pick IK chain with a safe retreat fallback."""
        joints = self._current_joints()
        if joints is None:
            return False, "no fresh joint state for IK"
        if self._plan_only():
            planned_observe = np.asarray(
                self._param("observe_pose"), dtype=float)
            if planned_observe.shape == joints.shape:
                joints = planned_observe
        rotation = tool_rotation(candidate.approach, candidate.closing)
        try:
            pregrasp_ee = self._ee_target(pregrasp, rotation)
            grasp_ee = self._ee_target(grasp, rotation)
            lift_ee = grasp_ee + np.array(
                [0.0, 0.0, float(self._param("lift_height"))])
            q_pre, pre_ok, pre_pos, pre_ori = self._solve_ik_checked(
                joints, pregrasp_ee, rotation)
            candidate.extras["ik_pregrasp_position_error"] = pre_pos
            candidate.extras["ik_pregrasp_orientation_error"] = pre_ori
            if not pre_ok:
                return (False, "pregrasp residual "
                        f"{pre_pos * 1000.0:.1f} mm/{math.degrees(pre_ori):.1f} deg")
            q_grasp, grasp_ok, grasp_pos, grasp_ori = self._solve_ik_checked(
                q_pre, grasp_ee, rotation)
            candidate.extras["ik_grasp_position_error"] = grasp_pos
            candidate.extras["ik_grasp_orientation_error"] = grasp_ori
            if not grasp_ok:
                return (False, "grasp residual "
                        f"{grasp_pos * 1000.0:.1f} mm/"
                        f"{math.degrees(grasp_ori):.1f} deg")
            _q_lift, lift_ok, lift_pos, lift_ori = self._solve_ik_checked(
                q_grasp, lift_ee, rotation)
            candidate.extras["ik_lift_position_error"] = lift_pos
            candidate.extras["ik_lift_orientation_error"] = lift_ori
            if lift_ok:
                candidate.extras["post_grasp_mode"] = "vertical_lift"
                candidate.extras["post_grasp_target_x"] = float(lift_ee[0])
                candidate.extras["post_grasp_target_y"] = float(lift_ee[1])
                candidate.extras["post_grasp_target_z"] = float(lift_ee[2])
                candidate.extras["ik_post_grasp_position_error"] = lift_pos
                candidate.extras["ik_post_grasp_orientation_error"] = lift_ori
                return True, (f"IK residual pre={pre_pos * 1000.0:.1f} mm, "
                              f"grasp={grasp_pos * 1000.0:.1f} mm, "
                              f"lift={lift_pos * 1000.0:.1f} mm")

            # For a near-horizontal approach, world +Z can be unreachable even
            # though entering the grasp was valid.  The exact reverse path to
            # the already checked pregrasp is the only fallback accepted here;
            # it preserves the candidate orientation and never flips approach.
            _q_retreat, retreat_ok, retreat_pos, retreat_ori = \
                self._solve_ik_checked(q_grasp, pregrasp_ee, rotation)
            candidate.extras["ik_retreat_position_error"] = retreat_pos
            candidate.extras["ik_retreat_orientation_error"] = retreat_ori
            if not retreat_ok:
                return (False, "post-grasp unreachable: lift residual "
                        f"{lift_pos * 1000.0:.1f} mm/"
                        f"{math.degrees(lift_ori):.1f} deg; reverse retreat "
                        f"residual {retreat_pos * 1000.0:.1f} mm/"
                        f"{math.degrees(retreat_ori):.1f} deg")
            candidate.extras["post_grasp_mode"] = "reverse_to_pregrasp"
            candidate.extras["post_grasp_target_x"] = float(pregrasp_ee[0])
            candidate.extras["post_grasp_target_y"] = float(pregrasp_ee[1])
            candidate.extras["post_grasp_target_z"] = float(pregrasp_ee[2])
            candidate.extras["ik_post_grasp_position_error"] = retreat_pos
            candidate.extras["ik_post_grasp_orientation_error"] = retreat_ori
            return True, (f"IK residual pre={pre_pos * 1000.0:.1f} mm, "
                          f"grasp={grasp_pos * 1000.0:.1f} mm; vertical lift "
                          f"failed ({lift_pos * 1000.0:.1f} mm), reverse "
                          f"retreat={retreat_pos * 1000.0:.1f} mm")
        except (RuntimeError, ValueError, TypeError) as exc:
            return False, f"IK error: {exc}"

    def _gripper_collision_options(
            self, filter_cfg: FilterConfig) -> Dict[str, float]:
        """One geometry definition shared by safety and its diagnostics."""
        return {
            "pregrasp_standoff": float(filter_cfg.pregrasp_standoff),
            "finger_back": float(self._param(
                "gripper_collision_finger_back_m")),
            "finger_front": float(self._param(
                "gripper_collision_finger_front_m")),
            "finger_thickness": float(self._param(
                "gripper_collision_finger_thickness_m")),
            "gripper_height": float(self._param(
                "gripper_collision_height_m")),
            "margin": float(self._param("gripper_collision_margin_m")),
        }

    def select_grasp(self, scene: GraspScene
                     ) -> Tuple[Optional[GraspCandidate], List[GraspCandidate]]:
        """Detect on the scene, then associate/filter/rank a Gemini target."""
        # A new request must never leave a stale target/grasp visible.
        self._publish_target_cloud(None)
        self._publish_markers([], [])
        self._publish_debug_markers_safely([], [])
        self._publish_near_miss_markers_safely(
            None, None, float(self.filter_cfg.pregrasp_standoff))
        target_scene: Optional[GraspScene] = None
        target_pixel: Optional[Tuple[float, float]] = None
        if self.mode == "target":
            candidates, target_scene, target_pixel = \
                self._detect_in_target(scene)
        else:
            self._set_stage("detect")
            candidates = self.backend.detect(scene)
            self._publish_status(
                f"{self.backend.name}: {len(candidates)} raw candidates")
        if not candidates:
            return None, []

        self._set_stage("filter")
        filter_cfg = self.filter_cfg
        if self.mode == "target":
            filter_cfg = copy.deepcopy(self.filter_cfg)
            filter_cfg.table_z = self._target_surface_z()
        if target_scene is not None:
            # Robust component bounds prevent a candidate from being selected
            # merely because its approach volume overlaps the Gemini box.
            target_low, target_high = np.percentile(
                target_scene.points_base, [2, 98], axis=0)
            filter_cfg.target_min = target_low
            filter_cfg.target_max = target_high
            filter_cfg.target_margin = float(
                self._param("target_bounds_margin_m"))

        # Finger swapping is a true symmetry of the parallel gripper. Align
        # every candidate before IK so reachability is tested for exactly the
        # orientation that could later be sent to MoveIt.
        if self.mode == "target" and scene.tool_rotation_base is not None:
            for candidate in candidates:
                align_parallel_jaw_orientation(
                    candidate, scene.tool_rotation_base)
        reachable_chain = (self._reachable_chain
                           if bool(self._param("check_reachability")) else None)
        collision_options = {
            "pregrasp_standoff": float(filter_cfg.pregrasp_standoff),
        }
        collision_min_points = 3
        collision_note = "disabled"
        collision_diagnostic_points = scene.points_base
        scene_collision_check = None
        if bool(self._param("gripper_scene_collision_enabled")):
            collision_note = "legacy full-scene envelope"
            collision_options = self._gripper_collision_options(filter_cfg)
            collision_min_points = int(self._param(
                "gripper_scene_collision_min_points"))
            collision_error = None
            if self.mode == "target":
                try:
                    if target_scene is None:
                        raise ValueError(
                            "exact target component/provenance is unavailable")
                    exact_target_mask = target_region_mask(scene, target_scene)
                except (TypeError, ValueError) as exc:
                    collision_error = str(exc)
                else:
                    plan_only = self._plan_only()
                    try:
                        measured_open = float(self._param(
                            "gripper_width_at_open_pos"))
                    except (TypeError, ValueError):
                        measured_open = math.nan
                    measured_available = (
                        math.isfinite(measured_open) and measured_open > 0.0)
                    if not plan_only:
                        calibration_error = self._gripper_calibration_error()
                        if calibration_error is not None:
                            collision_error = calibration_error
                        elif not bool(self._param(
                                "gripper_calibration_validated")):
                            collision_error = (
                                "gripper aperture calibration has not been "
                                "physically validated")
                        elif not measured_available:
                            collision_error = (
                                "measured open gripper aperture is unavailable")
                        else:
                            open_aperture = measured_open
                            collision_note = (
                                f"target-aware measured-open="
                                f"{open_aperture * 1000.0:.1f}mm")
                    elif measured_available:
                        open_aperture = measured_open
                        validation = ("validated" if bool(self._param(
                            "gripper_calibration_validated")) else
                                      "unvalidated PREVIEW ONLY")
                        collision_note = (
                            f"target-aware open={open_aperture * 1000.0:.1f}mm "
                            f"({validation})")
                    else:
                        # Planning and RViz preview must remain usable before
                        # commissioning, but this assumption is never allowed
                        # into an executing run.
                        open_aperture = float(filter_cfg.max_width)
                        if (not math.isfinite(open_aperture)
                                or open_aperture <= 0.0):
                            collision_error = (
                                "neither measured nor positive preview open "
                                "aperture is available")
                        else:
                            collision_note = (
                                f"target-aware ASSUMED PREVIEW open="
                                f"{open_aperture * 1000.0:.1f}mm; NOT "
                                "EXECUTABLE")
                            self.get_logger().warn(collision_note)
                    if collision_error is None:
                        collision_options.update({
                            "target_mask": exact_target_mask,
                            "open_aperture": open_aperture,
                        })
            if collision_error is not None:
                detail = (
                    "target-aware collision unavailable (fail-closed): "
                    f"{collision_error}")
                collision_note = detail
                collision_diagnostic_points = None
                scene_collision_check = (
                    lambda _candidate, message=detail: (False, message))
            else:
                scene_collision_check = lambda candidate: conservative_gripper_collision(
                    candidate, scene.points_base, **collision_options,
                    min_points=collision_min_points)
        accepted, rejected = filter_and_rank(
            candidates, filter_cfg, reachable_chain=reachable_chain,
            scene_collision_check=scene_collision_check)
        self._last_perception = (
            f"raw={len(candidates)}, valid={len(accepted)}, "
            f"rejected={len(rejected)} "
            f"({rejection_summary(rejected)}); collision={collision_note}")
        self._publish_status(
            f"{len(accepted)} candidates passed, {len(rejected)} rejected "
            f"({rejection_summary(rejected)}); collision={collision_note}")
        if rejected:
            details = "; ".join(
                f"#{index + 1} {reason.reason}: {reason.detail}"
                for index, (_candidate, reason) in enumerate(rejected[:10]))
            self.get_logger().info(f"rejection details: {details}")
        selected = accepted[0] if accepted else None
        if (accepted and self.mode == "target" and target_pixel is not None):
            selected = select_target_candidate(
                accepted, target_scene or scene, target_pixel,
                score_slack=float(self._param("selection_score_slack")),
                tilt_slack_rad=float(
                    self._param("selection_tilt_slack_rad")),
                reference_rotation=scene.tool_rotation_base)
            if selected is not None:
                # Put the orientation-selected grasp first so bounded MoveIt
                # prevalidation evaluates the user's preferred policy first,
                # without discarding safe fallback candidates.
                accepted = [selected] + [candidate for candidate in accepted
                                         if candidate is not selected]
                orientation_delta = selected.extras.get(
                    "orientation_delta_rad")
                orientation_text = ("" if orientation_delta is None else
                                    f", dEE={math.degrees(float(orientation_delta)):.1f}deg")
                post_mode = selected.extras.get("post_grasp_mode")
                post_pos = selected.extras.get(
                    "ik_post_grasp_position_error")
                post_ori = selected.extras.get(
                    "ik_post_grasp_orientation_error")
                post_text = ""
                if post_mode is not None:
                    post_text = f", post={post_mode}"
                    if post_pos is not None and post_ori is not None:
                        post_text += (
                            f" IK={float(post_pos) * 1000.0:.1f}mm/"
                            f"{math.degrees(float(post_ori)):.1f}deg")
                policy = ("orientation-first"
                          if scene.tool_rotation_base is not None
                          else "score-pool/near-vertical fallback")
                self._publish_status(
                    f"target grasp selected ({policy}): "
                    f"score={selected.score:.3f}"
                    f"{orientation_text}{post_text}")
        if self.mode == "target":
            self.get_logger().info(
                "target candidate details: " + candidate_diagnostic_summary(
                    candidates, rejected,
                    reference_rotation=scene.tool_rotation_base))
        near_miss = None
        if selected is None:
            try:
                near_miss = best_near_miss(
                    rejected, filter_cfg,
                    scene_points=collision_diagnostic_points,
                    collision_kwargs=collision_options,
                    collision_min_points=collision_min_points,
                    ik_position_tolerance=float(self._param(
                        "ik_position_tolerance")),
                    ik_orientation_tolerance=float(self._param(
                        "ik_orientation_tolerance")))
            except Exception as exc:  # noqa: BLE001 - diagnostic only
                self.get_logger().warn(
                    "cannot choose best near miss (motion unaffected): "
                    f"{exc}")
            if near_miss is not None:
                rejection = near_miss.rejection
                near_miss_text = (
                    f"best near miss (DEBUG ONLY / NOT SELECTED): "
                    f"{rejection.reason}: {rejection.detail}")
                self._last_perception += "; " + near_miss_text
                self._publish_status(near_miss_text)

        # The normal topic contains only the candidate actually selected after
        # every safety gate.  A near miss is intentionally never passed here.
        self._publish_markers(
            [selected] if selected is not None else [],
            [c for c, _ in rejected])
        self._publish_debug_markers_safely(accepted, rejected)
        self._publish_near_miss_markers_safely(
            near_miss, scene.points_base,
            float(filter_cfg.pregrasp_standoff))
        return selected, accepted

    def _locate_target(self, scene: GraspScene) -> Optional[Localization]:
        """Ask Gemini where the described target is. ``None`` on any failure
        (no description set, Gemini disabled, request failed, or not found) —
        the caller logs specifics; this just signals "stop here"."""
        self._set_stage("gemini_locate")
        if not self._target_description:
            self.get_logger().error(
                "mode 'target' needs a description: set the target_description "
                "parameter or publish on ~/set_target")
            return None
        if not self.gemini.enabled:
            self.get_logger().error(
                f"mode 'target' needs Gemini. {self.gemini.describe()}")
            return None
        try:
            found: Localization = self.gemini.locate(scene.color_image,
                                                     self._target_description)
        except GeminiError as exc:
            self.get_logger().error(f"Gemini locate failed: {exc}")
            return None
        self._last_gemini = (f"locate('{self._target_description}') -> "
                             f"found={found.found} pixel={found.pixel} "
                             f"conf={found.confidence:.2f} {found.reason}")
        self._publish_status(self._last_gemini)
        return found if found.found else None

    def _detect_in_target(
            self, scene: GraspScene,
            ) -> Tuple[List[GraspCandidate], Optional[GraspScene],
                       Optional[Tuple[float, float]]]:
        """Locate/segment the target, then infer with complete scene geometry.

        This follows ROBOTIS target mode: Gemini identifies the requested
        object, the learned backend proposes grasps with surrounding geometry,
        and later safety filters associate proposals with the target component.
        AnyGrasp additionally receives that component as exact region steering.
        """
        located = self._locate_target(scene)
        if located is None:
            return [], None, None
        self._set_stage("detect")
        if located.box is not None:
            target_surface_z = self._target_surface_z()
            segmented = segment_target_component(
                scene, located.box, located.pixel,
                pad_px=float(self._param("target_crop_pad_px")),
                seed_radius_px=float(self._param("target_seed_radius_px")),
                depth_tolerance=float(
                    self._param("target_depth_tolerance_m")),
                voxel_size=float(self._param("target_component_voxel_m")),
                min_points=int(self._param("target_component_min_points")),
                table_z=target_surface_z,
                table_margin=float(self._param("target_table_margin_m")))
            if segmented is None:
                self.get_logger().error(
                    "Gemini's box did not yield one connected target component "
                    f"above target support z={target_surface_z}")
                return [], None, located.pixel
            if getattr(self.backend, "supports_region_steering", False):
                # AnyGrasp must see every obstacle in the original optical
                # cloud.  The exact mask steers proposals to the yellow target
                # component without deleting the object underneath it.
                region_mask = target_region_mask(scene, segmented)
                candidates = self.backend.detect(
                    scene, collision_scene=scene, region_mask=region_mask)
                network_point_count = scene.points_base.shape[0]
            elif self.backend.name == "graspnet":
                network_scene = crop_to_workspace(
                    scene, self.filter_cfg.workspace_min,
                    self.filter_cfg.workspace_max)
                if network_scene.points_base.shape[0] < int(
                        self._param("target_component_min_points")):
                    self.get_logger().error(
                        "too few scene points inside the configured robot "
                        "workspace for GraspNet")
                    return [], segmented, located.pixel
                candidates = self._detect_graspnet_multi_seed(
                    network_scene, collision_scene=scene)
                network_point_count = network_scene.points_base.shape[0]
            elif hasattr(self.backend, "detect_single"):
                candidates = self.backend.detect_single(
                    segmented,
                    foreground_band_m=float(
                        self._param("target_foreground_band_m")),
                    table_margin=float(self._param("target_table_margin_m")))
                network_point_count = segmented.points_base.shape[0]
            else:
                candidates = self.backend.detect(segmented)
                network_point_count = segmented.points_base.shape[0]
            self._publish_status(
                f"{self.backend.name}: {len(candidates)} raw scene candidates; "
                f"target '{self._target_description}' has "
                f"{segmented.points_base.shape[0]} segmented points; "
                f"network scene has {network_point_count} points")
            target_scene = segmented
            self._publish_target_cloud(target_scene)
        else:
            # No box, just a point (e.g. a model that only returns "point").
            # Fall back to detecting the whole scene and matching by pixel.
            candidates = self.backend.detect(scene)
            for candidate in candidates:
                if candidate.pixel is None:
                    candidate.pixel = self._project_into_image(scene, candidate)
            candidates = nearest_to_pixel(
                candidates, located.pixel,
                float(self._param("target_match_radius_px")))
            target_scene = None
        if not candidates:
            self.get_logger().error(
                f"no grasp candidate found for '{self._target_description}'")
        return candidates, target_scene, located.pixel

    def _detect_graspnet_multi_seed(
            self, network_scene: GraspScene, *,
            collision_scene: GraspScene) -> List[GraspCandidate]:
        """Pool several deterministic sparse-cloud samples before filtering.

        Every returned candidate still passes through the unchanged geometry,
        collision, tilt, workspace, and IK gates in :meth:`select_grasp`.
        """
        attempts = max(1, min(
            8, int(self._param("graspnet_sampling_attempts"))))
        first_seed = int(self._param("graspnet_sampling_seed"))
        combined: List[GraspCandidate] = []
        counts: List[int] = []
        seeds: List[int] = []
        for offset in range(attempts):
            seed = first_seed + offset
            batch = self.backend.detect(
                network_scene, collision_scene=collision_scene,
                sampling_seed=seed)
            seeds.append(seed)
            counts.append(len(batch))
            combined.extend(batch)
        self._publish_status(
            f"graspnet multi-seed {seeds}: per-seed={counts}, "
            f"combined={len(combined)} candidates")
        return combined

    # ---------------- motion ----------------
    def _pose(self, position: np.ndarray, rotation: np.ndarray) -> Pose:
        quat = matrix_to_quat(rotation)
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = \
            [float(v) for v in position]
        (pose.orientation.x, pose.orientation.y,
         pose.orientation.z, pose.orientation.w) = [float(v) for v in quat]
        return pose

    def _plan_only(self) -> bool:
        return not bool(self._param("execute_motion"))

    def _gripper_calibration_error(self) -> Optional[str]:
        """Validate measured aperture endpoints and accepted grasp widths."""
        return gripper_calibration_error(
            open_pos=self._param("gripper_open_pos"),
            close_pos=self._param("gripper_close_pos"),
            width_at_open_pos=self._param("gripper_width_at_open_pos"),
            width_at_close_pos=self._param("gripper_width_at_close_pos"),
            min_width=self._param("min_width"),
            max_width=self._param("max_width"))

    def _gripper_width_mapping(self) -> Optional[Tuple[float, float]]:
        """Return calibrated widths, or the documented plan-only fallback."""
        error = self._gripper_calibration_error()
        if error is None:
            if (not self._plan_only()
                    and not bool(self._param(
                        "gripper_calibration_validated"))):
                self.get_logger().error(
                    "physical gripper command refused: "
                    "gripper_calibration_validated is false")
                return None
            return (float(self._param("gripper_width_at_open_pos")),
                    float(self._param("gripper_width_at_close_pos")))
        if not self._plan_only():
            self.get_logger().error(
                f"physical gripper command refused: {error}")
            return None
        # Preserve existing plan-only behaviour without pretending that zero
        # is a measured closed aperture.  Physical execution cannot reach this
        # branch because it requires the calibration gate above.
        legacy_max = float(self._param("max_width"))
        if not math.isfinite(legacy_max) or legacy_max <= 0.0:
            self.get_logger().error(
                "plan-only gripper preview refused: max_width must be "
                "positive and finite")
            return None
        self.get_logger().warn(
            "plan-only gripper preview is using the legacy [0, max_width] "
            f"aperture assumption: {error}")
        return legacy_max, 0.0

    def _target_surface_z(self) -> float:
        """Support surface used only for target segmentation and clearance."""
        name = ("target_support_z"
                if bool(self._param("target_support_enabled")) else "table_z")
        value = float(self._param(name))
        if not math.isfinite(value):
            raise ValueError(f"{name} must be finite")
        return value

    def _apply_collision_surface(self, object_id: str, size: Sequence[float],
                                 center_xy: Sequence[float], top_z: float,
                                 label: str) -> bool:
        """Add one measured axis-aligned box whose top is ``top_z``."""
        size = [float(v) for v in size]
        center_xy = [float(v) for v in center_xy]
        if (len(size) != 3 or len(center_xy) != 2
                or not all(math.isfinite(v) and v > 0.0 for v in size)
                or not all(math.isfinite(v) for v in center_xy)
                or not math.isfinite(float(top_z))):
            self.get_logger().error(
                f"{label} collision box needs 3 positive finite dimensions, "
                "2 finite centre coordinates, and a finite top height")
            return False
        pose = Pose()
        pose.position.x, pose.position.y = center_xy
        pose.position.z = float(top_z) - size[2] * 0.5
        pose.orientation.w = 1.0
        return self.moveit.apply_collision_box(object_id, size, pose)

    def _apply_table_collision(self) -> bool:
        if bool(self._param("table_collision_enabled")):
            if not self._apply_collision_surface(
                    "pick_table", self._param("table_collision_size"),
                    self._param("table_collision_center_xy"),
                    float(self._param("table_z")), "main table"):
                return False
        else:
            self.get_logger().warn(
                "table collision object disabled; table avoidance is not checked")

        if bool(self._param("target_support_enabled")):
            size = [self._param("target_support_collision_size_x"),
                    self._param("target_support_collision_size_y"),
                    self._param("target_support_collision_size_z")]
            center_xy = [self._param("target_support_collision_center_x"),
                         self._param("target_support_collision_center_y")]
            if not self._apply_collision_surface(
                    "target_support", size, center_xy,
                    self._target_surface_z(), "target support"):
                return False
        return True

    def _camera_preflight(self) -> bool:
        """Prove camera ownership/data and capture-time transform before motion."""
        try:
            frame = self.camera.capture(warmup=1)
        except Exception as exc:  # noqa: BLE001 - camera SDK raises many types
            self.get_logger().error(
                f"camera preflight failed: {type(exc).__name__}: {exc}")
            return False
        if frame is None:
            detail = getattr(self.camera, "last_error", "")
            self.get_logger().error(
                "camera preflight returned no frame"
                + (f": {detail}" if detail else ""))
            return False
        joints = self._joints_for_capture(frame.stamp)
        if joints is None:
            self.get_logger().error(
                "camera preflight has no timestamp-matched joint sample")
            return False
        try:
            self._camera_pose(joints, frame.frame_id, frame.stamp)
        except RuntimeError as exc:
            self.get_logger().error(f"camera transform preflight failed: {exc}")
            return False
        self._publish_status(
            f"camera preflight passed ({frame.frame_id}, "
            f"{frame.color.shape[1]}x{frame.color.shape[0]})")
        return True

    def _execution_interlocks_ready(self) -> bool:
        """Require the controller ownership state used for autonomous motion."""
        with self._lock:
            operation_mode = self._operation_mode
            remote_enabled = self._remote_enabled
        mode_topic = str(self._param("operation_mode_state_topic"))
        remote_topic = str(self._param("remote_enabled_state_topic"))
        if (self.count_publishers(mode_topic) < 1
                or self.count_publishers(remote_topic) < 1):
            self.get_logger().error(
                "execution refused: controller state publishers are missing")
            return False
        if operation_mode != "AUTONOMOUS":
            self.get_logger().error(
                "execution refused: operation mode must be AUTONOMOUS "
                f"(reported {operation_mode!r})")
            return False
        if remote_enabled is not False:
            self.get_logger().error(
                "execution refused: remote control must report disabled "
                f"(reported {remote_enabled!r})")
            return False
        if not self._dynamixel_bus_ready():
            return False
        return True

    def _motion_command_allowed(self) -> bool:
        if self._plan_only():
            return True
        if self._stop_requested:
            return False
        if self._execution_interlocks_ready():
            return True
        self._trip_execution_interlock(
            "controller ownership was not safe before a command")
        return False

    def _motion_preflight(self) -> bool:
        """Verify prerequisites before any sequence step can move the arm."""
        executing = bool(self._param("execute_motion"))
        configured_base = str(self._param("base_frame"))
        ik_base = str(self._param("ik_base_link"))
        if configured_base != self.base_frame or ik_base != self.base_frame:
            self.get_logger().error(
                "motion refused: base_frame and ik_base_link must both match "
                f"the initialized MoveIt frame {self.base_frame!r} "
                f"(configured {configured_base!r}, IK {ik_base!r})")
            return False
        if self.mode not in ("classify", "target"):
            self.get_logger().error(
                f"unsupported mode '{self.mode}' (expected classify|target)")
            return False
        if self.moveit.motion_faulted:
            self.get_logger().error(
                "motion preflight refused: an earlier action timed out; verify "
                "the robot state and restart this node")
            return False
        if not self.backend.available():
            detail = getattr(self.backend, "availability_error", "unavailable")
            self.get_logger().error(
                f"grasp backend '{self.backend.name}' unavailable: {detail}")
            return False
        loader = getattr(self.backend, "load", None)
        if callable(loader):
            try:
                loader()
            except Exception as exc:  # noqa: BLE001 - optional ML stack
                self.get_logger().error(
                    f"grasp backend '{self.backend.name}' failed to load: "
                    f"{type(exc).__name__}: {exc}")
                return False
        if self.mode == "target" and not self.gemini.enabled:
            self.get_logger().error(
                f"target mode requires Gemini before motion: "
                f"{self.gemini.describe()}")
            return False
        if self._current_joints() is None:
            self.get_logger().error(
                f"motion refused: no fresh {self._param('joint_state_topic')}")
            return False
        try:
            self._tcp_offset()
            for name in ("observe_pose", "home_pose"):
                values = np.asarray(self._param(name), dtype=float)
                if (values.shape != (len(self.arm_joints),)
                        or not np.all(np.isfinite(values))):
                    raise ValueError(
                        f"{name} must contain {len(self.arm_joints)} finite values")
        except (TypeError, ValueError) as exc:
            self.get_logger().error(f"motion configuration invalid: {exc}")
            return False
        if executing:
            gripper_error = self._gripper_calibration_error()
            if gripper_error is not None:
                self.get_logger().error(
                    "execution refused: gripper calibration is invalid or "
                    f"unmeasured: {gripper_error}")
                return False
            if not bool(self._param("gripper_calibration_validated")):
                self.get_logger().error(
                    "execution refused: set "
                    "gripper_calibration_validated:=true only after measuring "
                    "the clear aperture at gripper_open_pos and "
                    "gripper_close_pos")
                return False
        if executing and not bool(self._param("calibration_validated")):
            self.get_logger().error(
                "execution refused: set calibration_validated:=true only after "
                "camera/TCP/table calibration has been checked on this robot")
            return False
        if (executing and bool(self._param("place_enabled"))
                and not bool(self._param("place_poses_validated"))):
            self.get_logger().error(
                "execution refused: place poses are not validated")
            return False
        if executing and not self._execution_interlocks_ready():
            return False

        timeout = float(self._param("moveit_server_timeout_s"))
        ready = (self.moveit.wait_for_servers(timeout_sec=timeout)
                 if executing
                 else self.moveit.wait_for_move_server(timeout_sec=timeout))
        if not ready:
            return False
        if not self.moveit.verify_planning_pipeline(
                "pilz_industrial_motion_planner", timeout_sec=timeout):
            return False
        self.moveit.reset_plan_only_state()
        if not self._apply_table_collision():
            return False
        if not self._camera_preflight():
            return False
        self._publish_status(
            "motion preflight passed (execution)" if executing
            else "motion preflight passed (plan-only; controllers untouched)")
        return True

    def _move_joints(self, values: Sequence[float], label: str) -> bool:
        plan_only = self._plan_only()
        if not self._motion_command_allowed():
            return False
        self._publish_status(
            f"{'plan-only' if plan_only else 'move'} joints: {label}")
        return self.moveit.move_to_joint_values(
            [float(v) for v in values], plan_only=plan_only)

    def _move_pose(self, position: np.ndarray, rotation: np.ndarray,
                   label: str,
                   position_tolerance: Optional[float] = None) -> bool:
        if not self._motion_command_allowed():
            return False
        self._publish_status(f"{label} -> {np.round(position, 3).tolist()}")
        return self.moveit.move_to_pose(self._pose(position, rotation),
                                        position_tolerance=position_tolerance,
                                        plan_only=self._plan_only())

    def _move_linear_pose(self, position: np.ndarray, rotation: np.ndarray,
                          label: str,
                          position_tolerance: Optional[float] = None) -> bool:
        if not self._motion_command_allowed():
            return False
        self._publish_status(
            f"{label} (LIN) -> {np.round(position, 3).tolist()}")
        return self.moveit.move_linear_to_pose(
            self._pose(position, rotation),
            position_tolerance=position_tolerance,
            orientation_tolerance=float(
                self._param("linear_orientation_tolerance")),
            plan_only=self._plan_only())

    def _gripper(self, position: float, label: str) -> bool:
        if not self._motion_command_allowed():
            return False
        self._publish_status(f"gripper {label} -> {position:+.4f}")
        if self._plan_only():
            updated = self.moveit.set_plan_only_joint_state(
                [str(name) for name in self._param("gripper_joint_names")],
                float(position))
            self._publish_status(
                f"(plan-only) gripper {label} state "
                f"{'updated' if updated else 'could not be updated'}")
            return updated
        ok = self.moveit.set_gripper(
            position, max_effort=float(self._param("gripper_max_effort")))
        time.sleep(float(self._param("gripper_settle_s")))
        return ok

    def _open_gripper(self) -> bool:
        return self._gripper(float(self._param("gripper_open_pos")), "open")

    def _gripper_position_for_candidate(
            self, candidate: GraspCandidate) -> Optional[float]:
        """Compute the calibrated close command without issuing a goal."""
        mapping = self._gripper_width_mapping()
        if mapping is None:
            return None
        width_at_open_pos, width_at_close_pos = mapping
        return gripper_position_for_width(
            candidate.width, open_pos=float(self._param("gripper_open_pos")),
            close_pos=float(self._param("gripper_close_pos")),
            width_at_open_pos=width_at_open_pos,
            width_at_close_pos=width_at_close_pos,
            bias=float(self._param("gripper_close_bias")))

    def _close_on(self, candidate: GraspCandidate) -> bool:
        position = self._gripper_position_for_candidate(candidate)
        if position is None:
            return False
        return self._gripper(position, f"close on {candidate.width * 1000:.0f} mm")

    def _candidate_motion_targets(
            self, candidate: GraspCandidate
            ) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        """Return rotation, pregrasp, grasp and preferred world-Z lift."""
        rotation = tool_rotation(candidate.approach, candidate.closing)
        standoff = float(self._param("pregrasp_standoff"))
        pregrasp = self._ee_target(candidate.pregrasp(standoff), rotation)
        grasp = self._ee_target(candidate.motion_position(), rotation)
        vertical_lift = grasp + np.array(
            [0.0, 0.0, float(self._param("lift_height"))])
        return rotation, pregrasp, grasp, vertical_lift

    @staticmethod
    def _record_post_grasp_target(candidate: GraspCandidate, mode: str,
                                  target: np.ndarray) -> None:
        candidate.extras["post_grasp_mode"] = str(mode)
        for axis, value in zip("xyz", np.asarray(target, dtype=float)):
            candidate.extras[f"post_grasp_target_{axis}"] = float(value)

    def _post_grasp_target(self, candidate: GraspCandidate,
                           pregrasp: np.ndarray,
                           vertical_lift: np.ndarray
                           ) -> Tuple[str, np.ndarray]:
        """Resolve the exact post-grasp path chosen during validation."""
        mode = str(candidate.extras.get(
            "post_grasp_mode", "vertical_lift"))
        if mode == "vertical_lift":
            expected = np.asarray(vertical_lift, dtype=float)
        elif mode == "reverse_to_pregrasp":
            expected = np.asarray(pregrasp, dtype=float)
        else:
            raise ValueError(f"unsupported post-grasp mode {mode!r}")
        keys = [f"post_grasp_target_{axis}" for axis in "xyz"]
        if all(key in candidate.extras for key in keys):
            recorded = np.asarray(
                [candidate.extras[key] for key in keys], dtype=float)
            if (recorded.shape != (3,) or not np.all(np.isfinite(recorded))
                    or np.linalg.norm(recorded - expected) > 1e-6):
                raise ValueError(
                    "recorded post-grasp target disagrees with current "
                    f"{mode} geometry")
        return mode, expected

    def _prevalidate_candidate_chain(self, candidate: GraspCandidate) -> bool:
        """Prove one complete chain through MoveIt without moving hardware."""
        candidate.extras["moveit_chain_validated"] = 0.0
        self.moveit.reset_plan_only_state()
        try:
            close_position = self._gripper_position_for_candidate(candidate)
            if close_position is None:
                return False
            rotation, pregrasp, grasp, vertical_lift = \
                self._candidate_motion_targets(candidate)
            linear_orientation_tolerance = float(
                self._param("linear_orientation_tolerance"))
            if not self._motion_command_allowed():
                return False
            if not self.moveit.move_to_pose(
                    self._pose(pregrasp, rotation), plan_only=True):
                self._publish_status("candidate prevalidation: pregrasp failed")
                return False
            if self._should_stop() or not self._motion_command_allowed():
                return False
            if not self.moveit.move_linear_to_pose(
                    self._pose(grasp, rotation),
                    position_tolerance=float(
                        self._param("grasp_position_tolerance")),
                    orientation_tolerance=linear_orientation_tolerance,
                    plan_only=True):
                self._publish_status("candidate prevalidation: grasp LIN failed")
                return False
            if self._should_stop() or not self._motion_command_allowed():
                return False
            if not self.moveit.set_plan_only_joint_state(
                    [str(name) for name in self._param("gripper_joint_names")],
                    close_position):
                self._publish_status(
                    "candidate prevalidation: simulated gripper close failed")
                return False
            if self._should_stop() or not self._motion_command_allowed():
                return False

            # Prefer the current world-Z lift. A failed plan does not advance
            # MoveItClient's cached endpoint, so a fallback plan still starts
            # at the same grasp pose with the simulated jaws closed.
            if self.moveit.move_linear_to_pose(
                    self._pose(vertical_lift, rotation),
                    orientation_tolerance=linear_orientation_tolerance,
                    plan_only=True):
                self._record_post_grasp_target(
                    candidate, "vertical_lift", vertical_lift)
                candidate.extras["moveit_chain_validated"] = 1.0
                return True
            if self._should_stop() or not self._motion_command_allowed():
                return False
            # Exact reverse of the unchanged approach is a standard withdrawal.
            # Require a real upward component even if max_tilt is later changed.
            if float(pregrasp[2] - grasp[2]) <= 1e-6:
                self._publish_status(
                    "candidate prevalidation: vertical lift failed and reverse "
                    "retreat has no upward component")
                return False
            if not self.moveit.move_linear_to_pose(
                    self._pose(pregrasp, rotation),
                    orientation_tolerance=linear_orientation_tolerance,
                    plan_only=True):
                self._publish_status(
                    "candidate prevalidation: vertical lift and reverse "
                    "retreat LIN both failed")
                return False
            self._record_post_grasp_target(
                candidate, "reverse_to_pregrasp", pregrasp)
            candidate.extras["moveit_chain_validated"] = 1.0
            return True
        finally:
            # A partial plan must never seed the next candidate or real goal.
            self.moveit.reset_plan_only_state()

    def _select_prevalidated_candidate(
            self, candidates: Sequence[GraspCandidate]
            ) -> Optional[GraspCandidate]:
        """Return the first ranked candidate whose complete chain plans."""
        selected: Optional[GraspCandidate] = None
        self.moveit.reset_plan_only_state()
        try:
            limit = int(self._param("max_prevalidation_candidates"))
            if not 1 <= limit <= MAX_PREVALIDATION_CANDIDATES:
                self.get_logger().error(
                    "max_prevalidation_candidates must be in [1, "
                    f"{MAX_PREVALIDATION_CANDIDATES}]")
                return None
            inspected = min(len(candidates), limit)
            self._set_stage("candidate_prevalidation")
            for index, candidate in enumerate(candidates[:limit]):
                if self._should_stop():
                    return None
                self._publish_status(
                    f"prevalidating candidate {index + 1}/{inspected}: "
                    f"score={candidate.score:.3f}")
                if self._prevalidate_candidate_chain(candidate):
                    selected = candidate
                    mode = candidate.extras.get("post_grasp_mode")
                    self._publish_status(
                        f"candidate {index + 1} passed complete MoveIt chain "
                        f"({mode})")
                    break
                if self._should_stop() or self.moveit.motion_faulted:
                    return None
            if selected is None:
                self._publish_status(
                    f"no complete MoveIt chain among {inspected} candidate(s)")
            return selected
        finally:
            # Defensive second reset immediately before any physical replan.
            self.moveit.reset_plan_only_state()

    def execute_grasp(self, candidate: GraspCandidate) -> bool:
        """pregrasp -> grasp -> close -> lift. False on the first failed step."""
        if (not self._plan_only()
                and not bool(candidate.extras.get(
                    "moveit_chain_validated", 0.0))):
            self.get_logger().error(
                "physical grasp refused: candidate has no complete MoveIt "
                "plan-only chain validation")
            return False
        rotation, pregrasp, grasp, vertical_lift = \
            self._candidate_motion_targets(candidate)
        try:
            post_mode, post_target = self._post_grasp_target(
                candidate, pregrasp, vertical_lift)
        except (TypeError, ValueError) as exc:
            self.get_logger().error(f"physical grasp refused: {exc}")
            return False
        self._last_recovery_ok = True
        self._set_stage("pregrasp")
        if not self._move_pose(pregrasp, rotation, "pregrasp"):
            return False
        if self._should_stop():
            self._last_recovery_ok = False
            return False
        self._set_stage("grasp")
        if not self._move_linear_pose(
                grasp, rotation, "grasp",
                position_tolerance=float(
                    self._param("grasp_position_tolerance"))):
            self._last_recovery_ok = self._recover_failed_grasp(
                pregrasp, rotation)
            return False
        if self._should_stop():
            self._last_recovery_ok = False
            return False
        if not self._close_on(candidate):
            self._last_recovery_ok = self._recover_failed_grasp(
                pregrasp, rotation)
            return False
        if self._should_stop():
            self._last_recovery_ok = False
            return False
        stage = ("lift" if post_mode == "vertical_lift"
                 else "postgrasp_retreat")
        self._set_stage(stage)
        if not self._move_linear_pose(post_target, rotation, stage):
            self._last_recovery_ok = self._recover_failed_grasp(
                pregrasp, rotation)
            return False
        return True

    def _recover_failed_grasp(self, pregrasp: np.ndarray,
                              rotation: np.ndarray) -> bool:
        """Release at the object and retreat before another attempt is allowed."""
        if self._stop_requested or self.moveit.motion_faulted:
            self.get_logger().warn(
                "stop/arm fault is active; no automatic gripper or retreat "
                "command will be sent (manual recovery is required)")
            return False
        self._publish_status(
            "grasp failed/stopped — opening and retreating to pregrasp")
        opened = self._open_gripper()
        if self._should_stop() or self.moveit.motion_faulted:
            self.get_logger().warn(
                "recovery interrupted before retreat; manual recovery is required")
            return False
        retreated = self._move_linear_pose(
            pregrasp, rotation, "failed-grasp retreat")
        if not (opened and retreated):
            self.get_logger().error(
                "automatic recovery failed; no further candidate will be tried")
        return bool(opened and retreated)

    def _place_pose_for(self, label: str) -> Optional[List[float]]:
        pose = self.place_poses.get(str(label).lower())
        if pose is None and "unknown" in self.place_poses:
            pose = self.place_poses["unknown"]
            self._publish_status(
                f"no place pose for '{label}', using validated 'unknown' bin")
        if pose is None:
            self.get_logger().error(
                f"no place pose for '{label}' and no 'unknown' bin")
        return pose

    def execute_place(self, label: str) -> bool:
        """place-approach -> place -> release -> retreat -> home.

        The bins are joint poses, so the approach point is FK of the bin pose
        raised by ``place_approach_height``: the object arrives from directly
        above the bin instead of swinging in at bin level.
        """
        if not bool(self._param("place_enabled")):
            self._publish_status(
                "place_enabled is false — pick ends after lift; object remains held")
            return True
        pose = self._place_pose_for(label)
        if pose is None:
            return False
        hover_height = float(self._param("place_approach_height"))
        p_bin, rotation = self.ik.fk_pose(np.array(pose, dtype=float))
        hover = p_bin + np.array([0.0, 0.0, hover_height])
        self._set_stage("place_approach")
        if not self._move_pose(hover, rotation, f"above the '{label}' bin"):
            self.get_logger().error(
                "place approach failed; direct motion to the bin is forbidden")
            return False
        if self._should_stop():
            return False
        self._set_stage("place")
        if not self._move_linear_pose(
                p_bin, rotation, f"lower into the '{label}' bin"):
            return False
        if self._should_stop():
            return False
        if not self._open_gripper():
            return False
        if self._should_stop():
            return False
        if not self._move_linear_pose(
                hover, rotation, "retreat from the bin"):
            return False
        if self._should_stop():
            return False
        self._set_stage("home")
        return self._move_joints(self._param("home_pose"), "home")

    # ---------------- sequences ----------------
    def _run_perceive_only(self) -> None:
        try:
            self._run_perceive_only_impl()
        except Exception as exc:  # noqa: BLE001 - keep worker/node alive
            self._last_perception = (
                f"failed: {type(exc).__name__}: {exc}")
            self.get_logger().error(
                f"perception worker failed: {type(exc).__name__}: {exc}")
            self._set_stage("failed")

    def _run_perceive_only_impl(self) -> None:
        self._last_perception = "running"
        self._set_stage("capture")
        scene = self.capture_scene()
        if scene is None:
            self._set_stage("failed")
            return
        best, accepted = self.select_grasp(scene)
        if best is None:
            if self._last_perception == "running":
                self._last_perception = "no usable grasp in this frame"
            self._publish_status("no usable grasp in this frame")
            self._set_stage("idle")
            return
        best_text = (
            f"best grasp {np.round(best.position, 3).tolist()} "
            f"width={best.width * 1000:.0f} mm score={best.score:.3f} "
            f"({len(accepted)} usable)")
        self._last_perception += "; " + best_text
        self._publish_status(best_text)
        self._set_stage("idle")

    def _run_sequence(self) -> None:
        try:
            self._run_sequence_impl()
        except Exception as exc:  # noqa: BLE001 - fail closed on runtime faults
            self.moveit.cancel_current_goal()
            self.get_logger().error(
                f"pick worker failed: {type(exc).__name__}: {exc}")
            self._set_stage("failed")

    def _run_sequence_impl(self) -> None:
        started = time.monotonic()
        self._set_stage("preflight")
        if not self._motion_preflight():
            self._set_stage("failed")
            return
        if self._should_stop():
            return
        self._set_stage("observe")
        if not self._move_joints(self._param("observe_pose"), "observe pose"):
            if self._stop_requested:
                self._should_stop()
                return
            self._set_stage("failed")
            return
        if self._should_stop():
            return
        if not self._open_gripper():
            self._set_stage("failed")
            return
        if self._should_stop():
            return

        self._set_stage("capture")
        scene = self.capture_scene()
        if scene is None:
            self._set_stage("failed")
            return

        _, accepted = self.select_grasp(scene)
        if not accepted:
            self._publish_status("no grasp survived filtering — nothing to pick")
            self._set_stage("idle")
            return

        picked: Optional[GraspCandidate] = None
        if bool(self._param("execute_motion")):
            # Planning failures are safe to skip here because no
            # candidate-specific physical motion has started. Once one fully
            # validated candidate is chosen, physical execution is one-shot.
            candidate = self._select_prevalidated_candidate(accepted)
            if candidate is not None and not self._should_stop():
                self.moveit.reset_plan_only_state()
                self._publish_status(
                    "physical attempt 1/1 after complete-chain validation: "
                    f"score={candidate.score:.3f}")
                if self.execute_grasp(candidate):
                    picked = candidate
                else:
                    self._publish_status(
                        "physical attempt failed; stale candidates will not "
                        "be retried")
        else:
            # Preserve the existing end-to-end dry-run chain, including its
            # simulated observe/open state and optional plan-only recovery.
            attempts = max(1, int(self._param("max_grasp_attempts")))
            for index, candidate in enumerate(accepted[:attempts]):
                if self._should_stop():
                    return
                self._publish_status(
                    f"attempt {index + 1}/{min(len(accepted), attempts)}: "
                    f"score={candidate.score:.3f} "
                    f"tilt={math.degrees(candidate.extras.get('tilt', 0.0)):.0f} deg")
                # Collect only the chosen global dry-run chain.  MoveIt's
                # normal per-segment display messages are too brief to make a
                # complete pick path obvious in RViz; the combined message is
                # published only after every segment succeeds.
                self.moveit.begin_plan_only_display()
                try:
                    if (self.execute_grasp(candidate)
                            and not self._stop_requested):
                        self.moveit.publish_plan_only_display()
                        picked = candidate
                        break
                finally:
                    # No-op after a successful publish; essential on a failed,
                    # stopped, or exceptional partial chain.
                    self.moveit.discard_plan_only_display()
                if self._stop_requested:
                    return
                if not self._last_recovery_ok:
                    break
                self._publish_status(
                    "plan-only attempt failed after safe retreat; trying next "
                    "candidate")
        if self._stop_requested:
            return
        if picked is None:
            self._publish_status("every candidate failed to execute")
            self._set_stage("failed")
            return

        label = self._classify_pick(scene, picked)
        self._last_pick = {
            "position": np.round(picked.position, 4).tolist(),
            "width_mm": round(picked.width * 1000.0, 1),
            "score": round(picked.score, 4),
            "label": label,
        }
        if self._should_stop():
            return
        if not bool(self._param("place_enabled")):
            if self._plan_only():
                self._publish_status(
                    "plan-only pick path succeeded; place disabled; nothing moved")
                self._set_stage("idle")
            else:
                self._publish_status(
                    f"pick complete in {time.monotonic() - started:.1f} s; "
                    "place is disabled and the object remains held")
                self._set_stage("holding")
            return
        if not self.execute_place(label):
            if self._stop_requested:
                return
            self._set_stage("failed")
            return
        self._publish_status(
            f"done in {time.monotonic() - started:.1f} s — picked '{label}'")
        self._set_stage("idle")

    def _classify_pick(self, scene: GraspScene,
                       candidate: GraspCandidate) -> str:
        """Ask Gemini what was picked; fall back to the last category."""
        categories = self.place_categories
        fallback = categories[-1]
        if self.mode == "target":
            return self._target_description or fallback
        if not self.gemini.enabled or scene.color_image is None:
            self._publish_status(
                f"skipping classification ({self.gemini.describe()})")
            return fallback
        self._set_stage("gemini_classify")
        pixel = candidate.pixel or self._project_into_image(scene, candidate)
        if pixel is None:
            self._publish_status(
                "the grasp point is not in view — skipping classification")
            return fallback
        crop = crop_around(scene.color_image, pixel,
                           int(self._param("gemini_crop_half_px")),
                           bbox=candidate.bbox)
        try:
            result: Classification = self.gemini.classify(crop, categories)
        except GeminiUnavailable as exc:
            self._publish_status(f"Gemini unavailable: {exc}")
            return fallback
        except GeminiError as exc:
            self.get_logger().error(f"Gemini classify failed: {exc}")
            return fallback
        self._last_gemini = (f"classify -> {result.label} "
                             f"({result.confidence:.2f}) {result.reason}")
        self._publish_status(self._last_gemini)
        return result.label

    @staticmethod
    def _project_into_image(scene: GraspScene, candidate: GraspCandidate
                            ) -> Optional[Tuple[float, float]]:
        """Where a base-frame grasp point lands in the colour image.

        The analytic backend already knows each candidate's source pixels; a
        learned backend returns geometry only, so the crop Gemini classifies has
        to be found by projecting back through the capture-time camera pose.
        """
        optical = np.asarray(scene.R_wc, dtype=float).T @ (
            np.asarray(candidate.position, dtype=float)
            - np.asarray(scene.p_wc, dtype=float))
        pixel = project(optical[np.newaxis], scene.intrinsics)[0]
        if not np.all(np.isfinite(pixel)):
            return None
        height, width = scene.color_image.shape[:2]
        if not (0 <= pixel[0] < width and 0 <= pixel[1] < height):
            return None
        return float(pixel[0]), float(pixel[1])

    # ---------------- markers ----------------
    def _publish_target_cloud(self, scene: Optional[GraspScene]) -> None:
        """Latch the Gemini-selected component as an opaque yellow cloud."""
        if scene is None:
            points = np.zeros((0, 3), dtype=float)
            colors = None
            frame = self.base_frame
        else:
            points = scene.points_base
            # point_cloud() carries BGR colours; yellow is BGR [0, 215, 255].
            colors = np.tile(
                np.array([[0, 215, 255]], dtype=np.uint8),
                (points.shape[0], 1))
            frame = scene.base_frame
        self.pub_target_cloud.publish(_cloud_message(
            points, colors, frame_id=frame, stamp=self.get_clock().now()))

    def _publish_markers(self, accepted: Sequence[GraspCandidate],
                         rejected: Sequence[GraspCandidate]) -> None:
        """Show only the selected safe grasp as a solid parallel gripper."""
        array = MarkerArray()
        clear = Marker()
        clear.header.frame_id = self.base_frame
        # Marker's default type is ARROW; give the DELETEALL command a neutral
        # type so marker-array diagnostics/tests cannot count it as a path.
        clear.type = Marker.CUBE
        clear.action = Marker.DELETEALL
        array.markers.append(clear)
        del rejected  # Rejections remain available in logs, not as RViz clutter.
        if accepted:
            candidate = accepted[0]
            marker = Marker()
            marker.header.frame_id = self.base_frame
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "parallel_gripper_selected"
            marker.id = 0
            marker.type = Marker.LINE_LIST
            marker.action = Marker.ADD
            marker.pose.orientation.w = 1.0
            marker.points = []
            for xyz in parallel_gripper_points(candidate):
                point = Point()
                point.x, point.y, point.z = [float(v) for v in xyz]
                marker.points.append(point)
            marker.scale.x = 0.014
            marker.color = ColorRGBA(r=1.0, g=0.05, b=0.05, a=1.0)
            marker.lifetime.sec = 0
            array.markers.append(marker)
        self.pub_markers.publish(array)

    @staticmethod
    def _debug_grasp_color(reason: str) -> ColorRGBA:
        """Opaque reason colours for the non-executable RViz debug layer."""
        colors = {
            "valid_unselected": (0.05, 1.0, 0.10),
            "scene_collision": (1.0, 0.05, 0.85),
            "reachability": (1.0, 0.45, 0.02),
            "tilt": (0.05, 0.75, 1.0),
            "width": (0.55, 0.20, 1.0),
            "workspace": (0.10, 0.35, 1.0),
            "off_target": (0.10, 0.35, 1.0),
            "clearance": (1.0, 0.85, 0.05),
            "score": (0.65, 0.65, 0.65),
        }
        red, green, blue = colors.get(
            str(reason), (0.80, 0.80, 0.80))
        return ColorRGBA(r=red, g=green, b=blue, a=1.0)

    def _publish_debug_markers(
            self, accepted: Sequence[GraspCandidate],
            rejected: Sequence[Tuple[GraspCandidate, Rejection]]) -> None:
        """Publish all non-selected proposals as opaque, read-only glyphs.

        The normal ``~/grasp_markers`` topic remains tutorial-style: at most
        one thick red gripper that passed every safety gate.  This separate
        topic visualises which other proposals were rejected or retained only
        as fallbacks. It is never read back by selection or motion code.
        """
        # Some geometry-only unit tests construct the node without __init__.
        # Real nodes always own this latched publisher.
        if not hasattr(self, "pub_debug_markers"):
            return
        array = MarkerArray()
        clear = Marker()
        clear.header.frame_id = self.base_frame
        clear.type = Marker.CUBE
        clear.action = Marker.DELETEALL
        array.markers.append(clear)
        if not bool(self._param("debug_grasp_markers_enabled")):
            self.pub_debug_markers.publish(array)
            return

        stamp = self.get_clock().now().to_msg()

        def append(candidate: GraspCandidate, reason: str, marker_id: int) -> None:
            marker = Marker()
            marker.header.frame_id = self.base_frame
            marker.header.stamp = stamp
            safe_reason = "".join(
                character if character.isalnum() or character == "_" else "_"
                for character in str(reason)) or "unknown"
            marker.ns = f"debug_{safe_reason}"
            marker.id = marker_id
            marker.type = Marker.LINE_LIST
            marker.action = Marker.ADD
            marker.pose.orientation.w = 1.0
            for xyz in parallel_gripper_points(candidate):
                point = Point()
                point.x, point.y, point.z = [float(value) for value in xyz]
                marker.points.append(point)
            # Clearly visible and fully opaque, but thinner than the selected
            # 14 mm red marker so unsafe debug proposals cannot be mistaken for
            # the executable result.
            marker.scale.x = 0.006
            marker.color = self._debug_grasp_color(reason)
            marker.lifetime.sec = 0
            array.markers.append(marker)

        marker_id = 0
        for candidate in accepted[1:]:
            append(candidate, "valid_unselected", marker_id)
            marker_id += 1
        for candidate, rejection in rejected:
            append(candidate, rejection.reason, marker_id)
            marker_id += 1
        self.pub_debug_markers.publish(array)

    def _publish_debug_markers_safely(
            self, accepted: Sequence[GraspCandidate],
            rejected: Sequence[Tuple[GraspCandidate, Rejection]]) -> None:
        """Keep an optional visualization failure out of control flow."""
        try:
            self._publish_debug_markers(accepted, rejected)
        except Exception as exc:  # noqa: BLE001 - debug output is fail-soft
            self.get_logger().warn(
                f"cannot publish debug grasp markers (motion unaffected): {exc}")

    def _publish_near_miss_markers(
            self, near_miss: Optional[NearMiss],
            scene_points: Optional[np.ndarray],
            pregrasp_standoff: float) -> None:
        """Show exactly one rejected proposal and its measured failure.

        This topic is write-only diagnostic output.  Nothing in selection,
        planning, or execution subscribes to it, and the candidate remains in
        the rejected list regardless of what RViz displays.
        """
        # Geometry-only unit tests may construct the node without __init__.
        if not hasattr(self, "pub_near_miss_markers"):
            return

        array = MarkerArray()
        clear = Marker()
        clear.header.frame_id = self.base_frame
        clear.type = Marker.CUBE
        clear.action = Marker.DELETEALL
        array.markers.append(clear)
        if (near_miss is None
                or not bool(self._param("near_miss_markers_enabled"))):
            self.pub_near_miss_markers.publish(array)
            return

        candidate = near_miss.candidate
        rejection = near_miss.rejection
        stamp = self.get_clock().now().to_msg()
        reason_color = self._debug_grasp_color(rejection.reason)

        gripper = Marker()
        gripper.header.frame_id = self.base_frame
        gripper.header.stamp = stamp
        gripper.ns = "best_near_miss_gripper_not_executable"
        gripper.id = 0
        gripper.type = Marker.LINE_LIST
        gripper.action = Marker.ADD
        gripper.pose.orientation.w = 1.0
        for xyz in parallel_gripper_points(candidate):
            point = Point()
            point.x, point.y, point.z = [float(value) for value in xyz]
            gripper.points.append(point)
        # Bold and fully opaque as requested, but still slightly thinner than
        # the 14 mm red marker reserved for a genuinely selected safe grasp.
        gripper.scale.x = 0.010
        gripper.color = reason_color
        gripper.lifetime.sec = 0
        array.markers.append(gripper)

        path = Marker()
        path.header.frame_id = self.base_frame
        path.header.stamp = stamp
        path.ns = "best_near_miss_pregrasp_path_not_executable"
        path.id = 0
        path.type = Marker.ARROW
        path.action = Marker.ADD
        path.pose.orientation.w = 1.0
        for xyz in (candidate.pregrasp(float(pregrasp_standoff)),
                    candidate.motion_position()):
            point = Point()
            point.x, point.y, point.z = [float(value) for value in xyz]
            path.points.append(point)
        path.scale.x = 0.007       # shaft diameter
        path.scale.y = 0.016       # head diameter
        path.scale.z = 0.022       # head length
        path.color = ColorRGBA(r=1.0, g=0.95, b=0.05, a=1.0)
        path.lifetime.sec = 0
        array.markers.append(path)

        label = Marker()
        label.header.frame_id = self.base_frame
        label.header.stamp = stamp
        label.ns = "best_near_miss_reason_not_executable"
        label.id = 0
        label.type = Marker.TEXT_VIEW_FACING
        label.action = Marker.ADD
        label.pose.orientation.w = 1.0
        label.pose.position.x = float(candidate.position[0])
        label.pose.position.y = float(candidate.position[1])
        label.pose.position.z = float(candidate.position[2]) + 0.065
        label.scale.z = 0.022
        label.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
        if math.isfinite(float(near_miss.violation)):
            violation_text = f"normalized miss={near_miss.violation:.2f}"
        else:
            violation_text = "normalized miss=not measurable"
        label.text = (
            "BEST NEAR MISS - DEBUG ONLY / NOT SELECTED\n"
            f"{rejection.reason}: {rejection.detail}\n"
            f"{violation_text}; score={candidate.score:.3f}; "
            f"width={candidate.width * 1000.0:.1f} mm")
        label.lifetime.sec = 0
        array.markers.append(label)

        # Use exactly the boolean mask computed with the safety envelope.  If
        # upstream data are unexpectedly misaligned, omit only this diagnostic
        # layer rather than inventing/guessing collision points.
        if near_miss.collision_mask is not None and scene_points is not None:
            raw_points = np.asarray(scene_points)
            raw_mask = np.asarray(near_miss.collision_mask)
            aligned = (
                raw_points.ndim == 2 and raw_points.shape[1] == 3
                and raw_mask.ndim == 1 and raw_mask.dtype == np.bool_
                and raw_mask.shape[0] == raw_points.shape[0])
            if aligned:
                points = raw_points.astype(float, copy=False)
                mask = raw_mask
                collision = Marker()
                collision.header.frame_id = self.base_frame
                collision.header.stamp = stamp
                collision.ns = "best_near_miss_exact_collision_points"
                collision.id = 0
                collision.type = Marker.POINTS
                collision.action = Marker.ADD
                collision.pose.orientation.w = 1.0
                for xyz in points[mask]:
                    point = Point()
                    point.x, point.y, point.z = [float(value) for value in xyz]
                    collision.points.append(point)
                collision.scale.x = 0.011
                collision.scale.y = 0.011
                collision.color = ColorRGBA(
                    r=1.0, g=0.0, b=0.0, a=1.0)
                collision.lifetime.sec = 0
                array.markers.append(collision)

        self.pub_near_miss_markers.publish(array)

    def _publish_near_miss_markers_safely(
            self, near_miss: Optional[NearMiss],
            scene_points: Optional[np.ndarray],
            pregrasp_standoff: float) -> None:
        """Keep the optional one-candidate diagnostic out of control flow."""
        try:
            self._publish_near_miss_markers(
                near_miss, scene_points, pregrasp_standoff)
        except Exception as exc:  # noqa: BLE001 - visualization is fail-soft
            self.get_logger().warn(
                "cannot publish best near-miss markers "
                f"(motion unaffected): {exc}")

    def request_shutdown(self) -> None:
        """Latch stop and request action cancellation while ROS still spins."""
        with self._worker_lock:
            self._shutdown_requested = True
            self._stop_requested = True
            self.moveit.cancel_current_goal()

    def shutdown_pending(self) -> bool:
        with self._worker_lock:
            local_work_pending = self._busy_unlocked()
        return bool(self.moveit.action_in_flight or local_work_pending)

    def destroy_node(self) -> None:
        self.request_shutdown()
        try:
            self.camera.stop()
        except Exception:       # noqa: BLE001 - shutdown is best effort
            pass
        worker = self._worker
        if (worker is not None and worker.is_alive()
                and worker is not threading.current_thread()):
            worker.join(timeout=2.0)
            if worker.is_alive():
                self.get_logger().warn(
                    "pick worker did not exit within the shutdown timeout")
        super().destroy_node()


def main(args=None) -> None:
    # Keep the rclpy context alive after SIGINT/SIGTERM.  The default rclpy
    # handlers shut the context down before ``finally`` runs, which prevents
    # action cancellation responses from being serviced.  A bounded spin_once
    # loop also guarantees Python gets regular chances to dispatch the local
    # signal handler, even when no ROS entity is otherwise ready.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    shutdown_signal = threading.Event()

    def _request_signal_shutdown(_signum, _frame) -> None:
        shutdown_signal.set()

    previous_handlers = {
        signum: signal.getsignal(signum)
        for signum in (signal.SIGINT, signal.SIGTERM)
    }
    for signum in previous_handlers:
        signal.signal(signum, _request_signal_shutdown)

    node = None
    executor = None
    try:
        node = GeminiPickNode()
        executor = MultiThreadedExecutor(num_threads=4)
        executor.add_node(node)
        while rclpy.ok() and not shutdown_signal.is_set():
            executor.spin_once(timeout_sec=0.1)
    except KeyboardInterrupt:
        shutdown_signal.set()
    finally:
        if node is not None:
            node.request_shutdown()
            next_spin_error_warning = 0.0

            def _shutdown_spin_once(stage: str) -> None:
                """Keep safety cancellation alive despite a bad callback."""
                nonlocal next_spin_error_warning
                try:
                    executor.spin_once(timeout_sec=0.1)
                except Exception as exc:  # noqa: BLE001 - safety drain continues
                    now = time.monotonic()
                    if now >= next_spin_error_warning:
                        node.get_logger().error(
                            f"callback raised during {stage}; continuing "
                            f"shutdown cancellation: {exc}")
                        next_spin_error_warning = now + 5.0

            # Most callbacks and plan-only work should drain quickly. Physical
            # ownership is handled separately below and is never abandoned on
            # an automatic timeout.
            deadline = time.monotonic() + 5.0
            while (executor is not None and rclpy.ok()
                   and node.shutdown_pending()
                   and time.monotonic() < deadline):
                _shutdown_spin_once("normal drain")
            safety_wait = bool(
                node.moveit.physical_action_in_flight
                or node.moveit.controller_cancel_in_flight)
            if safety_wait and executor is not None:
                node.get_logger().error(
                    "physical action/cancellation is still unconfirmed; "
                    "refusing automatic teardown. Use the hardware emergency "
                    "stop before force-killing this process")
                next_warning = time.monotonic() + 5.0
                while (rclpy.ok()
                       and (node.moveit.physical_action_in_flight
                            or node.moveit.controller_cancel_in_flight)):
                    node.moveit.cancel_controller_goals()
                    _shutdown_spin_once("physical-action safety drain")
                    if time.monotonic() >= next_warning:
                        node.get_logger().error(
                            "still waiting for physical stop confirmation")
                        next_warning = time.monotonic() + 5.0
            if node.shutdown_pending():
                node.get_logger().error(
                    "shutdown timed out with an action/worker still in flight; "
                    "verify the robot is stopped before restarting")
            if executor is not None:
                if not executor.shutdown(timeout_sec=1.0):
                    node.get_logger().error(
                        "executor callbacks did not stop within one second")
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        for signum, handler in previous_handlers.items():
            signal.signal(signum, handler)


if __name__ == "__main__":
    main()
