#!/usr/bin/env python3
"""Desktop semantic-target selector with fail-closed planning status."""

import json
import math
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from std_srvs.srv import Trigger


COCO_CLASSES = [
    "person", "bicycle", "car", "motorcycle", "airplane", "bus", "train", "truck", "boat",
    "traffic light", "fire hydrant", "stop sign", "parking meter", "bench", "bird", "cat", "dog",
    "horse", "sheep", "cow", "elephant", "bear", "zebra", "giraffe", "backpack", "umbrella",
    "handbag", "tie", "suitcase", "frisbee", "skis", "snowboard", "sports ball", "kite",
    "baseball bat", "baseball glove", "skateboard", "surfboard", "tennis racket", "bottle",
    "wine glass", "cup", "fork", "knife", "spoon", "bowl", "banana", "apple", "sandwich",
    "orange", "broccoli", "carrot", "hot dog", "pizza", "donut", "cake", "chair", "couch",
    "potted plant", "bed", "dining table", "toilet", "tv", "laptop", "mouse", "remote",
    "keyboard", "cell phone", "microwave", "oven", "toaster", "sink", "refrigerator", "book",
    "clock", "vase", "scissors", "teddy bear", "hair drier", "toothbrush",
]


def friendly_blocker(value):
    if value.startswith("grasp_execution_validation_rejected:"):
        return "Validasi sebelum gerak menolak: " + friendly_blocker(value.split(":", 1)[1])
    for prefix in ("grasp_approach_unavailable:", "grasp_approach_search_budget:"):
        if value.startswith(prefix):
            return "Pendekatan untuk menjepit belum ditemukan: " + friendly_blocker(value[len(prefix):])
    if value.startswith("pregrasp_bridge_"):
        code, _, contact = value[len("pregrasp_bridge_"):].partition(":")
        explanation = {
            "too_long": "jarak koreksi melebihi batas sambungan pendek",
            "start_out_of_bounds": "pose awal berada di luar limit joint",
            "approach_not_horizontal": "arah depan gripper belum horizontal",
            "ik_singular": "pose mendekati konfigurasi singular",
            "ik_no_progress": "IK belum menemukan langkah yang mendekat ke pose pra-jepit",
            "ik_iteration_limit": "batas iterasi IK tercapai",
            "ik_numerical_failure": "perhitungan IK tidak valid",
            "invalid_kinematics": "data kinematika tidak valid",
            "invalid_parameters": "parameter sambungan tidak valid",
            "timeout": "waktu pencarian sambungan habis",
            "waypoint_budget_exceeded": "jumlah titik sambungan melampaui batas",
            "collision_budget_exceeded": "jumlah pemeriksaan collision melampaui batas",
            "joint_jump": "perubahan joint antar titik terlalu besar",
            "start_collision": "pose awal sambungan bertabrakan dengan model",
            "collision": "jalur sambungan bertabrakan dengan model",
            "capsule_clearance": "jalur sambungan terhalang batas jarak aman kapsul",
            "terminal_alignment_or_standoff": "arah atau jarak akhir pra-jepit belum sesuai",
            "terminal_error": "posisi atau orientasi akhir belum mencapai toleransi",
        }.get(code, code.replace("_", " "))
        if contact == "pregrasp_bridge_capsule_clearance":
            explanation = "jalur sambungan terhalang batas jarak aman kapsul"
        elif contact:
            explanation += ": " + contact.replace("__", " dengan ")
        return "Sambungan graph ke pose pra-jepit belum berhasil: " + explanation
    exact = {
        "final_grasp_approach_not_horizontal": "arah depan gripper belum horizontal",
        "final_grasp_approach_misaligned": "arah jari belum sejajar dengan pendekatan ke benda",
        "final_grasp_ik_singular": "pose gripper mendekati konfigurasi singular",
        "final_grasp_ik_no_progress": "IK belum menemukan gerak ke pusat jepitan",
        "final_grasp_timeout": "waktu pencarian pendekatan habis",
        "final_grasp_joint_jump": "pendekatan membutuhkan perubahan joint terlalu besar",
        "grasp_closing_sweep_in_collision": "gerak menutup jari terhalang obstacle atau bagian robot",
        "grasp_opening_sweep_in_collision": "gerak membuka jari terhalang pada posisi saat ini",
        "grasp_gripper_state_missing_or_stale": "posisi aktual gripper belum tersedia atau kedaluwarsa",
        "pregrasp_pose_unavailable_for_target":
            "Data pose hijau belum memiliki pose pra-jepit yang sesuai untuk target ini",
        "current_joint_state_invalid_or_self_colliding":
            "Joint state robot tidak valid atau model mendeteksi self-collision",
        "current_joint_state_out_of_bounds":
            "Joint state berada di luar limit URDF",
        "joint_state_shape_invalid": "Joint state tidak memuat enam joint arm dengan benar",
        "controller_state_missing_or_stale": "Status dua controller belum tersedia atau kedaluwarsa",
        "controller_snapshot_ambiguous": "Identitas dua controller ambigu",
        "current_robot_mesh_intersects_environment_or_target":
            "Posisi robot saat ini bertabrakan dengan model environment/target; periksa RViz dan posisi objek",
        "target_intersection_blocked_or_disconnected":
            "Belum ada jalur edge bebas collision ke irisan target",
        "target_intersection_exact_blocked_or_disconnected":
            "Pose di calon jalur menabrak model environment/target; bukan berarti pose robot sekarang bertabrakan",
        "target_intersection_clearance_blocked_or_disconnected":
            "Jalur tertutup batas jarak aman kapsul; FCL tidak menemukan tabrakan mesh pada kandidat yang ditolak",
        "target_intersection_mixed_blocked_or_disconnected":
            "Calon jalur ditolak oleh batas jarak aman kapsul dan pemeriksaan tabrakan mesh",
        "target_intersection_capsule_filtered_or_disconnected":
            "Graph terputus setelah filter kapsul; tabrakan mesh pada bagian yang tersaring belum diperiksa",
        "exact_collision_replan_exhausted":
            "Batas pencarian tercapai setelah calon jalur bertabrakan pada pemeriksaan mesh",
        "clearance_replan_exhausted":
            "Batas pencarian tercapai karena batas jarak aman kapsul; bukan tabrakan mesh yang terkonfirmasi",
        "mixed_collision_replan_exhausted":
            "Batas pencarian tercapai: ada penolakan batas jarak aman kapsul dan tabrakan mesh",
        "graph_replan_exhausted": "Batas pencarian jalur graph tercapai",
        "current_full_body_intersects_environment":
            "Kapsul jarak aman pada posisi robot sekarang menyentuh model environment; belum membuktikan tabrakan mesh",
        "target_outside_reachability_intersection":
            "Target belum beririsan dengan area kerja graph robot",
        "current_state_cannot_connect_to_roadmap":
            "Posisi EoE saat ini belum terhubung ke graph dengan jalur bebas collision",
        "planner_target_collision_check_required":
            "Planner lama belum memeriksa collision objek target; jalankan ulang launch planning",
        "grasp_selected_target_collision_policy_mismatch":
            "Aturan target planner dan coordinator berbeda; jalankan ulang launch grasp",
        "execution_disabled_at_launch": "Eksekusi tidak diaktifkan saat launch",
        "frozen_plan_stale": "Preview kedaluwarsa; buat Preview baru",
        "motion_faulted": "Coordinator motion sedang fault",
        "dynamixel_failure_counter_advanced_since_preview":
            "Ada kegagalan komunikasi Dynamixel setelah Preview",
        "dynamixel_health_missing_or_stale": "Status Dynamixel hilang atau stale",
        "frozen_object_track_not_current":
            "Botol pada Preview belum ditemukan kembali secara konsisten di pengamatan terbaru",
        "pickup_target_reacquiring":
            "Menunggu pengamatan konsisten untuk mengenali kembali botol setelah nomor track berubah",
        "pickup_target_identity_ambiguous":
            "Ada lebih dari satu objek yang cocok di dekat target; identitas botol belum pasti",
        "semantic_object_moved_during_pickup":
            "Posisi botol berubah melampaui toleransi sejak Preview",
        "object_tracks_missing_or_stale": "Pengamatan objek hilang atau kedaluwarsa",
        "perception_missing_stale_or_rejected": "Data persepsi belum tersedia atau tidak valid",
        "grasp_execution_target_moved_missing_or_ambiguous":
            "Pusat cluster 3D target bergeser, hilang, atau ambigu pada validasi terbaru",
    }
    if value in exact:
        return exact[value]
    for prefix, description in {
        "required_controller_not_active:": "Controller belum aktif",
        "controller_type_mismatch:": "Jenis controller tidak sesuai",
        "controller_command_interfaces_mismatch:": "Interface controller tidak sesuai jalur position/offset",
        "controller_command_interfaces_conflict:": "Interface controller berkonflik",
    }.items():
        if value.startswith(prefix):
            return f"{description}: {value.split(':', 1)[1]}"
    if value.startswith("camera_calibration_not_verified:"):
        reasons = {
            "artifact_not_configured": "file kalibrasi hand-eye belum dipasang",
            "camera_serial_mismatch": "serial file kalibrasi tidak cocok dengan kamera",
            "camera_model_mismatch": "model file kalibrasi tidak cocok dengan kamera",
            "validation_quality_failed": "residual kalibrasi melewati batas",
            "parent_frame_mismatch": "parent frame kalibrasi tidak cocok",
            "transform_invalid": "transform kalibrasi tidak valid",
            "quaternion_not_normalized": "rotasi kalibrasi tidak valid",
        }
        reason = value.split(":", 1)[1]
        return "Kalibrasi kamera belum diverifikasi: " + reasons.get(reason, reason)
    if value.startswith("operation_mode_not_autonomous:"):
        return f"Mode robot masih {value.split(':', 1)[1]}"
    if value.startswith("remote_control_not_disabled:"):
        return "Remote control masih aktif"
    if value.startswith("dynamixel_bus_not_healthy:"):
        reason = value.split(":", 1)[1]
        if reason == "torque is not enabled on every actuator":
            return "Torque actuator belum semuanya ON"
        return f"Bus Dynamixel tidak sehat: {reason}"
    if value.startswith("current_self_collision:"):
        pair = value.split(":", 1)[1].replace("__", " dengan ")
        return f"Model mesh mendeteksi self-collision: {pair}"
    for prefix in ("final_grasp_collision:", "final_grasp_start_collision:"):
        if value.startswith(prefix):
            return "jalur pendekatan bertabrakan: " + value.split(":", 1)[1].replace("__", " dengan ")
    return value.replace("_", " ")


def summarize_graph_status(payload):
    """Return headline, details and whether Execute may be enabled."""
    if not isinstance(payload, dict) or not payload:
        return "Menunggu status planning", "", False
    message = str(payload.get("message", ""))
    stages = {
        "opening_gripper": "membuka gripper",
        "following_graph_path": "menuju pose pra-jepit",
        "following_final_approach": "melanjutkan dari pra-jepit ke pusat benda",
        "closing_gripper": "menutup gripper",
    }
    if payload.get("state") == "motion_failed":
        stage = stages.get(payload.get("failure_stage"), "eksekusi pickup")
        return ("Gerakan berhenti saat " + stage,
                friendly_blocker(message) + ". Buat Preview baru setelah penyebab teratasi.", False)
    if payload.get("busy", False):
        stage = stages.get(payload.get("state"), "memulai eksekusi")
        return "Sedang " + stage, friendly_blocker(message), False
    if not payload.get("plan_ready", False):
        reason = message.split("reachability_plan_invalid:", 1)[-1].strip()
        explanation = friendly_blocker(reason)
        if reason == "pregrasp_pose_unavailable_for_target":
            return explanation, (
                "Belum ada sampel eksperimen yang cocok dengan posisi dan arah pendekatan; "
                "ini belum membuktikan target di luar jangkauan robot"), False
        if explanation != reason.replace("_", " "):
            return explanation, "Periksa model environment dan graph robot di RViz", False
        return message or str(payload.get("state", "Menunggu Preview")), "", False
    target = str(payload.get("target_class", "object"))
    track_id = payload.get("target_track_id", "?")
    points = payload.get("trajectory_points", 0)
    age = float(payload.get("plan_age_sec", 0.0))
    headline = f"Preview: {target} track #{track_id} | {points} titik | umur {age:.1f}s"
    blockers = [friendly_blocker(str(item))
                for item in payload.get("execution_blockers", [])]
    fault_reason = str(payload.get("motion_fault_reason", "")).strip()
    if "motion_faulted" in payload.get("execution_blockers", []) and fault_reason:
        blockers = [
            f"Coordinator motion fault: {fault_reason}"
            if item == "Coordinator motion sedang fault" else item
            for item in blockers
        ]
    if blockers:
        return headline, "Belum bisa Execute: " + "; ".join(blockers), False
    executable = bool(payload.get("executable", False)) and not bool(payload.get("busy", False))
    detail = "Siap dieksekusi" if executable else message
    if payload.get("task_mode") == "move_to_target":
        detail += " | Menuju target melalui edge graph; gripper tidak dijepit"
    elif payload.get("task_mode") == "pickup" and executable:
        detail += " | Buka jari → jalur graph → dekati pusat benda → jepit"
        if payload.get("selected_target_excluded_from_collision") is True:
            detail += " | Cluster target tidak dihitung sebagai obstacle"
    return headline, detail, executable


def latest_collision_detail(payload, diagnostic, receipt_age_sec, service_notice=""):
    """Describe matching recent planner evidence; never participate in Execute gating."""
    if (not isinstance(payload, dict) or payload.get("plan_ready", False)
            or not isinstance(diagnostic, dict) or diagnostic.get("plan_valid") is not False):
        return ""
    if (not isinstance(receipt_age_sec, (int, float))
            or not math.isfinite(receipt_age_sec) or not 0.0 <= receipt_age_sec < 3.0):
        return ""
    reason = diagnostic.get("reason")
    if not isinstance(reason, str) or not reason:
        return ""
    # last_rejection describes graph-goal/edge candidates inspected earlier in
    # this cycle. It is not evidence for a later IK, bridge, gripper, or current
    # state failure, even when the diagnostic's top-level reason matches the UI.
    if reason not in {
            "target_intersection_blocked_or_disconnected",
            "target_intersection_exact_blocked_or_disconnected",
            "target_intersection_clearance_blocked_or_disconnected",
            "target_intersection_mixed_blocked_or_disconnected",
            "target_intersection_capsule_filtered_or_disconnected",
            "exact_collision_replan_exhausted", "clearance_replan_exhausted",
            "mixed_collision_replan_exhausted", "graph_replan_exhausted"}:
        return ""

    def matches(message):
        if not isinstance(message, str):
            return False
        return message.split("reachability_plan_invalid:", 1)[-1].strip() == reason

    if not any(matches(text) for text in (payload.get("message", ""), service_notice)):
        return ""
    rejection = diagnostic.get("last_rejection")
    if not isinstance(rejection, dict) or rejection.get("stage") not in {"graph_edge", "goal_state"}:
        return ""
    names = {
        "gripper_left_link": "jari kiri",
        "gripper_right_link": "jari kanan",
        "link7": "pangkal gripper (link7)",
        "dd_gng_environment": "model environment/target",
        "dd_gng_grasp_obstacles": "model obstacle",
        "dd_gng_grasp_target": "model benda target",
    }
    target_excluded = diagnostic.get("selected_target_excluded_from_collision") is True
    if target_excluded:
        names["dd_gng_environment"] = "obstacle di luar cluster target"
    policy_note = "\nCluster target tidak dihitung sebagai obstacle." if target_excluded else ""
    stage = "calon jalur" if rejection["stage"] == "graph_edge" else "pose tujuan"
    if rejection.get("mesh_checked") is True and rejection.get("mesh_blocked") is True:
        mesh = rejection.get("mesh")
        if not isinstance(mesh, dict):
            return ""
        pair = [mesh.get(key) for key in ("body_a", "body_b")]
        if not all(isinstance(name, str) and name for name in pair):
            return ""
        text = ("Diagnostik planner terbaru: " + " ↔ ".join(names.get(name, name) for name in pair)
                + f" ({stage}).")
        contact = mesh.get("contact_world_m")
        if (isinstance(contact, (list, tuple)) and len(contact) == 3
                and all(isinstance(v, (int, float)) and math.isfinite(v) for v in contact)):
            text += "\nTitik kontak model (global): " + ", ".join(
                f"{axis} {value * 100.0:.1f}" for axis, value in zip("XYZ", contact)) + " cm."
        return text + policy_note
    if rejection.get("capsule_blocked") is True:
        capsule = rejection.get("capsule")
        if not isinstance(capsule, dict):
            return ""
        pair = capsule.get("body_links")
        if not isinstance(pair, list) or len(pair) != 2 or not all(isinstance(v, str) and v for v in pair):
            return ""
        text = "Diagnostik planner terbaru: batas jarak aman di " + "–".join(
            names.get(name, name) for name in pair) + f" terhalang ({stage})."
        if rejection.get("mesh_checked") is True and rejection.get("mesh_blocked") is False:
            text += " Mesh pada kandidat ini lolos pemeriksaan."
        return text + policy_note
    return ""


class SemanticTargetGui(Node):
    def __init__(self):
        super().__init__("semantic_target_gui")
        self.declare_parameter("preview_only", False)
        self.preview_only = bool(self.get_parameter("preview_only").value)
        self.declare_parameter("collision_diagnostics_topic",
                               "/om6dof_topo_gng_v2/reachability_plan/collision_diagnostics")
        self.publisher = self.create_publisher(
            String, "/om6dof_topo_gng_v2/set_target_classes", 1)
        self.create_subscription(
            String, "/om6dof_topo_gng_v2/object_clusters", self._clusters, 2)
        self.create_subscription(
            String, "/om6dof_topo_gng_v2/graph_pick/status", self._graph_pick_status, 2)
        self.create_subscription(
            String, str(self.get_parameter("collision_diagnostics_topic").value),
            self._collision_diagnostics,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                       reliability=ReliabilityPolicy.RELIABLE))
        self.plan_client = self.create_client(Trigger, "/plan_graph_pick")
        self.execute_client = (None if self.preview_only else
                               self.create_client(Trigger, "/execute_graph_pick"))
        self.visible_classes = set()
        self.graph_status = {}
        self.collision_diagnostic = {}
        self.collision_diagnostic_received = 0.0
        self.service_notice = ""
        self.previewed_target = None
        self.selected_target = "all"
        self._lock = threading.Lock()

    def _clusters(self, message):
        try:
            clusters = json.loads(message.data)
            classes = {entry["class"] for entry in clusters
                       if entry.get("class") in COCO_CLASSES}
        except (json.JSONDecodeError, TypeError, KeyError):
            return
        with self._lock:
            self.visible_classes = classes

    def _graph_pick_status(self, message):
        try:
            payload = json.loads(message.data)
            if not isinstance(payload, dict):
                return
        except (json.JSONDecodeError, TypeError):
            return
        with self._lock:
            self.graph_status = payload

    def _collision_diagnostics(self, message):
        try:
            payload = json.loads(message.data)
            if not isinstance(payload, dict):
                return
        except (json.JSONDecodeError, TypeError):
            return
        with self._lock:
            self.collision_diagnostic = payload
            self.collision_diagnostic_received = time.monotonic()

    def choose(self, value):
        target = "all" if value == "All detected classes" else value
        message = String()
        message.data = target
        self.publisher.publish(message)
        with self._lock:
            self.selected_target = target
            self.previewed_target = None
            self.collision_diagnostic = {}
            self.service_notice = (
                "Target diubah ke semua kelas; tekan Preview" if target == "all"
                else f"Target diubah ke {target}; tekan Preview")

    def preview_path(self):
        if not self.plan_client.wait_for_service(timeout_sec=0.2):
            with self._lock:
                self.service_notice = "Service planning belum siap"
            return
        with self._lock:
            self.service_notice = "Menghitung preview EoE ke object..."
            self.collision_diagnostic = {}
        future = self.plan_client.call_async(Trigger.Request())

        def completed(done):
            try:
                response = done.result()
                text = response.message
                payload = json.loads(text) if response.success else None
            except (Exception, json.JSONDecodeError) as error:
                response = None
                payload = None
                text = f"Planning request gagal: {error}"
            self.get_logger().info(text)
            with self._lock:
                if response is not None and response.success and isinstance(payload, dict):
                    self.graph_status = payload
                    self.previewed_target = self.selected_target
                    self.service_notice = "Preview baru diterima"
                else:
                    self.previewed_target = None
                    self.service_notice = text

        future.add_done_callback(completed)

    def execute_path(self):
        if self.preview_only:
            with self._lock:
                self.service_notice = "Jendela ini hanya untuk preview; tidak menggerakkan robot."
            return
        if not self.execute_client.wait_for_service(timeout_sec=0.2):
            with self._lock:
                self.service_notice = "Service execution belum siap"
            return
        with self._lock:
            self.service_notice = "Memeriksa ulang semua interlock..."
        future = self.execute_client.call_async(Trigger.Request())

        def completed(done):
            try:
                response = done.result()
                text = response.message
            except Exception as error:
                text = f"Execution request gagal: {error}"
            self.get_logger().info(text)
            with self._lock:
                self.service_notice = text

        future.add_done_callback(completed)

    def run(self):
        import tkinter as tk
        from tkinter import ttk

        root = tk.Tk()
        root.title("DD-GNG Combined Preview" if self.preview_only else "DD-GNG Planning Target")
        root.geometry("640x520")
        root.minsize(570, 500)
        root.resizable(True, True)
        ttk.Label(
            root, text="Object target for EoE planning",
            font=("Sans", 13, "bold")).pack(pady=(18, 6))
        ttk.Label(
            root, text=("Pilih objek, Set planning target, lalu Preview. Robot tidak bergerak."
                        if self.preview_only else
                        "Preview dibekukan dan divalidasi ulang sebelum robot bergerak.")).pack()
        selected = tk.StringVar(value="All detected classes")
        menu = ttk.Combobox(root, textvariable=selected, state="readonly", width=40)
        menu["values"] = ["All detected classes"] + COCO_CLASSES
        menu.pack(pady=12)
        headline = tk.StringVar(value="Menunggu valid DD-GNG object track")
        detail = tk.StringVar(value="")
        notice = tk.StringVar(value="")
        collision_detail = tk.StringVar(value="")
        ttk.Label(
            root, textvariable=headline, font=("Sans", 10, "bold"),
            wraplength=530, justify="center").pack(pady=(4, 2))
        ttk.Label(
            root, textvariable=detail, foreground="#9b3d00",
            wraplength=530, justify="center").pack(pady=2)
        ttk.Label(
            root, textvariable=notice, wraplength=530,
            justify="center").pack(pady=(2, 8))
        ttk.Label(
            root, textvariable=collision_detail, wraplength=580,
            foreground="#9b3d00", justify="center").pack(pady=(0, 6))
        ttk.Button(
            root, text="Set planning target",
            command=lambda: self.choose(selected.get())).pack(pady=4)
        ttk.Button(
            root, text="Preview EoE path in RViz",
            command=self.preview_path).pack(pady=4)
        execute_button = ttk.Button(
            root, text="Execute planned motion", command=self.execute_path)
        if not self.preview_only:
            execute_button.pack(pady=4)
        execute_button.state(["disabled"])

        def refresh_visible():
            with self._lock:
                visible = sorted(self.visible_classes)
                payload = dict(self.graph_status)
                service_notice = self.service_notice
                selected_target = self.selected_target
                previewed_target = self.previewed_target
                diagnostic = dict(self.collision_diagnostic)
                diagnostic_age = time.monotonic() - self.collision_diagnostic_received
            if visible:
                menu["values"] = ["All detected classes"] + visible + [
                    item for item in COCO_CLASSES if item not in visible]
            status_headline, status_detail, executable = summarize_graph_status(payload)
            execute_button.configure(text=(
                "Gerakkan EoE ke target" if payload.get("task_mode") == "move_to_target"
                else "Dekati dan jepit benda"))
            if self.preview_only:
                if payload.get("plan_ready", False):
                    status_detail = "Preview gabungan: hijau = acuan; magenta = koreksi joystick."
                else:
                    status_detail = "Tekan Preview untuk mengirim trajectory baru ke tampilan gabungan."
            target_matches = previewed_target == selected_target
            if selected_target != "all" and payload.get("target_class") != selected_target:
                target_matches = False
            headline.set(status_headline)
            detail.set(status_detail)
            notice.set(service_notice)
            collision_detail.set(latest_collision_detail(
                payload, diagnostic, diagnostic_age, service_notice))
            if executable and target_matches:
                execute_button.state(["!disabled"])
            else:
                execute_button.state(["disabled"])
            root.after(250, refresh_visible)

        root.protocol("WM_DELETE_WINDOW", lambda: (root.destroy(), rclpy.shutdown()))
        refresh_visible()
        root.mainloop()


def main():
    rclpy.init()
    node = SemanticTargetGui()
    spinner = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spinner.start()
    try:
        node.run()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        spinner.join(timeout=1)


if __name__ == "__main__":
    main()
