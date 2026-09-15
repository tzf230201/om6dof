"""Exercise actual GUI reason formatting without starting ROS or Tk."""

import ast
import math
from pathlib import Path

import pytest


SOURCE = Path(__file__).resolve().parents[1] / "scripts/semantic_target_gui.py"
tree = ast.parse(SOURCE.read_text())
functions = ast.Module(body=[node for node in tree.body if isinstance(node, ast.FunctionDef)
                            and node.name in {"friendly_blocker", "summarize_graph_status", "latest_collision_detail"}],
                       type_ignores=[])
namespace = {"math": math}
exec(compile(functions, str(SOURCE), "exec"), namespace)
friendly = namespace["friendly_blocker"]
summarize = namespace["summarize_graph_status"]
collision_detail = namespace["latest_collision_detail"]


@pytest.mark.parametrize("reason", [
    "target_intersection_clearance_blocked_or_disconnected",
    "clearance_replan_exhausted",
    "target_intersection_capsule_filtered_or_disconnected",
    "target_intersection_exact_blocked_or_disconnected",
    "exact_collision_replan_exhausted",
    "target_intersection_mixed_blocked_or_disconnected",
    "mixed_collision_replan_exhausted",
    "graph_replan_exhausted",
])
def test_rejection_explained_and_never_enables_execution(reason):
    headline, detail, enabled = summarize({
        "plan_ready": False, "executable": True,
        "message": "graph trajectory rejected: reachability_plan_invalid:" + reason,
    })
    assert headline == friendly(reason)
    assert headline != reason.replace("_", " ")
    assert not enabled
    assert "RViz" in detail


def test_capsule_rejection_does_not_claim_mesh_collision():
    text = friendly("target_intersection_clearance_blocked_or_disconnected")
    assert "jarak aman kapsul" in text
    assert "FCL tidak menemukan" in text
    assert "belum diperiksa" in friendly("target_intersection_capsule_filtered_or_disconnected")


def test_missing_archived_pregrasp_does_not_claim_robot_cannot_reach_target():
    headline, detail, executable = summarize({
        "plan_ready": False, "executable": True,
        "message": "graph trajectory rejected: reachability_plan_invalid:"
                   "pregrasp_pose_unavailable_for_target",
    })
    assert "Data pose hijau" in headline
    assert "pose pra-jepit" in headline
    assert "posisi dan arah pendekatan" in detail
    assert "belum membuktikan target di luar jangkauan" in detail
    assert not executable


@pytest.mark.parametrize("code,expected", [
    ("too_long", "jarak koreksi"),
    ("approach_not_horizontal", "belum horizontal"),
    ("ik_no_progress", "IK belum menemukan langkah"),
    ("timeout", "waktu pencarian"),
    ("joint_jump", "perubahan joint"),
    ("collision", "jalur sambungan bertabrakan"),
    ("terminal_error", "posisi atau orientasi akhir"),
])
def test_bridge_failure_is_local_and_never_enables_execution(code, expected):
    reason = "grasp_approach_unavailable:pregrasp_bridge_" + code
    headline, _, executable = summarize({
        "plan_ready": False, "executable": True,
        "message": "reachability_plan_invalid:" + reason,
    })
    assert "Sambungan graph ke pose pra-jepit" in headline
    assert expected in headline
    assert not executable


def test_bridge_collision_preserves_available_contact_pair():
    text = friendly("pregrasp_bridge_collision:link6__dd_gng_obstacles")
    assert "jalur sambungan bertabrakan" in text
    assert "link6 dengan dd_gng_obstacles" in text


def test_exact_path_rejection_is_not_current_pose_collision():
    text = friendly("target_intersection_exact_blocked_or_disconnected")
    assert "calon jalur" in text
    assert "bukan berarti pose robot sekarang" in text


def test_excluded_target_is_not_named_as_the_reported_obstacle(failed_plan_and_contact):
    payload, diagnostic = failed_plan_and_contact
    diagnostic["selected_target_excluded_from_collision"] = True
    text = namespace["latest_collision_detail"](payload, diagnostic, 0.1)
    assert "obstacle di luar cluster target" in text
    assert "Cluster target tidak dihitung sebagai obstacle" in text
    assert "model environment/target" not in text


def test_preview_displays_active_target_exclusion_policy():
    _, text, executable = summarize({
        "plan_ready": True, "executable": True, "task_mode": "pickup",
        "selected_target_excluded_from_collision": True,
    })
    assert executable
    assert "Cluster target tidak dihitung sebagai obstacle" in text


def test_new_diagnostics_do_not_enable_a_blocked_ready_preview():
    headline, detail, enabled = summarize({
        "plan_ready": True, "executable": True,
        "execution_blockers": ["exact_collision_replan_exhausted"],
    })
    assert headline.startswith("Preview:")
    assert "pemeriksaan mesh" in detail
    assert not enabled


@pytest.fixture
def failed_plan_and_contact():
    reason = "target_intersection_exact_blocked_or_disconnected"
    return ({"plan_ready": False, "message": "reachability_plan_invalid:" + reason},
            {"reason": reason, "plan_valid": False, "last_rejection": {
                "stage": "graph_edge", "mesh_checked": True, "mesh_blocked": True,
                "capsule_blocked": False, "mesh": {
                    "body_a": "gripper_right_link", "body_b": "dd_gng_environment",
                    "contact_world_m": [0.29, -0.012, 0.212],
                    "penetration_m": -0.004,
                }}})


def test_fresh_matching_contact_uses_readable_pair_and_centimeters(failed_plan_and_contact):
    payload, diagnostic = failed_plan_and_contact
    text = collision_detail(payload, diagnostic, 0.2)
    assert "Diagnostik planner terbaru: jari kanan ↔ model environment/target" in text
    assert "calon jalur" in text
    assert "X 29.0, Y -1.2, Z 21.2 cm" in text
    assert "penetration" not in text and "-0.004" not in text
    # Display evidence must never change execution gating or input data.
    assert not summarize(payload)[2]
    assert payload["plan_ready"] is False


@pytest.mark.parametrize("age", [-0.1, 3.0, 20.0, math.nan, math.inf, "0.1"])
def test_stale_or_invalid_receipt_age_hides_contact(failed_plan_and_contact, age):
    assert collision_detail(*failed_plan_and_contact, age) == ""


def test_successful_or_unrelated_plan_hides_previous_contact(failed_plan_and_contact):
    payload, diagnostic = failed_plan_and_contact
    assert collision_detail(dict(payload, plan_ready=True), diagnostic, 0.1) == ""
    assert collision_detail(payload, dict(diagnostic, plan_valid=True), 0.1) == ""
    assert collision_detail(dict(payload, message="waiting_for_joint_states"), diagnostic, 0.1) == ""
    assert collision_detail(payload, dict(diagnostic, reason="exact_collision_replan_exhausted"), 0.1) == ""


@pytest.mark.parametrize("terminal_reason", [
    "grasp_approach_unavailable:pregrasp_bridge_joint_jump",
    "grasp_approach_search_budget:pregrasp_bridge_ik_no_progress",
    "grasp_approach_unavailable:pregrasp_bridge_collision:link6__dd_gng_grasp_obstacles",
    "grasp_approach_unavailable:final_grasp_ik_singular",
    "grasp_approach_unavailable:grasp_closing_sweep_in_collision",
    "current_joint_state_out_of_bounds",
    "current_robot_mesh_intersects_environment_or_target",
])
def test_matching_terminal_failure_does_not_inherit_earlier_graph_collision(
        failed_plan_and_contact, terminal_reason):
    payload, diagnostic = failed_plan_and_contact
    payload["message"] = "reachability_plan_invalid:" + terminal_reason
    diagnostic["reason"] = terminal_reason
    # The reason and receipt are current, but this evidence is from an earlier
    # goal candidate. It must not be presented as the later failure's cause.
    diagnostic["last_rejection"] = {
        "stage": "goal_state", "mesh_checked": True, "mesh_blocked": False,
        "capsule_blocked": True,
        "capsule": {"body_links": ["link7", "gripper_left_link"]},
    }
    assert collision_detail(payload, diagnostic, 0.1) == ""
    assert collision_detail({}, diagnostic, 0.1,
                            "graph trajectory rejected: " + payload["message"]) == ""
    assert not summarize(payload)[2]
    # The same mismatch must also hide a genuine mesh contact from the old edge.
    diagnostic["last_rejection"] = {
        "stage": "graph_edge", "mesh_checked": True, "mesh_blocked": True,
        "mesh": {"body_a": "link7", "body_b": "dd_gng_environment"},
    }
    assert collision_detail(payload, diagnostic, 0.1) == ""


def test_matching_failed_service_response_can_supply_reason(failed_plan_and_contact):
    payload, diagnostic = failed_plan_and_contact
    service_notice = "graph trajectory rejected: " + payload["message"]
    assert "jari kanan" in collision_detail({}, diagnostic, 0.1, service_notice)
    assert collision_detail({"plan_ready": True}, diagnostic, 0.1, service_notice) == ""


def test_contact_location_optional_and_nonfinite_coordinates_hidden(failed_plan_and_contact):
    payload, diagnostic = failed_plan_and_contact
    diagnostic["last_rejection"]["mesh"]["contact_world_m"] = [math.nan, 0, 0]
    text = collision_detail(payload, diagnostic, 0.1)
    assert "jari kanan" in text and "global" not in text
    diagnostic["last_rejection"]["mesh"]["body_a"] = ""
    assert collision_detail(payload, diagnostic, 0.1) == ""


def test_capsule_display_explicitly_distinguishes_mesh_pass(failed_plan_and_contact):
    payload, diagnostic = failed_plan_and_contact
    diagnostic["last_rejection"] = {
        "stage": "goal_state", "mesh_checked": True, "mesh_blocked": False,
        "capsule_blocked": True,
        "capsule": {"body_links": ["link7", "gripper_left_link"]},
    }
    text = collision_detail(payload, diagnostic, 0.1)
    assert "jarak aman" in text and "jari kiri" in text and "pose tujuan" in text
    assert "Mesh pada kandidat ini lolos" in text


def test_failed_pickup_reports_real_cause_even_after_preview_expiry():
    headline, detail, executable = summarize({
        'state': 'motion_failed', 'failure_stage': 'following_final_approach',
        'plan_ready': True, 'execution_blockers': ['frozen_plan_stale'],
        'message': 'frozen_object_track_not_current', 'executable': True,
    })
    assert 'pra-jepit ke pusat benda' in headline
    assert 'ditemukan kembali' in detail
    assert not executable


def test_busy_pickup_reports_stage_instead_of_preview_expiry():
    headline, detail, executable = summarize({
        'state': 'following_final_approach', 'busy': True, 'plan_ready': True,
        'execution_blockers': ['frozen_plan_stale'], 'message': 'pickup_target_reacquiring',
    })
    assert 'pra-jepit ke pusat benda' in headline
    assert 'nomor track berubah' in detail
    assert not executable
