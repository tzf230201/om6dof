"""Geometry helpers for the vision-only target/GraspNet viewer."""

import threading
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

import om6dof_pick_and_place_gemini.target_grasp_viewer as viewer_module
from om6dof_pick_and_place_gemini.gemini_client import Localization
from om6dof_pick_and_place_gemini.grasp_backends import (
    GraspCandidate, GraspScene)
from om6dof_pick_and_place_gemini.grasp_filter import Rejection
from om6dof_pick_and_place_gemini.target_grasp_viewer import (
    TargetGraspViewer, cloud_message, parallel_gripper_points)
from om6dof_pick_and_place_gemini.target_selection import (
    candidate_diagnostic_summary, candidate_pixel,
    closest_parallel_jaw_orientation, select_target_candidate)
from om6dof_pick_and_place_gemini.transforms import tool_rotation


def scene():
    return GraspScene(
        points_optical=np.zeros((0, 3)), points_base=np.zeros((0, 3)),
        pixels=np.zeros((0, 2)), colors=None, p_wc=np.array([1.0, 2.0, 3.0]),
        R_wc=np.eye(3), intrinsics=(100.0, 100.0, 3.0, 2.0),
        color_image=np.zeros((5, 7, 3), np.uint8))


def candidate(position):
    return GraspCandidate(
        position=np.asarray(position, dtype=float), approach=np.array([0, 0, -1]),
        closing=np.array([0, 1, 0]), width=0.02, score=0.9)


def test_candidate_pixel_reprojects_world_point_into_rgb_image():
    assert candidate_pixel(candidate([1.0, 2.0, 3.5]), scene()) == (3, 2)


def test_candidate_pixel_rejects_points_outside_the_image_or_camera():
    assert candidate_pixel(candidate([5.0, 2.0, 3.5]), scene()) is None
    assert candidate_pixel(candidate([1.0, 2.0, 2.5]), scene()) is None


def test_target_mode_selects_nearest_projected_valid_grasp_not_best_score():
    near = candidate([1.0, 2.0, 3.5])
    near.score = 0.90
    far = candidate([1.01, 2.0, 3.5])
    far.score = 0.99
    assert select_target_candidate(
        [far, near], scene(), (3.0, 2.0)) is near


def test_target_mode_prefers_sensible_top_down_orientation():
    near_but_sideways = candidate([1.0, 2.0, 3.5])
    near_but_sideways.score = 0.90
    near_but_sideways.approach = np.array([0.8, 0.0, -0.6])
    vertical = candidate([1.01, 2.0, 3.5])
    vertical.score = 0.82
    vertical.approach = np.array([0.0, 0.0, -1.0])

    assert select_target_candidate(
        [near_but_sideways, vertical], scene(), (3.0, 2.0)) is vertical


def test_low_quality_vertical_candidate_does_not_beat_sane_high_score_grasp():
    good = candidate([1.0, 2.0, 3.5])
    good.score = 0.90
    good.approach = np.array([0.3, 0.0, -0.954])
    poor = candidate([1.0, 2.0, 3.5])
    poor.score = 0.20
    poor.approach = np.array([0.0, 0.0, -1.0])

    assert select_target_candidate(
        [poor, good], scene(), (3.0, 2.0)) is good


def test_target_mode_prefers_orientation_nearest_the_current_gripper():
    current_like = candidate([1.01, 2.0, 3.5])
    current_like.score = 0.90
    current_like.approach = np.array([1.0, 0.0, 0.0])
    vertical = candidate([1.0, 2.0, 3.5])
    vertical.score = 0.99
    reference = tool_rotation(current_like.approach, current_like.closing)

    selected = select_target_candidate(
        [vertical, current_like], scene(), (3.0, 2.0),
        reference_rotation=reference)

    assert selected is current_like
    assert selected.extras["orientation_delta_rad"] == pytest.approx(0.0)


def test_parallel_jaw_symmetry_avoids_an_unnecessary_half_turn():
    grasp = candidate([1.0, 2.0, 3.5])
    grasp.closing = np.array([0.0, -1.0, 0.0])
    reference = tool_rotation(grasp.approach, -grasp.closing)

    distance, aligned_closing, flipped = \
        closest_parallel_jaw_orientation(grasp, reference)

    assert distance == pytest.approx(0.0)
    assert aligned_closing == pytest.approx([0.0, 1.0, 0.0])
    assert flipped

    selected = select_target_candidate(
        [grasp], scene(), (3.0, 2.0), reference_rotation=reference)
    assert selected.closing == pytest.approx([0.0, 1.0, 0.0])
    assert selected.extras["closing_axis_flipped"] == 1.0


def test_reference_orientation_is_primary_after_upstream_safety_filters():
    low = candidate([1.0, 2.0, 3.5])
    low.score = 0.10
    low.approach = np.array([1.0, 0.0, 0.0])
    high = candidate([1.01, 2.0, 3.5])
    high.score = 0.90
    reference = tool_rotation(low.approach, low.closing)

    assert select_target_candidate(
        [low, high], scene(), (3.0, 2.0), score_slack=0.15,
        reference_rotation=reference) is low


def test_reference_orientation_uses_score_before_pixel_as_tiebreak():
    farther_high_score = candidate([1.01, 2.0, 3.5])
    farther_high_score.score = 0.90
    near_low_score = candidate([1.0, 2.0, 3.5])
    near_low_score.score = 0.80
    reference = tool_rotation(
        farther_high_score.approach, farther_high_score.closing)

    assert select_target_candidate(
        [near_low_score, farther_high_score], scene(), (3.0, 2.0),
        reference_rotation=reference) is farther_high_score


def test_world_cloud_message_has_xyz_and_rgb_fields_in_the_world_frame():
    cloud = cloud_message(
        np.array([[1.0, 2.0, 3.0]]), np.array([[10, 20, 30]], np.uint8),
        frame_id="world", stamp=123.0)

    assert cloud.header.frame_id == "world"
    assert cloud.width == 1
    assert cloud.point_step == 16
    assert [field.name for field in cloud.fields] == ["x", "y", "z", "rgb"]
    # BGR [10, 20, 30] is packed as RGB 0x1e140a for RViz RGB8.
    assert int.from_bytes(bytes(cloud.data)[12:16], "little") == 0x1E140A


def test_parallel_gripper_glyph_has_two_fingers_palm_and_rear_stem():
    grasp = candidate([0.30, 0.0, 0.10])
    grasp.approach = np.array([1.0, 0.0, 0.0])
    grasp.closing = np.array([0.0, 1.0, 0.0])
    grasp.width = 0.040
    grasp.extras["graspnet_depth"] = 0.030

    points = parallel_gripper_points(
        grasp, palm_depth=0.020, tail_length=0.040)

    assert points.shape == (8, 3)
    # Both fingers run from x=0.28 behind the centre to x=0.33 forward.
    assert points[0] == pytest.approx([0.28, -0.02, 0.10])
    assert points[1] == pytest.approx([0.33, -0.02, 0.10])
    assert points[2] == pytest.approx([0.28, 0.02, 0.10])
    assert points[3] == pytest.approx([0.33, 0.02, 0.10])
    # Palm joins the fingers; stem points backward along -approach.
    assert points[4] == pytest.approx(points[0])
    assert points[5] == pytest.approx(points[2])
    assert points[6] == pytest.approx([0.24, 0.0, 0.10])
    assert points[7] == pytest.approx([0.28, 0.0, 0.10])


def test_parallel_gripper_glyph_respects_rotated_grasp_axes():
    grasp = candidate([0.0, 0.0, 0.0])
    grasp.approach = np.array([0.0, 0.0, -1.0])
    grasp.closing = np.array([1.0, 0.0, 0.0])
    grasp.width = 0.020
    grasp.extras["graspnet_depth"] = 0.010
    points = parallel_gripper_points(grasp)

    assert points[1] - points[0] == pytest.approx([0.0, 0.0, -0.030])
    assert points[2] - points[0] == pytest.approx([0.020, 0.0, 0.0])
    assert points[7] - points[6] == pytest.approx([0.0, 0.0, -0.040])


def test_parallel_gripper_glyph_stays_on_centre_when_tcp_is_offset():
    grasp = candidate([0.30, 0.0, 0.10])
    grasp.approach = np.array([1.0, 0.0, 0.0])
    grasp.extras["tcp_position"] = np.array([0.60, 0.0, 0.10])

    points = parallel_gripper_points(grasp)

    assert np.max(points[:, 0]) < 0.40
    assert np.min(points[:, 0]) > 0.20


def test_parallel_gripper_glyph_does_not_mutate_candidate_axes():
    grasp = candidate([0.30, 0.0, 0.10])
    grasp.approach = np.array([0.0, 0.0, -2.0])
    grasp.closing = np.array([0.0, 3.0, 0.0])
    original_approach = grasp.approach.copy()
    original_closing = grasp.closing.copy()

    parallel_gripper_points(grasp)

    assert np.array_equal(grasp.approach, original_approach)
    assert np.array_equal(grasp.closing, original_closing)


def test_viewer_capture_assigns_post_exclusion_source_indices(monkeypatch):
    viewer = object.__new__(TargetGraspViewer)
    values = {
        "camera_optical_frame": "camera_optical",
        "cloud_stride": 1,
        "cloud_z_min": 0.05,
        "cloud_z_max": 0.80,
        "self_exclusion_radius_m": 0.09,
        "base_frame": "world",
    }
    viewer.get_parameter = lambda name: SimpleNamespace(value=values[name])
    viewer._lookup_pose = lambda frame, _stamp: (
        (np.zeros(3), np.eye(3)) if frame == "camera_optical"
        else (np.ones(3), np.eye(3)))
    viewer.get_logger = lambda: SimpleNamespace(warn=pytest.fail)
    points = np.array([
        [0.10, 0.0, 0.20],
        [0.20, 0.0, 0.20],
        [0.30, 0.0, 0.20],
    ])
    pixels = np.array([[10, 10], [20, 20], [30, 30]], dtype=float)
    colors = np.array([[1, 2, 3], [4, 5, 6], [7, 8, 9]], np.uint8)
    monkeypatch.setattr(
        viewer_module, "point_cloud",
        lambda *_args, **_kwargs: (points.copy(), colors.copy(), pixels.copy()))
    monkeypatch.setattr(
        viewer_module, "self_exclusion_mask",
        lambda *_args, **_kwargs: np.array([True, False, True]))
    frame = SimpleNamespace(
        frame_id="camera_optical", stamp=12.0,
        depth=np.ones((1, 1), np.uint16),
        color=np.zeros((1, 1, 3), np.uint8),
        intrinsics=(100.0, 100.0, 0.0, 0.0), depth_scale=0.001)

    captured = TargetGraspViewer._scene_from_frame(viewer, frame)

    assert np.array_equal(captured.points_optical, points[[0, 2]])
    assert np.array_equal(captured.pixels, pixels[[0, 2]])
    assert np.array_equal(captured.colors, colors[[0, 2]])
    assert np.array_equal(captured.source_indices, [0, 1])


def test_anygrasp_viewer_uses_full_scene_and_exact_target_mask(monkeypatch):
    points = np.array([
        [-0.01, -0.01, 0.50],
        [0.01, 0.01, 0.50],
        [0.00, 0.00, 0.52],
        [0.40, 0.40, 0.50],
    ])
    full = GraspScene(
        points_optical=points.copy(), points_base=points.copy(),
        pixels=np.array([[1, 1], [5, 3], [3, 2], [6, 4]], dtype=float),
        colors=np.zeros((4, 3), np.uint8), p_wc=np.zeros(3),
        R_wc=np.eye(3), intrinsics=(100.0, 100.0, 3.0, 2.0),
        color_image=np.zeros((5, 7, 3), np.uint8),
        source_indices=np.array([10, 11, 12, 13]))
    segmented = GraspScene(
        points_optical=points[[2, 0]].copy(),
        points_base=points[[2, 0]].copy(),
        pixels=np.array([[3, 2], [1, 1]], dtype=float),
        colors=np.zeros((2, 3), np.uint8), p_wc=np.zeros(3),
        R_wc=np.eye(3), intrinsics=full.intrinsics,
        color_image=full.color_image,
        source_indices=np.array([12, 10]))
    located = Localization(
        found=True, pixel=(3.0, 2.0), box=(0.0, 0.0, 6.0, 4.0),
        confidence=0.95, reason="found")
    grasp = candidate([0.0, 0.0, 0.51])
    calls = {}

    class Backend:
        name = "anygrasp"
        supports_region_steering = True
        max_width = 0.065
        last_stats = {}

        def detect(self, network_scene, collision_scene=None,
                   region_mask=None):
            calls["detect"] = (
                network_scene, collision_scene, region_mask.copy())
            return [grasp]

    values = {
        "target_crop_pad_px": 4.0,
        "target_seed_radius_px": 14.0,
        "target_depth_tolerance_m": 0.05,
        "target_component_voxel_m": 0.008,
        "target_component_min_points": 1,
        "table_z": 0.0,
        "target_table_margin_m": 0.006,
        "gripper_min_width_m": 0.010,
        "gripper_max_width_m": 0.065,
        "gripper_width_at_open_pos": -1.0,
        "grasp_min_clearance_m": 0.005,
        "grasp_max_tilt_rad": 1.50,
        "workspace_min": [-1.0, -1.0, -1.0],
        "workspace_max": [1.0, 1.0, 1.0],
        "marker_pregrasp_standoff": 0.08,
        "gripper_scene_collision_enabled": False,
        "target_bounds_margin_m": 0.020,
        "selection_score_slack": 0.15,
        "selection_tilt_slack_rad": np.deg2rad(10.0),
    }
    viewer = object.__new__(TargetGraspViewer)
    viewer._gemini = SimpleNamespace(
        enabled=True, locate=lambda _image, _target: located)
    viewer._backend = Backend()
    viewer._scene_from_frame = lambda _frame: full
    viewer.get_parameter = lambda name: SimpleNamespace(value=values[name])
    viewer.get_logger = lambda: SimpleNamespace(
        info=lambda _message: None, error=lambda _message: None,
        warn=lambda _message: None)
    viewer._lock = threading.Lock()
    viewer._result = None
    viewer._result_generation = 0
    viewer._inference_thread = object()
    monkeypatch.setattr(
        viewer_module, "segment_target_component",
        lambda source, *_args, **_kwargs: (
            segmented if source is full else pytest.fail(
                "segmentation must use the full capture")))
    monkeypatch.setattr(
        viewer_module, "crop_to_workspace",
        lambda *_args, **_kwargs: pytest.fail(
            "AnyGrasp viewer must not crop the inference scene"))

    TargetGraspViewer._run_inference(viewer, object(), "red cube")

    assert viewer._result is not None, viewer._status
    assert viewer._result.target_scene is segmented
    assert viewer._result.selected is grasp
    assert calls["detect"][0] is full
    assert calls["detect"][1] is full
    assert np.array_equal(calls["detect"][2], [True, False, True, False])
    assert "network-scene=4 points" in viewer._status


def test_viewer_rejects_candidate_when_conservative_collision_finds_obstacle(
        monkeypatch):
    points = np.array([
        [-0.01, -0.01, 0.50],
        [0.01, 0.01, 0.50],
        [0.00, 0.00, 0.52],
        [0.20, 0.20, 0.50],
    ])
    full = GraspScene(
        points_optical=points.copy(), points_base=points.copy(),
        pixels=np.array([[1, 1], [5, 3], [3, 2], [6, 4]], dtype=float),
        colors=np.zeros((4, 3), np.uint8), p_wc=np.zeros(3),
        R_wc=np.eye(3), intrinsics=(100.0, 100.0, 3.0, 2.0),
        color_image=np.zeros((5, 7, 3), np.uint8),
        source_indices=np.arange(4))
    segmented = GraspScene(
        points_optical=points[:3].copy(), points_base=points[:3].copy(),
        pixels=full.pixels[:3].copy(), colors=full.colors[:3].copy(),
        p_wc=np.zeros(3), R_wc=np.eye(3), intrinsics=full.intrinsics,
        color_image=full.color_image, source_indices=np.arange(3))
    located = Localization(
        found=True, pixel=(3.0, 2.0), box=(0.0, 0.0, 6.0, 4.0),
        confidence=0.95, reason="found")
    grasp = candidate([0.0, 0.0, 0.51])

    class Backend:
        name = "anygrasp"
        supports_region_steering = True
        last_stats = {}

        def detect(self, *_args, **_kwargs):
            return [grasp]

    values = {
        "target_crop_pad_px": 4.0,
        "target_seed_radius_px": 14.0,
        "target_depth_tolerance_m": 0.05,
        "target_component_voxel_m": 0.008,
        "target_component_min_points": 1,
        "table_z": 0.0,
        "target_table_margin_m": 0.006,
        "gripper_min_width_m": 0.010,
        "gripper_max_width_m": 0.065,
        "gripper_width_at_open_pos": -1.0,
        "grasp_min_clearance_m": 0.005,
        "grasp_max_tilt_rad": 1.50,
        "workspace_min": [-1.0, -1.0, -1.0],
        "workspace_max": [1.0, 1.0, 1.0],
        "marker_pregrasp_standoff": 0.08,
        "gripper_scene_collision_enabled": True,
        "gripper_collision_finger_back_m": 0.070,
        "gripper_collision_finger_front_m": 0.021,
        "gripper_collision_finger_thickness_m": 0.040,
        "gripper_collision_height_m": 0.058,
        "gripper_collision_margin_m": 0.002,
        "gripper_scene_collision_min_points": 3,
        "target_bounds_margin_m": 0.020,
        "selection_score_slack": 0.15,
        "selection_tilt_slack_rad": np.deg2rad(10.0),
    }
    viewer = object.__new__(TargetGraspViewer)
    viewer._gemini = SimpleNamespace(
        enabled=True, locate=lambda _image, _target: located)
    viewer._backend = Backend()
    viewer._scene_from_frame = lambda _frame: full
    viewer.get_parameter = lambda name: SimpleNamespace(value=values[name])
    warnings = []
    viewer.get_logger = lambda: SimpleNamespace(
        info=lambda _message: None, error=lambda _message: None,
        warn=warnings.append)
    viewer._lock = threading.Lock()
    viewer._result = None
    viewer._result_generation = 0
    viewer._inference_thread = object()
    collision_calls = []
    monkeypatch.setattr(
        viewer_module, "segment_target_component",
        lambda *_args, **_kwargs: segmented)
    monkeypatch.setattr(
        viewer_module, "conservative_gripper_collision",
        lambda checked, scene_points, **kwargs: (
            collision_calls.append((checked, scene_points, kwargs))
            or (False, "obstacle under finger")))

    TargetGraspViewer._run_inference(viewer, object(), "red cube")

    assert len(collision_calls) == 1
    assert collision_calls[0][0] is grasp
    assert collision_calls[0][1] is full.points_base
    assert collision_calls[0][2]["open_aperture"] == pytest.approx(0.065)
    assert np.array_equal(
        collision_calls[0][2]["target_mask"], [True, True, True, False])
    assert warnings and "ASSUMED PREVIEW" in warnings[0]
    assert viewer._result is not None, viewer._status
    assert viewer._result.candidates == []
    assert viewer._result.selected is None
    assert len(viewer._result.rejected) == 1
    assert viewer._result.rejected[0][0] is grasp
    assert viewer._result.rejected[0][1].reason == "scene_collision"
    assert viewer._result.rejected[0][1].detail == "obstacle under finger"
    assert "scene_collisionx1" in viewer._status
    assert "ASSUMED PREVIEW" in viewer._status


def test_candidate_diagnostics_show_clipping_tilt_and_rejection():
    accepted = candidate([0.0, 0.0, 0.0])
    accepted.width = 0.065
    accepted.extras["graspnet_width_raw"] = 0.093
    accepted.extras["post_grasp_mode"] = "reverse_to_pregrasp"
    accepted.extras["ik_post_grasp_position_error"] = 0.004
    accepted.extras["ik_post_grasp_orientation_error"] = np.deg2rad(2.0)
    tilted = candidate([0.0, 0.0, 0.0])
    tilted.approach = np.array([1.0, 0.0, 0.0])
    rejected = [(tilted, Rejection("tilt", "90 deg > limit"))]

    text = candidate_diagnostic_summary([accepted, tilted], rejected)

    assert "#1 score=0.900 width=65.0mm(raw=93.0) tilt=0.0deg OK" in text
    assert "post=reverse_to_pregrasp IK=4.0mm/2.0deg" in text
    assert "#2 score=0.900 width=20.0mm tilt=90.0deg REJECT:tilt" in text


def test_rviz_reads_robot_description_and_markers_from_their_real_topics():
    config = (Path(__file__).parents[1] / "config" /
              "target_grasp_viewer.rviz").read_text()
    assert "Description Source: Topic" in config
    assert "Value: /robot_description" in config
    assert "Value: /target_grasp_viewer/markers" in config
    assert "Description Parameter: robot_description" not in config


def test_target_viewer_launch_exposes_the_hardware_geometry_limits():
    launch = (Path(__file__).parents[1] / "launch" /
              "target_grasp_viewer.launch.py").read_text()
    assert '"gripper_max_width_m", default_value="0.065"' in launch
    assert '"gripper_width_at_open_pos", default_value="-1.0"' in launch
    assert 'LaunchConfiguration("gripper_width_at_open_pos")' in launch
    assert '"grasp_max_tilt_rad", default_value="1.50"' in launch
    assert 'LaunchConfiguration("grasp_max_tilt_rad")' in launch
    assert '"graspnet_sampling_seed", default_value="0"' in launch
