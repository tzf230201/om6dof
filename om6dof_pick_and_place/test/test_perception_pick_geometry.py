import json
import threading
import time
from collections import deque
from types import SimpleNamespace
from unittest.mock import Mock

import numpy as np
import pytest

from om6dof_pick_and_place.direct_pick_node import (
    DirectPickNode,
    PERCEPTION_CAMERA_FRAME,
    PERCEPTION_METADATA,
    approach_standoff_distances,
    axis_aligned_bbox_top_world,
    consecutive_detection_streak,
    direct_approach_direction,
    image_axis_tracking_target,
    linear_waypoints,
    optical_point_to_world,
    safe_approach_standoff_distances,
    stable_point_median,
    yaw_tracking_target,
)


def test_optical_point_transform_identity():
    point = optical_point_to_world(
        np.array([0.1, 0.2, 0.3]),
        np.array([1.0, 2.0, 3.0]),
        np.eye(3),
        np.array([0.01, 0.02, 0.03]),
        np.eye(3),
    )
    assert np.allclose(point, [1.11, 2.22, 3.33])


def test_stable_point_median_accepts_tight_cluster():
    samples = [
        np.array([0.10, 0.20, 0.30]),
        np.array([0.11, 0.19, 0.30]),
        np.array([0.09, 0.20, 0.31]),
    ]
    assert np.allclose(
        stable_point_median(samples, max_spread=0.02),
        [0.10, 0.20, 0.30],
    )


def test_stable_point_median_rejects_depth_jump():
    samples = [np.array([0.0, 0.0, 0.3]), np.array([0.0, 0.0, 0.8])]
    assert stable_point_median(samples, max_spread=0.03) is None


def test_linear_waypoints_advance_at_constant_height_and_bounded_step():
    points = linear_waypoints(
        np.array([0.20, 0.0, 0.10]),
        np.array([0.27, 0.0, 0.10]),
        max_step=0.015,
    )
    previous = np.array([0.20, 0.0, 0.10])
    assert np.allclose(points[-1], [0.27, 0.0, 0.10])
    assert all(np.isclose(point[2], 0.10) for point in points)
    for point in points:
        assert np.linalg.norm(point - previous) <= 0.015 + 1e-9
        previous = point


def test_direct_approach_direction_includes_vertical_component():
    direction = direct_approach_direction(
        np.array([0.10, -0.05, 0.25]),
        np.array([0.40, 0.10, 0.10]),
    )
    expected = np.array([0.30, 0.15, -0.15])
    expected /= np.linalg.norm(expected)
    assert np.allclose(direction, expected)
    assert direction[2] < 0.0


def test_bbox_top_world_uses_vertical_extent_after_rotation():
    top = axis_aligned_bbox_top_world(
        center_optical=np.array([0.0, 0.0, 0.40]),
        size_optical=np.array([0.04, 0.10, 0.06]),
        p_we=np.zeros(3),
        R_we=np.eye(3),
        t_ec=np.zeros(3),
        R_eo=np.eye(3),
    )
    assert np.allclose(top, [0.0, 0.0, 0.43])


def test_linear_waypoints_follow_diagonal_3d_ray():
    start = np.array([0.10, 0.00, 0.25])
    end = np.array([0.30, 0.10, 0.05])
    points = linear_waypoints(start, end, max_step=0.04)
    ray = end - start
    for point in points:
        progress = (point[0] - start[0]) / ray[0]
        assert np.allclose(point, start + progress * ray)


def test_visual_approach_distances_descend_and_end_at_final_standoff():
    assert np.allclose(
        approach_standoff_distances(0.16, 0.08, 0.04),
        [0.16, 0.12, 0.08],
    )


def test_visual_approach_distances_handle_non_even_step():
    assert np.allclose(
        approach_standoff_distances(0.15, 0.08, 0.04),
        [0.15, 0.11, 0.08],
    )


def test_adaptive_approach_skips_folded_near_base_waypoint():
    assert np.allclose(
        safe_approach_standoff_distances(
            0.406, [0.16, 0.12, 0.08], minimum_target_radius=0.28),
        [0.12, 0.08],
    )


def test_detection_streak_requires_strictly_consecutive_frames():
    streak = 0
    for detected in [True, True, False, True, True, True]:
        streak = consecutive_detection_streak(streak, detected)
    assert streak == 3


def test_yaw_tracking_ignores_target_inside_deadband():
    target, error, move = yaw_tracking_target(
        0.2, 0.001, 1.0, 0.5, -1.0, 0.05, 0.04, -2.6, 2.6)
    assert np.isclose(target, 0.2)
    assert abs(error) < 0.05
    assert move is False


def test_yaw_tracking_moves_right_and_limits_each_step():
    target, error, move = yaw_tracking_target(
        0.2, 0.5, 1.0, 1.0, -1.0, 0.02, 0.04, -2.6, 2.6)
    assert error > 0.0
    assert np.isclose(target, 0.16)
    assert move is True


def test_yaw_tracking_respects_joint_limit():
    target, _error, move = yaw_tracking_target(
        -2.59, 0.5, 1.0, 1.0, -1.0, 0.02, 0.20, -2.6, 2.6)
    assert np.isclose(target, -2.6)
    assert move is True


def test_vertical_tracking_moves_joint5_positive_for_object_below_image():
    target, error, move = image_axis_tracking_target(
        0.89, 0.14, 0.37, 0.35, 1.0, 0.045, 0.04, -1.8, 1.8)
    assert error > 0.0
    assert np.isclose(target, 0.93)
    assert move is True


def test_pick_preflight_rejects_unreachable_target():
    node = object.__new__(DirectPickNode)
    import threading
    node._run_lock = threading.Lock()
    node._busy = False
    node._object_world = lambda allow_stale=False: (
        np.zeros(3), np.array([0.6, 0.0, 0.0]), np.eye(3)
    )
    node._reachable = lambda _point: {
        "reachable": False,
        "radius": 0.6,
        "standoff_ok": False,
        "grasp_ok": False,
        "obj": [0.6, 0.0, 0.0],
    }
    node.get_parameter = lambda name: type("P", (), {"value": (
        [0.0, -1.0, -1.0] if name == "perception_world_min"
        else [1.0, 1.0, 1.0]
    )})()

    ready, message = node._pick_preflight()

    assert ready is False
    assert "outside arm workspace" in message


def _guarded_picker():
    """Exercise callbacks without creating a ROS node, clients, or hardware."""
    node = object.__new__(DirectPickNode)
    node._perception_guard_lock = threading.RLock()
    node._search_state_lock = threading.Lock()
    node._run_lock = threading.Lock()
    node._perception_input_error = ""
    node._perception_frame_seen = False
    node._perception_status_compatible = True
    node._perception_samples = deque(maxlen=60)
    node._perception_bbox_optical = None
    node._last_obj = None
    node._last_obj_t = 0.0
    node._search_detection_streak = 0
    node._search_target_description = "object"
    node._pickup_mode = False
    node._tracking_mode = False
    node._search_mode = False
    node._busy = False
    node._worker_thread = None
    node._cancel = threading.Event()
    node.object_source = "perception"
    node.client = Mock()
    node.client.begin_sequence.return_value = True
    node.get_logger = lambda: Mock()
    return node


def _target(frame=PERCEPTION_CAMERA_FRAME):
    return SimpleNamespace(
        header=SimpleNamespace(frame_id=frame),
        point=SimpleNamespace(x=0.1, y=0.0, z=0.3),
    )


def _status(**metadata):
    return SimpleNamespace(data=json.dumps({
        **metadata,
        "target": {
            "state": "tracking", "class": "bottle", "point": [0.1, 0.0, 0.3],
            "bbox3d": {"center": [0.1, 0.0, 0.3], "size": [0.1, 0.1, 0.1]},
        },
    }))


@pytest.mark.parametrize("frame", ["camera_color_optical_frame", "world", ""])
def test_wrong_camera_frame_clears_cached_geometry_and_rejects_pick(frame):
    node = _guarded_picker()
    node._on_perception_target(_target())
    node._perception_bbox_optical = (time.monotonic(), np.zeros(3), np.ones(3))
    node._last_obj = (np.zeros(3), np.ones(3), np.eye(3))
    node._last_obj_t = time.monotonic()
    node._search_detection_streak = 4

    node._on_perception_target(_target(frame))

    assert not node._perception_samples
    assert node._perception_bbox_optical is None
    assert node._last_obj is None
    assert node._last_obj_t == 0.0
    assert node._search_detection_streak == 0
    assert node._perception_object_world(allow_stale=True) is None
    assert node._perception_bbox_top_world() is None
    ready, reason = node._pick_preflight()
    assert ready is False
    assert "camera frame" in reason


def test_correct_v2_frame_and_unlabelled_status_are_supported_initially():
    node = _guarded_picker()
    node._search_mode = True
    node._on_perception_target(_target())
    node._on_perception_status(_status())

    assert len(node._perception_samples) == 1
    assert node._perception_input_error == ""
    assert node._perception_bbox_optical is not None
    assert node._search_detection_streak == 1


@pytest.mark.parametrize("camera_model", ["D435", "D435i"])
def test_both_supported_v2_camera_models_restore_compatible_status(camera_model):
    node = _guarded_picker()
    node._perception_status_compatible = False
    node._perception_input_error = "previous mismatch"
    node._on_perception_status(_status(
        **PERCEPTION_METADATA, camera_model=camera_model))
    assert node._perception_status_compatible


@pytest.mark.parametrize("key,value", [
    ("model_version", "v1"), ("camera_model", "D405"),
    ("camera_frame", "camera_color_optical_frame"),
    ("model_version", "unknown"),
])
def test_status_model_mismatch_latches_until_complete_v2_metadata_and_new_point(
        key, value):
    node = _guarded_picker()
    node._on_perception_target(_target())
    node._on_perception_status(_status(**{key: value}))
    assert not node._perception_status_compatible
    assert not node._perception_samples

    # A seemingly compatible header (or old unlabelled status) cannot undo
    # evidence that the producer is publishing a different robot/camera model.
    node._on_perception_status(_status())
    node._on_perception_target(_target())
    assert not node._perception_samples
    assert node._perception_input_error

    node._on_perception_status(_status(
        **PERCEPTION_METADATA, camera_model="D435i"))
    assert node._perception_status_compatible
    assert node._perception_input_error  # no usable new frame yet
    assert node._perception_bbox_optical is None
    node._on_perception_target(_target())
    node._on_perception_status(_status(
        **PERCEPTION_METADATA, camera_model="D435i"))
    assert len(node._perception_samples) == 1
    assert node._perception_input_error == ""
    assert node._perception_bbox_optical is not None


def test_correct_frame_recovers_after_bad_frame_without_resurrecting_cache():
    node = _guarded_picker()
    node._on_perception_target(_target(""))
    node._on_perception_target(_target())
    assert node._perception_input_error == ""
    assert len(node._perception_samples) == 1
    assert node._last_obj is None


@pytest.mark.parametrize("mode", ["_pickup_mode", "_tracking_mode", "_search_mode"])
def test_model_mismatch_cancels_active_perception_run_without_auto_resume(mode):
    node = _guarded_picker()
    setattr(node, mode, True)
    node._busy = True
    node._on_perception_status(_status(model_version="v1"))

    assert node._cancel.is_set()
    node.client.cancel_current_goal.assert_called_once_with()
    node._on_perception_status(_status(
        **PERCEPTION_METADATA, camera_model="D435"))
    node._on_perception_target(_target())
    assert node._perception_input_error == ""
    assert node._cancel.is_set()  # a new explicit run is still required


def test_bad_perception_input_does_not_cancel_apriltag_sequence():
    node = _guarded_picker()
    node.object_source = "apriltag"
    node._pickup_mode = True
    cached = (np.zeros(3), np.ones(3), np.eye(3))
    node._last_obj = cached
    node._on_perception_target(_target("camera_color_optical_frame"))
    assert node._last_obj is cached
    assert not node._cancel.is_set()
    node.client.cancel_current_goal.assert_not_called()


def test_rejected_input_blocks_tracking_and_search_service_starts():
    node = _guarded_picker()
    node._on_perception_status(_status(camera_model="D405"))
    for callback in (node._on_track, node._on_search):
        response = SimpleNamespace(success=None, message="")
        callback(None, response)
        assert response.success is False
        assert "rejected" in response.message
    assert not node._busy


def test_rejected_input_blocks_next_trajectory_but_allows_explicit_hold():
    node = _guarded_picker()
    node._publish_arm_trajectory_unchecked = Mock()
    node._on_perception_target(_target("camera_color_optical_frame"))
    node._publish_arm_trajectory(np.zeros(6))
    node._publish_arm_trajectory_unchecked.assert_not_called()
    node._publish_arm_trajectory(np.zeros(6), 0.1, hold_current=True)
    node._publish_arm_trajectory_unchecked.assert_called_once()


def test_new_explicit_run_waits_for_client_cancellation_to_finish():
    node = _guarded_picker()
    node._cancel.set()
    node.client.begin_sequence.return_value = False
    assert node._preempt() is False
    assert node._cancel.is_set()
    node.client.begin_sequence.return_value = True
    assert node._preempt() is True
    assert not node._cancel.is_set()
