"""Every safety gate, one candidate at a time."""

import math

import numpy as np
import pytest

from om6dof_pick_and_place_gemini.grasp_backends import GraspCandidate
from om6dof_pick_and_place_gemini.grasp_filter import (
    FilterConfig, Rejection, best_near_miss, check,
    conservative_gripper_collision, filter_and_rank, gripper_collision_mask,
    nearest_to_pixel, rejection_summary, target_aware_gripper_collision,
    target_aware_gripper_collision_masks)
from om6dof_pick_and_place_gemini.transforms import tool_rotation


def candidate(position=(0.30, 0.00, 0.05), approach=(0.0, 0.0, -1.0),
              width=0.025, score=0.8, pixel=(320.0, 240.0),
              bbox=(300, 220, 340, 260)):
    return GraspCandidate(position=np.array(position, dtype=float),
                          approach=np.array(approach, dtype=float),
                          closing=np.array([0.0, 1.0, 0.0]), width=width,
                          score=score, pixel=pixel, bbox=bbox)


def test_a_sane_candidate_passes():
    assert check(candidate(), FilterConfig()) is None


@pytest.mark.parametrize("kwargs,reason", [
    ({"width": 0.002}, "width"),
    ({"width": 0.090}, "width"),
    ({"position": (0.30, 0.0, 0.001)}, "clearance"),
    ({"approach": (1.0, 0.0, -0.2)}, "tilt"),
    ({"position": (0.90, 0.0, 0.05)}, "workspace"),
    ({"score": -1.0}, "score"),
])
def test_each_gate_rejects_for_its_own_reason(kwargs, reason):
    cfg = FilterConfig(min_score=0.0)
    rejection = check(candidate(**kwargs), cfg)
    assert rejection is not None and rejection.reason == reason


def test_an_unreachable_pregrasp_is_rejected_even_when_the_grasp_is_fine():
    grasp = candidate()
    reachable = lambda position, _c: bool(  # noqa: E731 - one-line stub
        np.allclose(position, grasp.position))
    rejection = check(grasp, FilterConfig(), reachable)
    assert rejection is not None and rejection.reason == "reachability"


def test_chain_reachability_is_called_once_with_both_waypoints():
    grasp = candidate(position=(0.25, 0.0, 0.10))
    calls = []

    def chain(pregrasp, grasp_point, received):
        calls.append((pregrasp.copy(), grasp_point.copy(), received))
        return False, "grasp residual 25.0 mm/2.0 deg"

    cfg = FilterConfig()
    rejection = check(grasp, cfg, reachable_chain=chain)
    assert rejection.reason == "reachability"
    assert "25.0 mm" in rejection.detail
    assert len(calls) == 1
    assert calls[0][0] == pytest.approx(grasp.pregrasp(
        cfg.pregrasp_standoff))
    assert calls[0][1] == pytest.approx(grasp.position)
    assert calls[0][2] is grasp


def test_anygrasp_tcp_is_used_for_reachability_but_not_target_association():
    grasp = candidate(position=(0.30, 0.0, 0.10))
    grasp.extras["tcp_position"] = np.array([0.34, 0.0, 0.10])
    calls = []

    def chain(pregrasp, grasp_point, received):
        calls.append((pregrasp.copy(), grasp_point.copy(), received))
        return True

    cfg = FilterConfig(
        target_min=(0.29, -0.01, 0.09),
        target_max=(0.31, 0.01, 0.11),
        target_margin=0.0)
    rejection = check(grasp, cfg, reachable_chain=chain)

    assert rejection is None, \
        "target bounds must be checked at the grasp centre, not the TCP tip"
    assert len(calls) == 1
    assert calls[0][0] == pytest.approx([0.34, 0.0, 0.20])
    assert calls[0][1] == pytest.approx([0.34, 0.0, 0.10])
    assert calls[0][2] is grasp


def test_swept_gripper_envelope_rejects_a_thin_support_surface():
    grasp = candidate(position=(0.30, 0.0, 0.20), width=0.025)
    rotation = tool_rotation(grasp.approach, grasp.closing)
    # Three points only: a volume-ratio detector can dilute these in its
    # denominator, while all three physically occupy the right finger.
    local = np.array([
        [0.000, 0.020, 0.000],
        [0.002, 0.021, 0.001],
        [-0.002, 0.022, -0.001],
    ])
    scene_points = grasp.position + local @ rotation.T

    clear, detail = conservative_gripper_collision(
        grasp, scene_points, pregrasp_standoff=0.10, min_points=3)

    assert not clear
    assert "3 scene points" in detail


def test_collision_mask_is_the_exact_point_set_counted_by_the_verdict():
    grasp = candidate(position=(0.30, 0.0, 0.20), width=0.025)
    rotation = tool_rotation(grasp.approach, grasp.closing)
    # First three rows occupy a finger.  The fourth is in the open aperture;
    # the fifth has the right closing coordinate but is above the finger.
    local = np.array([
        [0.000, 0.020, 0.000],
        [0.002, 0.021, 0.001],
        [-0.002, 0.022, -0.001],
        [0.000, 0.000, 0.000],
        [0.080, 0.020, 0.000],
    ])
    scene_points = grasp.position + local @ rotation.T

    mask = gripper_collision_mask(
        grasp, scene_points, pregrasp_standoff=0.10)
    clear, detail = conservative_gripper_collision(
        grasp, scene_points, pregrasp_standoff=0.10, min_points=3)

    assert mask.dtype == np.bool_
    assert mask.tolist() == [True, True, True, False, False]
    assert not clear
    assert detail == (
        "3 scene points occupy the swept gripper envelope (limit 2)")


def test_collision_mask_preserves_empty_scene_shape():
    mask = gripper_collision_mask(
        candidate(), np.zeros((0, 3)), pregrasp_standoff=0.10)

    assert mask.shape == (0,)
    assert mask.dtype == np.bool_


def test_swept_collision_stays_anchored_at_geometric_grasp_centre():
    grasp = candidate(position=(0.30, 0.0, 0.20), width=0.025)
    grasp.extras["tcp_position"] = np.array([0.50, 0.0, 0.20])
    rotation = tool_rotation(grasp.approach, grasp.closing)
    local = np.array([
        [0.000, 0.020, 0.000],
        [0.002, 0.021, 0.001],
        [-0.002, 0.022, -0.001],
    ])
    scene_points = grasp.position + local @ rotation.T

    clear, detail = conservative_gripper_collision(
        grasp, scene_points, pregrasp_standoff=0.10, min_points=3)

    assert not clear
    assert "3 scene points" in detail


def test_swept_collision_uses_candidate_approach_not_tool_x():
    """An obstacle on a finger's approach sweep must be rejected.

    ``tool_rotation`` stores (remaining, closing, approach) as its X/Y/Z
    columns.  Keep the obstacle far back along approach so this test catches a
    collision implementation that accidentally treats tool X as approach and
    tool Z as finger height.
    """
    grasp = candidate(position=(0.30, 0.0, 0.20),
                      approach=(0.60, 0.0, -0.80), width=0.040)
    rotation = tool_rotation(grasp.approach, grasp.closing)
    # Local rows are [remaining/height, closing, approach].  These points lie
    # on one finger, 8 cm back along its 10 cm pregrasp sweep.
    local = np.array([
        [-0.001, 0.024, -0.080],
        [0.000, 0.025, -0.079],
        [0.001, 0.026, -0.078],
    ])
    scene_points = grasp.position + local @ rotation.T

    clear, detail = conservative_gripper_collision(
        grasp, scene_points, pregrasp_standoff=0.10, min_points=3)

    assert not clear
    assert "3 scene points" in detail


def test_swept_collision_uses_remaining_axis_for_gripper_height():
    """Points beyond finger height must not become a false collision."""
    grasp = candidate(position=(0.30, 0.0, 0.20),
                      approach=(0.60, 0.0, -0.80), width=0.040)
    rotation = tool_rotation(grasp.approach, grasp.closing)
    # The points align with a finger in closing/approach, but sit 8 cm out of
    # its height plane (the envelope half-height is only about 3 cm).
    local = np.array([
        [-0.080, 0.024, -0.001],
        [-0.079, 0.025, 0.000],
        [-0.078, 0.026, 0.001],
    ])
    scene_points = grasp.position + local @ rotation.T

    clear, detail = conservative_gripper_collision(
        grasp, scene_points, pregrasp_standoff=0.10, min_points=3)

    assert clear
    assert "0 scene points" in detail


def test_points_inside_the_open_jaw_are_not_solid_collision():
    grasp = candidate(position=(0.30, 0.0, 0.20), width=0.040)
    rotation = tool_rotation(grasp.approach, grasp.closing)
    local = np.array([[0.0, 0.0, z] for z in (-0.005, 0.0, 0.005)])
    scene_points = grasp.position + local @ rotation.T

    clear, _ = conservative_gripper_collision(
        grasp, scene_points, pregrasp_standoff=0.10, min_points=3)

    assert clear


def test_exact_target_contact_is_allowed_only_in_final_closure_band():
    grasp = candidate(position=(0.30, 0.0, 0.20), width=0.025)
    rotation = tool_rotation(grasp.approach, grasp.closing)
    local = np.array([
        [-0.002, 0.018, -0.005],
        [0.000, 0.020, 0.000],
        [0.002, 0.022, 0.005],
    ])
    points = grasp.position + local @ rotation.T

    clear, detail = target_aware_gripper_collision(
        grasp, points, target_mask=np.ones(3, dtype=bool),
        open_aperture=0.065, pregrasp_standoff=0.10,
        min_points=3)
    masks = target_aware_gripper_collision_masks(
        grasp, points, target_mask=np.ones(3, dtype=bool),
        open_aperture=0.065, pregrasp_standoff=0.10)

    assert clear, detail
    assert not np.any(masks.collision)
    assert np.all(masks.allowed_target_contact)
    assert "allowed-target-contact=3" in detail


def test_same_closure_points_from_environment_are_never_exempted():
    grasp = candidate(position=(0.30, 0.0, 0.20), width=0.025)
    rotation = tool_rotation(grasp.approach, grasp.closing)
    local = np.array([
        [0.000, 0.000, 0.000],  # exact target, clear in central gap
        [0.001, 0.000, 0.002],
        [-0.001, 0.000, -0.002],
        [-0.002, 0.018, -0.005],  # unrelated object in closure sweep
        [0.000, 0.020, 0.000],
        [0.002, 0.022, 0.005],
    ])
    points = grasp.position + local @ rotation.T
    target = np.array([True, True, True, False, False, False])

    clear, detail = target_aware_gripper_collision(
        grasp, points, target_mask=target, open_aperture=0.065,
        pregrasp_standoff=0.10, min_points=3)

    assert not clear
    assert "environment-closure=3" in detail


@pytest.mark.parametrize("local,expected_detail", [
    (np.array([[0.000, 0.000, -0.060],
               [0.001, 0.001, -0.059],
               [-0.001, -0.001, -0.061]]), "approach/palm=3"),
    (np.array([[0.000, 0.034, 0.000],
               [0.001, 0.035, 0.001],
               [-0.001, 0.036, -0.001]]), "target-outside-open=3"),
])
def test_target_label_never_excuses_palm_rear_or_outer_finger(
        local, expected_detail):
    grasp = candidate(position=(0.30, 0.0, 0.20), width=0.025)
    points = grasp.position + local @ tool_rotation(
        grasp.approach, grasp.closing).T

    clear, detail = target_aware_gripper_collision(
        grasp, points, target_mask=np.ones(3, dtype=bool),
        open_aperture=0.065, pregrasp_standoff=0.10,
        min_points=3)

    assert not clear
    assert expected_detail in detail


def test_environment_on_open_finger_approach_sweep_is_rejected():
    grasp = candidate(position=(0.30, 0.0, 0.20), width=0.025)
    local = np.array([
        [0.000, 0.032, -0.080],
        [0.001, 0.033, -0.079],
        [-0.001, 0.034, -0.078],
        [0.000, 0.000, 0.000],  # exact target keeps provenance non-empty
    ])
    points = grasp.position + local @ tool_rotation(
        grasp.approach, grasp.closing).T
    target = np.array([False, False, False, True])

    clear, detail = target_aware_gripper_collision(
        grasp, points, target_mask=target, open_aperture=0.065,
        pregrasp_standoff=0.10, min_points=3)

    assert not clear
    assert "approach/palm=3" in detail


@pytest.mark.parametrize("target_mask,open_aperture,phrase", [
    (np.array([1, 0, 0]), 0.065, "bool dtype"),
    (np.array([True, False]), 0.065, "expected (3,)"),
    (np.zeros(3, dtype=bool), 0.065, "empty"),
    (np.ones(3, dtype=bool), -1.0, "unmeasured or invalid"),
])
def test_target_aware_collision_fails_closed_on_invalid_mask_or_aperture(
        target_mask, open_aperture, phrase):
    points = np.array([[0.30, 0.0, 0.20],
                       [0.30, 0.01, 0.20],
                       [0.30, -0.01, 0.20]])

    clear, detail = target_aware_gripper_collision(
        candidate(position=(0.30, 0.0, 0.20)), points,
        target_mask=target_mask, open_aperture=open_aperture,
        pregrasp_standoff=0.10)

    assert not clear
    assert "fail-closed" in detail
    assert phrase in detail


def test_target_aware_collision_fails_closed_if_candidate_exceeds_opening():
    grasp = candidate(position=(0.30, 0.0, 0.20), width=0.060)
    points = np.array([[0.30, 0.0, 0.20]])

    clear, detail = target_aware_gripper_collision(
        grasp, points, target_mask=np.ones(1, dtype=bool),
        open_aperture=0.050, pregrasp_standoff=0.10)

    assert not clear
    assert "exceeds open aperture" in detail


def test_scene_collision_gate_runs_before_reachability():
    grasp = candidate()
    reachability_calls = []
    rejection = check(
        grasp, FilterConfig(),
        reachable_chain=lambda *_args: reachability_calls.append(True) or True,
        scene_collision_check=lambda _candidate: (False, "support occupied"))
    assert rejection.reason == "scene_collision"
    assert reachability_calls == []


def test_a_pregrasp_outside_the_workspace_is_rejected():
    # 40 cm up from a 0.05 m grasp leaves the 0.45 m ceiling
    cfg = FilterConfig(pregrasp_standoff=0.45)
    rejection = check(candidate(), cfg)
    assert rejection is not None and rejection.reason == "workspace"
    assert "pregrasp" in rejection.detail


def test_candidate_outside_selected_target_component_is_rejected():
    cfg = FilterConfig(target_min=(0.28, -0.02, 0.02),
                       target_max=(0.32, 0.02, 0.08),
                       target_margin=0.01)
    assert check(candidate(position=(0.30, 0.0, 0.05)), cfg) is None
    rejection = check(candidate(position=(0.30, 0.10, 0.05)), cfg)
    assert rejection is not None and rejection.reason == "off_target"


def test_filter_and_rank_sorts_survivors_by_score():
    accepted, rejected = filter_and_rank(
        [candidate(score=0.4), candidate(score=0.9), candidate(width=0.2)],
        FilterConfig())
    assert [c.score for c in accepted] == [0.9, 0.4]
    assert len(rejected) == 1


def test_best_near_miss_prefers_the_later_proven_gate_over_model_score():
    early = candidate(width=0.090, score=0.99)
    late = candidate(score=0.10)
    late.extras.update({
        "ik_pregrasp_position_error": 0.0031,
        "ik_pregrasp_orientation_error": 0.01,
    })
    early_rejection = Rejection("width", "too wide")
    late_rejection = Rejection("reachability", "pregrasp residual")

    result = best_near_miss(
        [(early, early_rejection), (late, late_rejection)], FilterConfig())

    assert result is not None
    assert result.candidate is late
    assert result.rejection is late_rejection
    assert result.gate_progress == 10


def test_best_near_miss_uses_normalized_residual_before_score_within_gate():
    close = candidate(score=0.10)
    close.extras.update({
        "ik_grasp_position_error": 0.0031,
        "ik_grasp_orientation_error": 0.01,
    })
    far = candidate(score=0.99)
    far.extras.update({
        "ik_grasp_position_error": 0.0060,
        "ik_grasp_orientation_error": 0.01,
    })

    result = best_near_miss([
        (far, Rejection("reachability", "far")),
        (close, Rejection("reachability", "close")),
    ], FilterConfig(), ik_position_tolerance=0.003,
        ik_orientation_tolerance=0.05)

    assert result is not None
    assert result.candidate is close
    assert result.gate_progress == 11
    assert result.violation == pytest.approx(0.0031 / 0.003)


def test_best_near_miss_uses_the_better_safe_post_grasp_alternative():
    near_retreat = candidate(score=0.20)
    near_retreat.extras.update({
        "ik_lift_position_error": 0.030,
        "ik_lift_orientation_error": 0.50,
        "ik_retreat_position_error": 0.0031,
        "ik_retreat_orientation_error": 0.01,
    })
    mediocre = candidate(score=0.90)
    mediocre.extras.update({
        "ik_lift_position_error": 0.006,
        "ik_lift_orientation_error": 0.10,
        "ik_retreat_position_error": 0.006,
        "ik_retreat_orientation_error": 0.10,
    })

    result = best_near_miss([
        (mediocre, Rejection("reachability", "both alternatives far")),
        (near_retreat, Rejection("reachability", "retreat nearly usable")),
    ], FilterConfig())

    assert result is not None
    assert result.candidate is near_retreat
    assert result.gate_progress == 12
    assert result.violation == pytest.approx(0.0031 / 0.003)


def test_best_near_miss_attaches_the_selected_collision_mask():
    grasp = candidate(position=(0.30, 0.0, 0.20), width=0.025)
    rotation = tool_rotation(grasp.approach, grasp.closing)
    local = np.array([
        [0.000, 0.020, 0.000],
        [0.002, 0.021, 0.001],
        [-0.002, 0.022, -0.001],
        [0.000, 0.000, 0.000],
    ])
    scene_points = grasp.position + local @ rotation.T

    result = best_near_miss(
        [(grasp, Rejection("scene_collision", "literal detail"))],
        FilterConfig(), scene_points=scene_points, collision_min_points=3)

    assert result is not None
    assert result.rejection.detail == "literal detail"
    assert result.gate_progress == 9
    assert result.violation == pytest.approx(1.0)
    assert result.collision_mask is not None
    assert result.collision_mask.tolist() == [True, True, True, False]


def test_best_near_miss_demotes_reachability_without_complete_finite_evidence():
    incomplete_ik = candidate(score=0.99)
    incomplete_ik.extras["ik_pregrasp_position_error"] = 0.0031
    collision = candidate(position=(0.32, 0.0, 0.20), score=0.10)
    rotation = tool_rotation(collision.approach, collision.closing)
    local = np.array([
        [0.000, 0.020, 0.000],
        [0.002, 0.021, 0.001],
        [-0.002, 0.022, -0.001],
    ])
    scene_points = collision.position + local @ rotation.T

    result = best_near_miss([
        (incomplete_ik, Rejection("reachability", "missing orientation")),
        (collision, Rejection("scene_collision", "three points")),
    ], FilterConfig(), scene_points=scene_points)

    assert result is not None
    assert result.candidate is collision
    assert math.isfinite(result.violation)


def test_best_near_miss_geometry_tie_break_is_input_order_independent():
    left = candidate(position=(0.25, 0.0, 0.05), width=0.090, score=0.5)
    right = candidate(position=(0.35, 0.0, 0.05), width=0.090, score=0.5)
    left_item = (left, Rejection("width", "same violation"))
    right_item = (right, Rejection("width", "same violation"))

    forward = best_near_miss([right_item, left_item], FilterConfig())
    reverse = best_near_miss([left_item, right_item], FilterConfig())

    assert forward is not None and reverse is not None
    assert forward.candidate is left
    assert reverse.candidate is left


def test_best_near_miss_of_no_rejections_is_none():
    assert best_near_miss([], FilterConfig()) is None


def test_rejection_summary_counts_reasons():
    _, rejected = filter_and_rank(
        [candidate(width=0.2), candidate(width=0.2),
         candidate(approach=(1.0, 0.0, -0.1))], FilterConfig())
    summary = rejection_summary(rejected)
    assert "widthx2" in summary and "tiltx1" in summary


def test_rejection_summary_of_nothing():
    assert rejection_summary([]) == "none rejected"


def test_max_tilt_is_honoured_at_the_boundary():
    cfg = FilterConfig(max_tilt=math.radians(45.0))
    tilted = candidate(approach=(math.sin(math.radians(30.0)), 0.0,
                                 -math.cos(math.radians(30.0))))
    assert check(tilted, cfg) is None
    steeper = candidate(approach=(math.sin(math.radians(60.0)), 0.0,
                                  -math.cos(math.radians(60.0))))
    assert check(steeper, cfg).reason == "tilt"


def test_nearest_to_pixel_prefers_a_containing_box_over_a_closer_centre():
    inside = candidate(pixel=(400.0, 240.0), bbox=(300, 200, 500, 280))
    close_by = candidate(pixel=(322.0, 240.0), bbox=(600, 600, 610, 610))
    ordered = nearest_to_pixel([close_by, inside], (320.0, 240.0), 90.0)
    assert ordered[0] is inside


def test_nearest_to_pixel_drops_candidates_beyond_the_radius():
    far = candidate(pixel=(10.0, 10.0), bbox=(0, 0, 20, 20))
    assert nearest_to_pixel([far], (600.0, 400.0), 50.0) == []


def test_nearest_to_pixel_skips_candidates_without_an_image_footprint():
    blind = candidate(pixel=None, bbox=None)
    assert nearest_to_pixel([blind], (320.0, 240.0), 500.0) == []
