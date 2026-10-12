"""Shared target-grasp selection helpers for viewing and execution.

Keeping these functions outside either ROS node prevents the vision-only viewer
and the motion node from silently choosing different grasps for the same frame.
"""

from __future__ import annotations

import math
from typing import List, Optional, Tuple

import numpy as np

from .grasp_backends import GraspCandidate, GraspScene
from .grasp_filter import Rejection
from .transforms import (approach_tilt_from_vertical, project,
                         rotation_distance, tool_rotation)


def candidate_pixel(candidate: GraspCandidate,
                    scene: GraspScene) -> Optional[Tuple[int, int]]:
    """Project a world-frame GraspNet candidate into the source RGB image."""
    optical = (np.asarray(scene.R_wc, dtype=float).T @ (
        np.asarray(candidate.position, dtype=float) - scene.p_wc))
    pixel = project(optical[np.newaxis], scene.intrinsics)[0]
    if not np.all(np.isfinite(pixel)):
        return None
    height, width = scene.color_image.shape[:2]
    if not (0 <= pixel[0] < width and 0 <= pixel[1] < height):
        return None
    return int(round(pixel[0])), int(round(pixel[1]))


def closest_parallel_jaw_orientation(
        candidate: GraspCandidate,
        reference_rotation: np.ndarray,
        ) -> Tuple[float, np.ndarray, bool]:
    """Return the nearest equivalent tool orientation to ``reference``.

    A parallel gripper is unchanged when its two identical fingers swap sides.
    Therefore ``closing`` and ``-closing`` describe the same grasp geometry,
    while producing tool frames separated by a 180-degree roll. The approach
    sign is deliberately not treated as symmetric: reversing it could approach
    through the object or table.
    """
    reference = np.asarray(reference_rotation, dtype=float).reshape(3, 3)
    closing = np.asarray(candidate.closing, dtype=float).reshape(3)
    normal_rotation = tool_rotation(candidate.approach, closing)
    flipped_rotation = tool_rotation(candidate.approach, -closing)
    normal_distance = rotation_distance(reference, normal_rotation)
    flipped_distance = rotation_distance(reference, flipped_rotation)
    if flipped_distance + 1e-9 < normal_distance:
        return flipped_distance, -closing, True
    return normal_distance, closing, False


def align_parallel_jaw_orientation(candidate: GraspCandidate,
                                   reference_rotation: np.ndarray) -> float:
    """Align one candidate's symmetric jaw axis and return its SO(3) delta.

    The motion node calls this before IK, ensuring reachability is checked for
    exactly the equivalent orientation that may later be executed.
    """
    distance, closing, flipped = closest_parallel_jaw_orientation(
        candidate, reference_rotation)
    was_flipped = bool(candidate.extras.get("closing_axis_flipped", 0.0))
    candidate.closing = closing
    candidate.extras["orientation_delta_rad"] = float(distance)
    candidate.extras["closing_axis_flipped"] = float(was_flipped != flipped)
    return float(distance)


def select_target_candidate(
        candidates: List[GraspCandidate], scene: GraspScene,
        target_pixel: Tuple[float, float], *, score_slack: float = 0.15,
        tilt_slack_rad: float = math.radians(10.0),
        reference_rotation: Optional[np.ndarray] = None,
        ) -> Optional[GraspCandidate]:
    """Choose one safe, high-quality grasp on Gemini's target.

    Candidates have already passed score, geometry, workspace and reachability
    safety filters. With a capture-time tool rotation, rank all survivors by
    the smallest full SO(3) wrist change, including parallel-jaw symmetry;
    model score and target-pixel distance break ties. If tool orientation is
    unavailable, retain the score-pool/near-vertical fallback.
    """
    if not candidates:
        return None
    target = np.asarray(target_pixel, dtype=float)
    best_score = max(float(candidate.score) for candidate in candidates)
    quality_floor = best_score - max(0.0, float(score_slack))
    ranked = []
    reference_ranked = []
    for candidate in candidates:
        approach = np.asarray(candidate.approach, dtype=float)
        approach /= max(float(np.linalg.norm(approach)), 1e-9)
        vertical_dot = float(np.clip(
            np.dot(approach, np.array([0.0, 0.0, -1.0])), -1.0, 1.0))
        tilt = math.acos(vertical_dot)
        pixel = candidate_pixel(candidate, scene)
        distance = (float("inf") if pixel is None else float(np.linalg.norm(
            np.asarray(pixel, dtype=float) - target)))
        if reference_rotation is not None:
            orientation_delta, aligned_closing, closing_flipped = \
                closest_parallel_jaw_orientation(
                    candidate, reference_rotation)
            reference_ranked.append((
                orientation_delta, -float(candidate.score), distance,
                candidate, aligned_closing, closing_flipped))
        if float(candidate.score) < quality_floor:
            continue
        ranked.append((tilt, distance, -float(candidate.score), candidate))
    if reference_ranked:
        reference_ranked.sort(key=lambda item: (item[0], item[1], item[2]))
        (orientation_delta, _negative_score, _distance, selected,
         _aligned_closing, _closing_flipped) = reference_ranked[0]
        # This is also safe when the execution node already aligned every
        # candidate before IK; the XOR state preserves the original jaw flip.
        align_parallel_jaw_orientation(selected, reference_rotation)
        selected.extras["orientation_delta_rad"] = float(orientation_delta)
        return selected
    if not ranked:
        return candidates[0]
    best_tilt = min(item[0] for item in ranked)
    orientation_limit = best_tilt + max(0.0, float(tilt_slack_rad))
    sensible = [item for item in ranked if item[0] <= orientation_limit]
    sensible.sort(key=lambda item: (item[1], item[2], item[0]))
    return sensible[0][3]


def candidate_diagnostic_summary(
        candidates: List[GraspCandidate],
        rejected: List[Tuple[GraspCandidate, Rejection]], *,
        max_items: int = 10,
        reference_rotation: Optional[np.ndarray] = None) -> str:
    """Compact per-candidate evidence for logs and status output."""
    rejection_by_id = {id(candidate): reason for candidate, reason in rejected}
    parts = []
    shown = max(0, int(max_items))
    for index, candidate in enumerate(candidates[:shown]):
        tilt_deg = math.degrees(
            approach_tilt_from_vertical(candidate.approach))
        width_mm = float(candidate.width) * 1000.0
        raw_width_mm = float(candidate.extras.get(
            "graspnet_width_raw", candidate.width)) * 1000.0
        width_text = f"{width_mm:.1f}mm"
        if raw_width_mm > width_mm + 0.05:
            width_text += f"(raw={raw_width_mm:.1f})"
        rejection = rejection_by_id.get(id(candidate))
        verdict = "OK" if rejection is None else f"REJECT:{rejection.reason}"
        orientation_text = ""
        if reference_rotation is not None:
            orientation_delta, _closing, _flipped = \
                closest_parallel_jaw_orientation(
                    candidate, reference_rotation)
            orientation_text = (
                f" dEE={math.degrees(orientation_delta):.1f}deg")
        post_text = ""
        post_mode = candidate.extras.get("post_grasp_mode")
        post_pos = candidate.extras.get("ik_post_grasp_position_error")
        post_ori = candidate.extras.get("ik_post_grasp_orientation_error")
        if post_mode is not None:
            post_text = f" post={post_mode}"
            if post_pos is not None and post_ori is not None:
                post_text += (
                    f" IK={float(post_pos) * 1000.0:.1f}mm/"
                    f"{math.degrees(float(post_ori)):.1f}deg")
        parts.append(
            f"#{index + 1} score={candidate.score:.3f} "
            f"width={width_text} tilt={tilt_deg:.1f}deg"
            f"{orientation_text} {verdict}{post_text}")
    if len(candidates) > shown:
        parts.append(f"+{len(candidates) - shown} more")
    return "; ".join(parts) if parts else "none"
