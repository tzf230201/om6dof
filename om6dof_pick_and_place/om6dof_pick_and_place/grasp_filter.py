"""Safety filtering and ranking of grasp candidates.

The ROBOTIS story filters on reachability, gripper width, table clearance and
approach angle before it picks the top-scoring survivor. Same four here, plus
the workspace box the other OM6DOF pick nodes already enforce, and one this arm
specifically needs: the pregrasp point has to be reachable too, otherwise the
arm plans into the object on its way to the standoff.

Everything is pure — no rclpy, no MoveIt — so the ranking is unit-testable and
the node stays a thin caller.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import (Callable, List, Mapping, Optional, Sequence, Tuple,
                    Union)

import numpy as np

from .grasp_backends import GraspCandidate
from .transforms import approach_tilt_from_vertical, tool_rotation

ReachabilityVerdict = Union[bool, Tuple[bool, str]]
ReachabilityChain = Callable[
    [np.ndarray, np.ndarray, GraspCandidate], ReachabilityVerdict
]
SceneCollisionCheck = Callable[[GraspCandidate], ReachabilityVerdict]


@dataclass(frozen=True)
class TargetAwareCollisionMasks:
    """Point classes used by the two-phase target-aware collision gate.

    ``approach_solid`` is occupancy of the open fingers or palm anywhere from
    pregrasp to grasp. ``environment_closure`` is non-target geometry swept by
    the fingers while they close at the final pose. ``target_outside_open``
    catches a target that does not fit through the measured open aperture.
    Only ``allowed_target_contact`` is deliberately ignored: those are exact
    target rows in the final, inner-finger closure band.
    """

    collision: np.ndarray
    approach_solid: np.ndarray
    environment_closure: np.ndarray
    target_outside_open: np.ndarray
    allowed_target_contact: np.ndarray


def target_aware_collision_input_error(
        scene_points: np.ndarray, target_mask: np.ndarray,
        open_aperture: float) -> Optional[str]:
    """Return why exact target-aware collision inputs are unusable.

    This is intentionally stricter than NumPy's normal coercion. A numeric or
    reshaped mask must never be interpreted as target provenance, because that
    could exempt arbitrary environment points from collision checking.
    """
    points = np.asarray(scene_points)
    if points.ndim != 2 or points.shape[1:] != (3,):
        return (f"scene_points has shape {points.shape}, expected (N, 3)")
    if not np.all(np.isfinite(points)):
        return "scene_points contains non-finite coordinates"
    mask = np.asarray(target_mask)
    if mask.dtype != np.bool_:
        return "target_mask must have bool dtype from exact source provenance"
    if mask.ndim != 1 or mask.shape != (points.shape[0],):
        return (f"target_mask has shape {mask.shape}, expected "
                f"({points.shape[0]},)")
    if not np.any(mask):
        return "target_mask is empty"
    try:
        aperture = float(open_aperture)
    except (TypeError, ValueError):
        return "open gripper aperture must be numeric"
    if not math.isfinite(aperture) or aperture <= 0.0:
        return ("open gripper aperture is unmeasured or invalid; supply the "
                "clear inner-jaw width in metres")
    return None


def target_aware_gripper_collision_masks(
        candidate: GraspCandidate, scene_points: np.ndarray, *,
        target_mask: np.ndarray, open_aperture: float,
        pregrasp_standoff: float,
        finger_back: float = 0.070,
        finger_front: float = 0.021,
        finger_thickness: float = 0.040,
        gripper_height: float = 0.058,
        contact_relief: float = 0.0015,
        margin: float = 0.002) -> TargetAwareCollisionMasks:
    """Classify occupancy for open approach followed by final jaw closure.

    The open-aperture geometry is the physical shape swept during approach.
    At the final pose, exact target points may occupy only the inner closure
    band. Environment points in that same band remain collisions, as do target
    points in the palm/rear or outside the clear open aperture.
    """
    error = target_aware_collision_input_error(
        scene_points, target_mask, open_aperture)
    if error is not None:
        raise ValueError(error)
    points = np.asarray(scene_points, dtype=float)
    target = np.asarray(target_mask, dtype=bool)
    aperture = float(open_aperture)
    width = float(candidate.width)
    if not math.isfinite(width) or width < 0.0:
        raise ValueError("candidate width must be finite and non-negative")
    if width > aperture + 1e-9:
        raise ValueError(
            f"candidate width {width:.6f} m exceeds open aperture "
            f"{aperture:.6f} m")

    centre = np.asarray(candidate.position, dtype=float).reshape(3)
    rotation = tool_rotation(candidate.approach, candidate.closing)
    local = (points - centre) @ rotation
    height_coord, closing_coord, approach_coord = local.T

    half_open = 0.5 * aperture
    half_closed = 0.5 * width
    front = max(float(finger_front), float(
        candidate.extras.get("graspnet_depth", finger_front)))
    back = max(0.0, float(finger_back))
    sweep = max(0.0, float(pregrasp_standoff))
    thickness = max(0.0, float(finger_thickness))
    half_height = 0.5 * max(0.0, float(gripper_height))
    relief = max(0.0, float(contact_relief))
    pad = max(0.0, float(margin))
    if not all(math.isfinite(value) for value in (
            front, back, sweep, thickness, half_height, relief, pad)):
        raise ValueError("gripper collision geometry must be finite")

    within_height = np.abs(height_coord) <= half_height + pad
    swept_approach = ((approach_coord >= -back - sweep - pad)
                      & (approach_coord <= front + pad))
    abs_closing = np.abs(closing_coord)

    # During approach the actual fingers remain at the open aperture. Expand
    # their solids inward by ``margin`` so an unmodelled clearance cannot turn
    # a grazing obstacle into a false pass.
    open_fingers = (within_height & swept_approach
                    & (abs_closing >= max(0.0, half_open - pad))
                    & (abs_closing <= half_open + thickness + pad))

    # The bridge/palm is always solid, including for target points. Its swept
    # union extends from the pregrasp pose to the final rear face.
    swept_palm = (
        within_height
        & (approach_coord >= -back - sweep - pad)
        & (approach_coord <= -back + thickness + pad)
        & (abs_closing <= half_open + thickness + pad))
    approach_solid = open_fingers | swept_palm

    # Only the finger length forward of the palm is a legitimate capture
    # region. The moving inner faces sweep from the open aperture toward the
    # candidate width at the final pose.
    final_contact_span = (
        within_height
        & (approach_coord > -back + thickness + pad)
        & (approach_coord <= front + pad))
    closure_band = (
        final_contact_span
        & (abs_closing >= max(0.0, half_closed - relief))
        & (abs_closing < max(0.0, half_open - pad)))
    environment_closure = (~target) & closure_band

    # A target surface beyond the usable clear gap cannot be excused merely
    # because it carries the right semantic label.
    target_outside_open = (
        target & final_contact_span
        & (abs_closing >= max(0.0, half_open - pad)))
    allowed_target_contact = (
        target & closure_band & ~approach_solid & ~target_outside_open)
    collision = approach_solid | environment_closure | target_outside_open
    return TargetAwareCollisionMasks(
        collision=collision,
        approach_solid=approach_solid,
        environment_closure=environment_closure,
        target_outside_open=target_outside_open,
        allowed_target_contact=allowed_target_contact)


def target_aware_gripper_collision(
        candidate: GraspCandidate, scene_points: np.ndarray, *,
        target_mask: np.ndarray, open_aperture: float,
        pregrasp_standoff: float,
        finger_back: float = 0.070,
        finger_front: float = 0.021,
        finger_thickness: float = 0.040,
        gripper_height: float = 0.058,
        contact_relief: float = 0.0015,
        margin: float = 0.002,
        min_points: int = 3) -> Tuple[bool, str]:
    """Fail-closed target-aware collision verdict with inspectable counts."""
    try:
        masks = target_aware_gripper_collision_masks(
            candidate, scene_points, target_mask=target_mask,
            open_aperture=open_aperture,
            pregrasp_standoff=pregrasp_standoff,
            finger_back=finger_back, finger_front=finger_front,
            finger_thickness=finger_thickness,
            gripper_height=gripper_height,
            contact_relief=contact_relief, margin=margin)
    except (TypeError, ValueError) as exc:
        return False, f"target-aware collision unavailable (fail-closed): {exc}"

    counts = {
        "approach/palm": int(np.count_nonzero(masks.approach_solid)),
        "environment-closure": int(np.count_nonzero(
            masks.environment_closure)),
        "target-outside-open": int(np.count_nonzero(
            masks.target_outside_open)),
        "allowed-target-contact": int(np.count_nonzero(
            masks.allowed_target_contact)),
    }
    occupied = int(np.count_nonzero(masks.collision))
    required = max(1, int(min_points))
    detail = ", ".join(f"{name}={count}" for name, count in counts.items())
    if occupied >= required:
        return False, (
            f"{occupied} points violate target-aware gripper geometry "
            f"(limit {required - 1}; {detail})")
    return True, (
        f"target-aware gripper geometry clear ({occupied} collision points; "
        f"{detail})")


def gripper_collision_mask(
        candidate: GraspCandidate, scene_points: np.ndarray, *,
        pregrasp_standoff: float,
        finger_back: float = 0.070,
        finger_front: float = 0.021,
        finger_thickness: float = 0.040,
        gripper_height: float = 0.058,
        contact_relief: float = 0.0015,
        margin: float = 0.002) -> np.ndarray:
    """Return the scene-point mask inside the swept physical gripper solid.

    Candidate directions map onto the robot tool frame as approach=+Z,
    closing=+Y, and the remaining jaw-height axis=+X.  The defaults
    conservatively bound the installed left/right STL meshes about
    ``end_effector_link``.  Keeping this geometry in one pure function ensures
    the safety verdict and RViz collision-point diagnostics use exactly the
    same envelope.
    """
    points = np.asarray(scene_points, dtype=float).reshape(-1, 3)
    if points.size == 0:
        return np.zeros(points.shape[0], dtype=bool)
    centre = np.asarray(candidate.position, dtype=float).reshape(3)
    rotation = tool_rotation(candidate.approach, candidate.closing)
    local = (points - centre) @ rotation
    height_coord, closing_coord, approach_coord = local.T

    half_gap = max(0.0, float(candidate.width)) * 0.5
    front = max(float(finger_front), float(
        candidate.extras.get("graspnet_depth", finger_front)))
    back = max(0.0, float(finger_back))
    sweep = max(0.0, float(pregrasp_standoff))
    thickness = max(0.0, float(finger_thickness))
    half_height = 0.5 * max(0.0, float(gripper_height))
    pad = max(0.0, float(margin))
    relief = max(0.0, float(contact_relief))

    within_height = np.abs(height_coord) <= half_height + pad
    within_sweep = ((approach_coord >= -back - sweep - pad)
                    & (approach_coord <= front + pad))
    # Preserve the intended empty aperture and a small contact band at its
    # boundary; points farther into either physical finger are collisions.
    left_finger = ((closing_coord >= half_gap + relief)
                   & (closing_coord <= half_gap + thickness + pad))
    right_finger = ((closing_coord <= -half_gap - relief)
                    & (closing_coord >= -half_gap - thickness - pad))
    side_collision = within_height & within_sweep & (
        left_finger | right_finger)

    # The bridge/palm spans the complete jaw width behind the fingertips and
    # sweeps forward from the pregrasp pose along the approach axis.
    palm_approach = ((approach_coord >= -back - sweep - pad)
                     & (approach_coord <= -back + thickness + pad))
    palm_closing = np.abs(closing_coord) <= half_gap + thickness + pad
    palm_collision = within_height & palm_approach & palm_closing

    return side_collision | palm_collision


def conservative_gripper_collision(
        candidate: GraspCandidate, scene_points: np.ndarray, *,
        pregrasp_standoff: float,
        finger_back: float = 0.070,
        finger_front: float = 0.021,
        finger_thickness: float = 0.040,
        gripper_height: float = 0.058,
        contact_relief: float = 0.0015,
        margin: float = 0.002,
        min_points: int = 3,
        target_mask: Optional[np.ndarray] = None,
        open_aperture: Optional[float] = None) -> Tuple[bool, str]:
    """Reject point occupancy in the OM6DOF gripper's swept solid envelope.

    Unlike a volume-ratio test, a thin support surface cannot disappear into a
    large denominator: a few independent points inside solid hardware reject
    the pose.  :func:`gripper_collision_mask` exposes those same points for
    read-only diagnostics without changing this verdict.
    """
    if target_mask is not None or open_aperture is not None:
        if target_mask is None or open_aperture is None:
            return False, (
                "target-aware collision unavailable (fail-closed): both "
                "target_mask and open_aperture are required")
        return target_aware_gripper_collision(
            candidate, scene_points, target_mask=target_mask,
            open_aperture=open_aperture,
            pregrasp_standoff=pregrasp_standoff,
            finger_back=finger_back, finger_front=finger_front,
            finger_thickness=finger_thickness,
            gripper_height=gripper_height,
            contact_relief=contact_relief, margin=margin,
            min_points=min_points)

    mask = gripper_collision_mask(
        candidate, scene_points, pregrasp_standoff=pregrasp_standoff,
        finger_back=finger_back, finger_front=finger_front,
        finger_thickness=finger_thickness, gripper_height=gripper_height,
        contact_relief=contact_relief, margin=margin)
    count = int(np.count_nonzero(mask))
    required = max(1, int(min_points))
    if count >= required:
        return False, (
            f"{count} scene points occupy the swept gripper envelope "
            f"(limit {required - 1})")
    return True, f"swept gripper envelope contains {count} scene points"


@dataclass
class FilterConfig:
    min_width: float = 0.010
    max_width: float = 0.065
    table_z: float = 0.0
    min_clearance: float = 0.005
    max_tilt: float = 1.20              # rad from straight down
    workspace_min: Sequence[float] = (0.08, -0.35, -0.05)
    workspace_max: Sequence[float] = (0.50, 0.35, 0.45)
    pregrasp_standoff: float = 0.10
    min_score: float = 0.0
    # Optional robust bounds of the selected target component.  They are left
    # unset for ordinary scene-wide picking and filled by target-mode viewers.
    target_min: Optional[Sequence[float]] = None
    target_max: Optional[Sequence[float]] = None
    target_margin: float = 0.020


@dataclass
class Rejection:
    """Why one candidate was dropped — logged so tuning has something to read."""
    reason: str
    detail: str = ""


@dataclass
class NearMiss:
    """One rejected candidate selected strictly for read-only diagnostics."""
    candidate: GraspCandidate
    rejection: Rejection
    gate_progress: int
    violation: float
    collision_mask: Optional[np.ndarray] = None


def _outside_box_violation(point: np.ndarray, low: Sequence[float],
                           high: Sequence[float]) -> float:
    """Scale-invariant Euclidean distance outside an axis-aligned box."""
    value = np.asarray(point, dtype=float)
    lower = np.asarray(low, dtype=float)
    upper = np.asarray(high, dtype=float)
    span = np.maximum(upper - lower, 1e-9)
    outside = np.maximum(lower - value, 0.0) + np.maximum(value - upper, 0.0)
    return float(np.linalg.norm(outside / span))


def _ik_violation(candidate: GraspCandidate, stage: str,
                  position_tolerance: float,
                  orientation_tolerance: float) -> float:
    position = candidate.extras.get(f"ik_{stage}_position_error")
    orientation = candidate.extras.get(f"ik_{stage}_orientation_error")
    try:
        position = float(position)
        orientation = float(orientation)
    except (TypeError, ValueError):
        return math.inf
    if not math.isfinite(position) or not math.isfinite(orientation):
        return math.inf
    return max(position / max(float(position_tolerance), 1e-9),
               orientation / max(float(orientation_tolerance), 1e-9))


def best_near_miss(
        rejected: Sequence[Tuple[GraspCandidate, Rejection]],
        cfg: "FilterConfig", *, scene_points: Optional[np.ndarray] = None,
        collision_kwargs: Optional[Mapping[str, object]] = None,
        collision_min_points: int = 3,
        ik_position_tolerance: float = 0.003,
        ik_orientation_tolerance: float = 0.05) -> Optional[NearMiss]:
    """Choose one deterministic near miss without changing safety results.

    A later first-fail gate is preferred because that candidate is proven to
    have passed more preceding checks. Within the same gate, the smaller
    normalized violation wins, followed by backend score and a stable geometry
    fingerprint. A reachability failure without numeric IK evidence is demoted
    below a collision candidate whose occupancy can be measured.
    """
    if not rejected:
        return None
    collision_options = dict(collision_kwargs or {})
    collision_options.setdefault("pregrasp_standoff",
                                 float(cfg.pregrasp_standoff))
    scene = (None if scene_points is None else
             np.asarray(scene_points, dtype=float).reshape(-1, 3))
    choices = []
    width_span = max(float(cfg.max_width) - float(cfg.min_width), 1e-9)

    for ordinal, (candidate, rejection) in enumerate(rejected):
        reason = str(rejection.reason)
        position = np.asarray(candidate.position, dtype=float)
        motion_position = candidate.motion_position()
        pregrasp = candidate.pregrasp(float(cfg.pregrasp_standoff))
        collision_mask = None
        progress = -1
        violation = math.inf

        if reason == "score":
            progress = 0
            violation = max(0.0, float(cfg.min_score) - float(candidate.score))
        elif reason == "width":
            progress = 1
            width = float(candidate.width)
            distance = (float(cfg.min_width) - width if width < cfg.min_width
                        else width - float(cfg.max_width))
            violation = max(0.0, distance) / width_span
        elif reason == "clearance":
            center_clearance = float(position[2]) - float(cfg.table_z)
            tcp_clearance = float(motion_position[2]) - float(cfg.table_z)
            if center_clearance < cfg.min_clearance:
                progress, measured = 2, center_clearance
            else:
                progress, measured = 6, tcp_clearance
            violation = max(0.0, float(cfg.min_clearance) - measured) / max(
                float(cfg.min_clearance), 1e-9)
        elif reason == "tilt":
            progress = 3
            tilt = approach_tilt_from_vertical(candidate.approach)
            violation = max(0.0, tilt - float(cfg.max_tilt)) / max(
                float(cfg.max_tilt), 1e-9)
        elif reason == "workspace":
            center_violation = _outside_box_violation(
                position, cfg.workspace_min, cfg.workspace_max)
            tcp_violation = _outside_box_violation(
                motion_position, cfg.workspace_min, cfg.workspace_max)
            pregrasp_violation = _outside_box_violation(
                pregrasp, cfg.workspace_min, cfg.workspace_max)
            if center_violation > 0.0:
                progress, violation = 4, center_violation
            elif tcp_violation > 0.0:
                progress, violation = 5, tcp_violation
            else:
                progress, violation = 8, pregrasp_violation
        elif reason == "off_target":
            progress = 7
            if cfg.target_min is not None and cfg.target_max is not None:
                low = (np.asarray(cfg.target_min, dtype=float)
                       - float(cfg.target_margin))
                high = (np.asarray(cfg.target_max, dtype=float)
                        + float(cfg.target_margin))
                violation = _outside_box_violation(position, low, high)
        elif reason == "scene_collision":
            progress = 9
            if scene is not None:
                target_mask = collision_options.get("target_mask")
                open_aperture = collision_options.get("open_aperture")
                geometry_options = {
                    name: value for name, value in collision_options.items()
                    if name not in {"target_mask", "open_aperture"}
                }
                if target_mask is None and open_aperture is None:
                    collision_mask = gripper_collision_mask(
                        candidate, scene, **geometry_options)
                elif target_mask is not None and open_aperture is not None:
                    collision_mask = target_aware_gripper_collision_masks(
                        candidate, scene, target_mask=target_mask,
                        open_aperture=float(open_aperture),
                        **geometry_options).collision
                else:
                    raise ValueError(
                        "both target_mask and open_aperture are required for "
                        "target-aware collision diagnostics")
                violation = float(np.count_nonzero(collision_mask)) / max(
                    1, int(collision_min_points))
        elif reason == "reachability":
            if ("ik_retreat_position_error" in candidate.extras
                    or "ik_lift_position_error" in candidate.extras):
                progress = 12
                violation = min(
                    _ik_violation(candidate, "lift", ik_position_tolerance,
                                  ik_orientation_tolerance),
                    _ik_violation(candidate, "retreat", ik_position_tolerance,
                                  ik_orientation_tolerance))
            elif "ik_grasp_position_error" in candidate.extras:
                progress = 11
                violation = _ik_violation(
                    candidate, "grasp", ik_position_tolerance,
                    ik_orientation_tolerance)
            elif "ik_pregrasp_position_error" in candidate.extras:
                progress = 10
                violation = _ik_violation(
                    candidate, "pregrasp", ik_position_tolerance,
                    ik_orientation_tolerance)
            else:
                # No numeric proof (for example, stale joint state): a finite
                # collision diagnosis is more useful than this as a near miss.
                progress = 8
            # A lone or non-finite residual is not numeric proof of closeness.
            # Demote it exactly like missing IK data so this read-only marker
            # cannot imply that an unmeasured pose was almost executable.
            if not math.isfinite(violation):
                progress = 8

        score = float(candidate.score)
        score_key = -score if math.isfinite(score) else math.inf

        def finite_geometry(values) -> Tuple[float, ...]:
            output = []
            for value in np.asarray(values, dtype=float).reshape(-1):
                output.append(round(float(value), 9)
                              if math.isfinite(float(value)) else math.inf)
            return tuple(output)

        fingerprint = (
            finite_geometry(position)
            + finite_geometry(candidate.approach)
            + finite_geometry(candidate.closing)
            + finite_geometry([candidate.width]))
        key = (-progress, violation, score_key, fingerprint, ordinal)
        choices.append((key, NearMiss(
            candidate=candidate, rejection=rejection,
            gate_progress=progress, violation=violation,
            collision_mask=collision_mask)))

    return min(choices, key=lambda item: item[0])[1]


def _in_box(point: np.ndarray, low: Sequence[float],
            high: Sequence[float]) -> bool:
    point = np.asarray(point, dtype=float)
    return bool(np.all(point >= np.asarray(low, dtype=float))
                and np.all(point <= np.asarray(high, dtype=float)))


def check(candidate: GraspCandidate, cfg: FilterConfig,
          reachable: Optional[Callable[[np.ndarray, GraspCandidate], bool]] = None,
          reachable_chain: Optional[ReachabilityChain] = None,
          scene_collision_check: Optional[SceneCollisionCheck] = None,
          ) -> Optional[Rejection]:
    """Return ``None`` if the candidate is safe, else why it is not."""
    if candidate.score < cfg.min_score:
        return Rejection("score", f"{candidate.score:.3f} < {cfg.min_score:.3f}")
    if not (cfg.min_width <= candidate.width <= cfg.max_width):
        return Rejection("width", f"{candidate.width * 1000:.0f} mm outside "
                                  f"[{cfg.min_width * 1000:.0f}, "
                                  f"{cfg.max_width * 1000:.0f}] mm")

    # ``position`` is the geometric grasp centre used for object association
    # and collision geometry.  AnyGrasp separately documents a final gripper
    # tip/TCP target, exposed by motion_position().  Both must be safe.
    position = np.asarray(candidate.position, dtype=float)
    motion_position = candidate.motion_position()
    clearance = float(position[2]) - cfg.table_z
    if clearance < cfg.min_clearance:
        return Rejection("clearance", f"z {position[2]:.3f} is {clearance * 1000:.0f} "
                                      f"mm above the table")

    tilt = approach_tilt_from_vertical(candidate.approach)
    if tilt > cfg.max_tilt:
        return Rejection("tilt", f"{math.degrees(tilt):.0f} deg from vertical "
                                 f"> {math.degrees(cfg.max_tilt):.0f} deg")

    if not _in_box(position, cfg.workspace_min, cfg.workspace_max):
        return Rejection("workspace", f"grasp {np.round(position, 3).tolist()}")
    if not _in_box(motion_position, cfg.workspace_min, cfg.workspace_max):
        return Rejection(
            "workspace",
            f"grasp TCP {np.round(motion_position, 3).tolist()}")
    tcp_clearance = float(motion_position[2]) - cfg.table_z
    if tcp_clearance < cfg.min_clearance:
        return Rejection(
            "clearance",
            f"grasp TCP z {motion_position[2]:.3f} is "
            f"{tcp_clearance * 1000:.0f} mm above the table")

    if cfg.target_min is not None and cfg.target_max is not None:
        target_low = (np.asarray(cfg.target_min, dtype=float)
                      - float(cfg.target_margin))
        target_high = (np.asarray(cfg.target_max, dtype=float)
                       + float(cfg.target_margin))
        if not _in_box(position, target_low, target_high):
            return Rejection(
                "off_target",
                f"grasp {np.round(position, 3).tolist()} outside target bounds")

    pregrasp = candidate.pregrasp(cfg.pregrasp_standoff)
    if not _in_box(pregrasp, cfg.workspace_min, cfg.workspace_max):
        return Rejection("workspace", f"pregrasp {np.round(pregrasp, 3).tolist()}")

    if scene_collision_check is not None:
        verdict = scene_collision_check(candidate)
        if isinstance(verdict, tuple):
            clear, detail = bool(verdict[0]), str(verdict[1])
        else:
            clear, detail = bool(verdict), "swept gripper collision"
        if not clear:
            return Rejection("scene_collision", detail)

    if reachable_chain is not None:
        verdict = reachable_chain(pregrasp, motion_position, candidate)
        if isinstance(verdict, tuple):
            ok, detail = bool(verdict[0]), str(verdict[1])
        else:
            ok, detail = bool(verdict), "pregrasp/grasp IK chain failed"
        if not ok:
            return Rejection("reachability", detail)
    elif reachable is not None:
        if not reachable(pregrasp, candidate):
            return Rejection("reachability", "pregrasp has no IK solution")
        if not reachable(motion_position, candidate):
            return Rejection("reachability", "grasp has no IK solution")
    return None


def filter_and_rank(candidates: Sequence[GraspCandidate], cfg: FilterConfig,
                    reachable: Optional[Callable[[np.ndarray, GraspCandidate], bool]] = None,
                    reachable_chain: Optional[ReachabilityChain] = None,
                    scene_collision_check: Optional[SceneCollisionCheck] = None,
                    ) -> Tuple[List[GraspCandidate], List[Tuple[GraspCandidate, Rejection]]]:
    """Split candidates into (accepted, highest score first) and (rejected, why)."""
    accepted: List[GraspCandidate] = []
    rejected: List[Tuple[GraspCandidate, Rejection]] = []
    for candidate in candidates:
        rejection = check(candidate, cfg, reachable, reachable_chain,
                          scene_collision_check)
        if rejection is None:
            accepted.append(candidate)
        else:
            rejected.append((candidate, rejection))
    accepted.sort(key=lambda c: -c.score)
    return accepted, rejected


def rejection_summary(rejected: Sequence[Tuple[GraspCandidate, Rejection]]) -> str:
    """One line naming how many candidates each reason accounted for."""
    counts: dict = {}
    for _, rejection in rejected:
        counts[rejection.reason] = counts.get(rejection.reason, 0) + 1
    if not counts:
        return "none rejected"
    return ", ".join(f"{reason}x{count}" for reason, count
                     in sorted(counts.items(), key=lambda kv: -kv[1]))


def nearest_to_pixel(candidates: Sequence[GraspCandidate],
                     pixel: Tuple[float, float],
                     max_distance_px: float) -> List[GraspCandidate]:
    """Keep candidates whose image footprint sits near a pixel Gemini named.

    A candidate whose bounding box contains the pixel wins over one that is
    merely close to it, so a small object inside a larger one is still pickable.
    """
    target = np.asarray(pixel, dtype=float)
    scored = []
    for candidate in candidates:
        if candidate.pixel is None:
            continue
        distance = float(np.linalg.norm(np.asarray(candidate.pixel) - target))
        inside = False
        if candidate.bbox is not None:
            x0, y0, x1, y1 = candidate.bbox
            inside = (x0 <= target[0] <= x1) and (y0 <= target[1] <= y1)
        if inside or distance <= float(max_distance_px):
            scored.append((0 if inside else 1, distance, candidate))
    scored.sort(key=lambda item: (item[0], item[1]))
    return [candidate for _, _, candidate in scored]
