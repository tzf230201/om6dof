"""Grasp proposal backends.

Both backends answer the same question — *where can the jaws close?* — and
return the same :class:`GraspCandidate` list in the arm base frame.

``analytic``
    Pure numpy. Segments the table off, clusters what is left on a voxel grid,
    and proposes a top-down-ish grasp across each cluster's narrow horizontal
    axis. No GPU, no model download, no network. It handles the separated
    objects on a table that the OMY story demonstrates; it does not reason about
    clutter or occlusion the way a learned model does.

``graspnet``
    Adapter for `graspnet-baseline <https://github.com/graspnet/graspnet-baseline>`_
    (MVIG-SJTU), the model the ROBOTIS story runs. It needs torch with CUDA, the
    repo's compiled ``pointnet2``/``knn`` ops and a checkpoint. The launch files
    select this backend by default on this Orin and load its isolated runtime
    through ``activate_om6dof_graspnet.sh``. Imports remain lazy so an
    unconfigured machine gets an actionable availability error rather than a
    startup crash.

``anygrasp``
    Adapter for the licensed AnyGrasp SDK.  Target mode gives it the complete
    scene plus an exactly aligned ``region_steering`` mask: the mask guides
    proposal generation, while the complete cloud remains available to the
    SDK's collision detector.
"""

from __future__ import annotations

import importlib
import math
import os
import sys
import threading
from argparse import Namespace
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np

from .transforms import directions_to_base, points_to_base

DOWN = np.array([0.0, 0.0, -1.0])


@dataclass
class GraspCandidate:
    """One proposed grasp, expressed the way the tool frame wants it.

    ``position`` is the geometric grasp centre (the midpoint between the
    fingertips). ``approach`` is tool +Z and ``closing`` is tool +Y (the
    direction the fingers travel).  Backends whose motion target differs from
    that centre put it in ``extras['tcp_position']``; target association and
    collision geometry deliberately continue to use the centre.
    """
    position: np.ndarray
    approach: np.ndarray
    closing: np.ndarray
    width: float
    score: float
    frame: str = "world"
    support: int = 0
    pixel: Optional[Tuple[float, float]] = None
    bbox: Optional[Tuple[int, int, int, int]] = None
    source: str = ""
    extras: Dict[str, object] = field(default_factory=dict)

    def motion_position(self) -> np.ndarray:
        """Return the TCP point the robot should reach for this proposal."""
        value = self.extras.get("tcp_position", self.position)
        point = np.asarray(value, dtype=float)
        if point.shape != (3,) or not np.all(np.isfinite(point)):
            raise ValueError("candidate tcp_position must be three finite values")
        return point

    def pregrasp(self, standoff: float) -> np.ndarray:
        """Point ``standoff`` metres back along the approach direction."""
        return self.motion_position() \
            - float(standoff) * np.asarray(self.approach, dtype=float)


@dataclass
class GraspScene:
    """One capture, already available in both frames the backends want."""
    points_optical: np.ndarray
    points_base: np.ndarray
    pixels: np.ndarray
    colors: Optional[np.ndarray]
    p_wc: np.ndarray
    R_wc: np.ndarray
    intrinsics: Tuple[float, float, float, float]
    color_image: Optional[np.ndarray] = None
    base_frame: str = "world"
    tool_rotation_base: Optional[np.ndarray] = None
    # Stable row IDs from the original post-self-exclusion capture.  AnyGrasp
    # region steering requires exact point/mask correspondence; reconstructing
    # it later by approximate XYZ equality is unsafe around duplicate pixels.
    source_indices: Optional[np.ndarray] = None


def _rodrigues(axis: np.ndarray, angle: float) -> np.ndarray:
    axis = np.asarray(axis, dtype=float)
    n = float(np.linalg.norm(axis))
    if n < 1e-9 or abs(angle) < 1e-12:
        return np.eye(3)
    k = axis / n
    K = np.array([[0.0, -k[2], k[1]], [k[2], 0.0, -k[0]], [-k[1], k[0], 0.0]])
    return np.eye(3) + math.sin(angle) * K + (1.0 - math.cos(angle)) * (K @ K)


def _sample_point_indices(point_count: int, sample_count: int,
                          seed: int = 0) -> np.ndarray:
    """Choose a deterministic, balanced GraspNet input point set.

    Balanced tiling is retained for unusually sparse captures, but normal
    target-mode inference passes the complete scene here. This keeps support
    and neighbouring-object geometry in GraspNet's input rather than making a
    20,000-point cloud from only a few hundred target pixels.
    """
    point_count, sample_count = int(point_count), int(sample_count)
    if point_count <= 0 or sample_count <= 0:
        return np.zeros(0, dtype=np.int64)
    rng = np.random.default_rng(int(seed))
    if point_count >= sample_count:
        return rng.choice(point_count, sample_count, replace=False)
    repeats, remainder = divmod(sample_count, point_count)
    indices = np.tile(np.arange(point_count, dtype=np.int64), repeats)
    if remainder:
        # Spread the extra repeats across the cloud instead of favouring its
        # first camera-raster rows.
        extras = np.linspace(
            0, point_count - 1, remainder, dtype=np.int64)
        indices = np.concatenate([indices, extras])
    rng.shuffle(indices)
    return indices


def self_exclusion_mask(points_base: np.ndarray, center: np.ndarray,
                        radius: float) -> np.ndarray:
    """True for points further than ``radius`` from ``center`` — not the gripper.

    The D405 is wrist-mounted close enough to the jaws that, with no filter,
    the gripper's own fingers are the nearest, cleanest, most cluster-shaped
    thing in every frame — and score *higher* than any real object, since they
    sit exactly jaw-width apart. Measured on hardware 2026-09-02: the
    highest-scoring "candidates" sat 3.3-5.1 cm from ``end_effector_link``,
    consistently failed IK (the arm cannot approach its own current gripper
    position from a new direction), and were mistaken for two stray objects
    across an entire debugging session before this filter existed.

    ``center`` is FK of the current joint state — cheap, and needs no
    camera-specific geometry, since any point near the tool origin is self
    regardless of which extrinsic produced the cloud. Apply the returned mask
    to every per-point array (optical points, base points, pixels, colours) so
    they all stay aligned.
    """
    points_base = np.asarray(points_base, dtype=float).reshape(-1, 3)
    if points_base.shape[0] == 0:
        return np.zeros(0, dtype=bool)
    return np.linalg.norm(points_base - np.asarray(center, dtype=float),
                          axis=1) > float(radius)


def crop_to_bbox(scene: "GraspScene", bbox: Tuple[float, float, float, float],
                 pad_px: float = 40.0) -> Optional["GraspScene"]:
    """Keep only the points whose source pixel falls inside ``bbox`` (+ pad).

    This crop is an association and visualisation aid. It must not replace the
    complete scene passed to GraspNet because that would remove surrounding
    objects and support geometry from proposal generation.
    """
    pixels = np.asarray(scene.pixels, dtype=float).reshape(-1, 2)
    if pixels.shape[0] != scene.points_optical.shape[0]:
        return None
    x0, y0, x1, y1 = [float(v) for v in bbox]
    x0, y0 = x0 - pad_px, y0 - pad_px
    x1, y1 = x1 + pad_px, y1 + pad_px
    finite = np.all(np.isfinite(pixels), axis=1)
    inside = (finite & (pixels[:, 0] >= x0) & (pixels[:, 0] <= x1)
              & (pixels[:, 1] >= y0) & (pixels[:, 1] <= y1))
    if not np.any(inside):
        return None
    colors = scene.colors[inside] if scene.colors is not None else None
    return GraspScene(points_optical=scene.points_optical[inside],
                      points_base=scene.points_base[inside],
                      pixels=pixels[inside], colors=colors,
                      p_wc=scene.p_wc, R_wc=scene.R_wc,
                      intrinsics=scene.intrinsics,
                      color_image=scene.color_image,
                      base_frame=scene.base_frame,
                      tool_rotation_base=scene.tool_rotation_base,
                      source_indices=(
                          np.asarray(scene.source_indices)[inside]
                          if scene.source_indices is not None else None))


def _masked_scene(scene: "GraspScene", mask: np.ndarray) -> "GraspScene":
    """Return a point-wise subset while keeping every scene array aligned."""
    keep = np.asarray(mask, dtype=bool).reshape(-1)
    point_count = np.asarray(scene.points_optical).reshape(-1, 3).shape[0]
    if keep.shape != (point_count,):
        raise ValueError(
            f"scene subset mask has shape {keep.shape}, expected ({point_count},)")
    colors = scene.colors[keep] if scene.colors is not None else None
    return GraspScene(points_optical=scene.points_optical[keep],
                      points_base=scene.points_base[keep],
                      pixels=scene.pixels[keep], colors=colors,
                      p_wc=scene.p_wc, R_wc=scene.R_wc,
                      intrinsics=scene.intrinsics,
                      color_image=scene.color_image,
                      base_frame=scene.base_frame,
                      tool_rotation_base=scene.tool_rotation_base,
                      source_indices=(
                          np.asarray(scene.source_indices)[keep]
                          if scene.source_indices is not None else None))


def crop_to_workspace(scene: "GraspScene", workspace_min,
                      workspace_max, *, margin: float = 0.0) -> "GraspScene":
    """Keep the complete observed geometry inside the robot workspace.

    Unlike target segmentation this is still a scene: the target, its support,
    and neighbouring objects all remain available to GraspNet. It only removes
    walls/furniture that the robot cannot reach and that would otherwise
    dominate point sampling and proposal generation.
    """
    low = np.asarray(workspace_min, dtype=float).reshape(3) - max(
        0.0, float(margin))
    high = np.asarray(workspace_max, dtype=float).reshape(3) + max(
        0.0, float(margin))
    points = np.asarray(scene.points_base, dtype=float).reshape(-1, 3)
    finite = np.all(np.isfinite(points), axis=1)
    inside = finite & np.all(points >= low, axis=1) & np.all(
        points <= high, axis=1)
    return _masked_scene(scene, inside)


def target_region_mask(scene: "GraspScene",
                       target_scene: "GraspScene") -> np.ndarray:
    """Map an exact target subset back onto ``scene`` for AnyGrasp steering.

    Both scenes must carry stable capture-row IDs.  Every target ID must occur
    in the network scene, and the result must contain at least one point.  The
    function raises instead of returning an all-false mask because the current
    AnyGrasp SDK treats an empty region mask as *steering disabled*, which could
    silently select an unrelated object.
    """
    scene_count = np.asarray(scene.points_optical).reshape(-1, 3).shape[0]
    target_count = np.asarray(target_scene.points_optical).reshape(-1, 3).shape[0]
    if scene.source_indices is None or target_scene.source_indices is None:
        raise ValueError(
            "source_indices are required for exact AnyGrasp region steering")
    scene_ids = np.asarray(scene.source_indices)
    target_ids = np.asarray(target_scene.source_indices)
    if scene_ids.shape != (scene_count,):
        raise ValueError(
            f"scene source_indices has shape {scene_ids.shape}, expected "
            f"({scene_count},)")
    if target_ids.shape != (target_count,):
        raise ValueError(
            f"target source_indices has shape {target_ids.shape}, expected "
            f"({target_count},)")
    if np.asarray(scene.points_base).shape != (scene_count, 3):
        raise ValueError("scene points_base is not aligned with points_optical")
    if np.asarray(target_scene.points_base).shape != (target_count, 3):
        raise ValueError(
            "target points_base is not aligned with points_optical")
    if target_count == 0:
        raise ValueError("target region is empty")
    if scene_ids.dtype.kind not in "iu" or target_ids.dtype.kind not in "iu":
        raise ValueError("source_indices must use an integer dtype")
    if np.unique(scene_ids).shape[0] != scene_ids.shape[0]:
        raise ValueError("scene source_indices must be unique")
    if np.unique(target_ids).shape[0] != target_ids.shape[0]:
        raise ValueError("target source_indices must be unique")
    missing = target_ids[~np.isin(target_ids, scene_ids)]
    if missing.size:
        raise ValueError(
            f"target region contains {missing.size} source IDs outside scene")
    # IDs are only trustworthy when they still name the exact capture rows.
    # Verify both coordinate arrays so a stale/reordered mask cannot exempt an
    # unrelated obstacle from target-aware collision filtering.
    scene_row_by_id = {
        int(source_id): row for row, source_id in enumerate(scene_ids)}
    mapped_rows = np.asarray(
        [scene_row_by_id[int(source_id)] for source_id in target_ids],
        dtype=np.int64)
    scene_optical = np.asarray(scene.points_optical)
    scene_base = np.asarray(scene.points_base)
    target_optical = np.asarray(target_scene.points_optical)
    target_base = np.asarray(target_scene.points_base)
    if (not np.array_equal(scene_optical[mapped_rows], target_optical)
            or not np.array_equal(scene_base[mapped_rows], target_base)):
        raise ValueError(
            "target source_indices do not identify the supplied target points")
    mask = np.isin(scene_ids, target_ids)
    if not np.any(mask):
        raise ValueError("target region must contain at least one scene point")
    return np.asarray(mask, dtype=bool)


def segment_target_component(
        scene: "GraspScene", bbox: Tuple[float, float, float, float],
        target_pixel: Tuple[float, float], *, pad_px: float = 4.0,
        seed_radius_px: float = 14.0, depth_tolerance: float = 0.05,
        voxel_size: float = 0.008, min_points: int = 30,
        table_z: float = 0.0, table_margin: float = 0.006,
        ) -> Optional["GraspScene"]:
    """Extract one RGB-D object component around Gemini's target point.

    A 2-D box is not an object mask: it also contains the table, objects behind
    it and often part of the robot.  This routine first applies the box, removes
    the horizontal support plane in the calibrated world frame, validates
    component depth around Gemini's on-object point, then chooses the connected
    3-D component nearest that point. The result is used for the yellow RViz
    overlay and target bounds after scene-wide GraspNet inference; it is not
    the network input.

    The implementation deliberately uses only NumPy and the existing voxel
    connectivity helper, keeping it usable on the Jetson runtime without a new
    segmentation model or Python dependency.
    """
    cropped = crop_to_bbox(scene, bbox, pad_px=pad_px)
    if cropped is None or cropped.points_optical.shape[0] == 0:
        return None

    min_points = max(1, int(min_points))
    finite = (np.all(np.isfinite(cropped.points_optical), axis=1)
              & np.all(np.isfinite(cropped.points_base), axis=1)
              & np.all(np.isfinite(cropped.pixels), axis=1))
    # A grasp target must be above the support plane.  This is the most useful
    # guard against a box that covers much more table than object.
    foreground = finite & (cropped.points_base[:, 2]
                           > float(table_z) + float(table_margin))
    if np.count_nonzero(foreground) < min_points:
        return None
    cropped = _masked_scene(cropped, foreground)

    target = np.asarray(target_pixel, dtype=float).reshape(2)
    pixel_distance = np.linalg.norm(cropped.pixels - target, axis=1)
    near_order = np.argsort(pixel_distance)
    within_seed = near_order[
        pixel_distance[near_order] <= max(1.0, float(seed_radius_px))]
    # A robust depth from the nearest few on-object pixels avoids one bad D405
    # sample moving the gate to the background.  If the requested radius is
    # sparse, the globally nearest foreground samples are the safe fallback.
    seed_indices = within_seed[:16] if within_seed.size else near_order[:16]
    if seed_indices.size == 0:
        return None
    seed_depth = float(np.median(cropped.points_optical[seed_indices, 2]))
    components = voxel_clusters(
        cropped.points_optical, max(0.001, float(voxel_size)), min_points)
    if not components:
        return None
    # Select the component whose image footprint lies nearest Gemini's point.
    # Median of its eight closest pixels is resistant to isolated depth specks.
    # The seed depth only validates component identity; it does not truncate the
    # selected object, which would otherwise cut the far side of a large object.

    def component_distance(indices: np.ndarray) -> float:
        distances = np.sort(np.linalg.norm(
            cropped.pixels[indices] - target, axis=1))
        return float(np.median(distances[:min(8, distances.size)]))

    tolerance = max(0.001, float(depth_tolerance))
    viable = []
    for indices in components:
        distances = np.linalg.norm(cropped.pixels[indices] - target, axis=1)
        closest = indices[np.argsort(distances)[:min(8, indices.size)]]
        component_depth = float(np.median(
            cropped.points_optical[closest, 2]))
        if abs(component_depth - seed_depth) <= tolerance:
            viable.append(indices)
    if not viable:
        return None
    selected = min(viable, key=component_distance)
    mask = np.zeros(cropped.points_optical.shape[0], dtype=bool)
    mask[selected] = True
    return _masked_scene(cropped, mask)


def voxel_clusters(points: np.ndarray, voxel: float,
                   min_points: int) -> List[np.ndarray]:
    """Label points by connected occupied voxels (26-neighbourhood).

    Returns one index array per cluster, largest first. Small enough to stay
    pure python: a 640x480 frame at stride 2 and a 6 mm voxel leaves a few
    thousand occupied cells.
    """
    points = np.asarray(points, dtype=float).reshape(-1, 3)
    if points.shape[0] == 0:
        return []
    keys = np.floor(points / float(voxel)).astype(np.int64)
    uniq, inverse = np.unique(keys, axis=0, return_inverse=True)
    index_of = {tuple(int(c) for c in key): i for i, key in enumerate(uniq)}

    neighbours = [(dx, dy, dz)
                  for dx in (-1, 0, 1) for dy in (-1, 0, 1) for dz in (-1, 0, 1)
                  if (dx, dy, dz) != (0, 0, 0)]
    labels = np.full(uniq.shape[0], -1, dtype=np.int64)
    label = 0
    for seed in range(uniq.shape[0]):
        if labels[seed] >= 0:
            continue
        stack = [seed]
        labels[seed] = label
        while stack:
            current = stack.pop()
            cx, cy, cz = (int(v) for v in uniq[current])
            for dx, dy, dz in neighbours:
                found = index_of.get((cx + dx, cy + dy, cz + dz))
                if found is not None and labels[found] < 0:
                    labels[found] = label
                    stack.append(found)
        label += 1

    point_labels = labels[inverse]
    clusters = []
    for value in range(label):
        idx = np.flatnonzero(point_labels == value)
        if idx.size >= int(min_points):
            clusters.append(idx)
    clusters.sort(key=lambda a: -a.size)
    return clusters


def horizontal_axes(points_xy: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    """Major and minor horizontal principal axes of a cluster's footprint."""
    centred = points_xy - points_xy.mean(axis=0)
    cov = np.cov(centred.T) if centred.shape[0] > 1 else np.eye(2)
    cov = np.atleast_2d(cov)
    values, vectors = np.linalg.eigh(cov)
    order = np.argsort(values)[::-1]
    major = np.array([vectors[0, order[0]], vectors[1, order[0]], 0.0])
    minor = np.array([vectors[0, order[1]], vectors[1, order[1]], 0.0])
    return major, minor


def robust_extent(values: np.ndarray, low: float = 5.0,
                  high: float = 95.0) -> float:
    """Percentile span — a stray depth pixel must not inflate an object's width."""
    if values.size == 0:
        return 0.0
    return float(np.percentile(values, high) - np.percentile(values, low))


class AnalyticGraspBackend:
    """Table-plane removal, voxel clustering, one grasp family per cluster."""

    name = "analytic"

    def __init__(self, *, voxel: float = 0.006, min_points: int = 40,
                 table_z: float = 0.0, table_margin: float = 0.010,
                 grasp_depth: float = 0.020, finger_clearance: float = 0.008,
                 min_width: float = 0.010, max_width: float = 0.065,
                 approach_tilts: Sequence[float] = (0.0, 0.35, 0.70, 1.05),
                 max_clusters: int = 8, support_reference: int = 400,
                 logger=None) -> None:
        self.voxel = float(voxel)
        self.min_points = int(min_points)
        self.table_z = float(table_z)
        self.table_margin = float(table_margin)
        self.grasp_depth = float(grasp_depth)
        self.finger_clearance = float(finger_clearance)
        self.min_width = float(min_width)
        self.max_width = float(max_width)
        self.approach_tilts = [float(t) for t in approach_tilts]
        self.max_clusters = int(max_clusters)
        self.support_reference = max(1, int(support_reference))
        self._log = logger

    def available(self) -> bool:
        return True

    def _width_score(self, width: float) -> float:
        """1.0 well inside the jaw range, falling to 0 at either limit."""
        if width >= self.max_width or width <= 0.0:
            return 0.0
        span = max(self.max_width - self.min_width, 1e-4)
        ideal = self.min_width + 0.45 * span
        return float(max(0.0, 1.0 - abs(width - ideal) / span))

    def detect(self, scene: GraspScene) -> List[GraspCandidate]:
        points = np.asarray(scene.points_base, dtype=float).reshape(-1, 3)
        pixels = np.asarray(scene.pixels, dtype=float).reshape(-1, 2)
        if pixels.shape[0] != points.shape[0]:
            pixels = np.full((points.shape[0], 2), np.nan)
        above = points[:, 2] > self.table_z + self.table_margin
        points = points[above]
        pixels = pixels[above]
        if points.shape[0] < self.min_points:
            if self._log:
                self._log.warn(
                    f"analytic backend: only {points.shape[0]} points above the "
                    f"table plane at z={self.table_z + self.table_margin:.3f}")
            return []

        candidates: List[GraspCandidate] = []
        for cluster in voxel_clusters(points, self.voxel,
                                      self.min_points)[:self.max_clusters]:
            candidates.extend(self._cluster_grasps(points[cluster],
                                                   pixels[cluster]))
        candidates.sort(key=lambda c: -c.score)
        return candidates

    def detect_single(self, scene: GraspScene,
                      foreground_band_m: float = 0.03,
                      table_margin: Optional[float] = None
                      ) -> List[GraspCandidate]:
        """Propose grasps treating one foreground surface as ONE object.

        For target mode: once ``crop_to_bbox`` has already isolated Gemini's
        box, identity is *mostly* solved — but a box drawn around a thin or
        angled object routinely includes a slice of whatever is behind or
        under it too (measured on hardware: a box for "the blue pen lying on
        the box" included the supporting box's own surface, and treating both
        depths as one object gave a single blob 31 cm wide, nothing like a
        pen). Two narrowing steps handle this without going back to general
        multi-object clustering — which would reintroduce the connectivity
        problem this method exists to avoid, since a thin or glossy target
        gives a real depth camera a sparse, gappy return that fails
        ``voxel_clusters``'s 26-neighbourhood chain long before it fails
        ``min_points``:

        1. **Foreground band** — keep only points within ``foreground_band_m``
           of the crop's nearest depth (5th percentile, not the bare minimum,
           so one noisy near pixel cannot set the whole band). The target is
           presumably what Gemini's box centred on, which is the nearest
           coherent surface in frame far more often than the backdrop is.
        2. **Above-table** — the existing table-plane cut, applied after, in
           case the foreground band alone still includes a slice of the table.
        """
        points = np.asarray(scene.points_base, dtype=float).reshape(-1, 3)
        pixels = np.asarray(scene.pixels, dtype=float).reshape(-1, 2)
        depth = np.asarray(scene.points_optical, dtype=float).reshape(-1, 3)[:, 2]
        if pixels.shape[0] != points.shape[0]:
            pixels = np.full((points.shape[0], 2), np.nan)

        if foreground_band_m > 0.0 and depth.size:
            near = float(np.percentile(depth, 5.0))
            band = depth <= near + foreground_band_m
            if int(band.sum()) >= self.min_points:
                points, pixels, depth = points[band], pixels[band], depth[band]

        # A separate, smaller margin than self.table_margin: measured on
        # hardware, a thin flat object (a pen) sits within about +/-1 cm of
        # table_z, well under this backend's default 1 cm scene-wide margin —
        # that margin exists to decline whole-scene table noise, not to
        # separate a single already-isolated object from the table beneath it.
        margin = self.table_margin if table_margin is None else float(table_margin)
        above = points[:, 2] > self.table_z + margin
        points, pixels = points[above], pixels[above]
        if points.shape[0] < self.min_points:
            if self._log:
                self._log.warn(
                    f"analytic backend (single): only {points.shape[0]} "
                    f"points above the table plane at "
                    f"z={self.table_z + margin:.3f}")
            return []
        candidates = self._cluster_grasps(points, pixels)
        candidates.sort(key=lambda c: -c.score)
        return candidates

    def _cluster_grasps(self, pts: np.ndarray,
                        pix: np.ndarray) -> List[GraspCandidate]:
        centre_xy = pts[:, :2].mean(axis=0)
        top_z = float(np.percentile(pts[:, 2], 95.0))
        major, minor = horizontal_axes(pts[:, :2])
        along_minor = pts[:, :2] @ minor[:2]
        along_major = pts[:, :2] @ major[:2]
        width = robust_extent(along_minor) + self.finger_clearance
        length = robust_extent(along_major)
        if width < self.min_width or width > self.max_width:
            return []

        grasp_z = max(top_z - self.grasp_depth,
                      self.table_z + self.table_margin)
        position = np.array([centre_xy[0], centre_xy[1], grasp_z])

        radial = np.array([position[0], position[1], 0.0])
        radial_norm = float(np.linalg.norm(radial))
        tilt_axis = (np.cross(radial / radial_norm, np.array([0.0, 0.0, 1.0]))
                     if radial_norm > 1e-6 else np.array([0.0, 1.0, 0.0]))

        aspect = 1.0 - min(1.0, width / max(length, 1e-4))
        support = float(min(1.0, pts.shape[0] / self.support_reference))
        base_score = (0.40 * self._width_score(width)
                      + 0.25 * support
                      + 0.15 * max(0.0, aspect))
        has_pixels = pix.size > 0 and bool(np.all(np.isfinite(pix)))
        pixel = (float(pix[:, 0].mean()), float(pix[:, 1].mean())) \
            if has_pixels else None
        bbox = (int(pix[:, 0].min()), int(pix[:, 1].min()),
                int(pix[:, 0].max()), int(pix[:, 1].max())) \
            if has_pixels else None

        grasps = []
        for tilt in self.approach_tilts:
            approach = _rodrigues(tilt_axis, tilt) @ DOWN
            grasps.append(GraspCandidate(
                position=position.copy(),
                approach=approach / float(np.linalg.norm(approach)),
                closing=minor.copy(),
                width=float(width),
                score=float(base_score + 0.20 * math.cos(tilt)),
                support=int(pts.shape[0]),
                pixel=pixel,
                bbox=bbox,
                source=self.name,
                extras={"top_z": top_z, "length": float(length),
                        "tilt": float(tilt)},
            ))
        return grasps


class GraspNetBackend:
    """graspnet-baseline adapter — the model the ROBOTIS OMY story runs.

    Needs the repo on ``repo_path`` (for ``models/graspnet.py`` and its compiled
    CUDA ops), a ``checkpoint_path``, and a working torch. Everything is
    imported inside :meth:`load` so a machine without them can still run the
    analytic backend.
    """

    name = "graspnet"

    def __init__(self, *, repo_path: str = "", checkpoint_path: str = "",
                 num_point: int = 20000, num_view: int = 300,
                 collision_thresh: float = 0.01, voxel_size: float = 0.01,
                 empty_thresh: float = 0.01, device: str = "cuda",
                 top_k: int = 50, max_width: float = 0.065,
                 sampling_seed: int = 0, logger=None) -> None:
        self.repo_path = str(repo_path)
        self.checkpoint_path = str(checkpoint_path)
        self.num_point = int(num_point)
        self.num_view = int(num_view)
        self.collision_thresh = float(collision_thresh)
        self.voxel_size = float(voxel_size)
        self.empty_thresh = float(empty_thresh)
        self.device = str(device)
        self.top_k = int(top_k)
        self.max_width = float(max_width)
        self.sampling_seed = int(sampling_seed)
        self._log = logger
        self._net = None
        self._torch = None
        self._device = None
        self._pred_decode = None
        self._grasp_group_cls = None
        self._collision_detector_cls = None
        self._availability_error = "not checked"
        self.last_stats: Dict[str, int] = {}

    @property
    def availability_error(self) -> str:
        """Why :meth:`available` last returned false, safe to show in a log."""
        return self._availability_error

    def _prepare_repo_imports(self) -> None:
        """Validate the configured checkout and put its import roots first.

        GraspNet-baseline uses top-level imports (``from graspnet import ...``
        and ``from collision_detector import ...``) in its own demo, so the
        checkout's ``models`` and ``utils`` directories must be on
        :data:`sys.path`.  Putting the configured checkout first also avoids
        silently importing an unrelated module with either generic name.
        """
        if not self.repo_path or not os.path.isdir(self.repo_path):
            raise RuntimeError(
                f"graspnet_repo_path is not a directory: {self.repo_path!r}")

        required = [os.path.join(self.repo_path, "models", "graspnet.py")]
        if self.collision_thresh > 0.0:
            required.append(os.path.join(
                self.repo_path, "utils", "collision_detector.py"))
        missing = [path for path in required if not os.path.isfile(path)]
        if missing:
            names = ", ".join(os.path.relpath(path, self.repo_path)
                              for path in missing)
            raise RuntimeError(
                f"graspnet checkout is incomplete; missing: {names}")
        if not self.checkpoint_path or not os.path.isfile(self.checkpoint_path):
            raise RuntimeError(
                f"graspnet_checkpoint is not a file: {self.checkpoint_path!r}")

        roots = [self.repo_path,
                 os.path.join(self.repo_path, "models"),
                 os.path.join(self.repo_path, "utils"),
                 os.path.join(self.repo_path, "dataset")]
        # Reverse insertion preserves the order above at the front of sys.path.
        for path in reversed(roots):
            if path in sys.path:
                sys.path.remove(path)
            sys.path.insert(0, path)

    def _import_runtime(self) -> None:
        """Import every optional runtime component, with one actionable error.

        ``find_spec('torch')`` is insufficient: the broken Jetson installation
        this package was developed around has a discoverable module that raises
        ``OSError`` while loading ``libtorch_global_deps.so``.  Importing here
        makes :meth:`available` reflect whether inference can actually start.
        """
        if (self._torch is not None and self._pred_decode is not None
                and self._grasp_group_cls is not None
                and (self.collision_thresh <= 0.0
                     or self._collision_detector_cls is not None)):
            return
        self._prepare_repo_imports()
        try:
            torch = importlib.import_module("torch")
            graspnet_api = importlib.import_module("graspnetAPI")
            graspnet_model = importlib.import_module("graspnet")
            collision_module = (importlib.import_module("collision_detector")
                                if self.collision_thresh > 0.0 else None)

            pred_decode = getattr(graspnet_model, "pred_decode")
            grasp_group_cls = getattr(graspnet_api, "GraspGroup")
            collision_cls = (getattr(collision_module,
                                     "ModelFreeCollisionDetector")
                             if collision_module is not None else None)
            # Access now so an incomplete/wrong checkout fails availability,
            # rather than much later in the worker thread.
            getattr(graspnet_model, "GraspNet")
        except (ImportError, OSError, AttributeError, RuntimeError) as exc:
            raise RuntimeError(
                f"cannot import graspnet-baseline runtime: {exc}") from exc

        self._torch = torch
        self._graspnet_cls = graspnet_model.GraspNet
        self._pred_decode = pred_decode
        self._grasp_group_cls = grasp_group_cls
        self._collision_detector_cls = collision_cls

    def available(self) -> bool:
        try:
            self._import_runtime()
        except Exception as exc:  # noqa: BLE001 - optional stack fails many ways
            self._availability_error = str(exc)
            return False
        self._availability_error = ""
        return True

    def load(self) -> None:
        """Import torch + the repo and restore the checkpoint. Raises on failure."""
        if self._net is not None:
            return
        self._import_runtime()
        torch = self._torch
        net = self._graspnet_cls(
            input_feature_dim=0, num_view=self.num_view,
            num_angle=12, num_depth=4, cylinder_radius=0.05,
            hmin=-0.02, hmax_list=[0.01, 0.02, 0.03, 0.04],
            is_training=False)
        if (self.device.lower().startswith("cuda")
                and not torch.cuda.is_available()):
            raise RuntimeError(
                f"graspnet_device={self.device!r}, but CUDA is unavailable")
        device = torch.device(self.device)
        net.to(device)
        checkpoint = torch.load(
            self.checkpoint_path, map_location=device, weights_only=True)
        state = (checkpoint.get("model_state_dict")
                 if isinstance(checkpoint, dict) else None)
        if not isinstance(state, dict):
            raise RuntimeError("GraspNet checkpoint has no model_state_dict")
        net.load_state_dict(state, strict=True)
        net.eval()
        self._device = device
        self._net = net
        if self._log:
            self._log.info(
                f"graspnet-baseline loaded on {device} from "
                f"{os.path.basename(self.checkpoint_path)}")

    def detect(self, scene: GraspScene,
               collision_scene: Optional[GraspScene] = None,
               sampling_seed: Optional[int] = None,
               ) -> List[GraspCandidate]:
        """Infer on ``scene`` and optionally collision-check against a wider cloud.

        Target picking feeds a complete reachable scene to the network, never
        only the segmented object. Supplying a still wider ``collision_scene``
        preserves out-of-workspace support and neighbouring geometry for the
        upstream model-free collision detector.
        """
        self.last_stats = {}
        self.load()
        torch = self._torch
        points = np.asarray(scene.points_optical, dtype=np.float32)
        if points.shape[0] == 0:
            self.last_stats = {"decoded": 0, "returned": 0}
            return []
        seed = (self.sampling_seed if sampling_seed is None
                else int(sampling_seed))
        self.last_stats["sampling_seed"] = seed
        idx = _sample_point_indices(points.shape[0], self.num_point, seed)
        sampled = points[idx]

        with torch.no_grad():
            batch = {"point_clouds": torch.from_numpy(
                sampled[np.newaxis]).to(self._device)}
            end_points = self._net(batch)
            # This is the public inference path in graspnet-baseline's demo.py.
            # ``end_points`` is an internal feature dictionary, not a grasp
            # array; pred_decode converts it to one tensor per batch element.
            grasp_predictions = self._pred_decode(end_points)
            if not grasp_predictions:
                self.last_stats = {"decoded": 0, "returned": 0}
                return []
            grasp_array = grasp_predictions[0].detach().cpu().numpy()
            grasps = self._grasp_group_cls(grasp_array)
        self.last_stats["decoded"] = len(grasps)

        # The RealSense checkpoint predicts apertures for the 100 mm gripper
        # used during training.  GraspNet's own evaluator clips those widths to
        # the evaluated gripper's maximum instead of dropping the pose.  Do the
        # same for this 65 mm gripper *before* collision checking, so collision
        # geometry matches the aperture the real hardware can command.  A
        # genuinely too-large object will then collide with the narrower jaws
        # and is still rejected by the model-free detector.
        raw_width_by_pose = {
            self._grasp_geometry_key(grasp): float(grasp.width)
            for grasp in grasps
        }
        if len(grasps) and self.max_width > 0.0:
            raw_widths = np.asarray(grasps.widths, dtype=float)
            clipped_widths = np.minimum(raw_widths, self.max_width)
            clipped_count = int(np.count_nonzero(
                raw_widths > self.max_width + 1e-9))
            self.last_stats["width_clipped"] = clipped_count
            grasps.widths = clipped_widths
            if clipped_count and self._log:
                self._log.info(
                    f"GraspNet clipped {clipped_count}/{len(grasps)} "
                    f"predicted widths to {self.max_width * 1000:.0f} mm "
                    "before collision checking")

        # Upstream runs model-free collision detection on the complete scene
        # cloud before NMS.  It returns True for candidates that collide.
        if self.collision_thresh > 0.0 and len(grasps):
            collision_points = np.asarray(
                (collision_scene.points_optical
                 if collision_scene is not None else scene.points_optical),
                dtype=np.float32)
            detector = self._collision_detector_cls(
                collision_points, voxel_size=self.voxel_size)
            detected = detector.detect(
                grasps, approach_dist=0.05,
                collision_thresh=self.collision_thresh,
                return_empty_grasp=self.empty_thresh > 0.0,
                empty_thresh=max(0.0, self.empty_thresh))
            if self.empty_thresh > 0.0:
                collision_mask, empty_mask = detected
                collision_mask = np.asarray(collision_mask, dtype=bool)
                empty_mask = np.asarray(empty_mask, dtype=bool)
                self.last_stats["collision_rejected"] = int(
                    np.count_nonzero(collision_mask))
                self.last_stats["empty_rejected"] = int(np.count_nonzero(
                    empty_mask & ~collision_mask))
                collision_mask = collision_mask | empty_mask
            else:
                collision_mask = np.asarray(detected, dtype=bool)
                self.last_stats["collision_rejected"] = int(
                    np.count_nonzero(collision_mask))
            if collision_mask.shape != (len(grasps),):
                raise RuntimeError(
                    "GraspNet collision detector returned a mask of shape "
                    f"{collision_mask.shape}, expected ({len(grasps)},)")
            grasps = grasps[~collision_mask]
        self.last_stats["collision_kept"] = len(grasps)
        if not len(grasps):
            self.last_stats["after_nms"] = 0
            self.last_stats["returned"] = 0
            return []

        # GraspGroup.nms() returns a new group in current graspnetAPI releases;
        # ignoring it silently disables NMS. Accommodate older mutating builds
        # too, as these repositories are commonly pinned to old commits.
        nms_result = grasps.nms()
        if nms_result is not None:
            grasps = nms_result
        self.last_stats["after_nms"] = len(grasps)
        sort_result = grasps.sort_by_score()
        if sort_result is not None:
            grasps = sort_result
        candidates = self._to_candidates(
            grasps[:self.top_k], scene,
            raw_width_by_pose=raw_width_by_pose)
        self.last_stats["returned"] = len(candidates)
        return candidates

    @staticmethod
    def _grasp_geometry_key(grasp) -> Tuple[float, ...]:
        """Stable identity for one pose while its width is being clipped."""
        values = np.concatenate([
            np.asarray([grasp.score, grasp.depth], dtype=float),
            np.asarray(grasp.rotation_matrix, dtype=float).reshape(-1),
            np.asarray(grasp.translation, dtype=float).reshape(-1),
        ])
        return tuple(np.round(values, decimals=7).tolist())

    def _to_candidates(
            self, grasps, scene: GraspScene, *,
            raw_width_by_pose: Optional[Dict[Tuple[float, ...], float]] = None,
            ) -> List[GraspCandidate]:
        """graspnetAPI grasps (camera frame) -> base-frame candidates.

        graspnet-baseline writes a grasp rotation as columns
        (approach, binormal, minor).  Its ``translation`` is already the target
        point at the gripper centre (see graspnetAPI ``plot_gripper_pro_max``);
        ``depth`` sizes the fingers along the approach axis and must *not* be
        added to the translation.  Adding it shifts every displayed/executed
        grasp several centimetres past the object.
        """
        out: List[GraspCandidate] = []
        for grasp in grasps:
            R = np.asarray(grasp.rotation_matrix, dtype=float).reshape(3, 3)
            approach_opt = R[:, 0]
            closing_opt = R[:, 1]
            position_opt = np.asarray(grasp.translation, dtype=float)
            position = points_to_base(position_opt[np.newaxis],
                                      scene.p_wc, scene.R_wc)[0]
            approach = directions_to_base(approach_opt[np.newaxis],
                                          scene.R_wc)[0]
            closing = directions_to_base(closing_opt[np.newaxis],
                                         scene.R_wc)[0]
            width = float(grasp.width)
            raw_width = (raw_width_by_pose or {}).get(
                self._grasp_geometry_key(grasp), width)
            out.append(GraspCandidate(
                position=position,
                approach=approach / float(np.linalg.norm(approach)),
                closing=closing / float(np.linalg.norm(closing)),
                width=width,
                score=float(grasp.score),
                source=self.name,
                extras={
                    "graspnet_depth": float(grasp.depth),
                    "graspnet_width_raw": float(raw_width),
                    "graspnet_width_clipped": float(
                        raw_width > width + 1e-9),
                },
            ))
        return out


_ANYGRASP_LOAD_LOCK = threading.Lock()


class AnyGraspBackend:
    """Licensed AnyGrasp SDK adapter with exact target-region steering.

    AnyGrasp performs its own collision filtering against the same full point
    cloud it receives for inference.  Consequently ``collision_scene`` is
    accepted for backend API compatibility but never replaces ``scene``.
    Target mode supplies a boolean ``region_mask`` aligned to every scene row.
    """

    name = "anygrasp"
    supports_region_steering = True

    def __init__(self, *, runtime_dir: str, checkpoint_path: str,
                 license_dir: str = "", max_width: float = 0.065,
                 gripper_height: float = 0.058, top_k: int = 50,
                 dense_grasp: bool = False,
                 collision_detection: bool = True, logger=None) -> None:
        self.runtime_dir = os.path.abspath(os.path.expanduser(str(runtime_dir)))
        self.checkpoint_path = os.path.abspath(os.path.expanduser(
            str(checkpoint_path)))
        configured_license = str(license_dir).strip()
        self.license_dir = os.path.abspath(os.path.expanduser(
            configured_license)) if configured_license else os.path.join(
                self.runtime_dir, "license")
        self.max_width = float(max_width)
        self.gripper_height = float(gripper_height)
        self.top_k = int(top_k)
        self.dense_grasp = bool(dense_grasp)
        self.collision_detection = bool(collision_detection)
        self._log = logger
        self._create_detector = None
        self._detector = None
        self._availability_error = "not checked"
        self.last_stats: Dict[str, object] = {}

    @property
    def availability_error(self) -> str:
        return self._availability_error

    def _validate_artifacts(self) -> None:
        if not os.path.isdir(self.runtime_dir):
            raise RuntimeError(
                f"anygrasp_runtime_dir is not a directory: {self.runtime_dir!r}")
        binary = os.path.join(self.runtime_dir, "gsnet.so")
        if not os.path.isfile(binary):
            raise RuntimeError(f"AnyGrasp runtime is missing gsnet.so: {binary}")
        if not os.path.isfile(self.checkpoint_path):
            raise RuntimeError(
                f"anygrasp_checkpoint is not a file: {self.checkpoint_path!r}")
        license_cfg = os.path.join(self.license_dir, "licenseCfg.json")
        if not os.path.isfile(license_cfg):
            raise RuntimeError(
                f"AnyGrasp license is missing licenseCfg.json: {license_cfg}")
        # The July-2026 SDK's create_detector uses the relative path
        # license/licenseCfg.json.  A differently named directory would pass
        # our validation and then mysteriously fail inside the binary.
        if os.path.basename(self.license_dir) != "license":
            raise RuntimeError(
                "anygrasp_license_dir must end in 'license' for this SDK")
        if not (0.0 < self.max_width <= 0.1):
            raise RuntimeError("anygrasp_max_width must be in (0.0, 0.1] m")
        if self.gripper_height <= 0.0:
            raise RuntimeError("anygrasp_gripper_height must be positive")
        if self.top_k <= 0:
            raise RuntimeError("top_k must be positive")

    def _import_runtime(self) -> None:
        if self._create_detector is not None:
            return
        self._validate_artifacts()
        # ``gsnet.so`` imports sibling SDK packages using top-level names.
        # Put the selected installation first, including its parent for the
        # AnyGrasp pointnet2 package, instead of accepting a coincidental copy.
        for path in reversed([self.runtime_dir,
                              os.path.dirname(self.runtime_dir)]):
            if path in sys.path:
                sys.path.remove(path)
            sys.path.insert(0, path)
        try:
            module = importlib.import_module("gsnet")
            create_detector = getattr(module, "create_detector")
        except (ImportError, OSError, AttributeError, RuntimeError) as exc:
            raise RuntimeError(f"cannot import AnyGrasp runtime: {exc}") from exc
        module_file = getattr(module, "__file__", None)
        if module_file is not None:
            module_file = os.path.realpath(module_file)
            runtime_root = os.path.realpath(self.runtime_dir) + os.sep
            if not module_file.startswith(runtime_root):
                raise RuntimeError(
                    "imported gsnet from the wrong installation: "
                    f"{module_file} (expected under {self.runtime_dir})")
        if not callable(create_detector):
            raise RuntimeError("AnyGrasp gsnet module has no create_detector API")
        self._create_detector = create_detector

    def available(self) -> bool:
        try:
            self._import_runtime()
        except Exception as exc:  # noqa: BLE001 - native SDK failure surface
            self._availability_error = str(exc)
            return False
        self._availability_error = ""
        return True

    def load(self) -> None:
        """Load the detector exactly once and validate its licensed result."""
        if self._detector is not None:
            return
        self._import_runtime()
        config = Namespace(
            checkpoint_path=self.checkpoint_path,
            max_gripper_width=self.max_width,
            gripper_height=self.gripper_height,
        )
        # create_detector currently resolves ``license/licenseCfg.json`` from
        # cwd.  Keep the unavoidable process-wide cwd change short, serialized,
        # and always restore it even if the native extension raises.
        license_parent = os.path.dirname(self.license_dir)
        with _ANYGRASP_LOAD_LOCK:
            previous_cwd = os.getcwd()
            try:
                os.chdir(license_parent)
                try:
                    detector = self._create_detector(config)
                except Exception as exc:  # noqa: BLE001 - native SDK surface
                    raise RuntimeError(
                        f"AnyGrasp detector initialization failed: {exc}") \
                        from exc
            finally:
                os.chdir(previous_cwd)
        if detector is None:
            raise RuntimeError(
                "AnyGrasp create_detector returned None; license validation "
                "or model initialization failed")
        self._detector = detector
        if self._log:
            self._log.info(
                f"AnyGrasp loaded from {os.path.basename(self.checkpoint_path)} "
                f"(max width {self.max_width * 1000:.0f} mm, collision "
                f"{'on' if self.collision_detection else 'off'})")

    @staticmethod
    def _validated_region_mask(region_mask, point_count: int):
        if region_mask is None:
            return None
        mask = np.asarray(region_mask)
        if mask.ndim != 1 or mask.shape != (point_count,):
            raise ValueError(
                f"AnyGrasp region_mask has shape {mask.shape}, expected "
                f"({point_count},)")
        if mask.dtype != np.bool_:
            raise ValueError("AnyGrasp region_mask must have bool dtype")
        if not np.any(mask):
            raise ValueError(
                "AnyGrasp region_mask is empty; at least one target point is "
                "required")
        return np.ascontiguousarray(mask)

    def detect(self, scene: GraspScene,
               collision_scene: Optional[GraspScene] = None,
               sampling_seed: Optional[int] = None,
               region_mask: Optional[np.ndarray] = None,
               ) -> List[GraspCandidate]:
        """Run one SDK inference on the complete optical-frame scene."""
        # AnyGrasp's licensed API performs collision checking only against the
        # point array passed to ``get_grasp``.  It has no separate collision
        # cloud argument.  Silently accepting a different cloud here would let
        # a caller believe obstacles are checked when they are not.
        if collision_scene is not None and collision_scene is not scene:
            raise ValueError(
                "AnyGrasp collision_scene must be the same full GraspScene as "
                "scene; the SDK collision checker cannot consume a separate "
                "cloud")
        del sampling_seed  # deliberately not an SDK input
        self.last_stats = {}
        points = np.asarray(scene.points_optical)
        if points.ndim != 2 or points.shape[1:] != (3,):
            raise ValueError(
                f"AnyGrasp scene points have shape {points.shape}, expected (N, 3)")
        if points.shape[0] == 0:
            self.last_stats = {"returned": 0}
            return []
        if not np.all(np.isfinite(points)):
            raise ValueError("AnyGrasp scene points must all be finite")
        mask = self._validated_region_mask(region_mask, points.shape[0])
        self.load()
        points_f32 = np.ascontiguousarray(points, dtype=np.float32)
        options = {
            "dense_grasp": self.dense_grasp,
            "collision_detection": self.collision_detection,
            "region_steering": mask,
            "approach_steering": None,
            "approach_thresh": np.pi,
        }
        try:
            grasps = self._detector.get_grasp(points_f32, options)
        except Exception as exc:  # noqa: BLE001 - native SDK surface
            raise RuntimeError(f"AnyGrasp inference failed: {exc}") from exc
        self.last_stats["input_points"] = int(points_f32.shape[0])
        self.last_stats["region_points"] = (
            int(np.count_nonzero(mask)) if mask is not None else None)
        if grasps is None or len(grasps) == 0:
            self.last_stats.update({"decoded": 0, "collision_kept": 0,
                                    "after_nms": 0, "returned": 0})
            return []
        self.last_stats["decoded"] = len(grasps)
        # The licensed SDK applies collision filtering internally before it
        # returns the group; it does not expose the pre-collision count.
        self.last_stats["collision_kept"] = len(grasps)
        if not self.dense_grasp:
            nms_result = grasps.nms()
            if nms_result is not None:
                grasps = nms_result
        self.last_stats["after_nms"] = len(grasps)
        sort_result = grasps.sort_by_score()
        if sort_result is not None:
            grasps = sort_result
        candidates = self._to_candidates(grasps[:self.top_k], scene)
        self.last_stats["returned"] = len(candidates)
        return candidates

    def _to_candidates(self, grasps, scene: GraspScene) -> List[GraspCandidate]:
        """Convert AnyGrasp centres and documented tip targets to base frame.

        Unlike the generic visualisation centre, this SDK explicitly defines
        the executable gripper tip as ``translation + depth * approach`` in
        ``USAGE.md``.  Keep the geometric centre for association/collision and
        carry the SDK tip separately as the motion target.
        """
        out: List[GraspCandidate] = []
        for grasp in grasps:
            rotation = np.asarray(
                grasp.rotation_matrix, dtype=float).reshape(3, 3)
            centre_opt = np.asarray(grasp.translation, dtype=float).reshape(3)
            depth = float(grasp.depth)
            approach_opt = rotation[:, 0]
            closing_opt = rotation[:, 1]
            tip_opt = centre_opt + depth * approach_opt
            centre = points_to_base(
                centre_opt[np.newaxis], scene.p_wc, scene.R_wc)[0]
            tcp = points_to_base(
                tip_opt[np.newaxis], scene.p_wc, scene.R_wc)[0]
            approach = directions_to_base(
                approach_opt[np.newaxis], scene.R_wc)[0]
            closing = directions_to_base(
                closing_opt[np.newaxis], scene.R_wc)[0]
            approach_norm = float(np.linalg.norm(approach))
            closing_norm = float(np.linalg.norm(closing))
            if (approach_norm < 1e-9 or closing_norm < 1e-9
                    or not np.all(np.isfinite(centre))
                    or not np.all(np.isfinite(tcp))):
                continue
            out.append(GraspCandidate(
                position=centre,
                approach=approach / approach_norm,
                closing=closing / closing_norm,
                width=float(grasp.width),
                score=float(grasp.score),
                source=self.name,
                extras={
                    "graspnet_depth": depth,
                    "anygrasp_depth": depth,
                    "tcp_position": tcp,
                    "grasp_center": centre.copy(),
                },
            ))
        return out


def make_backend(name: str, *, logger=None, analytic: Optional[dict] = None,
                 graspnet: Optional[dict] = None,
                 anygrasp: Optional[dict] = None):
    """Build the requested analytic, GraspNet, or AnyGrasp backend."""
    name = str(name).lower()
    if name == "analytic":
        return AnalyticGraspBackend(logger=logger, **(analytic or {}))
    if name == "graspnet":
        return GraspNetBackend(logger=logger, **(graspnet or {}))
    if name == "anygrasp":
        return AnyGraspBackend(logger=logger, **(anygrasp or {}))
    raise ValueError(
        f"unknown grasp_backend '{name}' (analytic|graspnet|anygrasp)")
