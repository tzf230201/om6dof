"""The analytic backend on synthetic tabletop scenes."""

import importlib
import sys
import types

import numpy as np
import pytest

from om6dof_pick_and_place_gemini.grasp_backends import (AnalyticGraspBackend,
                                                         AnyGraspBackend,
                                                         GraspNetBackend,
                                                         GraspScene,
                                                         _masked_scene,
                                                         _sample_point_indices,
                                                         crop_to_bbox,
                                                         crop_to_workspace,
                                                         horizontal_axes,
                                                         make_backend,
                                                         robust_extent,
                                                         segment_target_component,
                                                         self_exclusion_mask,
                                                         target_region_mask,
                                                         voxel_clusters)

RNG = np.random.default_rng(20260902)


def test_workspace_crop_keeps_scene_geometry_not_only_the_target():
    points = np.array([
        [0.30, 0.00, 0.10],   # target
        [0.35, 0.05, 0.03],   # support/nearby geometry
        [0.70, 0.00, 0.20],   # unreachable room geometry
    ])
    source = GraspScene(
        points_optical=points.copy(), points_base=points.copy(),
        pixels=np.array([[1, 1], [2, 2], [3, 3]], dtype=float),
        colors=np.zeros((3, 3), dtype=np.uint8), p_wc=np.zeros(3),
        R_wc=np.eye(3), intrinsics=(1.0, 1.0, 0.0, 0.0))

    cropped = crop_to_workspace(
        source, [0.08, -0.35, -0.05], [0.50, 0.35, 0.45])

    assert cropped.points_base == pytest.approx(points[:2])
    assert cropped.pixels == pytest.approx(np.array([[1, 1], [2, 2]]))


def block(cx, cy, width_x, width_y, height, n=1200):
    p = RNG.random((n, 3))
    p[:, 0] = cx + (p[:, 0] - 0.5) * width_x
    p[:, 1] = cy + (p[:, 1] - 0.5) * width_y
    p[:, 2] = p[:, 2] * height
    return p


def test_sparse_graspnet_sampling_is_balanced_and_repeatable():
    first = _sample_point_indices(7, 25, seed=4)
    second = _sample_point_indices(7, 25, seed=4)
    counts = np.bincount(first, minlength=7)

    assert np.array_equal(first, second)
    assert len(first) == 25
    assert counts.max() - counts.min() <= 1
    assert np.all(counts > 0)


def test_dense_graspnet_sampling_has_no_duplicate_indices():
    indices = _sample_point_indices(30, 20, seed=2)
    assert len(indices) == len(np.unique(indices)) == 20


def scene_from(points):
    pixels = np.column_stack([np.linspace(0, 639, points.shape[0]),
                              np.linspace(0, 479, points.shape[0])])
    return GraspScene(points_optical=points, points_base=points, pixels=pixels,
                      colors=None, p_wc=np.zeros(3), R_wc=np.eye(3),
                      intrinsics=(600.0, 600.0, 320.0, 240.0))


def test_voxel_clusters_separates_two_piles():
    left = block(0.25, 0.10, 0.03, 0.03, 0.04)
    right = block(0.25, -0.10, 0.03, 0.03, 0.04)
    clusters = voxel_clusters(np.vstack([left, right]), 0.006, 40)
    assert len(clusters) == 2


def test_voxel_clusters_drops_specks_below_min_points():
    speck = block(0.25, 0.0, 0.005, 0.005, 0.005, n=5)
    assert voxel_clusters(speck, 0.006, 40) == []


def test_robust_extent_ignores_a_single_outlier():
    values = np.concatenate([np.linspace(0.0, 0.02, 200), [5.0]])
    assert robust_extent(values) < 0.03


def test_horizontal_axes_finds_the_long_direction():
    pts = block(0.3, 0.0, 0.02, 0.10, 0.03)[:, :2]
    major, minor = horizontal_axes(pts)
    assert abs(major[1]) > 0.9      # long axis is Y
    assert abs(minor[0]) > 0.9      # jaws close across X


def test_grasp_lands_over_the_object_and_below_its_top():
    points = np.vstack([block(0.30, 0.05, 0.025, 0.060, 0.05),
                        np.column_stack([RNG.uniform(0.15, 0.45, 2000),
                                         RNG.uniform(-0.3, 0.3, 2000),
                                         np.zeros(2000)])])
    candidates = AnalyticGraspBackend().detect(scene_from(points))
    assert candidates, "the block should produce at least one candidate"
    best = candidates[0]
    assert np.allclose(best.position[:2], [0.30, 0.05], atol=0.01)
    assert 0.0 < best.position[2] < 0.05
    assert 0.02 < best.width < 0.038


def test_an_object_wider_than_the_jaws_is_not_proposed():
    points = block(0.30, 0.0, 0.090, 0.090, 0.06)
    assert AnalyticGraspBackend().detect(scene_from(points)) == []


def test_the_least_tilted_candidate_scores_highest():
    points = block(0.30, 0.0, 0.025, 0.060, 0.05)
    candidates = AnalyticGraspBackend().detect(scene_from(points))
    tilts = [c.extras["tilt"] for c in candidates]
    assert tilts == sorted(tilts), "score order should follow tilt order"


def test_tilted_candidates_lean_away_from_the_base():
    points = block(0.30, 0.0, 0.025, 0.060, 0.05)
    candidates = AnalyticGraspBackend().detect(scene_from(points))
    tilted = [c for c in candidates if c.extras["tilt"] > 0.5][0]
    # object sits at +X, so a tilted wrist must reach out along +X, not back
    assert tilted.approach[0] > 0.0
    assert tilted.approach[2] < 0.0


def test_everything_at_table_height_yields_nothing():
    flat = np.column_stack([RNG.uniform(0.15, 0.45, 3000),
                            RNG.uniform(-0.3, 0.3, 3000),
                            np.zeros(3000)])
    assert AnalyticGraspBackend().detect(scene_from(flat)) == []


def test_pregrasp_backs_off_along_the_approach():
    points = block(0.30, 0.0, 0.025, 0.060, 0.05)
    best = AnalyticGraspBackend().detect(scene_from(points))[0]
    back = best.pregrasp(0.10)
    assert pytest.approx(0.10, abs=1e-9) == float(
        np.linalg.norm(back - best.position))
    assert back[2] > best.position[2]    # straight-down grasp backs off upward


# --- excluding the robot's own gripper from the cloud -----------------------

def test_self_exclusion_drops_points_near_the_gripper():
    # measured on hardware: gripper self-points sat 3.3-5.1 cm from EE
    center = np.array([0.163, 0.0, 0.252])
    points = np.array([
        [0.155, -0.05, 0.247],   # 5.1 cm away -> gripper finger
        [0.153, 0.031, 0.248],   # 3.3 cm away -> gripper finger
        [0.30, 0.05, 0.05],      # far away -> a real object on the table
    ])
    keep = self_exclusion_mask(points, center, radius=0.09)
    assert list(keep) == [False, False, True]


def test_self_exclusion_boundary_is_strictly_greater_than_radius():
    center = np.zeros(3)
    points = np.array([[0.09, 0.0, 0.0], [0.0901, 0.0, 0.0]])
    keep = self_exclusion_mask(points, center, radius=0.09)
    assert list(keep) == [False, True]


def test_self_exclusion_on_an_empty_cloud():
    assert self_exclusion_mask(np.zeros((0, 3)), np.zeros(3), 0.09).shape == (0,)


def test_a_real_object_survives_the_full_pipeline_once_self_points_are_removed():
    gripper = np.array([0.163, 0.0, 0.252]) + RNG.normal(scale=0.01, size=(200, 3))
    real_object = block(0.30, 0.0, 0.025, 0.060, 0.05)
    table = np.column_stack([RNG.uniform(0.15, 0.45, 2000),
                            RNG.uniform(-0.3, 0.3, 2000), np.zeros(2000)])
    points = np.vstack([gripper, real_object, table])
    keep = self_exclusion_mask(points, np.array([0.163, 0.0, 0.252]), 0.09)
    filtered = points[keep]
    candidates = AnalyticGraspBackend().detect(scene_from(filtered))
    assert candidates, "the real object should still be found"
    assert np.allclose(candidates[0].position[:2], [0.30, 0.0], atol=0.01)


# --- cropping the cloud to a Gemini bounding box, before detection ---------

def scene_with_pixels(points, pixels, colors=None):
    return GraspScene(points_optical=points, points_base=points, pixels=pixels,
                      colors=colors, p_wc=np.zeros(3), R_wc=np.eye(3),
                      intrinsics=(600.0, 600.0, 320.0, 240.0))


def test_crop_to_bbox_keeps_only_points_inside_the_box():
    points = np.array([[0.1, 0, 0.3], [0.2, 0, 0.3], [0.3, 0, 0.3]])
    pixels = np.array([[100.0, 100.0], [300.0, 300.0], [500.0, 500.0]])
    cropped = crop_to_bbox(scene_with_pixels(points, pixels),
                           bbox=(250.0, 250.0, 350.0, 350.0), pad_px=0.0)
    assert cropped.points_optical.shape[0] == 1
    assert np.allclose(cropped.points_optical[0], [0.2, 0, 0.3])


def test_crop_to_bbox_padding_widens_the_kept_region():
    points = np.array([[0.1, 0, 0.3], [0.2, 0, 0.3]])
    pixels = np.array([[100.0, 100.0], [240.0, 240.0]])
    tight = crop_to_bbox(scene_with_pixels(points, pixels),
                         bbox=(250.0, 250.0, 350.0, 350.0), pad_px=0.0)
    padded = crop_to_bbox(scene_with_pixels(points, pixels),
                          bbox=(250.0, 250.0, 350.0, 350.0), pad_px=20.0)
    assert tight is None
    assert padded is not None and padded.points_optical.shape[0] == 1


def test_crop_to_bbox_returns_none_when_nothing_is_inside():
    points = np.array([[0.1, 0, 0.3]])
    pixels = np.array([[10.0, 10.0]])
    assert crop_to_bbox(scene_with_pixels(points, pixels),
                        bbox=(500.0, 500.0, 550.0, 550.0), pad_px=5.0) is None


def test_crop_to_bbox_keeps_colours_aligned_with_points():
    points = np.array([[0.1, 0, 0.3], [0.2, 0, 0.3]])
    pixels = np.array([[100.0, 100.0], [300.0, 300.0]])
    colors = np.array([[1, 2, 3], [4, 5, 6]], dtype=np.uint8)
    cropped = crop_to_bbox(scene_with_pixels(points, pixels, colors),
                           bbox=(250.0, 250.0, 350.0, 350.0), pad_px=0.0)
    assert list(cropped.colors[0]) == [4, 5, 6]


def test_target_segmentation_selects_point_anchored_component_not_largest():
    local_rng = np.random.default_rng(7)

    def local_block(cx, cy, width_x, width_y, height, count):
        points = local_rng.random((count, 3))
        points[:, 0] = cx + (points[:, 0] - 0.5) * width_x
        points[:, 1] = cy + (points[:, 1] - 0.5) * width_y
        points[:, 2] *= height
        return points

    target_base = local_block(0.30, 0.00, 0.025, 0.025, 0.040, 300)
    distractor_base = local_block(0.30, 0.12, 0.060, 0.060, 0.080, 900)
    table_base = np.column_stack([local_rng.uniform(0.20, 0.40, 500),
                                  local_rng.uniform(-0.05, 0.18, 500),
                                  np.zeros(500)])
    points_base = np.vstack([target_base, distractor_base, table_base])
    # Optical coordinates need only preserve metric neighbourhoods here.
    points_optical = points_base + np.array([0.0, 0.0, 0.40])
    target_pixels = local_rng.normal([280.0, 240.0], [7.0, 7.0], (300, 2))
    distractor_pixels = local_rng.normal(
        [390.0, 240.0], [18.0, 18.0], (900, 2))
    table_pixels = local_rng.uniform(
        [240.0, 190.0], [440.0, 300.0], (500, 2))
    pixels = np.vstack([target_pixels, distractor_pixels, table_pixels])
    colors = np.arange(points_base.shape[0] * 3, dtype=np.uint8).reshape(-1, 3)
    rgbd = GraspScene(
        points_optical=points_optical, points_base=points_base, pixels=pixels,
        colors=colors, p_wc=np.zeros(3), R_wc=np.eye(3),
        intrinsics=(600.0, 600.0, 320.0, 240.0),
        source_indices=np.arange(points_base.shape[0], dtype=np.int64))

    selected = segment_target_component(
        rgbd, bbox=(230.0, 180.0, 450.0, 310.0),
        target_pixel=(280.0, 240.0), pad_px=0.0,
        depth_tolerance=0.10, min_points=30)

    assert selected is not None
    assert selected.points_base.shape[0] < distractor_base.shape[0]
    assert np.allclose(np.median(selected.points_base[:, :2], axis=0),
                       [0.30, 0.00], atol=0.015)
    assert np.all(selected.points_base[:, 2] > 0.006), "table must be removed"
    assert selected.colors.shape[0] == selected.points_base.shape[0]
    assert np.allclose(
        selected.points_base, points_base[selected.source_indices]), \
        "target segmentation must retain each point's capture-time identity"


def test_target_segmentation_fails_instead_of_feeding_the_table_to_graspnet():
    local_rng = np.random.default_rng(8)
    points = np.column_stack([local_rng.uniform(0.2, 0.4, 200),
                              local_rng.uniform(-0.1, 0.1, 200),
                              np.zeros(200)])
    pixels = local_rng.normal([320.0, 240.0], [20.0, 20.0], (200, 2))
    assert segment_target_component(
        scene_with_pixels(points, pixels),
        bbox=(250.0, 180.0, 390.0, 300.0),
        target_pixel=(320.0, 240.0), min_points=20) is None


# --- source-point provenance for exact AnyGrasp region steering ------------

def indexed_scene(points, source_indices, *, pixels=None, colors=None,
                  p_wc=None, R_wc=None):
    points = np.asarray(points, dtype=float).reshape(-1, 3)
    if pixels is None:
        pixels = np.column_stack([
            np.arange(points.shape[0], dtype=float) * 100.0 + 10.0,
            np.arange(points.shape[0], dtype=float) * 100.0 + 10.0,
        ])
    return GraspScene(
        points_optical=points.copy(), points_base=points.copy(),
        pixels=np.asarray(pixels, dtype=float).reshape(-1, 2), colors=colors,
        p_wc=np.zeros(3) if p_wc is None else np.asarray(p_wc, dtype=float),
        R_wc=np.eye(3) if R_wc is None else np.asarray(R_wc, dtype=float),
        intrinsics=(600.0, 600.0, 320.0, 240.0),
        source_indices=(None if source_indices is None else
                        np.asarray(source_indices, dtype=np.int64)))


def test_masked_scene_keeps_source_indices_aligned_with_every_point_array():
    points = np.array([[0.10, 0.0, 0.1], [0.20, 0.0, 0.2],
                       [0.30, 0.0, 0.3], [0.40, 0.0, 0.4]])
    pixels = np.array([[10, 11], [20, 21], [30, 31], [40, 41]])
    colors = np.arange(12, dtype=np.uint8).reshape(4, 3)
    scene = indexed_scene(points, [101, 102, 103, 104],
                          pixels=pixels, colors=colors)

    subset = _masked_scene(scene, np.array([False, True, False, True]))

    assert np.array_equal(subset.source_indices, [102, 104])
    assert np.array_equal(subset.points_optical, points[[1, 3]])
    assert np.array_equal(subset.points_base, points[[1, 3]])
    assert np.array_equal(subset.pixels, pixels[[1, 3]])
    assert np.array_equal(subset.colors, colors[[1, 3]])


def test_public_crops_preserve_capture_source_indices():
    points = np.array([[0.10, 0.0, 0.1], [0.20, 0.0, 0.2],
                       [0.30, 0.0, 0.3], [0.40, 0.0, 0.4]])
    pixels = np.array([[10, 10], [110, 110], [210, 210], [310, 310]])
    scene = indexed_scene(points, [10, 20, 30, 40], pixels=pixels)

    image_crop = crop_to_bbox(
        scene, bbox=(100.0, 100.0, 220.0, 220.0), pad_px=0.0)
    workspace_crop = crop_to_workspace(
        scene, workspace_min=[0.15, -0.1, 0.0],
        workspace_max=[0.35, 0.1, 0.5])

    assert image_crop is not None
    assert np.array_equal(image_crop.source_indices, [20, 30])
    assert np.array_equal(workspace_crop.source_indices, [20, 30])


def test_target_region_mask_matches_capture_ids_not_target_row_order():
    full = indexed_scene(
        np.array([[0.10, 0.0, 0.2], [0.20, 0.0, 0.2],
                  [0.30, 0.0, 0.2], [0.40, 0.0, 0.2]]),
        [10, 20, 30, 40])
    target = indexed_scene(full.points_optical[[3, 1]], [40, 20])

    mask = target_region_mask(full, target)

    assert mask.shape == (4,)
    assert mask.dtype == np.bool_
    assert np.array_equal(mask, [False, True, False, True])


@pytest.mark.parametrize("bad_case", [
    "scene_ids_missing",
    "target_ids_missing",
    "scene_length_mismatch",
    "target_length_mismatch",
    "target_id_outside_scene",
    "duplicate_target_id",
    "target_coordinate_mismatch",
    "noninteger_ids",
    "empty_target",
])
def test_target_region_mask_fails_closed_on_invalid_provenance(bad_case):
    full_points = np.array([[0.10, 0.0, 0.2], [0.20, 0.0, 0.2],
                            [0.30, 0.0, 0.2]])
    full = indexed_scene(full_points, [10, 20, 30])
    target = indexed_scene(full_points[[1]], [20])
    if bad_case == "scene_ids_missing":
        full.source_indices = None
    elif bad_case == "target_ids_missing":
        target.source_indices = None
    elif bad_case == "scene_length_mismatch":
        full.source_indices = np.array([10, 20])
    elif bad_case == "target_length_mismatch":
        target.source_indices = np.array([20, 30])
    elif bad_case == "target_id_outside_scene":
        target.source_indices = np.array([999])
    elif bad_case == "duplicate_target_id":
        target = indexed_scene(full_points[[1, 1]], [20, 20])
    elif bad_case == "target_coordinate_mismatch":
        target.points_base[0, 0] += 0.01
    elif bad_case == "noninteger_ids":
        target.source_indices = np.array([20.0])
    elif bad_case == "empty_target":
        target = indexed_scene(np.zeros((0, 3)), np.zeros(0, dtype=np.int64))

    with pytest.raises(ValueError):
        target_region_mask(full, target)


def test_a_gappy_target_is_found_by_detect_single_but_not_plain_detect():
    # A thin/glossy object (a pen, the standing real-hardware example) gives a
    # real depth camera a sparse, gappy return — n=60 here leaves real chain
    # gaps along the 14 cm length at the default 6 mm voxel size, which is
    # exactly what breaks plain multi-object detect()'s connectivity
    # requirement (min_points inside ONE connected voxel chain). This is not
    # an unrealistic synthetic case: it is what actually happened on hardware.
    pen = block(0.30, 0.0, 0.010, 0.140, 0.010, n=60)
    # table_margin lower than default: a pen lying on its side is only about
    # as tall as its own diameter, thin enough that the default margin would
    # filter out its points as "at table level" before clustering runs at all.
    backend = AnalyticGraspBackend(min_points=40, table_margin=0.003)
    pen_scene = scene_with_pixels(pen, np.zeros((60, 2)))

    assert backend.detect(pen_scene) == [], \
        "gappy connectivity should defeat plain multi-object detect()"

    single = backend.detect_single(pen_scene)
    assert single, "detect_single should use every above-table point directly"
    assert np.allclose(single[0].position[:2], [0.30, 0.0], atol=0.02)


def test_crop_to_bbox_then_detect_single_finds_a_target_buried_in_clutter():
    pen = block(0.30, 0.0, 0.010, 0.140, 0.010, n=60)
    pen_pixels = np.column_stack([
        np.full(60, 320.0) + RNG.normal(scale=5, size=60),
        np.full(60, 240.0) + RNG.normal(scale=30, size=60)])
    # Clutter's pixels are kept clear of the crop window, the way a real
    # camera's would be: a real surface's pixel footprint and its 3D position
    # move together, so clutter that shares none of the target's image region
    # cannot also share its depth. (Pen and clutter positions are already
    # 5+ cm apart in 3D, so voxel_clusters would separate them regardless —
    # this just keeps the crop step itself physically honest.)
    clutter = block(0.25, 0.15, 0.20, 0.20, 0.15, n=4000)
    clutter_pixels = np.column_stack([RNG.uniform(400, 640, 4000),
                                      RNG.uniform(0, 480, 4000)])
    points = np.vstack([pen, clutter])
    pixels = np.vstack([pen_pixels, clutter_pixels])
    backend = AnalyticGraspBackend(min_points=40, table_margin=0.003)

    cropped = crop_to_bbox(scene_with_pixels(points, pixels),
                           bbox=(280.0, 180.0, 360.0, 300.0), pad_px=10.0)
    assert cropped is not None
    candidates = backend.detect_single(cropped)
    assert candidates
    assert np.allclose(candidates[0].position[:2], [0.30, 0.0], atol=0.02)


# --- detect_single: foreground depth band and its own table margin --------

def test_foreground_band_drops_a_background_surface():
    # Foreground: a pen-height blob near the camera. Background: a much
    # larger, further surface sharing the same pixel window (a box the pen
    # sits on, sharing the crop as measured on hardware).
    near = block(0.30, 0.0, 0.03, 0.14, 0.010, n=200)
    near_opt = near.copy()
    near_opt[:, 2] = 0.20 + near[:, 2]     # ~0.20 m deep
    far = block(0.30, 0.0, 0.30, 0.30, 0.05, n=2000)
    far_opt = far.copy()
    far_opt[:, 2] = 0.45 + far[:, 2]       # ~0.45 m deep
    points_base = np.vstack([near, far])
    points_optical = np.vstack([near_opt, far_opt])
    pixels = np.zeros((points_base.shape[0], 2))
    scene = GraspScene(points_optical=points_optical, points_base=points_base,
                       pixels=pixels, colors=None, p_wc=np.zeros(3),
                       R_wc=np.eye(3), intrinsics=(600., 600., 320., 240.))

    backend = AnalyticGraspBackend(min_points=40)
    without_band = backend.detect_single(scene, foreground_band_m=0.0)
    with_band = backend.detect_single(scene, foreground_band_m=0.05,
                                      table_margin=0.003)
    assert without_band == [], \
        "the merged near+far blob should fail the jaw-width check"
    assert with_band, "the foreground band should isolate the near object alone"
    assert with_band[0].width * 1000 < 38


def test_detect_single_table_margin_override_does_not_mutate_the_backend():
    pts = block(0.30, 0.0, 0.010, 0.140, 0.006, n=200)
    scene = scene_from(pts)
    backend = AnalyticGraspBackend()
    original_margin = backend.table_margin
    backend.detect_single(scene, foreground_band_m=0.0, table_margin=0.001)
    assert backend.table_margin == original_margin


def test_detect_single_table_margin_none_falls_back_to_backend_default():
    pts = block(0.30, 0.0, 0.025, 0.060, 0.05, n=200)
    scene = scene_from(pts)
    backend = AnalyticGraspBackend()
    with_default = backend.detect_single(scene, foreground_band_m=0.0)
    with_explicit_same = backend.detect_single(
        scene, foreground_band_m=0.0, table_margin=backend.table_margin)
    assert len(with_default) == len(with_explicit_same)


# --- graspnet-baseline adapter, using lightweight fake runtime modules -------

def fake_graspnet_checkout(tmp_path, *, collision=True):
    repo = tmp_path / "graspnet-baseline"
    (repo / "models").mkdir(parents=True)
    (repo / "utils").mkdir()
    (repo / "dataset").mkdir()
    (repo / "models" / "graspnet.py").write_text("# fake for import tests\n")
    if collision:
        (repo / "utils" / "collision_detector.py").write_text(
            "# fake for import tests\n")
    checkpoint = repo / "checkpoint-rs.tar"
    checkpoint.write_bytes(b"fake checkpoint")
    return repo, checkpoint


def install_fake_graspnet_runtime(monkeypatch, events):
    class FakeTensor:
        def __init__(self, value):
            self.value = np.asarray(value)

        def to(self, device):
            events.append(("tensor_to", device))
            return self

        def detach(self):
            return self

        def cpu(self):
            return self

        def numpy(self):
            return self.value

    class FakeNet:
        def __init__(self, **kwargs):
            events.append(("net_init", kwargs))

        def to(self, device):
            events.append(("net_to", device))

        def load_state_dict(self, state, strict=True):
            events.append(("load_state", state, strict))

        def eval(self):
            events.append("eval")

        def __call__(self, batch):
            events.append(("net_call", batch))
            return {"encoded_features": "not already decoded grasps"}

    class FakeGrasp:
        def __init__(self, identifier):
            self.identifier = int(identifier)
            self.rotation_matrix = np.eye(3)
            self.translation = np.array([0.1 * identifier, 0.0, 0.3])
            self.depth = 0.02
            self.width = 0.025
            self.score = float(identifier)

    class FakeGraspGroup:
        def __init__(self, values):
            array = np.asarray(values).reshape(-1)
            self.items = [FakeGrasp(value) for value in array]
            events.append(("group", [g.identifier for g in self.items]))

        @classmethod
        def from_items(cls, items):
            obj = object.__new__(cls)
            obj.items = list(items)
            return obj

        def __len__(self):
            return len(self.items)

        def __iter__(self):
            return iter(self.items)

        @property
        def widths(self):
            return np.asarray([grasp.width for grasp in self.items])

        @widths.setter
        def widths(self, values):
            values = np.asarray(values, dtype=float)
            assert values.shape == (len(self.items),)
            for grasp, value in zip(self.items, values):
                grasp.width = float(value)

        def __getitem__(self, key):
            if isinstance(key, slice):
                return self.from_items(self.items[key])
            mask = np.asarray(key)
            if mask.dtype == bool:
                return self.from_items(
                    [item for item, keep in zip(self.items, mask) if keep])
            return self.items[key]

        def nms(self):
            events.append(("nms", [g.identifier for g in self.items]))
            # Deliberately return a NEW group. The adapter must retain it.
            return self.from_items(self.items[-1:])

        def sort_by_score(self):
            events.append(("sort", [g.identifier for g in self.items]))
            self.items.sort(key=lambda grasp: -grasp.score)
            # Older graspnetAPI builds mutate and return None.

    class FakeCollisionDetector:
        def __init__(self, cloud, voxel_size):
            events.append(("collision_init", cloud.copy(), voxel_size))

        def detect(self, grasps, *, approach_dist, collision_thresh,
                   return_empty_grasp, empty_thresh):
            events.append(("collision_widths",
                           [grasp.width for grasp in grasps]))
            events.append(("collision_detect", [g.identifier for g in grasps],
                           approach_dist, collision_thresh,
                           return_empty_grasp, empty_thresh))
            collision = np.array([False, True, False])
            empty = np.array([False, False, False])
            return (collision, empty) if return_empty_grasp else collision

    fake_torch = types.ModuleType("torch")
    fake_torch.cuda = types.SimpleNamespace(is_available=lambda: True)
    fake_torch.device = lambda name: f"device:{name}"
    fake_torch.load = lambda path, map_location, weights_only: {
        "model_state_dict": {"weight": 1}}
    fake_torch.from_numpy = FakeTensor

    # contextlib.nullcontext exists on every supported Python; defining this
    # tiny object here keeps the fake Torch surface explicit and dependency-free.
    class NoGrad:
        def __enter__(self):
            events.append("no_grad_enter")

        def __exit__(self, *_args):
            events.append("no_grad_exit")

    fake_torch.no_grad = NoGrad

    fake_model = types.ModuleType("graspnet")
    fake_model.GraspNet = FakeNet

    def pred_decode(end_points):
        events.append(("pred_decode", end_points))
        return [FakeTensor(np.array([1, 2, 3]))]

    fake_model.pred_decode = pred_decode
    fake_api = types.ModuleType("graspnetAPI")
    fake_api.GraspGroup = FakeGraspGroup
    fake_collision = types.ModuleType("collision_detector")
    fake_collision.ModelFreeCollisionDetector = FakeCollisionDetector
    monkeypatch.setitem(sys.modules, "torch", fake_torch)
    monkeypatch.setitem(sys.modules, "graspnet", fake_model)
    monkeypatch.setitem(sys.modules, "graspnetAPI", fake_api)
    monkeypatch.setitem(sys.modules, "collision_detector", fake_collision)


def test_graspnet_availability_imports_torch_instead_of_only_finding_it(
        tmp_path, monkeypatch):
    repo, checkpoint = fake_graspnet_checkout(tmp_path)
    backend = GraspNetBackend(repo_path=str(repo),
                              checkpoint_path=str(checkpoint))

    real_import = __import__("importlib").import_module

    def broken_torch(name):
        if name == "torch":
            raise OSError("libtorch_global_deps.so is missing")
        return real_import(name)

    monkeypatch.setattr(
        "om6dof_pick_and_place_gemini.grasp_backends.importlib.import_module",
        broken_torch)
    assert not backend.available()
    assert "libtorch_global_deps.so" in backend.availability_error


def test_graspnet_availability_rejects_an_incomplete_checkout(tmp_path):
    repo = tmp_path / "not-graspnet"
    repo.mkdir()
    checkpoint = repo / "checkpoint.tar"
    checkpoint.write_bytes(b"fake")
    backend = GraspNetBackend(repo_path=str(repo),
                              checkpoint_path=str(checkpoint))
    assert not backend.available()
    assert "models/graspnet.py" in backend.availability_error


def test_graspnet_uses_upstream_decode_collision_and_returned_nms(
        tmp_path, monkeypatch):
    repo, checkpoint = fake_graspnet_checkout(tmp_path)
    events = []
    install_fake_graspnet_runtime(monkeypatch, events)
    backend = GraspNetBackend(
        repo_path=str(repo), checkpoint_path=str(checkpoint),
        num_point=4, collision_thresh=0.07, voxel_size=0.006,
        device="cuda:0", top_k=10)
    scene = scene_from(np.array([
        [0.0, 0.0, 0.2], [0.1, 0.0, 0.2],
        [0.2, 0.0, 0.2], [0.3, 0.0, 0.2],
        [0.4, 0.0, 0.2]], dtype=np.float32))

    assert backend.available()
    collision_scene = scene_from(np.zeros((7, 3), dtype=np.float32))
    candidates = backend.detect(scene, collision_scene=collision_scene)

    assert any(event[0] == "pred_decode" for event in events
               if isinstance(event, tuple))
    collision = next(event for event in events
                     if isinstance(event, tuple)
                     and event[0] == "collision_detect")
    assert collision[1:] == ([1, 2, 3], 0.05, 0.07, True, 0.01)
    collision_init = next(event for event in events
                          if isinstance(event, tuple)
                          and event[0] == "collision_init")
    assert collision_init[1].shape == (7, 3), \
        "collision detection must use the wider scene, not target/network input"
    assert collision_init[2] == 0.006
    assert ("nms", [1, 3]) in events
    assert ("sort", [3]) in events, \
        "the new GraspGroup returned by nms() must be retained"
    assert len(candidates) == 1
    assert candidates[0].score == 3.0
    assert np.allclose(candidates[0].position, [0.3, 0.0, 0.3]), \
        "grasp.translation is already the centre; depth must not shift it"


def test_graspnet_clips_hardware_width_before_collision(tmp_path, monkeypatch):
    repo, checkpoint = fake_graspnet_checkout(tmp_path)
    events = []
    install_fake_graspnet_runtime(monkeypatch, events)
    backend = GraspNetBackend(
        repo_path=str(repo), checkpoint_path=str(checkpoint), num_point=3,
        max_width=0.020)

    candidates = backend.detect(
        scene_from(np.ones((3, 3), dtype=np.float32)))

    collision_widths = next(
        event[1] for event in events if event[0] == "collision_widths")
    assert collision_widths == pytest.approx([0.020, 0.020, 0.020])
    assert len(candidates) == 1
    assert candidates[0].width == pytest.approx(0.020)
    assert candidates[0].extras["graspnet_width_raw"] == pytest.approx(0.025)
    assert candidates[0].extras["graspnet_width_clipped"] == 1.0


def test_graspnet_returns_cleanly_when_collision_filter_removes_everything(
        tmp_path, monkeypatch):
    repo, checkpoint = fake_graspnet_checkout(tmp_path)
    events = []
    install_fake_graspnet_runtime(monkeypatch, events)

    class AllCollisionDetector:
        def __init__(self, _cloud, voxel_size):
            assert voxel_size == 0.01

        def detect(self, grasps, *, approach_dist, collision_thresh,
                   return_empty_grasp, empty_thresh):
            assert approach_dist == 0.05
            assert collision_thresh == 0.01
            assert return_empty_grasp and empty_thresh == 0.01
            return (np.ones(len(grasps), dtype=bool),
                    np.zeros(len(grasps), dtype=bool))

    sys.modules["collision_detector"].ModelFreeCollisionDetector = \
        AllCollisionDetector
    backend = GraspNetBackend(repo_path=str(repo),
                              checkpoint_path=str(checkpoint), num_point=3)
    assert backend.detect(scene_from(np.ones((3, 3), dtype=np.float32))) == []
    assert not any(isinstance(event, tuple) and event[0] == "nms"
                   for event in events)


def test_graspnet_rejects_an_empty_grasp(tmp_path, monkeypatch):
    repo, checkpoint = fake_graspnet_checkout(tmp_path)
    events = []
    install_fake_graspnet_runtime(monkeypatch, events)

    class EmptyDetector:
        def __init__(self, _cloud, voxel_size):
            assert voxel_size == 0.01

        def detect(self, grasps, *, approach_dist, collision_thresh,
                   return_empty_grasp, empty_thresh):
            assert return_empty_grasp and empty_thresh == 0.02
            return (np.zeros(len(grasps), dtype=bool),
                    np.ones(len(grasps), dtype=bool))

    sys.modules["collision_detector"].ModelFreeCollisionDetector = EmptyDetector
    backend = GraspNetBackend(
        repo_path=str(repo), checkpoint_path=str(checkpoint), num_point=3,
        empty_thresh=0.02)
    assert backend.detect(scene_from(np.ones((3, 3), dtype=np.float32))) == []


# --- licensed AnyGrasp adapter, using a fake gsnet binary/API ---------------

def fake_anygrasp_install(tmp_path, *, binary=True, checkpoint=True,
                          license_config=True):
    runtime = tmp_path / "grasp_detection"
    runtime.mkdir(parents=True)
    if binary:
        (runtime / "gsnet.so").write_bytes(b"fake extension")
    checkpoint_path = runtime / "checkpoint_detection.tar"
    if checkpoint:
        checkpoint_path.write_bytes(b"fake checkpoint")
    license_dir = runtime / "license"
    if license_config:
        license_dir.mkdir()
        (license_dir / "licenseCfg.json").write_text(
            '{"feature_id": "fake"}\n')
    return runtime, checkpoint_path, license_dir


def install_fake_anygrasp_runtime(monkeypatch, events):
    state = {
        "factory_returns_none": False,
        "grasp_output": None,
        "nms_ids": None,
    }

    class FakeGrasp:
        def __init__(self, identifier, score, *, translation=None,
                     rotation_matrix=None, width=0.025, depth=0.020):
            self.identifier = int(identifier)
            self.score = float(score)
            self.translation = np.asarray(
                ([0.1 * identifier, 0.0, 0.3]
                 if translation is None else translation), dtype=float)
            self.rotation_matrix = np.asarray(
                (np.eye(3) if rotation_matrix is None else rotation_matrix),
                dtype=float)
            self.width = float(width)
            self.depth = float(depth)

    class FakeGraspGroup:
        def __init__(self, items):
            self.items = list(items)

        def __len__(self):
            return len(self.items)

        def __iter__(self):
            return iter(self.items)

        def __getitem__(self, key):
            if isinstance(key, slice):
                return FakeGraspGroup(self.items[key])
            return self.items[key]

        def nms(self):
            events.append(("anygrasp_nms",
                           [grasp.identifier for grasp in self.items]))
            requested = state["nms_ids"]
            if requested is None:
                selected = list(self.items)
            else:
                by_id = {grasp.identifier: grasp for grasp in self.items}
                selected = [by_id[identifier] for identifier in requested]
            # The current API returns a new object. Ignoring this return value
            # silently keeps candidates that NMS explicitly removed.
            return FakeGraspGroup(selected)

        def sort_by_score(self):
            events.append(("anygrasp_sort",
                           [grasp.identifier for grasp in self.items]))
            # Return a new object here too, so the test catches code that only
            # handles old, in-place graspnetAPI builds.
            return FakeGraspGroup(sorted(
                self.items, key=lambda grasp: -grasp.score))

    class FakeDetector:
        def get_grasp(self, points, optional_params):
            copied_params = {
                key: (value.copy() if isinstance(value, np.ndarray) else value)
                for key, value in optional_params.items()
            }
            events.append(("anygrasp_get", np.asarray(points).copy(),
                           copied_params))
            return state["grasp_output"]

    detector = FakeDetector()
    fake_gsnet = types.ModuleType("gsnet")

    def create_detector(config):
        events.append(("anygrasp_create", config))
        return None if state["factory_returns_none"] else detector

    fake_gsnet.create_detector = create_detector
    monkeypatch.setitem(sys.modules, "gsnet", fake_gsnet)
    return state, FakeGrasp, FakeGraspGroup


def make_fake_anygrasp_backend(tmp_path, monkeypatch, events, **kwargs):
    runtime, checkpoint, license_dir = fake_anygrasp_install(tmp_path)
    state, grasp_cls, group_cls = install_fake_anygrasp_runtime(
        monkeypatch, events)
    backend = AnyGraspBackend(
        runtime_dir=str(runtime), checkpoint_path=str(checkpoint),
        license_dir=str(license_dir), **kwargs)
    return backend, state, grasp_cls, group_cls


def test_anygrasp_loads_create_detector_once_with_expected_config(
        tmp_path, monkeypatch):
    events = []
    backend, _state, _grasp_cls, _group_cls = make_fake_anygrasp_backend(
        tmp_path, monkeypatch, events, max_width=0.065,
        gripper_height=0.058)

    assert backend.available()
    backend.load()
    backend.load()

    creates = [event for event in events if event[0] == "anygrasp_create"]
    assert len(creates) == 1
    config = creates[0][1]
    assert config.checkpoint_path == backend.checkpoint_path
    assert config.max_gripper_width == pytest.approx(0.065)
    assert config.gripper_height == pytest.approx(0.058)


def test_anygrasp_passes_full_scene_aligned_region_mask_with_collision_enabled(
        tmp_path, monkeypatch):
    events = []
    backend, _state, _grasp_cls, _group_cls = make_fake_anygrasp_backend(
        tmp_path, monkeypatch, events)
    full = indexed_scene(
        np.array([[0.01, 0.0, 0.20], [0.02, 0.0, 0.20],
                  [0.03, 0.0, 0.20], [0.04, 0.0, 0.20]]),
        [10, 20, 30, 40])
    target = indexed_scene(full.points_optical[[3, 1]], [40, 20])
    region_mask = target_region_mask(full, target)
    assert backend.detect(
        full, collision_scene=full,
        region_mask=region_mask) == []

    _, received_points, params = next(
        event for event in events if event[0] == "anygrasp_get")
    assert received_points.dtype == np.float32
    assert np.array_equal(received_points,
                          full.points_optical.astype(np.float32))
    received_mask = params["region_steering"]
    assert received_mask.shape == (4,)
    assert received_mask.dtype == np.bool_
    assert np.array_equal(received_mask, [False, True, False, True])
    assert np.array_equal(received_points[received_mask],
                          full.points_optical[[1, 3]].astype(np.float32))
    assert params["collision_detection"] is True
    assert params["dense_grasp"] is False


def test_anygrasp_rejects_a_separate_collision_cloud_fail_closed(
        tmp_path, monkeypatch):
    events = []
    backend, _state, _grasp_cls, _group_cls = make_fake_anygrasp_backend(
        tmp_path, monkeypatch, events)
    full = indexed_scene(np.ones((4, 3)), [0, 1, 2, 3])
    unrelated_collision_scene = indexed_scene(
        np.array([[9.0, 9.0, 9.0]]), [999])

    with pytest.raises(ValueError, match="same full GraspScene"):
        backend.detect(full, collision_scene=unrelated_collision_scene)

    assert not any(event[0] == "anygrasp_get" for event in events)


@pytest.mark.parametrize(("region_mask", "error_word"), [
    pytest.param(np.array([True, False, True]), "shape", id="too-short"),
    pytest.param(np.ones((4, 1), dtype=bool), "shape", id="two-dimensional"),
    pytest.param(np.array([1, 0, 1, 0], dtype=np.uint8), "bool",
                 id="integer-dtype"),
    pytest.param(np.zeros(4, dtype=bool), "empty", id="no-target-points"),
])
def test_anygrasp_rejects_a_misaligned_or_unsafe_region_mask(
        tmp_path, monkeypatch, region_mask, error_word):
    events = []
    backend, _state, _grasp_cls, _group_cls = make_fake_anygrasp_backend(
        tmp_path, monkeypatch, events)
    scene = indexed_scene(np.ones((4, 3)), [0, 1, 2, 3])

    with pytest.raises(ValueError) as caught:
        backend.detect(scene, region_mask=region_mask)

    assert error_word in str(caught.value).lower()
    assert not any(event[0] == "anygrasp_get" for event in events)


def test_anygrasp_retains_groups_returned_by_nms_and_sort(
        tmp_path, monkeypatch):
    events = []
    backend, state, grasp_cls, group_cls = make_fake_anygrasp_backend(
        tmp_path, monkeypatch, events, top_k=1)
    state["grasp_output"] = group_cls([
        grasp_cls(1, 0.2), grasp_cls(2, 0.9), grasp_cls(3, 0.5)])
    state["nms_ids"] = [1, 3]

    candidates = backend.detect(scene_from(np.ones((3, 3))))

    assert ("anygrasp_nms", [1, 2, 3]) in events
    assert ("anygrasp_sort", [1, 3]) in events, \
        "sort must run on the new group returned by nms()"
    assert len(candidates) == 1
    assert candidates[0].score == pytest.approx(0.5), \
        "the new sorted group must be retained before the top-k slice"
    assert np.allclose(candidates[0].position, [0.3, 0.0, 0.3])


def test_anygrasp_transforms_center_axes_and_documented_tip_to_base(
        tmp_path, monkeypatch):
    events = []
    backend, state, grasp_cls, group_cls = make_fake_anygrasp_backend(
        tmp_path, monkeypatch, events)
    state["grasp_output"] = group_cls([
        grasp_cls(7, 0.8, translation=[0.1, 0.2, 0.3],
                  rotation_matrix=np.eye(3), width=0.031, depth=0.040)])
    R_wc = np.array([[0.0, -1.0, 0.0],
                     [1.0, 0.0, 0.0],
                     [0.0, 0.0, 1.0]])
    scene = indexed_scene(np.array([[0.0, 0.0, 0.2]]), [0],
                          p_wc=[1.0, 2.0, 3.0], R_wc=R_wc)

    candidate = backend.detect(scene)[0]

    assert np.allclose(candidate.position, [0.8, 2.1, 3.3]), \
        "candidate.position is the grasp centre, not the depth-shifted tip"
    assert np.allclose(candidate.approach, [0.0, 1.0, 0.0])
    assert np.allclose(candidate.closing, [-1.0, 0.0, 0.0])
    assert np.allclose(candidate.extras["tcp_position"],
                       [0.8, 2.14, 3.3]), \
        "AnyGrasp USAGE.md defines tip = translation + depth * approach"
    assert candidate.width == pytest.approx(0.031)
    assert candidate.score == pytest.approx(0.8)
    assert candidate.source == "anygrasp"


@pytest.mark.parametrize(("missing", "error_word"), [
    ("runtime", "runtime"),
    ("binary", "gsnet.so"),
    ("checkpoint", "checkpoint"),
    ("license", "licensecfg.json"),
])
def test_anygrasp_availability_reports_missing_install_artifacts(
        tmp_path, monkeypatch, missing, error_word):
    events = []
    install_fake_anygrasp_runtime(monkeypatch, events)
    if missing == "runtime":
        runtime = tmp_path / "missing-runtime"
        checkpoint = tmp_path / "checkpoint_detection.tar"
        checkpoint.write_bytes(b"fake")
        license_dir = tmp_path / "license"
        license_dir.mkdir()
        (license_dir / "licenseCfg.json").write_text("{}\n")
    else:
        runtime, checkpoint, license_dir = fake_anygrasp_install(
            tmp_path, binary=missing != "binary",
            checkpoint=missing != "checkpoint",
            license_config=missing != "license")
    backend = AnyGraspBackend(
        runtime_dir=str(runtime), checkpoint_path=str(checkpoint),
        license_dir=str(license_dir))

    assert not backend.available()
    assert error_word in backend.availability_error.lower()


def test_anygrasp_availability_reports_a_binary_dependency_import_error(
        tmp_path, monkeypatch):
    runtime, checkpoint, license_dir = fake_anygrasp_install(tmp_path)
    real_import = importlib.import_module

    def broken_import(name, package=None):
        if name == "gsnet":
            raise OSError("libMinkowskiEngine.so is missing")
        return real_import(name, package)

    monkeypatch.setattr(
        "om6dof_pick_and_place_gemini.grasp_backends.importlib.import_module",
        broken_import)
    backend = AnyGraspBackend(
        runtime_dir=str(runtime), checkpoint_path=str(checkpoint),
        license_dir=str(license_dir))

    assert not backend.available()
    assert "libminkowskiengine.so is missing" in \
        backend.availability_error.lower()


def test_anygrasp_availability_rejects_a_gsnet_without_create_detector(
        tmp_path, monkeypatch):
    runtime, checkpoint, license_dir = fake_anygrasp_install(tmp_path)
    monkeypatch.setitem(sys.modules, "gsnet", types.ModuleType("gsnet"))
    backend = AnyGraspBackend(
        runtime_dir=str(runtime), checkpoint_path=str(checkpoint),
        license_dir=str(license_dir))

    assert not backend.available()
    assert "create_detector" in backend.availability_error


def test_anygrasp_none_detector_is_reported_as_a_license_failure(
        tmp_path, monkeypatch):
    events = []
    backend, state, _grasp_cls, _group_cls = make_fake_anygrasp_backend(
        tmp_path, monkeypatch, events)
    state["factory_returns_none"] = True

    assert backend.available()
    with pytest.raises(RuntimeError, match="(?i)license"):
        backend.load()


def test_anygrasp_none_grasp_output_returns_cleanly_without_postprocessing(
        tmp_path, monkeypatch):
    events = []
    backend, _state, _grasp_cls, _group_cls = make_fake_anygrasp_backend(
        tmp_path, monkeypatch, events)

    assert backend.detect(scene_from(np.ones((3, 3)))) == []
    assert not any(event[0] in {"anygrasp_nms", "anygrasp_sort"}
                   for event in events)


def test_anygrasp_dense_mode_sorts_but_skips_nms(tmp_path, monkeypatch):
    events = []
    backend, state, grasp_cls, group_cls = make_fake_anygrasp_backend(
        tmp_path, monkeypatch, events, dense_grasp=True)
    state["grasp_output"] = group_cls([
        grasp_cls(1, 0.2), grasp_cls(2, 0.9)])

    candidates = backend.detect(scene_from(np.ones((3, 3))))

    assert not any(event[0] == "anygrasp_nms" for event in events)
    assert ("anygrasp_sort", [1, 2]) in events
    assert [candidate.score for candidate in candidates] == \
        pytest.approx([0.9, 0.2])


def test_make_backend_registers_anygrasp(tmp_path):
    runtime, checkpoint, license_dir = fake_anygrasp_install(tmp_path)

    backend = make_backend(
        "AnyGrasp", anygrasp={
            "runtime_dir": str(runtime),
            "checkpoint_path": str(checkpoint),
            "license_dir": str(license_dir),
        })

    assert isinstance(backend, AnyGraspBackend)
    with pytest.raises(ValueError, match="anygrasp"):
        make_backend("not-a-backend")
