import importlib.util
import unittest
from pathlib import Path
from types import SimpleNamespace


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
SCRIPT = PACKAGE_ROOT / "scripts" / "generate_reachability_scene_catalog.py"
if not SCRIPT.exists():
    SCRIPT = PACKAGE_ROOT / "generate_reachability_scene_catalog.py"
SPEC = importlib.util.spec_from_file_location("generate_reachability_scene_catalog", SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(MODULE)


class CounterStreamTest(unittest.TestCase):
    def test_sha256_counter_golden_values(self):
        key = bytes.fromhex(MODULE.DEFAULT_MASTER_KEY_HEX)
        expected = {
            "start": 0.082853115893609497,
            "target": 0.34128456951845437,
            "detour": 0.81816725881154984,
            "hit": 0.53697009108715554,
        }
        for tag, value in expected.items():
            self.assertAlmostEqual(MODULE.counter_u53(key, tag, 0, 0), value, places=16)

    def test_joint_samples_stay_inside_margin(self):
        key = bytes.fromhex(MODULE.DEFAULT_MASTER_KEY_HEX)
        sample = MODULE.sample_joints(key, "start", 7, [-1.0, -2.0], [1.0, 2.0], 0.05)
        self.assertTrue(-0.9 <= sample[0] <= 0.9)
        self.assertTrue(-1.8 <= sample[1] <= 1.8)
        self.assertEqual(
            sample,
            MODULE.sample_joints(key, "start", 7, [-1.0, -2.0], [1.0, 2.0], 0.05),
        )

    def test_runtime_model_hashes_are_exact_and_expected_is_normalized(self):
        graph = SimpleNamespace(
            expanded_urdf_sha256="1" * 64,
            srdf_sha256="2" * 64,
            reachability_parameters_sha256="a" * 64,
        )
        args = SimpleNamespace(
            urdf_sha256="1" * 64,
            srdf_sha256="2" * 64,
            parameters_sha256="A" * 64,
        )
        self.assertEqual(
            MODULE.runtime_model_hashes(graph), MODULE.expected_model_hashes(args)
        )


def candidate(counter, score, kind):
    positions = [[0.1, 0.0, 0.3]]
    if kind == "segment":
        positions = [[0.1, -0.05, 0.3], [0.1, 0.05, 0.3]]
    pose = {
        "position": [0.2, 0.0, 0.3],
        "orientation_xyzw": [0.0, 0.0, 0.0, 1.0],
    }
    return {
        "source_counter": counter,
        "base_trajectory_id": f"base_{counter:06d}",
        "detour_attempt": 0,
        "start_joint_positions": [0.0] * 6,
        "target_joint_positions": [0.1] * 6,
        "detour_joint_positions": [0.2] * 6,
        "start_ee_pose": pose,
        "target_ee_pose": pose,
        "hit_ee_pose": pose,
        "hit_fraction": 0.5,
        "obstacle_positions": positions,
        "target_obstacle_distance_m": 0.1,
        "joint_distance_normalized": score,
        "oracle_evidence": {
            "clear_evaluated": True,
            "clear_reason": "ok",
            "clear_start_self_valid": True,
            "clear_target_self_valid": True,
            "clear_detour_self_valid": True,
            "clear_capsule_direct_valid": True,
            "clear_exact_direct_valid": True,
            "dynamic_evaluated": True,
            "dynamic_reason": "ok",
            "dynamic_capsule_direct_blocked": True,
            "dynamic_exact_direct_blocked": True,
            "dynamic_capsule_detour_valid": True,
            "dynamic_exact_detour_valid": True,
            "dynamic_start_valid": True,
            "dynamic_target_valid": True,
            "dynamic_detour_state_valid": True,
        },
    }


class FinalizeScenesTest(unittest.TestCase):
    def test_six_scene_smoke_has_one_scene_per_cell(self):
        candidates = {
            kind: [candidate(index, score, kind) for index, score in enumerate((0.3, 0.1, 0.2))]
            for kind in MODULE.OBSTACLE_KINDS
        }
        scenes = MODULE.finalize_scenes(
            candidates,
            6,
            [f"joint{index}" for index in range(1, 7)],
            [-1.0] * 6,
            [1.0] * 6,
        )
        self.assertEqual(len(scenes), 6)
        cells = {
            (scene["stratum"]["difficulty"], scene["stratum"]["obstacle_kind"])
            for scene in scenes
        }
        self.assertEqual(len(cells), 6)
        self.assertEqual(len({scene["scene_id"] for scene in scenes}), 6)
        self.assertEqual(len({scene["target"]["id"] for scene in scenes}), 6)
        self.assertEqual(len({scene["base_trajectory_id"] for scene in scenes}), 3)
        for base_id in {scene["base_trajectory_id"] for scene in scenes}:
            variants = [scene for scene in scenes if scene["base_trajectory_id"] == base_id]
            self.assertEqual(
                {scene["dynamic"]["kind"] for scene in variants}, {"point", "segment"}
            )
        segment_scenes = [scene for scene in scenes if scene["dynamic"]["kind"] == "segment"]
        self.assertTrue(all(len(scene["dynamic"]["nodes"]) == 2 for scene in segment_scenes))
        self.assertTrue(all(len(scene["dynamic"]["edges"]) == 1 for scene in segment_scenes))

    def test_scene_count_must_be_multiple_of_six(self):
        with self.assertRaisesRegex(ValueError, "multiple of six"):
            MODULE.finalize_scenes({}, 5, ["joint1"], [-1.0], [1.0])

    def test_point_and_segment_sources_must_be_paired(self):
        candidates = {
            "point": [candidate(index, score, "point")
                      for index, score in enumerate((0.1, 0.2, 0.3))],
            "segment": [candidate(index + 10, score, "segment")
                        for index, score in enumerate((0.1, 0.2, 0.3))],
        }
        with self.assertRaisesRegex(ValueError, "identical base trajectories"):
            MODULE.finalize_scenes(
                candidates, 6, [f"joint{index}" for index in range(1, 7)],
                [-1.0] * 6, [1.0] * 6,
            )


if __name__ == "__main__":
    unittest.main()
