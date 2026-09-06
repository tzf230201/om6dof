import copy
import importlib.util
import math
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
SCRIPT = PACKAGE_ROOT / "scripts" / "reachability_multiscene_benchmark.py"
if not SCRIPT.exists():
    SCRIPT = PACKAGE_ROOT / "reachability_multiscene_benchmark.py"
SPEC = importlib.util.spec_from_file_location("reachability_multiscene_benchmark", SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(MODULE)


def oracle_evidence():
    evidence = {key: True for key in MODULE.REQUIRED_ORACLE_TRUE}
    evidence.update({
        "clear_reason": "ok",
        "dynamic_reason": "ok",
        "detour_joint_positions": [0.2] * 6,
        "hit_fraction": 0.5,
        "target_obstacle_distance_m": 0.1,
        "joint_distance_normalized": 0.2,
        "hit_ee_pose": {
            "position": [0.1, 0.0, 0.35],
            "orientation_xyzw": [0.0, 0.0, 0.0, 1.0],
        },
    })
    return evidence


def point_scene(
    scene_id="scene_000", target_id=1000, obstacle_id=1001,
    source_counter=0, obstacle_kind="point", difficulty="low"
):
    dynamic_nodes = [{
        "id": obstacle_id,
        "class_id": -1,
        "confidence": 1.0,
        "position": [0.1, 0.0, 0.35],
    }]
    dynamic_edges = []
    if obstacle_kind == "segment":
        dynamic_nodes = [
            {"id": obstacle_id, "class_id": -1, "position": [0.1, -0.05, 0.35]},
            {"id": obstacle_id + 1, "class_id": -1, "position": [0.1, 0.05, 0.35]},
        ]
        dynamic_edges = [{
            "source_id": obstacle_id, "target_id": obstacle_id + 1, "cost": 0.1
        }]
    return {
        "scene_id": scene_id,
        "catalog_index": 0,
        "source_counter": source_counter,
        "base_trajectory_id": f"base_{source_counter:06d}",
        "stratum": {"difficulty": difficulty, "obstacle_kind": obstacle_kind},
        "start_joint_positions": [0.0] * 6,
        "target": {
            "id": target_id,
            "class_id": 1,
            "confidence": 1.0,
            "position": [0.2, 0.0, 0.3],
            "source_joint_positions": [0.1] * 6,
        },
        "dynamic": {
            "kind": obstacle_kind,
            "nodes": dynamic_nodes,
            "edges": dynamic_edges,
        },
        "oracle": oracle_evidence(),
    }


def catalog(scene_count=6):
    if scene_count % 2:
        raise ValueError("unit catalog scene_count must be even")
    base_count = scene_count // 2
    scenes = []
    for index in range(scene_count):
        source_counter = index // 2
        obstacle_kind = "point" if index % 2 == 0 else "segment"
        difficulty = MODULE.DIFFICULTIES[
            min(len(MODULE.DIFFICULTIES) - 1,
                source_counter * len(MODULE.DIFFICULTIES) // max(1, base_count))
        ]
        scenes.append(point_scene(
            f"scene_{index:03d}", 1000 + 10 * index, 1001 + 10 * index,
            source_counter, obstacle_kind, difficulty,
        ))
    for index, scene in enumerate(scenes):
        scene["catalog_index"] = index
    return {
        "schema_version": MODULE.CATALOG_SCHEMA_VERSION,
        "catalog_id": "unit_catalog",
        "frame_id": "world",
        "group_name": "arm",
        "end_effector_link": "end_effector_link",
        "joint_names": [f"joint{index}" for index in range(1, 7)],
        "model": {
            "expanded_urdf_sha256": "1" * 64,
            "srdf_sha256": "2" * 64,
            "reachability_parameters_sha256": "3" * 64,
        },
        "generator": {
            "roadmap_independent": True,
            "paired_obstacle_design": True,
            "base_trajectory_count": base_count,
            "master_key_hex": "6" * 64,
            "joint_lower_bounds": [-1.0] * 6,
            "joint_upper_bounds": [1.0] * 6,
            "oracle": {
                "query_mode": True,
                "graph_method": "halton_prm",
                "collision_checks": ["conservative_capsules", "moveit_fcl"],
            },
            "implementation": {
                "generator_script_sha256": "4" * 64,
                "reachability_node_binary_sha256": "5" * 64,
            },
        },
        "scenes": scenes,
    }


class CatalogValidationTest(unittest.TestCase):
    def test_accepts_normalized_point_scene(self):
        normalized = MODULE.validate_catalog(catalog())
        self.assertEqual(normalized["catalog_id"], "unit_catalog")
        self.assertEqual(normalized["scenes"][0]["dynamic"]["kind"], "point")

    def test_accepts_segment_scene(self):
        payload = catalog()
        normalized = MODULE.validate_catalog(payload)
        self.assertEqual(len(normalized["scenes"][1]["dynamic"]["edges"]), 1)

    def test_rejects_nonfinite_joint_and_duplicate_ids(self):
        payload = catalog()
        payload["scenes"][0]["start_joint_positions"][0] = math.nan
        with self.assertRaisesRegex(ValueError, "non-finite"):
            MODULE.validate_catalog(payload)

        payload = catalog()
        payload["scenes"][0]["dynamic"]["nodes"][0]["id"] = 1000
        with self.assertRaisesRegex(ValueError, "duplicate"):
            MODULE.validate_catalog(payload)

    def test_validation_does_not_depend_on_method_or_stream(self):
        payload = catalog()
        first = MODULE.validate_catalog(copy.deepcopy(payload))
        second = MODULE.validate_catalog(copy.deepcopy(payload))
        self.assertEqual(first, second)

    def test_rejects_missing_oracle_evidence_and_duplicate_catalog_index(self):
        payload = catalog()
        payload["scenes"][1]["catalog_index"] = 0
        with self.assertRaisesRegex(ValueError, "catalog_index"):
            MODULE.validate_catalog(payload)

        payload = catalog()
        payload["scenes"][0]["oracle"]["dynamic_exact_direct_blocked"] = False
        with self.assertRaisesRegex(ValueError, "oracle evidence"):
            MODULE.validate_catalog(payload)


class ScheduleTest(unittest.TestCase):
    def test_smoke_schedule_has_18_builds_and_216_queries(self):
        streams = list(range(9000, 9006))
        schedule = MODULE.make_schedule(streams, list(MODULE.SUPPORTED_METHODS), 6)
        self.assertEqual(len(schedule), 18)
        query_ids = {
            query_id
            for run in schedule
            for phases in run["query_ids"].values()
            for query_id in phases.values()
        }
        self.assertEqual(len(query_ids), 216)
        observed_orders = []
        for stream_ordinal in range(6):
            runs = [run for run in schedule if run["stream_ordinal"] == stream_ordinal]
            observed_orders.append(tuple(
                run["method"] for run in sorted(runs, key=lambda item: item["method_order_position"])
            ))
        self.assertEqual(len(set(observed_orders)), 6)

    def test_confirmatory_schedule_has_180_builds_and_10800_queries(self):
        streams = list(range(100, 160))
        schedule = MODULE.make_schedule(streams, list(MODULE.SUPPORTED_METHODS), 30)
        self.assertEqual(len(schedule), 180)
        query_ids = {
            query_id
            for run in schedule
            for phases in run["query_ids"].values()
            for query_id in phases.values()
        }
        self.assertEqual(len(query_ids), 10800)
        order_counts = {}
        for stream_ordinal in range(60):
            runs = [run for run in schedule if run["stream_ordinal"] == stream_ordinal]
            order = tuple(
                run["method"] for run in sorted(runs, key=lambda item: item["method_order_position"])
            )
            order_counts[order] = order_counts.get(order, 0) + 1
        self.assertEqual(set(order_counts.values()), {10})

        first_phase_counts = {}
        for run in schedule:
            for catalog_index, order in run["phase_orders"].items():
                key = (run["method"], catalog_index, order[0])
                first_phase_counts[key] = first_phase_counts.get(key, 0) + 1
        for method in MODULE.SUPPORTED_METHODS:
            for catalog_index in range(30):
                self.assertEqual(first_phase_counts[(method, catalog_index, "clear")], 30)
                self.assertEqual(first_phase_counts[(method, catalog_index, "dynamic")], 30)

    def test_confirmatory_protocol_is_explicit_and_frozen(self):
        payload = catalog(60)
        payload["catalog_id"] = "om6dof_icra_scene_catalog_v3"
        payload["model"] = dict(MODULE.CONFIRMATORY_MODEL_HASHES)
        payload["generator"]["master_key_hex"] = MODULE.CONFIRMATORY_MASTER_KEY_HEX
        normalized = MODULE.validate_catalog(payload)
        args = SimpleNamespace(
            catalog="catalog.json",
            protocol_id=MODULE.CONFIRMATORY_PROTOCOL_ID,
            sample_count=800,
            halton_start_index=17,
            gng_guard_fraction=0.75,
            max_scenes=0,
            source_tree_sha256="a" * 64,
            expected_catalog_sha256="b" * 64,
            rmw_implementation="rmw_fastrtps_cpp",
            catalog_generation_log="catalog_generation.log",
            source_snapshot="source_snapshot.tar.gz",
            protocol_document="confirmatory_protocol.md",
            analyzer_script="analyze_reachability_multiscene.py",
        )
        MODULE.validate_protocol(
            args, normalized, "b" * 64, list(MODULE.SUPPORTED_METHODS),
            list(MODULE.CONFIRMATORY_STREAMS),
        )
        args.protocol_document = ""
        with self.assertRaisesRegex(ValueError, "frozen-input"):
            MODULE.validate_protocol(
                args, normalized, "b" * 64, list(MODULE.SUPPORTED_METHODS),
                list(MODULE.CONFIRMATORY_STREAMS),
            )
        args.protocol_document = "confirmatory_protocol.md"
        args.sample_count = 799
        with self.assertRaisesRegex(ValueError, "sample_count"):
            MODULE.validate_protocol(
                args, normalized, "b" * 64, list(MODULE.SUPPORTED_METHODS),
                list(MODULE.CONFIRMATORY_STREAMS),
            )

    def test_frozen_input_bundle_is_atomic_and_resume_is_fail_closed(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            source_dir = root / "source"
            source_dir.mkdir()
            values = {}
            for index, (name, argument) in enumerate(
                MODULE.FROZEN_INPUT_ARGUMENTS.items()
            ):
                path = source_dir / name
                path.write_bytes(f"frozen-{index}\n".encode("utf-8"))
                values[argument] = str(path)
            args = SimpleNamespace(**values)
            sources = MODULE.resolve_frozen_input_sources(args)
            output = root / "result"
            output.mkdir()
            first = MODULE.freeze_input_bundle(output, sources, resume=False)
            second = MODULE.freeze_input_bundle(output, sources, resume=True)
            self.assertEqual(first, second)
            self.assertEqual(set(first), set(MODULE.FROZEN_INPUT_ARGUMENTS))
            (output / "frozen_inputs" / "confirmatory_protocol.md").write_text(
                "tampered\n", encoding="utf-8"
            )
            with self.assertRaisesRegex(ValueError, "resume frozen input mismatch"):
                MODULE.freeze_input_bundle(output, sources, resume=True)


class RuntimeContractTest(unittest.TestCase):
    def setUp(self):
        self.args = SimpleNamespace(
            sample_count=800,
            halton_start_index=17,
            gng_guard_fraction=0.75,
        )
        self.catalog = {
            "joint_names": [f"joint{index}" for index in range(1, 7)],
            "model": {
                "expanded_urdf_sha256": "1" * 64,
                "srdf_sha256": "2" * 64,
                "reachability_parameters_sha256": "3" * 64,
            },
        }

    def graph(self, method):
        graph = {
            "reported_method": method,
            "graph_revision": 1,
            **self.catalog["model"],
            "requested_node_count": 800,
            "anchor_node_count": 2,
            "prototype_budget": 0,
            "prototype_node_count": 0,
            "requested_guard_node_count": 0,
            "guard_node_count": 0,
            "fill_sample_node_count": 0,
            "candidate_attempts": 900,
            "halton_start_index": 17,
            "sample_stream_seed": 123,
            "sample_stream_type": "digit_permuted_halton",
            "gng_training_sample_count": 0,
            "effective_guard_fraction": 0.75 if method == "guarded_gng" else 0.0,
            "nodes": 800,
            "edges": 100,
            "components": 1,
            "build_time_ms": 1.0,
            "joint_names": self.catalog["joint_names"],
            "graph_publisher_count": 1,
            "plan_publisher_count": 1,
            "query_subscriber_count": 1,
        }
        if method == "gng":
            graph.update(prototype_budget=798, prototype_node_count=798,
                         gng_training_sample_count=4000, candidate_attempts=4100)
        elif method == "guarded_gng":
            graph.update(prototype_budget=199, prototype_node_count=199,
                         requested_guard_node_count=599, guard_node_count=599,
                         gng_training_sample_count=4000, candidate_attempts=4100)
        else:
            graph.update(fill_sample_node_count=798)
        return graph

    def test_exact_method_compositions_are_enforced(self):
        for method in MODULE.SUPPORTED_METHODS:
            run = {"method": method, "stream_id": 123}
            self.assertEqual(
                MODULE.graph_contract_errors(
                    self.graph(method), run, self.args, self.catalog
                ),
                [],
            )
        broken = self.graph("gng")
        broken["prototype_budget"] = 800
        self.assertTrue(MODULE.graph_contract_errors(
            broken, {"method": "gng", "stream_id": 123}, self.args, self.catalog
        ))

    def test_query_contract_catches_valid_exact_mismatch(self):
        scene = point_scene()
        row = {
            "plan_graph_method": "gng",
            "graph_revision": 1,
            "requested_target_environment_node_id": 1000,
            "requested_target_position": [0.2, 0.0, 0.3],
            "preview_start_joints": [0.0] * 6,
            "valid": True,
            "exact_valid": True,
            "path_nodes": 2,
            "path_ids": [1, 2],
            "blocked_nodes": 0,
            "blocked_edges": 0,
            "exact_replans": 0,
            "selected_target_environment_node_id": 1000,
            "start_node_id": 1,
            "goal_node_id": 2,
            "graph_cost": 1.0,
            "start_connection_cost": 0.25,
            "total_joint_path_cost": 1.25,
            "target_distance": 0.01,
            "planning_time_ms": 1.0,
            "publish_to_plan_ms": 2.0,
            "exact_time_ms": 0.5,
        }
        self.assertEqual(
            MODULE.query_contract_errors(row, scene, "clear", 1, 6, "gng"), []
        )
        row["exact_valid"] = False
        self.assertIn(
            "valid_exact_valid_mismatch",
            MODULE.query_contract_errors(row, scene, "clear", 1, 6, "gng"),
        )

    def test_clear_exact_replan_is_valid_but_unexplained_blocking_is_rejected(self):
        scene = point_scene()
        row = {
            "plan_graph_method": "gng",
            "graph_revision": 1,
            "requested_target_environment_node_id": 1000,
            "requested_target_position": [0.2, 0.0, 0.3],
            "preview_start_joints": [0.0] * 6,
            "valid": True,
            "exact_valid": True,
            "path_nodes": 2,
            "path_ids": [1, 2],
            "blocked_nodes": 0,
            "blocked_edges": 1,
            "exact_replans": 1,
            "selected_target_environment_node_id": 1000,
            "start_node_id": 1,
            "goal_node_id": 2,
            "graph_cost": 1.0,
            "start_connection_cost": 0.25,
            "total_joint_path_cost": 1.25,
            "target_distance": 0.01,
            "planning_time_ms": 1.0,
            "publish_to_plan_ms": 2.0,
            "exact_time_ms": 0.5,
        }
        self.assertEqual(
            MODULE.query_contract_errors(row, scene, "clear", 1, 6, "gng"), []
        )
        row["exact_replans"] = 0
        self.assertIn(
            "clear_blocked_edges_exact_replans_mismatch",
            MODULE.query_contract_errors(row, scene, "clear", 1, 6, "gng"),
        )
        row["exact_replans"] = 1
        row["blocked_nodes"] = 1
        self.assertIn(
            "clear_query_reported_blocked_nodes",
            MODULE.query_contract_errors(row, scene, "clear", 1, 6, "gng"),
        )

    def test_resume_accepts_only_complete_matching_runs(self):
        schedule = MODULE.make_schedule([123], ["gng"], 6)
        graph_row = {
            "stream_ordinal": "0",
            "method": "gng",
            "infrastructure_error": "",
            "catalog_sha256": "abc",
            "catalog_id": "unit",
            "reported_method": "gng",
            "roadmap_stream_id": "123",
            "requested_node_count": "800",
            "halton_start_index": "17",
            "sample_stream_seed": "123",
            "expanded_urdf_sha256": "1" * 64,
            "srdf_sha256": "2" * 64,
            "reachability_parameters_sha256": "3" * 64,
            "graph_publisher_count": "1",
            "plan_publisher_count": "1",
            "query_subscriber_count": "1",
        }
        query_rows = [
            {
                "stream_ordinal": "0", "method": "gng", "query_id": str(query_id),
                "infrastructure_error": "", "timeout": "False",
                "plan_graph_method": "gng",
            }
            for query_id in range(1, 13)
        ]
        self.assertEqual(
            MODULE.validate_resume_rows(
                [graph_row], query_rows, schedule, self.args,
                {"catalog_id": "unit", "model": self.catalog["model"]}, "abc",
            ),
            {(0, "gng")},
        )
        with self.assertRaisesRegex(ValueError, "corrupt or failed"):
            MODULE.validate_resume_rows(
                [graph_row], query_rows[:-1], schedule, self.args,
                {"catalog_id": "unit", "model": self.catalog["model"]}, "abc",
            )
        with self.assertRaisesRegex(ValueError, "duplicate query IDs"):
            MODULE.validate_resume_rows(
                [graph_row], [*query_rows[:-1], query_rows[0]], schedule, self.args,
                {"catalog_id": "unit", "model": self.catalog["model"]}, "abc",
            )

    def test_binary_and_plan_method_provenance_are_enforced(self):
        payload = catalog()
        MODULE.validate_node_binary_provenance(payload, "5" * 64)
        with self.assertRaisesRegex(ValueError, "binary differs"):
            MODULE.validate_node_binary_provenance(payload, "a" * 64)

        scene = point_scene()
        errors = MODULE.query_contract_errors(
            {"plan_graph_method": "halton_prm"},
            scene,
            "dynamic",
            1,
            6,
            "gng",
        )
        self.assertIn("plan_graph_method_mismatch", errors)

    @patch.object(MODULE.subprocess, "run")
    def test_domain_preflight_requires_zero_visible_nodes(self, run_mock):
        run_mock.return_value = SimpleNamespace(
            returncode=0, stdout="\n", stderr=""
        )
        MODULE.assert_clean_ros_domain({"ROS_DOMAIN_ID": "42"})
        self.assertIn("--no-daemon", run_mock.call_args.args[0])

        run_mock.return_value = SimpleNamespace(
            returncode=0, stdout="/stale_reachability_graph_node\n", stderr=""
        )
        with self.assertRaisesRegex(ValueError, "not clean"):
            MODULE.assert_clean_ros_domain({"ROS_DOMAIN_ID": "42"})


if __name__ == "__main__":
    unittest.main()
