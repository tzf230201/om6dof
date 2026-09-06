import copy
import csv
import hashlib
import importlib.util
import itertools
import json
import tempfile
import unittest
from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
SCRIPT = PACKAGE_ROOT / "scripts" / "analyze_reachability_multiscene.py"
if not SCRIPT.exists():
    SCRIPT = PACKAGE_ROOT / "analyze_reachability_multiscene.py"
SPEC = importlib.util.spec_from_file_location(
    "analyze_reachability_multiscene", SCRIPT
)
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(MODULE)


METHODS = ["gng", "guarded_gng", "halton_prm"]
STREAMS = [100, 101, 102]
SCENE_COUNT = 2


def sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def manifest():
    return {
        "schema": MODULE.RUN_SCHEMA,
        "catalog_id": "unit_catalog",
        "catalog_sha256": "a" * 64,
        "methods": list(METHODS),
        "streams": list(STREAMS),
        "scene_count": SCENE_COUNT,
        "graph_builds": len(METHODS) * len(STREAMS),
        "query_rows": len(METHODS) * len(STREAMS) * SCENE_COUNT * 2,
        "sample_count": 800,
        "preview_only": True,
        "ros_localhost_only": True,
        "controller_topics_published": [],
    }


def v2_manifest():
    payload = manifest()
    payload.update({
        "schema": MODULE.RUN_SCHEMA_V2,
        "phase_order_design": MODULE.PHASE_ORDER_DESIGN,
        "catalog_model": {
            "expanded_urdf_sha256": "1" * 64,
            "srdf_sha256": "2" * 64,
            "reachability_parameters_sha256": "3" * 64,
        },
        "catalog_generator_implementation": {
            "generator_script_sha256": "4" * 64,
            "reachability_node_binary_sha256": "5" * 64,
        },
        "implementation": {
            "runner_script_sha256": "6" * 64,
            "reachability_node_binary_sha256": "7" * 64,
            "source_tree_sha256": "8" * 64,
        },
        "artifacts": {
            "run_config.json": "c" * 64,
            "graphs.csv": "9" * 64,
            "queries.csv": "a" * 64,
            "logs": {
                f"run_{index:04d}.log": "b" * 64
                for index in range(payload["graph_builds"])
            },
        },
    })
    return payload


def graph_rows():
    rows = []
    run_index = 0
    for stream_ordinal, stream in enumerate(STREAMS):
        for method_position, method in enumerate(METHODS):
            rows.append({
                "run_index": run_index,
                "stream_ordinal": stream_ordinal,
                "roadmap_stream_id": stream,
                "method": method,
                "method_order_position": method_position,
                "ros_domain_id": 20 + run_index,
                "reported_method": method,
                "graph_revision": 1,
                "requested_node_count": 800,
                "sample_stream_seed": stream,
                "nodes": 800,
                "edges": 4000 + run_index,
                "components": 1,
                "build_time_ms": 100.0 + 10.0 * method_position + stream_ordinal,
                "catalog_id": "unit_catalog",
                "catalog_sha256": "a" * 64,
                "ros_localhost_only": True,
                "infrastructure_error": "",
            })
            run_index += 1
    return rows


def success(method, stream_ordinal, catalog_index, phase):
    if phase == "clear":
        return True
    if method == "gng":
        return (stream_ordinal, catalog_index) not in {(1, 1), (2, 1)}
    if method == "guarded_gng":
        return True
    return (stream_ordinal, catalog_index) != (2, 1)


def query_rows(include_phase_order=True, include_pairing=False):
    rows = []
    query_id = 1
    graphs = {
        (row["roadmap_stream_id"], row["method"]): row for row in graph_rows()
    }
    for stream_ordinal, stream in enumerate(STREAMS):
        for method in METHODS:
            graph = graphs[(stream, method)]
            for catalog_index in range(SCENE_COUNT):
                scene_id = f"scene_{catalog_index:03d}"
                for phase in MODULE.PHASES:
                    valid = success(method, stream_ordinal, catalog_index, phase)
                    changed = phase == "dynamic" and method == "gng"
                    path_ids = [0, 2] if changed else [0, 1]
                    if not valid:
                        path_ids = []
                    target_position = (
                        [0.2, 0.0, 0.3]
                        if include_pairing
                        else [0.2 + catalog_index, 0.0, 0.3]
                    )
                    row = {
                        "run_index": graph["run_index"],
                        "stream_ordinal": stream_ordinal,
                        "roadmap_stream_id": stream,
                        "method": method,
                        "method_order_position": graph["method_order_position"],
                        "ros_domain_id": graph["ros_domain_id"],
                        "graph_revision": 1,
                        "scene_id": scene_id,
                        "catalog_index": catalog_index,
                        "scene_order_position": (
                            catalog_index + stream_ordinal
                        ) % SCENE_COUNT,
                        "phase": phase,
                        "obstacle_kind": "none" if phase == "clear" else "point",
                        "query_id": query_id,
                        "start_joint_positions": json.dumps([0.0] * 6),
                        "target_position": json.dumps(target_position),
                        "requested_target_environment_node_id": 1000 + catalog_index,
                        "requested_target_position": json.dumps(target_position),
                        "selected_target_environment_node_id": 1000 + catalog_index,
                        "valid": valid,
                        "exact_valid": valid,
                        "reason": "path_ready" if valid else "no_path",
                        "timeout": False,
                        "infrastructure_error": "",
                        "blocked_nodes": 0 if phase == "clear" else 2,
                        "blocked_edges": 0 if phase == "clear" else 3,
                        "planning_time_ms": (
                            2.0 + stream_ordinal + (1.0 if phase == "dynamic" else 0.0)
                        ),
                        "publish_to_plan_ms": 4.0,
                        "graph_cost": 3.0 if valid else 0.0,
                        "start_connection_cost": 1.0 if valid else 0.0,
                        "total_joint_path_cost": 4.0 if valid else 0.0,
                        "path_nodes": len(path_ids),
                        "path_ids": json.dumps(path_ids),
                        "preview_start_joints": json.dumps([0.0] * 6 if valid else []),
                    }
                    if include_phase_order:
                        clear_position = (stream_ordinal + catalog_index) % 2
                        row["phase_order_position"] = (
                            clear_position if phase == "clear" else 1 - clear_position
                        )
                    if include_pairing:
                        row["base_trajectory_id"] = "base_000"
                        row["source_counter"] = 42
                        scene_kind = "point" if catalog_index == 0 else "segment"
                        row["stratum"] = json.dumps({
                            "difficulty": "low", "obstacle_kind": scene_kind,
                        })
                        if phase == "dynamic":
                            row["obstacle_kind"] = scene_kind
                    rows.append(row)
                    query_id += 1
    return rows


def csv_raw(rows):
    return [{key: str(value) for key, value in row.items()} for row in rows]


def audited(include_phase_order=True):
    return MODULE.audit_bundle(
        manifest(), csv_raw(graph_rows()), csv_raw(query_rows(include_phase_order))
    )


def write_bundle(directory, include_phase_order=True):
    directory = Path(directory)
    directory.mkdir(parents=True, exist_ok=True)
    (directory / "manifest.json").write_text(
        json.dumps(manifest(), sort_keys=True, indent=2) + "\n", encoding="utf-8"
    )
    for filename, rows in (
        ("graphs.csv", graph_rows()),
        ("queries.csv", query_rows(include_phase_order)),
    ):
        with (directory / filename).open("w", newline="", encoding="utf-8") as stream:
            writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
            writer.writeheader()
            writer.writerows(rows)


def write_v2_bundle(directory):
    directory = Path(directory)
    write_bundle(directory, include_phase_order=True)
    logs = directory / "logs"
    logs.mkdir()
    run_config_path = directory / "run_config.json"
    run_config_path.write_text(
        json.dumps({"unit": True}, sort_keys=True) + "\n", encoding="utf-8"
    )
    log_hashes = {}
    for index in range(manifest()["graph_builds"]):
        path = logs / f"run_{index:04d}.log"
        path.write_text(f"unit log {index}\n", encoding="utf-8")
        log_hashes[path.name] = sha256(path)
    payload = v2_manifest()
    payload["artifacts"] = {
        "run_config.json": sha256(run_config_path),
        "graphs.csv": sha256(directory / "graphs.csv"),
        "queries.csv": sha256(directory / "queries.csv"),
        "logs": log_hashes,
    }
    (directory / "manifest.json").write_text(
        json.dumps(payload, sort_keys=True, indent=2) + "\n", encoding="utf-8"
    )


class AuditTest(unittest.TestCase):
    def test_complete_bundle_passes_with_and_without_phase_order_column(self):
        current = audited(True)
        self.assertTrue(current["phase_order_position_present"])
        self.assertEqual(current["counts"]["queries"], 36)
        legacy = audited(False)
        self.assertFalse(legacy["phase_order_position_present"])

    def test_rejects_infrastructure_error_immediately(self):
        rows = query_rows()
        rows[4]["infrastructure_error"] = "graph_revision_mismatch"
        with self.assertRaisesRegex(MODULE.AuditError, "infrastructure_error"):
            MODULE.audit_bundle(manifest(), csv_raw(graph_rows()), csv_raw(rows))

    def test_rejects_missing_pair_and_invalid_phase_counterbalance(self):
        rows = query_rows()
        with self.assertRaisesRegex(MODULE.AuditError, "expected 36 rows"):
            MODULE.audit_bundle(
                manifest(), csv_raw(graph_rows()), csv_raw(rows[:-1])
            )
        rows = query_rows()
        rows[0]["phase_order_position"] = 1
        with self.assertRaisesRegex(MODULE.AuditError, "phase order"):
            MODULE.audit_bundle(manifest(), csv_raw(graph_rows()), csv_raw(rows))

    def test_v2_requires_complete_provenance_hashes(self):
        payload = v2_manifest()
        data = MODULE.audit_bundle(
            payload, csv_raw(graph_rows()), csv_raw(query_rows())
        )
        self.assertTrue(data["phase_order_position_present"])

        missing_design = copy.deepcopy(payload)
        del missing_design["phase_order_design"]
        with self.assertRaisesRegex(MODULE.AuditError, "phase_order_design"):
            MODULE.audit_bundle(
                missing_design, csv_raw(graph_rows()), csv_raw(query_rows())
            )

        bad_hash = copy.deepcopy(payload)
        bad_hash["implementation"]["runner_script_sha256"] = "not-a-hash"
        with self.assertRaisesRegex(MODULE.AuditError, "runner_script_sha256"):
            MODULE.audit_bundle(
                bad_hash, csv_raw(graph_rows()), csv_raw(query_rows())
            )

        missing_log = copy.deepcopy(payload)
        missing_log["artifacts"]["logs"].pop("run_0000.log")
        with self.assertRaisesRegex(MODULE.AuditError, "logs count"):
            MODULE.audit_bundle(
                missing_log, csv_raw(graph_rows()), csv_raw(query_rows())
            )

    def test_exact_point_segment_pairing_and_base_count(self):
        rows = query_rows(include_pairing=True)
        data = MODULE.audit_bundle(
            manifest(), csv_raw(graph_rows()), csv_raw(rows)
        )
        self.assertEqual(data["counts"]["base_trajectories"], 1)
        self.assertTrue(data["trajectory_pair_audit"]["exact_point_segment_pairs"])

        wrong_kind = copy.deepcopy(rows)
        for row in wrong_kind:
            if row["catalog_index"] == 1 and row["phase"] == "dynamic":
                row["obstacle_kind"] = "point"
        with self.assertRaisesRegex(
            MODULE.AuditError, "obstacle kind disagrees|pair point and segment"
        ):
            MODULE.audit_bundle(
                manifest(), csv_raw(graph_rows()), csv_raw(wrong_kind)
            )

        wrong_target = copy.deepcopy(rows)
        for row in wrong_target:
            if row["catalog_index"] == 1:
                row["target_position"] = json.dumps([0.9, 0.0, 0.3])
                row["requested_target_position"] = json.dumps([0.9, 0.0, 0.3])
        with self.assertRaisesRegex(MODULE.AuditError, "different start/target"):
            MODULE.audit_bundle(
                manifest(), csv_raw(graph_rows()), csv_raw(wrong_target)
            )


class AnalysisTest(unittest.TestCase):
    def test_rates_path_change_and_paired_statistics(self):
        data = audited()
        methods, comparisons = MODULE.analyze(
            data, manifest(), "smoke_descriptive", repetitions=200, level=0.95,
            seed=7, permutation_repetitions=200,
        )
        self.assertAlmostEqual(methods["gng"]["clear_success"]["estimate"], 1.0)
        self.assertAlmostEqual(methods["gng"]["dynamic_success"]["estimate"], 4 / 6)
        self.assertAlmostEqual(
            methods["halton_prm"]["dynamic_success"]["estimate"], 5 / 6
        )
        self.assertAlmostEqual(
            methods["gng"]["path_change_joint_success"]["estimate"], 1.0
        )
        self.assertAlmostEqual(
            methods["halton_prm"]["path_change_joint_success"]["estimate"], 0.0
        )
        dynamic = next(
            row for row in comparisons
            if row["left_method"] == "gng"
            and row["right_method"] == "halton_prm"
            and row["metric"] == "dynamic_success"
        )
        self.assertAlmostEqual(dynamic["paired_effect"], -1 / 6)
        self.assertFalse(dynamic["inferential_use"])
        self.assertFalse(dynamic["binary_pair_description"]["mcnemar_test_reported"])

    def test_exact_sign_test_and_bootstrap_are_deterministic(self):
        self.assertEqual(MODULE.exact_two_sided_sign_test(3, 0), 0.25)
        contributions = {100: (1.0, 1), 101: (0.0, 1), 102: (1.0, 1)}
        first = MODULE.bootstrap_method_ci(contributions, STREAMS, 200, 0.95, 9)
        second = MODULE.bootstrap_method_ci(contributions, STREAMS, 200, 0.95, 9)
        self.assertEqual(first, second)

    def test_confirmatory_label_guard(self):
        self.assertEqual(
            MODULE.choose_study_kind("auto", 6, 6), "smoke_descriptive"
        )
        self.assertEqual(
            MODULE.choose_study_kind("auto", 60, 30), "smoke_descriptive"
        )
        with self.assertRaisesRegex(MODULE.AuditError, "requires at least"):
            MODULE.choose_study_kind("confirmatory", 49, 30)
        with self.assertRaisesRegex(MODULE.AuditError, "base trajectories"):
            MODULE.choose_study_kind("confirmatory", 50, 29)
        self.assertEqual(
            MODULE.choose_study_kind("confirmatory", 50, 30), "confirmatory"
        )

    def test_primary_family_is_exactly_two_and_holm_is_not_applied_elsewhere(self):
        data = audited()
        _, comparisons = MODULE.analyze(
            data, manifest(), "confirmatory", repetitions=200, level=0.95,
            seed=19, permutation_repetitions=300,
        )
        primary = [
            row for row in comparisons
            if row["hypothesis_role"] == "primary_confirmatory"
        ]
        self.assertEqual(len(primary), 2)
        self.assertEqual(
            {(row["left_method"], row["right_method"], row["metric"])
             for row in primary},
            {
                ("guarded_gng", "gng", "dynamic_success"),
                ("guarded_gng", "halton_prm", "dynamic_success"),
            },
        )
        for row in primary:
            self.assertTrue(row["inferential_use"])
            self.assertEqual(
                row["paired_cluster_bootstrap_ci"]["resampling_unit"],
                "paired_whole_roadmap_stream",
            )
            self.assertEqual(
                row["primary_permutation_test"]["holm_family_size"], 2
            )
        self.assertEqual(
            len({
                row["paired_cluster_bootstrap_ci"]["resampling_seed"]
                for row in primary
            }),
            1,
        )
        self.assertTrue(all(
            not row["inferential_use"] for row in comparisons if row not in primary
        ))

    def test_permutation_and_two_way_sensitivity_are_deterministic(self):
        data = MODULE.audit_bundle(
            manifest(), csv_raw(graph_rows()),
            csv_raw(query_rows(include_pairing=True)),
        )
        differences = MODULE.paired_stream_differences(
            data, "guarded_gng", "gng"
        )
        first = MODULE.paired_stream_permutation(differences, STREAMS, 500, 31)
        second = MODULE.paired_stream_permutation(differences, STREAMS, 500, 31)
        self.assertEqual(first, second)
        self.assertEqual(first["test"], "monte_carlo_paired_label_swap")
        interval = MODULE.two_way_stratified_base_bootstrap_ci(
            data, "guarded_gng", "gng", 200, 0.95, 5
        )
        self.assertTrue(interval["difficulty_stratified"])
        self.assertIn("point_segment_together", interval["resampling_unit"])

    def test_common_success_support_and_descriptive_conditioning(self):
        data = audited()
        excluded = data["query_by_key"][(100, "gng", "scene_000", "dynamic")]
        excluded["exact_valid"] = False
        excluded["total_joint_path_cost"] = 9999.0
        halton = data["query_by_key"][(100, "halton_prm", "scene_000", "dynamic")]
        halton["total_joint_path_cost"] = 1.0
        paired = MODULE.paired_metric_contributions(
            data, "gng", "halton_prm", "path_cost_dynamic"
        )
        self.assertEqual(paired["support_definition"],
                         "common_dynamic_exact_success_cells")
        self.assertEqual(paired["eligible_cells"], 3)
        self.assertNotEqual(MODULE.ratio_estimate(paired["differences"]), 9998.0)

        all_outcomes = MODULE.paired_metric_contributions(
            data, "gng", "halton_prm", "query_latency_dynamic_ms"
        )
        self.assertEqual(all_outcomes["support_definition"],
                         "all_dynamic_query_outcomes")
        self.assertEqual(all_outcomes["eligible_cells"], 6)

        _, comparisons = MODULE.analyze(
            data, manifest(), "smoke_descriptive", repetitions=200, level=0.95,
            seed=3, permutation_repetitions=200,
        )
        retention = next(
            row for row in comparisons
            if row["left_method"] == "gng"
            and row["right_method"] == "halton_prm"
            and row["metric"] == "conditional_retention"
        )
        self.assertFalse(retention["inferential_use"])
        self.assertIsNone(retention["paired_cluster_bootstrap_ci"]["lower"])
        self.assertEqual(
            retention["support_definition"],
            "method_specific_clear_success_denominators",
        )

    def test_stratum_reporting_and_sign_effect_name(self):
        data = MODULE.audit_bundle(
            manifest(), csv_raw(graph_rows()),
            csv_raw(query_rows(include_pairing=True)),
        )
        strata = MODULE.stratified_dynamic_results(data)
        self.assertEqual(
            {row["level"] for row in strata if row["dimension"] == "obstacle_kind"},
            {"point", "segment"},
        )
        paired = MODULE.paired_metric_contributions(
            data, "guarded_gng", "gng", "dynamic_success"
        )
        sign = MODULE.paired_stream_sign_summary(
            paired["differences"], data["streams"]
        )
        self.assertIn("sign_dominance", sign)
        self.assertNotIn("sign_rank_biserial", sign)

    def test_confirmatory_method_order_and_run_config_contract(self):
        methods = list(MODULE.CONFIRMATORY_METHODS)
        orders = list(itertools.permutations(methods))
        scenes = [f"scene_{index:03d}" for index in range(60)]
        scene_metadata = {}
        for index, scene in enumerate(scenes):
            base_index = index // 2
            scene_metadata[scene] = {
                "base_trajectory_id": f"base_{base_index:03d}",
                "difficulty": MODULE.DIFFICULTIES[base_index // 10],
                "obstacle_kind": "point" if index % 2 == 0 else "segment",
            }
        data = {
            "methods": methods,
            "streams": list(MODULE.CONFIRMATORY_STREAMS),
            "scenes": scenes,
            "scene_metadata": scene_metadata,
            "counts": {
                "methods": 3, "streams": 60, "scenes": 60,
                "graphs": 180, "queries": 21600, "base_trajectories": 30,
            },
            "phase_order_position_present": True,
            "phase_order_audit": {"balanced": True},
            "trajectory_pair_audit": {
                "present": True, "exact_point_segment_pairs": True,
            },
            "scene_order_audit": {
                "identical_across_methods": True,
                "expected_catalog_rotation": True,
            },
            "method_order_audit": {
                "permutation_counts": [
                    {"order": list(order), "count": 10} for order in orders
                ],
            },
            "v2_runtime_telemetry": {"complete": True},
        }
        frozen_manifest = v2_manifest()
        frozen_manifest.update({
            "protocol_id": MODULE.CONFIRMATORY_PROTOCOL_ID,
            "catalog_id": MODULE.CONFIRMATORY_CATALOG_ID,
            "catalog_sha256": "a" * 64,
            "methods": methods,
            "streams": list(MODULE.CONFIRMATORY_STREAMS),
            "scene_count": 60,
            "graph_builds": 180,
            "query_rows": 21600,
            "sample_count": 800,
            "halton_start_index": 17,
            "guarded_fraction": 0.75,
            "rmw_implementation": "rmw_fastrtps_cpp",
        })
        frozen_inputs = {
            "catalog.json": frozen_manifest["catalog_sha256"],
            "catalog_generation.log": "d" * 64,
            "source_snapshot.tar.gz": "e" * 64,
            "confirmatory_protocol.md": "f" * 64,
            "analyze_reachability_multiscene.py": MODULE.sha256_file(MODULE.__file__),
        }
        frozen_manifest["artifacts"]["frozen_inputs"] = frozen_inputs
        run_config = {
            "schema": "om6dof-reachability-multiscene-config-v2",
            "protocol_id": MODULE.CONFIRMATORY_PROTOCOL_ID,
            "catalog_id": MODULE.CONFIRMATORY_CATALOG_ID,
            "catalog_sha256": frozen_manifest["catalog_sha256"],
            "expected_catalog_sha256": frozen_manifest["catalog_sha256"],
            "streams": list(MODULE.CONFIRMATORY_STREAMS),
            "methods": methods,
            "scene_count": 60,
            "base_trajectory_count": 30,
            "graph_builds": 180,
            "query_rows": 21600,
            "sample_count": 800,
            "halton_start_index": 17,
            "guarded_fraction": 0.75,
            "rmw_implementation": "rmw_fastrtps_cpp",
            "ros_localhost_only": True,
            "phase_order_design": MODULE.PHASE_ORDER_DESIGN,
            "source_tree_sha256": frozen_manifest["implementation"][
                "source_tree_sha256"
            ],
            "runner_script_sha256": frozen_manifest["implementation"][
                "runner_script_sha256"
            ],
            "reachability_node_binary_sha256": frozen_manifest["implementation"][
                "reachability_node_binary_sha256"
            ],
            "catalog_model": frozen_manifest["catalog_model"],
            "catalog_generator_implementation": frozen_manifest[
                "catalog_generator_implementation"
            ],
            "frozen_input_sha256": frozen_inputs,
        }
        provenance = {
            "verified": True, "run_config_verified": True,
            "run_config": run_config,
            "frozen_inputs_verified": True,
            "executing_analyzer_sha256": MODULE.sha256_file(MODULE.__file__),
        }
        MODULE.require_confirmatory_design(data, frozen_manifest, provenance)

        unbalanced = copy.deepcopy(data)
        unbalanced["method_order_audit"]["permutation_counts"][0]["count"] = 9
        unbalanced["method_order_audit"]["permutation_counts"][1]["count"] = 11
        with self.assertRaisesRegex(MODULE.AuditError, "all 6 permutations"):
            MODULE.require_confirmatory_design(
                unbalanced, frozen_manifest, provenance
            )
        with self.assertRaisesRegex(MODULE.AuditError, "run_config.json"):
            MODULE.require_confirmatory_design(
                data, frozen_manifest, {"verified": True, "run_config_verified": False}
            )
        with self.assertRaisesRegex(MODULE.AuditError, "50000 bootstrap"):
            MODULE.require_confirmatory_resampling(49999, 100000)
        with self.assertRaisesRegex(MODULE.AuditError, "100000 permutations"):
            MODULE.require_confirmatory_resampling(50000, 99999)
        MODULE.require_confirmatory_resampling(50000, 100000)


class OutputTest(unittest.TestCase):
    def test_empty_csv_table_still_has_a_header(self):
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / "empty.csv"
            MODULE.write_csv(path, [], fieldnames=["left", "right"])
            self.assertEqual(path.read_text(encoding="utf-8"), "left,right\n")

    def test_writes_outputs_without_changing_raw_artifacts(self):
        with tempfile.TemporaryDirectory() as temporary:
            input_dir = Path(temporary) / "raw"
            output_dir = Path(temporary) / "analysis"
            write_bundle(input_dir)
            before = {
                name: sha256(input_dir / name)
                for name in ("graphs.csv", "queries.csv", "manifest.json")
            }
            result, actual_output = MODULE.run_analysis(
                input_dir, output_dir=output_dir, repetitions=200, seed=11
            )
            after = {
                name: sha256(input_dir / name)
                for name in ("graphs.csv", "queries.csv", "manifest.json")
            }
            self.assertEqual(before, after)
            self.assertEqual(actual_output, output_dir.resolve())
            self.assertEqual(result["study"]["kind"], "smoke_descriptive")
            self.assertTrue(result["audit"]["raw_inputs_unchanged"])
            for filename in (
                "analysis.json", "summary.md", "methods.csv", "pairwise.csv",
                "strata.csv",
            ):
                self.assertTrue((output_dir / filename).is_file(), filename)
            summary = (output_dir / "summary.md").read_text(encoding="utf-8")
            self.assertIn("SMOKE / DESCRIPTIVE ONLY", summary)

    def test_v2_verifies_declared_artifact_contents(self):
        with tempfile.TemporaryDirectory() as temporary:
            input_dir = Path(temporary) / "raw_v2"
            output_dir = Path(temporary) / "analysis_v2"
            write_v2_bundle(input_dir)
            frozen_dir = input_dir / "frozen_inputs"
            frozen_dir.mkdir()
            frozen_hashes = {}
            for index, filename in enumerate(sorted(MODULE.CONFIRMATORY_FROZEN_INPUTS)):
                path = frozen_dir / filename
                path.write_text(f"frozen {index}\n", encoding="utf-8")
                frozen_hashes[filename] = sha256(path)
            frozen_manifest = json.loads(
                (input_dir / "manifest.json").read_text(encoding="utf-8")
            )
            frozen_manifest["artifacts"]["frozen_inputs"] = frozen_hashes
            (input_dir / "manifest.json").write_text(
                json.dumps(frozen_manifest, sort_keys=True, indent=2) + "\n",
                encoding="utf-8",
            )
            result, _ = MODULE.run_analysis(
                input_dir, output_dir=output_dir, repetitions=200
            )
            self.assertEqual(
                result["audit"]["run_manifest_schema"], MODULE.RUN_SCHEMA_V2
            )
            self.assertTrue(result["audit"]["v2_provenance_verified"])
            self.assertTrue(result["audit"]["v2_run_config_verified"])
            self.assertTrue(result["audit"]["v2_frozen_inputs_verified"])
            self.assertEqual(
                result["source_provenance"]["phase_order_design"],
                MODULE.PHASE_ORDER_DESIGN,
            )

            payload = json.loads(
                (input_dir / "manifest.json").read_text(encoding="utf-8")
            )
            payload["artifacts"]["queries.csv"] = "f" * 64
            (input_dir / "manifest.json").write_text(
                json.dumps(payload, sort_keys=True, indent=2) + "\n",
                encoding="utf-8",
            )
            with self.assertRaisesRegex(MODULE.AuditError, "queries.csv digest"):
                MODULE.run_analysis(
                    input_dir,
                    output_dir=Path(temporary) / "analysis_v2_bad",
                    repetitions=200,
                )

    def test_v2_rejects_tampered_run_config_digest(self):
        with tempfile.TemporaryDirectory() as temporary:
            input_dir = Path(temporary) / "raw_v2"
            write_v2_bundle(input_dir)
            (input_dir / "run_config.json").write_text(
                json.dumps({"unit": False}) + "\n", encoding="utf-8"
            )
            with self.assertRaisesRegex(MODULE.AuditError, "run_config.json digest"):
                MODULE.run_analysis(
                    input_dir,
                    output_dir=Path(temporary) / "analysis_v2_bad_config",
                    repetitions=200,
                    permutation_repetitions=200,
                )

    def test_legacy_v2_smoke_without_run_config_remains_descriptive(self):
        with tempfile.TemporaryDirectory() as temporary:
            input_dir = Path(temporary) / "raw_v2_legacy"
            write_v2_bundle(input_dir)
            payload = json.loads(
                (input_dir / "manifest.json").read_text(encoding="utf-8")
            )
            del payload["artifacts"]["run_config.json"]
            (input_dir / "manifest.json").write_text(
                json.dumps(payload, sort_keys=True, indent=2) + "\n",
                encoding="utf-8",
            )
            (input_dir / "run_config.json").unlink()
            result, _ = MODULE.run_analysis(
                input_dir,
                output_dir=Path(temporary) / "analysis_v2_legacy",
                repetitions=200,
                permutation_repetitions=200,
            )
            self.assertEqual(result["study"]["kind"], "smoke_descriptive")
            self.assertFalse(result["audit"]["v2_run_config_verified"])


if __name__ == "__main__":
    unittest.main()
