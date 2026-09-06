#!/usr/bin/env python3
"""Audit and analyze paired multi-scene reachability benchmark artifacts.

The analyzer is deliberately offline: it reads ``graphs.csv``, ``queries.csv``,
and ``manifest.json`` and never imports ROS.  The confirmatory estimand treats
the deterministic scene catalog as a fixed test set; its CI therefore resamples
whole roadmap streams while preserving every scene and paired method outcome.
A separate two-way stream/base-trajectory bootstrap is labeled only as a
scene-generalization sensitivity.  ``auto`` is always descriptive: confirmatory
labeling must be requested explicitly and pass the frozen-protocol audit.
"""

import argparse
import csv
import hashlib
import itertools
import json
import math
import os
import random
import statistics
import sys
from collections import Counter
from datetime import datetime, timezone
from pathlib import Path


SCHEMA = "om6dof-reachability-multiscene-analysis-v2"
RUN_SCHEMA_V1 = "om6dof-reachability-multiscene-run-v1"
RUN_SCHEMA_V2 = "om6dof-reachability-multiscene-run-v2"
RUN_SCHEMAS = (RUN_SCHEMA_V1, RUN_SCHEMA_V2)
# Backward-compatible public constant used by existing unit fixtures.
RUN_SCHEMA = RUN_SCHEMA_V1
PHASE_ORDER_DESIGN = (
    "paired_across_methods_counterbalanced_by_stream_ordinal_plus_catalog_index"
)
PHASES = ("clear", "dynamic")
CONFIRMATORY_MIN_STREAMS = 50
CONFIRMATORY_MIN_BASE_TRAJECTORIES = 30
CONFIRMATORY_MIN_BOOTSTRAP_REPETITIONS = 50000
CONFIRMATORY_MIN_PERMUTATION_REPETITIONS = 100000
CONFIRMATORY_PROTOCOL_ID = "icra_confirmatory_v3"
CONFIRMATORY_CATALOG_ID = "om6dof_icra_scene_catalog_v3"
CONFIRMATORY_METHODS = ("gng", "guarded_gng", "halton_prm")
CONFIRMATORY_STREAMS = tuple(range(100, 160))
CONFIRMATORY_SCENES = 60
CONFIRMATORY_BASE_TRAJECTORIES = 30
CONFIRMATORY_GRAPH_BUILDS = 180
CONFIRMATORY_QUERY_ROWS = 21600
PRIMARY_METHOD = "guarded_gng"
PRIMARY_BASELINES = ("gng", "halton_prm")
PRIMARY_METRIC = "dynamic_success"
DIFFICULTIES = ("low", "medium", "high")

V2_GRAPH_TELEMETRY_FIELDS = {
    "expanded_urdf_sha256", "srdf_sha256", "reachability_parameters_sha256",
    "anchor_node_count", "prototype_budget", "prototype_node_count",
    "requested_guard_node_count", "guard_node_count", "fill_sample_node_count",
    "candidate_attempts", "halton_start_index", "sample_stream_type",
    "gng_training_sample_count", "effective_guard_fraction", "joint_names",
    "graph_publisher_count", "plan_publisher_count", "query_subscriber_count",
}
V2_QUERY_TELEMETRY_FIELDS = {
    "plan_graph_method", "base_trajectory_id", "source_counter", "stratum",
    "phase_order_position", "target_source_joint_positions", "exact_checks",
    "exact_replans", "exact_time_ms", "start_node_id", "goal_node_id",
    "target_distance",
}
CONFIRMATORY_FROZEN_INPUTS = frozenset({
    "catalog.json", "catalog_generation.log", "source_snapshot.tar.gz",
    "confirmatory_protocol.md", "analyze_reachability_multiscene.py",
})

GRAPH_REQUIRED_FIELDS = {
    "run_index", "stream_ordinal", "roadmap_stream_id", "method",
    "method_order_position", "ros_domain_id", "reported_method",
    "graph_revision", "requested_node_count", "sample_stream_seed", "nodes",
    "edges", "components", "build_time_ms", "catalog_id", "catalog_sha256",
    "ros_localhost_only", "infrastructure_error",
}
QUERY_REQUIRED_FIELDS = {
    "run_index", "stream_ordinal", "roadmap_stream_id", "method",
    "method_order_position", "ros_domain_id", "graph_revision", "scene_id",
    "catalog_index", "scene_order_position", "phase", "obstacle_kind",
    "query_id", "start_joint_positions", "target_position",
    "requested_target_environment_node_id", "requested_target_position",
    "selected_target_environment_node_id", "valid", "exact_valid", "reason",
    "timeout", "infrastructure_error", "blocked_nodes", "blocked_edges",
    "planning_time_ms", "publish_to_plan_ms", "graph_cost",
    "start_connection_cost", "total_joint_path_cost", "path_nodes", "path_ids",
    "preview_start_joints",
}


class AuditError(ValueError):
    """Raised on the first benchmark contract violation."""


def fail(message):
    raise AuditError(message)


def sha256_file(path):
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def strict_json_load(path):
    try:
        return json.loads(
            Path(path).read_text(encoding="utf-8"),
            parse_constant=lambda value: fail(
                f"{path}: non-standard JSON constant {value}"
            ),
        )
    except (OSError, json.JSONDecodeError) as exc:
        fail(f"cannot read valid JSON from {path}: {exc}")


def read_csv(path, required_fields):
    try:
        with Path(path).open(newline="", encoding="utf-8") as stream:
            reader = csv.DictReader(stream)
            if reader.fieldnames is None:
                fail(f"{path}: missing CSV header")
            if len(reader.fieldnames) != len(set(reader.fieldnames)):
                fail(f"{path}: duplicate CSV header")
            missing = required_fields - set(reader.fieldnames)
            if missing:
                fail(f"{path}: missing fields {sorted(missing)}")
            rows = []
            for line_number, row in enumerate(reader, start=2):
                if None in row:
                    fail(f"{path}:{line_number}: extra unlabelled CSV values")
                row["__line__"] = line_number
                rows.append(row)
    except OSError as exc:
        fail(f"cannot read {path}: {exc}")
    if not rows:
        fail(f"{path}: contains no data rows")
    return rows


def row_context(kind, row):
    return f"{kind}.csv:{row.get('__line__', '?')}"


def parse_int(value, name, context, minimum=None):
    try:
        parsed = int(value)
    except (TypeError, ValueError):
        fail(f"{context}: {name} must be an integer")
    if str(value).strip() != str(parsed):
        fail(f"{context}: {name} must use canonical integer syntax")
    if minimum is not None and parsed < minimum:
        fail(f"{context}: {name} must be >= {minimum}")
    return parsed


def parse_float(value, name, context, minimum=None):
    try:
        parsed = float(value)
    except (TypeError, ValueError):
        fail(f"{context}: {name} must be numeric")
    if not math.isfinite(parsed):
        fail(f"{context}: {name} must be finite")
    if minimum is not None and parsed < minimum:
        fail(f"{context}: {name} must be >= {minimum}")
    return parsed


def parse_bool(value, name, context):
    normalized = str(value).strip().lower()
    if normalized == "true":
        return True
    if normalized == "false":
        return False
    fail(f"{context}: {name} must be True or False")


def parse_json_cell(value, name, context, expected_type):
    try:
        parsed = json.loads(
            value,
            parse_constant=lambda item: fail(
                f"{context}: {name} contains non-standard JSON constant {item}"
            ),
        )
    except (TypeError, json.JSONDecodeError) as exc:
        fail(f"{context}: {name} is not valid JSON: {exc}")
    if not isinstance(parsed, expected_type):
        fail(f"{context}: {name} must encode {expected_type.__name__}")
    return parsed


def parse_numeric_vector(value, name, context):
    parsed = parse_json_cell(value, name, context, list)
    result = []
    for index, item in enumerate(parsed):
        result.append(parse_float(item, f"{name}[{index}]", context))
    return result


def normalize_graph(row):
    context = row_context("graphs", row)
    integers = (
        "run_index", "stream_ordinal", "roadmap_stream_id",
        "method_order_position", "ros_domain_id", "graph_revision",
        "requested_node_count", "sample_stream_seed", "nodes", "edges",
        "components",
    )
    result = dict(row)
    for field in integers:
        result[field] = parse_int(row[field], field, context, 0)
    result["build_time_ms"] = parse_float(
        row["build_time_ms"], "build_time_ms", context, 0.0
    )
    result["ros_localhost_only"] = parse_bool(
        row["ros_localhost_only"], "ros_localhost_only", context
    )
    result["method"] = row["method"].strip()
    result["reported_method"] = row["reported_method"].strip()
    result["infrastructure_error"] = row["infrastructure_error"].strip()
    if V2_GRAPH_TELEMETRY_FIELDS.issubset(row):
        for field in (
            "anchor_node_count", "prototype_budget", "prototype_node_count",
            "requested_guard_node_count", "guard_node_count",
            "fill_sample_node_count", "candidate_attempts", "halton_start_index",
            "gng_training_sample_count", "graph_publisher_count",
            "plan_publisher_count", "query_subscriber_count",
        ):
            result[field] = parse_int(row[field], field, context, 0)
        result["effective_guard_fraction"] = parse_float(
            row["effective_guard_fraction"], "effective_guard_fraction", context,
            0.0,
        )
        for field in (
            "expanded_urdf_sha256", "srdf_sha256",
            "reachability_parameters_sha256", "sample_stream_type",
        ):
            result[field] = row[field].strip()
        result["joint_names"] = parse_json_cell(
            row["joint_names"], "joint_names", context, list
        )
        require(
            result["joint_names"]
            and all(isinstance(name, str) and name for name in result["joint_names"])
            and len(result["joint_names"]) == len(set(result["joint_names"])),
            f"{context}: joint_names must be a non-empty unique string list",
        )
    result["__context__"] = context
    return result


def normalize_query(row):
    context = row_context("queries", row)
    integer_fields = (
        "run_index", "stream_ordinal", "roadmap_stream_id",
        "method_order_position", "ros_domain_id", "graph_revision",
        "catalog_index", "scene_order_position", "query_id",
        "requested_target_environment_node_id",
        "selected_target_environment_node_id", "blocked_nodes", "blocked_edges",
        "path_nodes",
    )
    result = dict(row)
    for field in integer_fields:
        result[field] = parse_int(row[field], field, context, 0)
    if "phase_order_position" in row:
        result["phase_order_position"] = parse_int(
            row["phase_order_position"], "phase_order_position", context, 0
        )
    if "source_counter" in row:
        result["source_counter"] = parse_int(
            row["source_counter"], "source_counter", context, 0
        )
    if "base_trajectory_id" in row:
        result["base_trajectory_id"] = row["base_trajectory_id"].strip()
        require(result["base_trajectory_id"],
                f"{context}: base_trajectory_id must be non-empty")
    if "stratum" in row:
        result["stratum"] = parse_json_cell(
            row["stratum"], "stratum", context, dict
        )
        difficulty = result["stratum"].get("difficulty")
        obstacle_kind = result["stratum"].get("obstacle_kind")
        require(difficulty in DIFFICULTIES,
                f"{context}: stratum.difficulty must be one of {DIFFICULTIES}")
        require(obstacle_kind in ("point", "segment"),
                f"{context}: stratum.obstacle_kind must be point or segment")
    if V2_QUERY_TELEMETRY_FIELDS.issubset(row):
        for field in (
            "exact_checks", "exact_replans", "start_node_id", "goal_node_id",
        ):
            result[field] = parse_int(row[field], field, context, 0)
        result["exact_time_ms"] = parse_float(
            row["exact_time_ms"], "exact_time_ms", context, 0.0
        )
        result["target_distance"] = parse_float(
            row["target_distance"], "target_distance", context, -1.0
        )
        result["target_source_joint_positions"] = parse_numeric_vector(
            row["target_source_joint_positions"],
            "target_source_joint_positions", context,
        )
        result["plan_graph_method"] = row["plan_graph_method"].strip()
    for field in ("planning_time_ms", "publish_to_plan_ms"):
        result[field] = parse_float(row[field], field, context, 0.0)
    # Invalid plans use negative finite sentinels for unavailable cost fields.
    # Successful rows are checked for non-negative costs below.
    for field in ("graph_cost", "start_connection_cost", "total_joint_path_cost"):
        result[field] = parse_float(row[field], field, context)
    for field in ("valid", "exact_valid", "timeout"):
        result[field] = parse_bool(row[field], field, context)
    result["start_joint_positions"] = parse_numeric_vector(
        row["start_joint_positions"], "start_joint_positions", context
    )
    result["target_position"] = parse_numeric_vector(
        row["target_position"], "target_position", context
    )
    result["requested_target_position"] = parse_numeric_vector(
        row["requested_target_position"], "requested_target_position", context
    )
    result["preview_start_joints"] = parse_numeric_vector(
        row["preview_start_joints"], "preview_start_joints", context
    )
    result["path_ids"] = parse_json_cell(
        row["path_ids"], "path_ids", context, list
    )
    if any(not isinstance(item, int) or isinstance(item, bool) or item < 0
           for item in result["path_ids"]):
        fail(f"{context}: path_ids must contain non-negative integers")
    for field in ("method", "scene_id", "phase", "obstacle_kind", "reason"):
        result[field] = row[field].strip()
    result["infrastructure_error"] = row["infrastructure_error"].strip()
    result["__context__"] = context
    return result


def vectors_close(left, right, tolerance=1e-9):
    return len(left) == len(right) and all(
        math.isclose(a, b, rel_tol=0.0, abs_tol=tolerance)
        for a, b in zip(left, right)
    )


def require(condition, message):
    if not condition:
        fail(message)


def is_sha256(value):
    return (
        isinstance(value, str)
        and len(value) == 64
        and all(character in "0123456789abcdefABCDEF" for character in value)
    )


def require_hash_mapping(payload, name, keys):
    require(isinstance(payload, dict), f"manifest.json: {name} must be an object")
    for key in keys:
        require(
            is_sha256(payload.get(key)),
            f"manifest.json: {name}.{key} must be a SHA-256 digest",
        )


def validate_manifest(manifest):
    require(isinstance(manifest, dict), "manifest.json: root must be an object")
    schema = manifest.get("schema")
    require(schema in RUN_SCHEMAS,
            f"manifest.json: schema must be one of {list(RUN_SCHEMAS)}")
    methods = manifest.get("methods")
    streams = manifest.get("streams")
    require(
        isinstance(methods, list) and methods
        and all(isinstance(item, str) and item for item in methods)
        and len(methods) == len(set(methods)),
        "manifest.json: methods must be a non-empty unique string list",
    )
    require(
        isinstance(streams, list) and streams
        and all(isinstance(item, int) and not isinstance(item, bool) and item >= 0
                for item in streams)
        and len(streams) == len(set(streams)),
        "manifest.json: streams must be a non-empty unique non-negative integer list",
    )
    for field in ("scene_count", "graph_builds", "query_rows", "sample_count"):
        value = manifest.get(field)
        require(isinstance(value, int) and not isinstance(value, bool) and value > 0,
                f"manifest.json: {field} must be a positive integer")
    require(manifest.get("preview_only") is True,
            "manifest.json: preview_only must be true")
    require(manifest.get("ros_localhost_only") is True,
            "manifest.json: ros_localhost_only must be true")
    require(manifest.get("controller_topics_published") == [],
            "manifest.json: controller_topics_published must be empty")
    require(isinstance(manifest.get("catalog_id"), str) and manifest["catalog_id"],
            "manifest.json: catalog_id must be non-empty")
    catalog_hash = manifest.get("catalog_sha256")
    require(
        isinstance(catalog_hash, str) and len(catalog_hash) == 64
        and all(character in "0123456789abcdef" for character in catalog_hash),
        "manifest.json: catalog_sha256 must be a lowercase SHA-256 digest",
    )
    if schema == RUN_SCHEMA_V2:
        require(
            manifest.get("phase_order_design") == PHASE_ORDER_DESIGN,
            f"manifest.json: phase_order_design must equal {PHASE_ORDER_DESIGN}",
        )
        require_hash_mapping(
            manifest.get("catalog_model"), "catalog_model",
            (
                "expanded_urdf_sha256", "srdf_sha256",
                "reachability_parameters_sha256",
            ),
        )
        require_hash_mapping(
            manifest.get("catalog_generator_implementation"),
            "catalog_generator_implementation",
            ("generator_script_sha256", "reachability_node_binary_sha256"),
        )
        require_hash_mapping(
            manifest.get("implementation"), "implementation",
            (
                "runner_script_sha256", "reachability_node_binary_sha256",
                "source_tree_sha256",
            ),
        )
        artifacts = manifest.get("artifacts")
        require(isinstance(artifacts, dict),
                "manifest.json: artifacts must be an object")
        for filename in ("graphs.csv", "queries.csv"):
            require(
                is_sha256(artifacts.get(filename)),
                f"manifest.json: artifacts.{filename} must be a SHA-256 digest",
            )
        if "run_config.json" in artifacts:
            require(
                is_sha256(artifacts["run_config.json"]),
                "manifest.json: artifacts.run_config.json must be a SHA-256 digest",
            )
        logs = artifacts.get("logs")
        require(isinstance(logs, dict),
                "manifest.json: artifacts.logs must be an object")
        require(len(logs) == manifest["graph_builds"],
                "manifest.json: artifacts.logs count must equal graph_builds")
        for filename, digest in logs.items():
            require(
                isinstance(filename, str)
                and filename == Path(filename).name
                and filename.startswith("run_") and filename.endswith(".log"),
                "manifest.json: artifact log names must be basename run_*.log paths",
            )
            require(
                is_sha256(digest),
                f"manifest.json: artifacts.logs.{filename} must be a SHA-256 digest",
            )
        if "frozen_inputs" in artifacts:
            frozen_inputs = artifacts["frozen_inputs"]
            require(isinstance(frozen_inputs, dict) and frozen_inputs,
                    "manifest.json: artifacts.frozen_inputs must be a non-empty object")
            for filename, digest in frozen_inputs.items():
                require(
                    isinstance(filename, str) and filename == Path(filename).name,
                    "manifest.json: frozen input names must be safe basenames",
                )
                require(
                    is_sha256(digest),
                    f"manifest.json: frozen input {filename} must have a SHA-256 digest",
                )
    return methods, streams


def audit_v2_artifact_contents(manifest, input_dir, input_hashes):
    """Verify every v2-declared artifact digest against immutable input files."""
    if manifest["schema"] != RUN_SCHEMA_V2:
        return None
    artifacts = manifest["artifacts"]
    require(
        artifacts["graphs.csv"].lower() == input_hashes["graphs"].lower(),
        "manifest.json: artifacts.graphs.csv digest does not match graphs.csv",
    )
    require(
        artifacts["queries.csv"].lower() == input_hashes["queries"].lower(),
        "manifest.json: artifacts.queries.csv digest does not match queries.csv",
    )
    log_dir = Path(input_dir) / "logs"
    require(log_dir.is_dir(), "manifest.json: declared logs directory is missing")
    declared = artifacts["logs"]
    actual_names = {path.name for path in log_dir.glob("run_*.log") if path.is_file()}
    require(actual_names == set(declared),
            "manifest.json: declared and on-disk run log sets differ")
    for filename, expected in declared.items():
        actual = sha256_file(log_dir / filename)
        require(
            actual.lower() == expected.lower(),
            f"manifest.json: artifact log digest mismatch for {filename}",
        )
    run_config_verified = False
    run_config = None
    if "run_config.json" in artifacts:
        run_config_path = Path(input_dir) / "run_config.json"
        require(run_config_path.is_file(),
                "manifest.json: declared run_config.json is missing")
        run_config = strict_json_load(run_config_path)
        require(isinstance(run_config, dict),
                "run_config.json: root must be an object")
        require(
            sha256_file(run_config_path).lower()
            == artifacts["run_config.json"].lower(),
            "manifest.json: artifact run_config.json digest mismatch",
        )
        run_config_verified = True
    frozen_inputs_verified = False
    if "frozen_inputs" in artifacts:
        frozen_dir = Path(input_dir) / "frozen_inputs"
        require(frozen_dir.is_dir(),
                "manifest.json: declared frozen_inputs directory is missing")
        declared_frozen = artifacts["frozen_inputs"]
        actual_frozen = {
            path.name for path in frozen_dir.iterdir() if path.is_file()
        }
        require(
            actual_frozen == set(declared_frozen),
            "manifest.json: declared and on-disk frozen input sets differ",
        )
        for filename, expected in declared_frozen.items():
            require(
                sha256_file(frozen_dir / filename).lower() == expected.lower(),
                f"manifest.json: frozen input digest mismatch for {filename}",
            )
        frozen_inputs_verified = True
    return {
        "verified": True,
        "run_config_declared": "run_config.json" in artifacts,
        "run_config_verified": run_config_verified,
        "run_config": run_config,
        "frozen_inputs_verified": frozen_inputs_verified,
        "executing_analyzer_sha256": sha256_file(Path(__file__).resolve()),
    }


def audit_bundle(manifest, raw_graphs, raw_queries):
    """Normalize rows and fail on the first contract violation."""
    methods, streams = validate_manifest(manifest)
    graphs = [normalize_graph(row) for row in raw_graphs]
    queries = [normalize_query(row) for row in raw_queries]
    phase_order_columns = {"phase_order_position" in row for row in raw_queries}
    require(len(phase_order_columns) == 1,
            "queries.csv: phase_order_position is present in only some rows")
    has_phase_order = phase_order_columns == {True}
    pairing_columns = {
        ("base_trajectory_id" in row, "source_counter" in row)
        for row in raw_queries
    }
    require(len(pairing_columns) == 1,
            "queries.csv: trajectory-pairing columns are present in only some rows")
    pairing_presence = next(iter(pairing_columns))
    require(pairing_presence in ((False, False), (True, True)),
            "queries.csv: base_trajectory_id and source_counter must appear together")
    has_trajectory_pairing = pairing_presence == (True, True)
    stratum_columns = {"stratum" in row for row in raw_queries}
    require(len(stratum_columns) == 1,
            "queries.csv: stratum is present in only some rows")
    has_strata = stratum_columns == {True}
    graph_columns = set(raw_graphs[0]) if raw_graphs else set()
    query_columns = set(raw_queries[0]) if raw_queries else set()
    has_v2_graph_telemetry = V2_GRAPH_TELEMETRY_FIELDS.issubset(graph_columns)
    has_v2_query_telemetry = V2_QUERY_TELEMETRY_FIELDS.issubset(query_columns)
    scene_count = manifest["scene_count"]
    expected_graphs = len(methods) * len(streams)
    expected_queries = expected_graphs * scene_count * len(PHASES)
    require(manifest["graph_builds"] == expected_graphs,
            "manifest.json: graph_builds disagrees with methods x streams")
    require(manifest["query_rows"] == expected_queries,
            "manifest.json: query_rows disagrees with methods x streams x scenes x phases")
    require(len(graphs) == expected_graphs,
            f"graphs.csv: expected {expected_graphs} rows, found {len(graphs)}")
    require(len(queries) == expected_queries,
            f"queries.csv: expected {expected_queries} rows, found {len(queries)}")

    stream_ordinal = {stream: index for index, stream in enumerate(streams)}
    graph_by_key = {}
    graph_run_indices = set()
    for row in graphs:
        context = row["__context__"]
        key = (row["roadmap_stream_id"], row["method"])
        require(key not in graph_by_key, f"{context}: duplicate graph key {key}")
        require(row["method"] in methods, f"{context}: unknown method")
        require(row["roadmap_stream_id"] in stream_ordinal,
                f"{context}: unknown roadmap stream")
        require(row["stream_ordinal"] == stream_ordinal[row["roadmap_stream_id"]],
                f"{context}: stream_ordinal does not match manifest order")
        require(row["reported_method"] == row["method"],
                f"{context}: reported_method mismatch")
        require(row["sample_stream_seed"] == row["roadmap_stream_id"],
                f"{context}: sample_stream_seed mismatch")
        require(row["requested_node_count"] == manifest["sample_count"],
                f"{context}: requested_node_count mismatch")
        require(row["nodes"] == row["requested_node_count"],
                f"{context}: graph node budget not met")
        require(row["components"] >= 1, f"{context}: graph has no component")
        require(row["graph_revision"] > 0, f"{context}: graph_revision must be positive")
        require(row["build_time_ms"] > 0.0, f"{context}: build time must be positive")
        require(row["ros_localhost_only"], f"{context}: ROS localhost isolation is false")
        require(not row["infrastructure_error"],
                f"{context}: infrastructure_error={row['infrastructure_error']!r}")
        require(row["catalog_id"] == manifest["catalog_id"],
                f"{context}: catalog_id mismatch")
        require(row["catalog_sha256"] == manifest["catalog_sha256"],
                f"{context}: catalog_sha256 mismatch")
        if has_v2_graph_telemetry:
            model = manifest.get("catalog_model", {})
            for field in (
                "expanded_urdf_sha256", "srdf_sha256",
                "reachability_parameters_sha256",
            ):
                require(
                    row[field] == model.get(field),
                    f"{context}: runtime {field} disagrees with frozen catalog model",
                )
            require(
                row["graph_publisher_count"] == 1
                and row["plan_publisher_count"] == 1
                and row["query_subscriber_count"] == 1,
                f"{context}: isolated ROS endpoint counts must all equal one",
            )
            require(
                row["sample_stream_type"] == "digit_permuted_halton",
                f"{context}: sample_stream_type mismatch",
            )
            require(row["anchor_node_count"] == 2,
                    f"{context}: anchor_node_count must equal two")
            remaining = row["requested_node_count"] - row["anchor_node_count"]
            require(
                sum(row[field] for field in (
                    "anchor_node_count", "prototype_node_count",
                    "guard_node_count", "fill_sample_node_count",
                )) == row["requested_node_count"],
                f"{context}: node composition does not meet the fixed budget",
            )
            if row["method"] == "gng":
                expected = (remaining, remaining, 0, 0, 0)
            elif row["method"] == "guarded_gng":
                requested_guards = min(
                    int(math.floor(manifest.get("guarded_fraction", -1.0)
                                   * remaining + 0.5)),
                    max(0, remaining - 2),
                )
                expected = (
                    remaining - requested_guards, remaining - requested_guards,
                    requested_guards, requested_guards, 0,
                )
            else:
                expected = (0, 0, 0, 0, remaining)
            observed = (
                row["prototype_budget"], row["prototype_node_count"],
                row["requested_guard_node_count"], row["guard_node_count"],
                row["fill_sample_node_count"],
            )
            require(observed == expected,
                    f"{context}: method-specific node composition mismatch")
            require(
                row["candidate_attempts"] >= remaining,
                f"{context}: candidate_attempts is below the remaining node budget",
            )
            if row["method"] == "halton_prm":
                require(row["gng_training_sample_count"] == 0,
                        f"{context}: Halton unexpectedly reports GNG training samples")
            else:
                require(row["gng_training_sample_count"] >= remaining,
                        f"{context}: GNG training sample count is too small")
            require(
                row["halton_start_index"] == manifest.get("halton_start_index"),
                f"{context}: halton_start_index mismatch",
            )
        require(row["run_index"] not in graph_run_indices,
                f"{context}: duplicate run_index")
        graph_run_indices.add(row["run_index"])
        graph_by_key[key] = row

    expected_keys = set(itertools.product(streams, methods))
    require(set(graph_by_key) == expected_keys,
            "graphs.csv: stream-method Cartesian product is incomplete")
    for stream in streams:
        positions = {
            graph_by_key[(stream, method)]["method_order_position"]
            for method in methods
        }
        require(positions == set(range(len(methods))),
                f"graphs.csv: method order is not a permutation for stream {stream}")
    method_orders = Counter(
        tuple(sorted(
            methods,
            key=lambda method: graph_by_key[(stream, method)][
                "method_order_position"
            ],
        ))
        for stream in streams
    )
    all_method_orders = list(itertools.permutations(methods))
    method_order_audit = {
        "permutation_counts": [
            {"order": list(order), "count": method_orders.get(order, 0)}
            for order in all_method_orders
        ],
        "all_permutations_present": all(
            method_orders.get(order, 0) > 0 for order in all_method_orders
        ),
        "count_range": (
            [min(method_orders.values()), max(method_orders.values())]
            if method_orders else None
        ),
    }

    query_by_key = {}
    query_ids = set()
    scene_ids = set()
    catalog_to_scene = {}
    scene_static = {}
    scene_pair_identity = {}
    scene_strata = {}
    for row in queries:
        context = row["__context__"]
        graph_key = (row["roadmap_stream_id"], row["method"])
        require(graph_key in graph_by_key, f"{context}: no matching graph row")
        graph = graph_by_key[graph_key]
        require(row["stream_ordinal"] == graph["stream_ordinal"],
                f"{context}: stream ordinal differs from graph")
        require(row["run_index"] == graph["run_index"],
                f"{context}: run_index differs from graph")
        require(row["method_order_position"] == graph["method_order_position"],
                f"{context}: method order differs from graph")
        require(row["ros_domain_id"] == graph["ros_domain_id"],
                f"{context}: ROS domain differs from graph")
        require(row["graph_revision"] == graph["graph_revision"],
                f"{context}: graph revision differs from graph")
        if has_v2_query_telemetry:
            require(row["plan_graph_method"] == row["method"],
                    f"{context}: plan_graph_method mismatch")
            require(
                len(row["target_source_joint_positions"])
                == len(row["start_joint_positions"]),
                f"{context}: target source joint vector length mismatch",
            )
        require(row["phase"] in PHASES, f"{context}: phase must be clear or dynamic")
        key = (
            row["roadmap_stream_id"], row["method"], row["scene_id"], row["phase"]
        )
        require(key not in query_by_key, f"{context}: duplicate query key {key}")
        require(row["query_id"] > 0, f"{context}: query_id must be positive")
        require(row["query_id"] not in query_ids,
                f"{context}: query_id is not globally unique")
        query_ids.add(row["query_id"])
        require(not row["timeout"], f"{context}: query timed out")
        require(not row["infrastructure_error"],
                f"{context}: infrastructure_error={row['infrastructure_error']!r}")
        require(row["valid"] == row["exact_valid"],
                f"{context}: valid and exact_valid disagree")
        require(bool(row["reason"]), f"{context}: reason is empty")
        require(len(row["target_position"]) == 3,
                f"{context}: target_position must have length 3")
        require(vectors_close(row["requested_target_position"], row["target_position"]),
                f"{context}: requested target position echo mismatch")
        require(row["path_nodes"] == len(row["path_ids"]),
                f"{context}: path_nodes does not match path_ids length")
        if row["exact_valid"]:
            require(row["path_nodes"] >= 1, f"{context}: successful path is empty")
            require(row["selected_target_environment_node_id"] ==
                    row["requested_target_environment_node_id"],
                    f"{context}: selected target ID mismatch")
            require(vectors_close(row["preview_start_joints"],
                                  row["start_joint_positions"]),
                    f"{context}: preview start-state echo mismatch")
            require(
                row["graph_cost"] >= 0.0
                and row["start_connection_cost"] >= 0.0
                and row["total_joint_path_cost"] >= 0.0,
                f"{context}: successful path costs must be non-negative",
            )
            require(math.isclose(
                row["graph_cost"] + row["start_connection_cost"],
                row["total_joint_path_cost"], rel_tol=1e-9, abs_tol=1e-8,
            ), f"{context}: total path cost decomposition mismatch")
        if row["phase"] == "clear":
            require(row["obstacle_kind"] == "none",
                    f"{context}: clear phase obstacle_kind must be none")
            require(row["blocked_nodes"] == 0 and row["blocked_edges"] == 0,
                    f"{context}: clear phase reports blocked graph elements")
        else:
            require(row["obstacle_kind"] in ("point", "segment"),
                    f"{context}: invalid dynamic obstacle kind")

        require(0 <= row["catalog_index"] < scene_count,
                f"{context}: catalog_index out of range")
        mapped_scene = catalog_to_scene.setdefault(row["catalog_index"], row["scene_id"])
        require(mapped_scene == row["scene_id"],
                f"{context}: catalog index maps to multiple scene IDs")
        scene_ids.add(row["scene_id"])
        static = (
            tuple(row["start_joint_positions"]), tuple(row["target_position"]),
            row["requested_target_environment_node_id"],
        )
        prior_static = scene_static.setdefault(row["scene_id"], static)
        require(prior_static == static,
                f"{context}: scene start/target metadata changes across pairs")
        if has_trajectory_pairing:
            pair_identity = (row["base_trajectory_id"], row["source_counter"])
            prior_identity = scene_pair_identity.setdefault(
                row["scene_id"], pair_identity
            )
            require(prior_identity == pair_identity,
                    f"{context}: scene trajectory-pair identity changes across rows")
        if has_strata:
            stratum = (
                row["stratum"]["difficulty"], row["stratum"]["obstacle_kind"]
            )
            prior_stratum = scene_strata.setdefault(row["scene_id"], stratum)
            require(prior_stratum == stratum,
                    f"{context}: scene stratum changes across rows")
            if row["phase"] == "dynamic":
                require(row["obstacle_kind"] == stratum[1],
                        f"{context}: dynamic obstacle kind disagrees with stratum")
        query_by_key[key] = row

    require(len(scene_ids) == scene_count,
            f"queries.csv: expected {scene_count} scene IDs, found {len(scene_ids)}")
    require(set(catalog_to_scene) == set(range(scene_count)),
            "queries.csv: catalog indexes are not contiguous")
    expected_query_keys = set(itertools.product(streams, methods, scene_ids, PHASES))
    require(set(query_by_key) == expected_query_keys,
            "queries.csv: stream-method-scene-phase Cartesian product is incomplete")

    base_trajectory_count = None
    trajectory_pair_audit = {
        "present": has_trajectory_pairing,
        "base_trajectory_count": None,
        "exact_point_segment_pairs": None,
    }
    if has_trajectory_pairing:
        source_to_base = {}
        base_to_source = {}
        paired_scenes = {}
        reference_stream = streams[0]
        reference_method = methods[0]
        for scene in scene_ids:
            base_id, source_counter = scene_pair_identity[scene]
            require(
                source_to_base.setdefault(source_counter, base_id) == base_id,
                f"queries.csv: source_counter {source_counter} maps to multiple "
                "base trajectories",
            )
            require(
                base_to_source.setdefault(base_id, source_counter) == source_counter,
                f"queries.csv: base trajectory {base_id} maps to multiple source counters",
            )
            dynamic = query_by_key[
                (reference_stream, reference_method, scene, "dynamic")
            ]
            paired_scenes.setdefault((base_id, source_counter), []).append(
                (scene, dynamic["obstacle_kind"])
            )
        for identity, members in paired_scenes.items():
            require(len(members) == 2,
                    f"queries.csv: trajectory {identity} must contain exactly two scenes")
            require({kind for _, kind in members} == {"point", "segment"},
                    f"queries.csv: trajectory {identity} must pair point and segment")
            left_static = scene_static[members[0][0]]
            right_static = scene_static[members[1][0]]
            require(left_static[:2] == right_static[:2],
                    f"queries.csv: trajectory {identity} pair has different start/target")
            if has_strata:
                member_strata = [scene_strata[scene] for scene, _ in members]
                require(
                    member_strata[0][0] == member_strata[1][0],
                    f"queries.csv: trajectory {identity} pair has different difficulty",
                )
                require(
                    {value[1] for value in member_strata} == {"point", "segment"},
                    f"queries.csv: trajectory {identity} stratum kinds are not paired",
                )
        base_trajectory_count = len(paired_scenes)
        require(base_trajectory_count * 2 == scene_count,
                "queries.csv: every scene must belong to one exact two-kind pair")
        trajectory_pair_audit = {
            "present": True,
            "base_trajectory_count": base_trajectory_count,
            "exact_point_segment_pairs": True,
        }

    for stream, method, scene in itertools.product(streams, methods, scene_ids):
        clear = query_by_key[(stream, method, scene, "clear")]
        dynamic = query_by_key[(stream, method, scene, "dynamic")]
        require(clear["catalog_index"] == dynamic["catalog_index"],
                f"queries.csv: phase catalog mismatch for {stream}/{method}/{scene}")
        require(clear["scene_order_position"] == dynamic["scene_order_position"],
                f"queries.csv: phase order mismatch for {stream}/{method}/{scene}")
        require(vectors_close(clear["start_joint_positions"],
                              dynamic["start_joint_positions"]),
                f"queries.csv: phase start mismatch for {stream}/{method}/{scene}")
        require(vectors_close(clear["target_position"], dynamic["target_position"]),
                f"queries.csv: phase target mismatch for {stream}/{method}/{scene}")
        if has_phase_order:
            require(
                {clear["phase_order_position"], dynamic["phase_order_position"]} == {0, 1},
                f"queries.csv: phase order is not a permutation for "
                f"{stream}/{method}/{scene}",
            )
            expected_clear_position = (
                clear["stream_ordinal"] + clear["catalog_index"]
            ) % 2
            require(clear["phase_order_position"] == expected_clear_position,
                    f"queries.csv: phase order violates stream+catalog parity for "
                    f"{stream}/{method}/{scene}")
            require(dynamic["phase_order_position"] == 1 - expected_clear_position,
                    f"queries.csv: dynamic phase order violates parity for "
                    f"{stream}/{method}/{scene}")

    first_counts = None
    if has_phase_order:
        for stream, scene in itertools.product(streams, scene_ids):
            observed = {
                (
                    query_by_key[(stream, method, scene, "clear")][
                        "phase_order_position"
                    ],
                    query_by_key[(stream, method, scene, "dynamic")][
                        "phase_order_position"
                    ],
                )
                for method in methods
            }
            require(len(observed) == 1,
                    f"queries.csv: methods use different phase orders for {stream}/{scene}")
        first_counts = {
            phase: sum(
                query_by_key[(stream, methods[0], scene, phase)][
                    "phase_order_position"
                ] == 0
                for stream, scene in itertools.product(streams, scene_ids)
            )
            for phase in PHASES
        }
        require(abs(first_counts["clear"] - first_counts["dynamic"]) <= 1,
                "queries.csv: phase-first assignments are not balanced")

    for stream, method in itertools.product(streams, methods):
        positions = {
            query_by_key[(stream, method, scene, "clear")]["scene_order_position"]
            for scene in scene_ids
        }
        require(positions == set(range(scene_count)),
                f"queries.csv: scene order is not a permutation for {stream}/{method}")
    scene_order_across_methods = True
    scene_order_expected_rotation = True
    for stream, scene in itertools.product(streams, scene_ids):
        observed = {
            query_by_key[(stream, method, scene, "clear")]["scene_order_position"]
            for method in methods
        }
        if len(observed) != 1:
            scene_order_across_methods = False
            continue
        catalog_index = query_by_key[
            (stream, methods[0], scene, "clear")
        ]["catalog_index"]
        expected_position = (catalog_index - stream_ordinal[stream]) % scene_count
        if next(iter(observed)) != expected_position:
            scene_order_expected_rotation = False

    return {
        "methods": methods,
        "streams": streams,
        "scenes": sorted(scene_ids, key=lambda item: next(
            index for index, value in catalog_to_scene.items() if value == item
        )),
        "graphs": graphs,
        "queries": queries,
        "graph_by_key": graph_by_key,
        "query_by_key": query_by_key,
        "counts": {
            "methods": len(methods),
            "streams": len(streams),
            "scenes": scene_count,
            "graphs": len(graphs),
            "queries": len(queries),
            "base_trajectories": base_trajectory_count,
        },
        "phase_order_position_present": has_phase_order,
        "phase_order_audit": {
            "present": has_phase_order,
            "scheme": (
                "clear_position=(stream_ordinal+catalog_index)%2;dynamic=complement"
                if has_phase_order else None
            ),
            "first_position_counts": first_counts,
            "balanced": (
                abs(first_counts["clear"] - first_counts["dynamic"]) <= 1
                if first_counts is not None else None
            ),
        },
        "trajectory_pair_audit": trajectory_pair_audit,
        "method_order_audit": method_order_audit,
        "scene_order_audit": {
            "identical_across_methods": scene_order_across_methods,
            "expected_catalog_rotation": scene_order_expected_rotation,
            "formula": "position=(catalog_index-stream_ordinal)%scene_count",
        },
        "difficulty_strata_present": has_strata,
        "v2_runtime_telemetry": {
            "graph_fields_present": has_v2_graph_telemetry,
            "query_fields_present": has_v2_query_telemetry,
            "complete": has_v2_graph_telemetry and has_v2_query_telemetry,
        },
        "scene_metadata": {
            scene: {
                "obstacle_kind": query_by_key[
                    (streams[0], methods[0], scene, "dynamic")
                ]["obstacle_kind"],
                "difficulty": scene_strata[scene][0] if has_strata else None,
                "base_trajectory_id": (
                    scene_pair_identity[scene][0]
                    if has_trajectory_pairing else None
                ),
            }
            for scene in scene_ids
        },
    }


METRIC_DEFINITIONS = {
    "clear_success": {
        "label": "Clear exact-valid success",
        "unit": "proportion",
        "family": "rate",
    },
    "dynamic_success": {
        "label": "Dynamic exact-valid success",
        "unit": "proportion",
        "family": "rate",
    },
    "conditional_retention": {
        "label": "Dynamic success conditional on clear success",
        "unit": "proportion",
        "family": "rate",
    },
    "path_change_joint_success": {
        "label": "Path changed among clear+dynamic joint successes",
        "unit": "proportion",
        "family": "rate",
    },
    "build_time_ms": {
        "label": "Roadmap build time",
        "unit": "ms",
        "family": "continuous",
    },
    "planning_time_clear_ms": {
        "label": "Clear internal planning time (exact-success only)",
        "unit": "ms",
        "family": "continuous",
    },
    "planning_time_dynamic_ms": {
        "label": "Dynamic internal planning time (exact-success only)",
        "unit": "ms",
        "family": "continuous",
    },
    "query_latency_clear_ms": {
        "label": "Clear publish-to-plan latency (all outcomes)",
        "unit": "ms",
        "family": "continuous",
    },
    "query_latency_dynamic_ms": {
        "label": "Dynamic publish-to-plan latency (all outcomes)",
        "unit": "ms",
        "family": "continuous",
    },
    "path_cost_clear": {
        "label": "Clear total joint-path cost (successes only)",
        "unit": "joint-space cost",
        "family": "continuous",
    },
    "path_cost_dynamic": {
        "label": "Dynamic total joint-path cost (successes only)",
        "unit": "joint-space cost",
        "family": "continuous",
    },
}


def stream_contributions(data, method, metric):
    contributions = {}
    raw_values = []
    for stream in data["streams"]:
        numerator = 0.0
        denominator = 0
        if metric == "build_time_ms":
            value = data["graph_by_key"][(stream, method)]["build_time_ms"]
            numerator, denominator = value, 1
            raw_values.append(value)
        else:
            for scene in data["scenes"]:
                clear = data["query_by_key"][(stream, method, scene, "clear")]
                dynamic = data["query_by_key"][(stream, method, scene, "dynamic")]
                if metric == "clear_success":
                    value = float(clear["exact_valid"])
                elif metric == "dynamic_success":
                    value = float(dynamic["exact_valid"])
                elif metric == "conditional_retention":
                    if not clear["exact_valid"]:
                        continue
                    value = float(dynamic["exact_valid"])
                elif metric == "path_change_joint_success":
                    if not (clear["exact_valid"] and dynamic["exact_valid"]):
                        continue
                    value = float(clear["path_ids"] != dynamic["path_ids"])
                elif metric == "planning_time_clear_ms":
                    if not clear["exact_valid"]:
                        continue
                    value = clear["planning_time_ms"]
                elif metric == "planning_time_dynamic_ms":
                    if not dynamic["exact_valid"]:
                        continue
                    value = dynamic["planning_time_ms"]
                elif metric == "query_latency_clear_ms":
                    value = clear["publish_to_plan_ms"]
                elif metric == "query_latency_dynamic_ms":
                    value = dynamic["publish_to_plan_ms"]
                elif metric == "path_cost_clear":
                    if not clear["exact_valid"]:
                        continue
                    value = clear["total_joint_path_cost"]
                elif metric == "path_cost_dynamic":
                    if not dynamic["exact_valid"]:
                        continue
                    value = dynamic["total_joint_path_cost"]
                else:
                    raise KeyError(metric)
                numerator += value
                denominator += 1
                raw_values.append(value)
        contributions[stream] = (numerator, denominator)
    return contributions, raw_values


def ratio_estimate(contributions, sampled_streams=None):
    keys = list(contributions) if sampled_streams is None else sampled_streams
    numerator = sum(contributions[key][0] for key in keys)
    denominator = sum(contributions[key][1] for key in keys)
    return None if denominator == 0 else numerator / denominator


def percentile(values, probability):
    if not values:
        return None
    ordered = sorted(values)
    if len(ordered) == 1:
        return ordered[0]
    location = probability * (len(ordered) - 1)
    lower = int(math.floor(location))
    upper = int(math.ceil(location))
    fraction = location - lower
    return ordered[lower] * (1.0 - fraction) + ordered[upper] * fraction


def distribution_summary(values):
    if not values:
        return {
            "n": 0, "mean": None, "sd": None, "median": None,
            "q1": None, "q3": None, "min": None, "max": None,
        }
    return {
        "n": len(values),
        "mean": statistics.fmean(values),
        "sd": statistics.stdev(values) if len(values) > 1 else None,
        "median": statistics.median(values),
        "q1": percentile(values, 0.25),
        "q3": percentile(values, 0.75),
        "min": min(values),
        "max": max(values),
    }


def derived_seed(seed, *parts):
    material = "\0".join(str(part) for part in parts).encode("utf-8")
    offset = int.from_bytes(hashlib.sha256(material).digest()[:8], "big")
    return seed ^ offset


def bootstrap_method_ci(contributions, streams, repetitions, level, seed):
    rng = random.Random(seed)
    estimates = []
    for _ in range(repetitions):
        sampled = [streams[rng.randrange(len(streams))] for _ in streams]
        estimate = ratio_estimate(contributions, sampled)
        if estimate is not None:
            estimates.append(estimate)
    alpha = (1.0 - level) / 2.0
    return {
        "level": level,
        "lower": percentile(estimates, alpha),
        "upper": percentile(estimates, 1.0 - alpha),
        "valid_replicates": len(estimates),
        "requested_replicates": repetitions,
        "resampling_unit": "roadmap_stream_cluster",
    }


def bootstrap_paired_difference(left, right, streams, repetitions, level, seed):
    rng = random.Random(seed)
    estimates = []
    for _ in range(repetitions):
        sampled = [streams[rng.randrange(len(streams))] for _ in streams]
        left_value = ratio_estimate(left, sampled)
        right_value = ratio_estimate(right, sampled)
        if left_value is not None and right_value is not None:
            estimates.append(left_value - right_value)
    alpha = (1.0 - level) / 2.0
    return {
        "level": level,
        "lower": percentile(estimates, alpha),
        "upper": percentile(estimates, 1.0 - alpha),
        "valid_replicates": len(estimates),
        "requested_replicates": repetitions,
        "resampling_unit": "paired_roadmap_stream_cluster",
    }


def exact_two_sided_sign_test(positive, negative):
    non_ties = positive + negative
    if non_ties == 0:
        return None
    lower_tail = sum(math.comb(non_ties, index) for index in range(
        min(positive, negative) + 1
    )) / (2 ** non_ties)
    return min(1.0, 2.0 * lower_tail)


def bootstrap_difference_ci(
    contributions, streams, repetitions, level, seed,
    resampling_unit="paired_roadmap_stream_cluster",
):
    """Bootstrap a paired mean/ratio difference stored per whole stream."""
    rng = random.Random(seed)
    estimates = []
    for _ in range(repetitions):
        sampled = [streams[rng.randrange(len(streams))] for _ in streams]
        estimate = ratio_estimate(contributions, sampled)
        if estimate is not None:
            estimates.append(estimate)
    alpha = (1.0 - level) / 2.0
    return {
        "level": level,
        "lower": percentile(estimates, alpha),
        "upper": percentile(estimates, 1.0 - alpha),
        "valid_replicates": len(estimates),
        "requested_replicates": repetitions,
        "resampling_unit": resampling_unit,
    }


def paired_metric_contributions(data, left_method, right_method, metric):
    """Return within-cell paired differences and their common support.

    Planning-time and path-cost effects are deliberately restricted to cells
    where both methods returned exact-valid paths.  Path-change differences use
    the stricter intersection where both phases succeeded for both methods.
    """
    differences = {}
    left_values = {}
    right_values = {}
    support_definition = "all_fixed_catalog_cells"
    for stream in data["streams"]:
        difference_sum = 0.0
        left_sum = 0.0
        right_sum = 0.0
        count = 0
        if metric == "build_time_ms":
            left_value = data["graph_by_key"][(stream, left_method)]["build_time_ms"]
            right_value = data["graph_by_key"][(stream, right_method)]["build_time_ms"]
            difference_sum = left_value - right_value
            left_sum = left_value
            right_sum = right_value
            count = 1
            support_definition = "all_paired_roadmap_streams"
        else:
            for scene in data["scenes"]:
                left_clear = data["query_by_key"][
                    (stream, left_method, scene, "clear")
                ]
                right_clear = data["query_by_key"][
                    (stream, right_method, scene, "clear")
                ]
                left_dynamic = data["query_by_key"][
                    (stream, left_method, scene, "dynamic")
                ]
                right_dynamic = data["query_by_key"][
                    (stream, right_method, scene, "dynamic")
                ]
                if metric == "clear_success":
                    left_value = float(left_clear["exact_valid"])
                    right_value = float(right_clear["exact_valid"])
                elif metric == "dynamic_success":
                    left_value = float(left_dynamic["exact_valid"])
                    right_value = float(right_dynamic["exact_valid"])
                elif metric == "path_change_joint_success":
                    if not all((
                        left_clear["exact_valid"], left_dynamic["exact_valid"],
                        right_clear["exact_valid"], right_dynamic["exact_valid"],
                    )):
                        continue
                    left_value = float(left_clear["path_ids"] != left_dynamic["path_ids"])
                    right_value = float(
                        right_clear["path_ids"] != right_dynamic["path_ids"]
                    )
                    support_definition = (
                        "common_four_way_clear_dynamic_joint_success_cells"
                    )
                elif metric in ("planning_time_clear_ms", "path_cost_clear"):
                    if not (left_clear["exact_valid"] and right_clear["exact_valid"]):
                        continue
                    left_value = (
                        left_clear["planning_time_ms"]
                        if metric == "planning_time_clear_ms"
                        else left_clear["total_joint_path_cost"]
                    )
                    right_value = (
                        right_clear["planning_time_ms"]
                        if metric == "planning_time_clear_ms"
                        else right_clear["total_joint_path_cost"]
                    )
                    support_definition = "common_clear_exact_success_cells"
                elif metric in ("planning_time_dynamic_ms", "path_cost_dynamic"):
                    if not (
                        left_dynamic["exact_valid"] and right_dynamic["exact_valid"]
                    ):
                        continue
                    left_value = (
                        left_dynamic["planning_time_ms"]
                        if metric == "planning_time_dynamic_ms"
                        else left_dynamic["total_joint_path_cost"]
                    )
                    right_value = (
                        right_dynamic["planning_time_ms"]
                        if metric == "planning_time_dynamic_ms"
                        else right_dynamic["total_joint_path_cost"]
                    )
                    support_definition = "common_dynamic_exact_success_cells"
                elif metric == "query_latency_clear_ms":
                    left_value = left_clear["publish_to_plan_ms"]
                    right_value = right_clear["publish_to_plan_ms"]
                    support_definition = "all_clear_query_outcomes"
                elif metric == "query_latency_dynamic_ms":
                    left_value = left_dynamic["publish_to_plan_ms"]
                    right_value = right_dynamic["publish_to_plan_ms"]
                    support_definition = "all_dynamic_query_outcomes"
                else:
                    raise KeyError(metric)
                difference_sum += left_value - right_value
                left_sum += left_value
                right_sum += right_value
                count += 1
        differences[stream] = (difference_sum, count)
        left_values[stream] = (left_sum, count)
        right_values[stream] = (right_sum, count)
    return {
        "differences": differences,
        "left": left_values,
        "right": right_values,
        "eligible_cells": sum(value[1] for value in differences.values()),
        "support_definition": support_definition,
    }


def paired_stream_differences(data, left_method, right_method, phase="dynamic"):
    """Per-stream risk differences over every scene in the fixed catalog."""
    metric = "dynamic_success" if phase == "dynamic" else "clear_success"
    contributions = paired_metric_contributions(
        data, left_method, right_method, metric
    )["differences"]
    result = {}
    for stream in data["streams"]:
        result[stream] = ratio_estimate(contributions, [stream])
    return result


def bootstrap_stream_mean_ci(differences, streams, repetitions, level, seed):
    """Primary fixed-catalog CI: resample complete roadmap streams only."""
    values = [differences[stream] for stream in streams]
    if any(value is None for value in values):
        fail("primary stream difference is undefined")
    rng = random.Random(seed)
    estimates = []
    for _ in range(repetitions):
        estimates.append(statistics.fmean(
            values[rng.randrange(len(values))] for _ in values
        ))
    alpha = (1.0 - level) / 2.0
    return {
        "level": level,
        "lower": percentile(estimates, alpha),
        "upper": percentile(estimates, 1.0 - alpha),
        "valid_replicates": len(estimates),
        "requested_replicates": repetitions,
        "resampling_unit": "paired_whole_roadmap_stream",
        "catalog_treatment": "fixed_scene_catalog_not_resampled",
        "resampling_seed": seed,
    }


def studentized_mean_statistic(values, tolerance=1e-15):
    if not values:
        return None
    mean = statistics.fmean(values)
    if len(values) < 2:
        if abs(mean) <= tolerance:
            return 0.0
        return math.copysign(math.inf, mean)
    sd = statistics.stdev(values)
    if sd <= tolerance:
        if abs(mean) <= tolerance:
            return 0.0
        return math.copysign(math.inf, mean)
    return mean / (sd / math.sqrt(len(values)))


def paired_stream_permutation(differences, streams, repetitions, seed):
    """Monte-Carlo paired method-label swap using one sign per stream."""
    values = [differences[stream] for stream in streams]
    if any(value is None for value in values):
        fail("permutation input contains an undefined stream difference")
    observed = studentized_mean_statistic(values)
    if all(value == 0.0 for value in values):
        return {
            "test": "monte_carlo_paired_label_swap",
            "statistic": "studentized_mean_stream_risk_difference",
            "observed_statistic": 0.0,
            "observed_statistic_is_infinite": False,
            "two_sided_p": 1.0,
            "requested_permutations": repetitions,
            "valid_permutations": repetitions,
            "extreme_permutations": repetitions,
            "plus_one_correction": True,
            "exchangeability_assumption": "method_labels_exchangeable_within_stream",
            "degenerate_case": "all_stream_differences_zero",
        }
    rng = random.Random(seed)
    extreme = 0
    observed_absolute = abs(observed)
    for _ in range(repetitions):
        permuted = [
            value if rng.getrandbits(1) else -value for value in values
        ]
        statistic = studentized_mean_statistic(permuted)
        if abs(statistic) >= observed_absolute - 1e-15:
            extreme += 1
    return {
        "test": "monte_carlo_paired_label_swap",
        "statistic": "studentized_mean_stream_risk_difference",
        "observed_statistic": finite_or_none(observed),
        "observed_statistic_is_infinite": math.isinf(observed),
        "two_sided_p": (extreme + 1.0) / (repetitions + 1.0),
        "requested_permutations": repetitions,
        "valid_permutations": repetitions,
        "extreme_permutations": extreme,
        "plus_one_correction": True,
        "exchangeability_assumption": "method_labels_exchangeable_within_stream",
        "degenerate_case": None,
    }


def paired_stream_sign_summary(contributions, streams, tolerance=1e-12):
    differences = []
    for stream in streams:
        value = ratio_estimate(contributions, [stream])
        if value is not None:
            differences.append(value)
    positive = sum(value > tolerance for value in differences)
    negative = sum(value < -tolerance for value in differences)
    ties = len(differences) - positive - negative
    non_ties = positive + negative
    sd = statistics.stdev(differences) if len(differences) > 1 else None
    return {
        "paired_streams": len(differences),
        "positive": positive,
        "negative": negative,
        "ties": ties,
        "exact_sign_test_two_sided_p": exact_two_sided_sign_test(positive, negative),
        "sign_dominance": (
            (positive - negative) / non_ties if non_ties else None
        ),
        "paired_standardized_mean_difference_dz": (
            statistics.fmean(differences) / sd
            if sd is not None and sd > tolerance else None
        ),
        "test_unit": "roadmap_stream_summary",
        "zero_difference_handling": "discarded",
    }


def two_way_stratified_base_bootstrap_ci(
    data, left_method, right_method, repetitions, level, seed,
):
    """Scene-generalization sensitivity, separate from fixed-catalog inference.

    Streams and base trajectories are independently resampled.  Base sampling
    occurs within difficulty strata and both point/segment variants of a sampled
    base trajectory always travel together.
    """
    pairing = data["trajectory_pair_audit"]
    if not pairing["present"] or not data["difficulty_strata_present"]:
        differences = paired_stream_differences(
            data, left_method, right_method, "dynamic"
        )
        interval = bootstrap_stream_mean_ci(
            differences, data["streams"], repetitions, level, seed
        )
        interval.update({
            "analysis_role": "legacy_fallback_not_scene_generalization",
            "resampling_unit": "paired_whole_roadmap_stream",
            "base_trajectory_pairing_available": False,
            "difficulty_stratified": False,
        })
        return interval

    base_to_scenes = {}
    difficulty_to_bases = {difficulty: [] for difficulty in DIFFICULTIES}
    for scene in data["scenes"]:
        metadata = data["scene_metadata"][scene]
        base_to_scenes.setdefault(metadata["base_trajectory_id"], []).append(scene)
    for base, scenes in base_to_scenes.items():
        if len(scenes) != 2:
            fail(f"base trajectory {base} does not contain both obstacle variants")
        scenes.sort(key=lambda item: data["scene_metadata"][item]["obstacle_kind"])
        difficulty = data["scene_metadata"][scenes[0]]["difficulty"]
        difficulty_to_bases[difficulty].append(base)
    difficulty_to_bases = {
        difficulty: sorted(bases)
        for difficulty, bases in difficulty_to_bases.items() if bases
    }
    base_stream_difference = {}
    for stream in data["streams"]:
        for base, scenes in base_to_scenes.items():
            values = []
            for scene in scenes:
                left = data["query_by_key"][
                    (stream, left_method, scene, "dynamic")
                ]["exact_valid"]
                right = data["query_by_key"][
                    (stream, right_method, scene, "dynamic")
                ]["exact_valid"]
                values.append(float(left) - float(right))
            base_stream_difference[(stream, base)] = statistics.fmean(values)

    rng = random.Random(seed)
    estimates = []
    streams = data["streams"]
    for _ in range(repetitions):
        sampled_streams = [streams[rng.randrange(len(streams))] for _ in streams]
        sampled_bases = []
        for bases in difficulty_to_bases.values():
            sampled_bases.extend(
                bases[rng.randrange(len(bases))] for _ in bases
            )
        estimates.append(statistics.fmean(
            base_stream_difference[(stream, base)]
            for stream in sampled_streams for base in sampled_bases
        ))
    alpha = (1.0 - level) / 2.0
    return {
        "level": level,
        "lower": percentile(estimates, alpha),
        "upper": percentile(estimates, 1.0 - alpha),
        "valid_replicates": len(estimates),
        "requested_replicates": repetitions,
        "resampling_unit": (
            "paired_roadmap_stream_x_base_trajectory_with_point_segment_together"
        ),
        "analysis_role": "scene_generalization_sensitivity_not_primary",
        "base_trajectory_pairing_available": True,
        "difficulty_stratified": True,
        "difficulty_base_counts": {
            key: len(value) for key, value in difficulty_to_bases.items()
        },
    }


def binary_discordance(data, left_method, right_method, phase):
    left_only = 0
    right_only = 0
    both = 0
    neither = 0
    for stream, scene in itertools.product(data["streams"], data["scenes"]):
        left = data["query_by_key"][(stream, left_method, scene, phase)]["exact_valid"]
        right = data["query_by_key"][(stream, right_method, scene, phase)]["exact_valid"]
        if left and right:
            both += 1
        elif left:
            left_only += 1
        elif right:
            right_only += 1
        else:
            neither += 1
    odds = left_only / right_only if right_only else None
    odds_note = None
    if right_only == 0:
        odds_note = "undefined_no_discordance" if left_only == 0 else "positive_infinity"
    return {
        "matched_pairs": left_only + right_only + both + neither,
        "both_success": both,
        "left_only_success": left_only,
        "right_only_success": right_only,
        "neither_success": neither,
        "discordant_odds_ratio": odds,
        "discordant_odds_ratio_note": odds_note,
        "mcnemar_test_reported": False,
        "mcnemar_reason": (
            "scene-level pairs repeat within roadmap streams; the independent-pair "
            "assumption is not justified. Primary inference uses a paired whole-stream "
            "method-label permutation test."
        ),
    }


def choose_study_kind(requested, stream_count, base_trajectory_count=None):
    """Never promote a dataset to confirmatory without an explicit request."""
    if requested == "auto":
        return "smoke_descriptive"
    if requested == "confirmatory":
        if stream_count < CONFIRMATORY_MIN_STREAMS:
            fail(
                "confirmatory labeling requires at least "
                f"{CONFIRMATORY_MIN_STREAMS} roadmap streams"
            )
        if (
            base_trajectory_count is None
            or base_trajectory_count < CONFIRMATORY_MIN_BASE_TRAJECTORIES
        ):
            fail(
                "confirmatory labeling requires paired telemetry for at least "
                f"{CONFIRMATORY_MIN_BASE_TRAJECTORIES} base trajectories"
            )
        return "confirmatory"
    return "smoke_descriptive"


def require_confirmatory_design(data, manifest, v2_artifact_audit):
    require(
        manifest.get("schema") == RUN_SCHEMA_V2,
        "confirmatory analysis requires a v2 run manifest",
    )
    require(
        manifest.get("protocol_id") == CONFIRMATORY_PROTOCOL_ID,
        f"confirmatory analysis requires protocol_id={CONFIRMATORY_PROTOCOL_ID}",
    )
    require(
        manifest.get("catalog_id") == CONFIRMATORY_CATALOG_ID,
        f"confirmatory catalog_id must equal {CONFIRMATORY_CATALOG_ID}",
    )
    require(
        tuple(data["methods"]) == CONFIRMATORY_METHODS,
        "confirmatory analysis requires exactly gng, guarded_gng, and halton_prm",
    )
    require(
        tuple(data["streams"]) == CONFIRMATORY_STREAMS,
        "confirmatory roadmap stream labels must equal the frozen sequence 100..159",
    )
    require(
        data["counts"]["scenes"] == CONFIRMATORY_SCENES
        and data["counts"]["base_trajectories"]
        == CONFIRMATORY_BASE_TRAJECTORIES
        and data["counts"]["graphs"] == CONFIRMATORY_GRAPH_BUILDS
        and data["counts"]["queries"] == CONFIRMATORY_QUERY_ROWS,
        "confirmatory design must contain exactly 60 scenes, 30 base trajectories, "
        "180 graphs, and 21600 queries",
    )
    require(
        manifest.get("sample_count") == 800
        and manifest.get("halton_start_index") == 17
        and math.isclose(
            float(manifest.get("guarded_fraction", math.nan)), 0.75,
            rel_tol=0.0, abs_tol=1.0e-12,
        )
        and manifest.get("rmw_implementation") == "rmw_fastrtps_cpp",
        "confirmatory roadmap/RMW parameters do not match the frozen protocol",
    )
    require(
        data.get("v2_runtime_telemetry", {}).get("complete") is True,
        "confirmatory analysis requires all v2 graph/query runtime telemetry fields",
    )
    require(
        data["phase_order_position_present"]
        and data["phase_order_audit"]["balanced"] is True,
        "confirmatory analysis requires explicit balanced phase-order telemetry",
    )
    require(
        data["trajectory_pair_audit"]["present"]
        and data["trajectory_pair_audit"]["exact_point_segment_pairs"] is True,
        "confirmatory analysis requires exact point/segment base-trajectory pairing",
    )
    require(
        data["scene_order_audit"]["identical_across_methods"]
        and data["scene_order_audit"]["expected_catalog_rotation"],
        "confirmatory scene order must use the expected catalog rotation identically "
        "across methods",
    )
    require(
        len(data["streams"]) == len(CONFIRMATORY_STREAMS),
        "confirmatory counterbalance requires exactly 60 roadmap streams",
    )
    counts = {
        tuple(payload["order"]): payload["count"]
        for payload in data["method_order_audit"]["permutation_counts"]
    }
    expected_orders = set(itertools.permutations(data["methods"]))
    require(
        set(counts) == expected_orders
        and all(counts[order] == 10 for order in expected_orders),
        "confirmatory method-order counterbalance requires all 6 permutations "
        "exactly 10 times each",
    )
    require(
        isinstance(v2_artifact_audit, dict)
        and v2_artifact_audit.get("run_config_verified") is True,
        "confirmatory analysis requires a valid v2 run_config.json artifact digest",
    )
    frozen_inputs = manifest.get("artifacts", {}).get("frozen_inputs")
    require(
        v2_artifact_audit.get("frozen_inputs_verified") is True
        and isinstance(frozen_inputs, dict)
        and set(frozen_inputs) == CONFIRMATORY_FROZEN_INPUTS,
        "confirmatory analysis requires the exact frozen catalog, generation log, "
        "source snapshot, protocol, and analyzer bundle",
    )
    require(
        frozen_inputs["catalog.json"] == manifest["catalog_sha256"],
        "bundled catalog digest disagrees with the run manifest",
    )
    require(
        frozen_inputs["analyze_reachability_multiscene.py"]
        == v2_artifact_audit.get("executing_analyzer_sha256"),
        "confirmatory results must be analyzed by the exact bundled analyzer",
    )
    run_config = v2_artifact_audit.get("run_config")
    require(
        isinstance(run_config, dict)
        and run_config.get("schema") == "om6dof-reachability-multiscene-config-v2",
        "confirmatory run_config.json has the wrong schema",
    )
    expected_config = {
        "protocol_id": CONFIRMATORY_PROTOCOL_ID,
        "catalog_id": CONFIRMATORY_CATALOG_ID,
        "catalog_sha256": manifest["catalog_sha256"],
        "expected_catalog_sha256": manifest["catalog_sha256"],
        "streams": list(CONFIRMATORY_STREAMS),
        "methods": list(CONFIRMATORY_METHODS),
        "scene_count": CONFIRMATORY_SCENES,
        "base_trajectory_count": CONFIRMATORY_BASE_TRAJECTORIES,
        "graph_builds": CONFIRMATORY_GRAPH_BUILDS,
        "query_rows": CONFIRMATORY_QUERY_ROWS,
        "sample_count": 800,
        "halton_start_index": 17,
        "guarded_fraction": 0.75,
        "rmw_implementation": "rmw_fastrtps_cpp",
        "ros_localhost_only": True,
        "phase_order_design": PHASE_ORDER_DESIGN,
        "source_tree_sha256": manifest["implementation"]["source_tree_sha256"],
        "runner_script_sha256": manifest["implementation"]["runner_script_sha256"],
        "reachability_node_binary_sha256": manifest["implementation"][
            "reachability_node_binary_sha256"
        ],
        "catalog_model": manifest["catalog_model"],
        "catalog_generator_implementation": manifest[
            "catalog_generator_implementation"
        ],
        "frozen_input_sha256": frozen_inputs,
    }
    for field, expected in expected_config.items():
        require(
            run_config.get(field) == expected,
            f"confirmatory run_config.json field {field} disagrees with the manifest",
        )
    quota = Counter(
        (
            data["scene_metadata"][scene]["difficulty"],
            data["scene_metadata"][scene]["obstacle_kind"],
        )
        for scene in data["scenes"]
    )
    require(
        quota == Counter({
            (difficulty, obstacle): 10
            for difficulty in DIFFICULTIES
            for obstacle in ("point", "segment")
        }),
        "confirmatory catalog telemetry must contain exactly 10 scenes in every "
        "difficulty-by-obstacle cell",
    )


def require_confirmatory_resampling(repetitions, permutation_repetitions):
    require(
        repetitions >= CONFIRMATORY_MIN_BOOTSTRAP_REPETITIONS,
        "confirmatory analysis requires at least "
        f"{CONFIRMATORY_MIN_BOOTSTRAP_REPETITIONS} bootstrap repetitions",
    )
    require(
        permutation_repetitions >= CONFIRMATORY_MIN_PERMUTATION_REPETITIONS,
        "confirmatory analysis requires at least "
        f"{CONFIRMATORY_MIN_PERMUTATION_REPETITIONS} permutations",
    )


def empty_interval(level, repetitions, reason):
    return {
        "level": level,
        "lower": None,
        "upper": None,
        "valid_replicates": 0,
        "requested_replicates": repetitions,
        "resampling_unit": None,
        "not_computed_reason": reason,
    }


def empty_sign_summary():
    return {
        "paired_streams": 0,
        "positive": 0,
        "negative": 0,
        "ties": 0,
        "exact_sign_test_two_sided_p": None,
        "sign_dominance": None,
        "paired_standardized_mean_difference_dz": None,
        "test_unit": None,
        "zero_difference_handling": None,
    }


def holm_adjust_primary_family(comparisons):
    family = [
        payload for payload in comparisons
        if payload["hypothesis_role"] == "primary_confirmatory"
    ]
    require(len(family) == 2,
            "confirmatory primary family must contain exactly two contrasts")
    ordered = sorted(
        family, key=lambda payload: payload["primary_permutation_test"]["two_sided_p"]
    )
    running = 0.0
    for rank, payload in enumerate(ordered):
        raw = payload["primary_permutation_test"]["two_sided_p"]
        adjusted = min(1.0, (len(ordered) - rank) * raw)
        running = max(running, adjusted)
        payload["primary_permutation_test"].update({
            "holm_adjusted_p_primary_family": running,
            "holm_family_size": 2,
            "holm_family": [
                "guarded_gng_minus_gng_dynamic_exact_valid_risk_difference",
                "guarded_gng_minus_halton_prm_dynamic_exact_valid_risk_difference",
            ],
        })


def stratified_dynamic_results(data):
    """Descriptive dynamic success and risk differences by catalog stratum."""
    groupings = []
    obstacle_values = sorted({
        data["scene_metadata"][scene]["obstacle_kind"] for scene in data["scenes"]
    })
    groupings.extend(("obstacle_kind", value) for value in obstacle_values)
    if data["difficulty_strata_present"]:
        difficulty_values = [
            value for value in DIFFICULTIES
            if any(data["scene_metadata"][scene]["difficulty"] == value
                   for scene in data["scenes"])
        ]
        groupings.extend(("difficulty", value) for value in difficulty_values)
        groupings.extend(
            ("difficulty_x_obstacle_kind", f"{difficulty}:{kind}")
            for difficulty in difficulty_values for kind in obstacle_values
        )

    rows = []
    for dimension, level_name in groupings:
        if dimension == "obstacle_kind":
            scenes = [
                scene for scene, metadata in data["scene_metadata"].items()
                if metadata["obstacle_kind"] == level_name
            ]
        elif dimension == "difficulty":
            scenes = [
                scene for scene, metadata in data["scene_metadata"].items()
                if metadata["difficulty"] == level_name
            ]
        else:
            difficulty, kind = level_name.split(":", 1)
            scenes = [
                scene for scene, metadata in data["scene_metadata"].items()
                if metadata["difficulty"] == difficulty
                and metadata["obstacle_kind"] == kind
            ]
        if not scenes:
            continue
        rates = {}
        for method in data["methods"]:
            values = [
                float(data["query_by_key"][(stream, method, scene, "dynamic")][
                    "exact_valid"
                ])
                for stream, scene in itertools.product(data["streams"], scenes)
            ]
            rates[method] = statistics.fmean(values)
        contrasts = {}
        if PRIMARY_METHOD in rates:
            for baseline in PRIMARY_BASELINES:
                if baseline in rates:
                    contrasts[f"{PRIMARY_METHOD}_minus_{baseline}"] = (
                        rates[PRIMARY_METHOD] - rates[baseline]
                    )
        rows.append({
            "dimension": dimension,
            "level": level_name,
            "scene_count": len(scenes),
            "stream_scene_cells_per_method": len(scenes) * len(data["streams"]),
            "method_dynamic_exact_valid_rates": rates,
            "prespecified_risk_differences": contrasts,
            "analysis_role": "descriptive_stratum_report_not_multiplicity_tested",
        })
    return rows


def analyze(
    data, manifest, study_kind, repetitions=50000, level=0.95, seed=20260824,
    permutation_repetitions=100000,
):
    method_results = {}
    contribution_cache = {}
    for method in data["methods"]:
        method_results[method] = {}
        for metric, definition in METRIC_DEFINITIONS.items():
            contributions, raw_values = stream_contributions(data, method, metric)
            contribution_cache[(method, metric)] = contributions
            estimate = ratio_estimate(contributions)
            denominator = sum(value[1] for value in contributions.values())
            numerator = sum(value[0] for value in contributions.values())
            method_results[method][metric] = {
                "definition": definition["label"],
                "unit": definition["unit"],
                "estimate": estimate,
                "numerator_or_sum": numerator,
                "eligible_or_count": denominator,
                "distribution": distribution_summary(raw_values),
                "cluster_bootstrap_ci": bootstrap_method_ci(
                    contributions, data["streams"], repetitions, level,
                    derived_seed(seed, "method", method, metric),
                ),
                "analysis_role": (
                    "method_specific_descriptive"
                    if metric in ("conditional_retention", "path_change_joint_success")
                    else "descriptive_method_summary"
                ),
            }

    comparisons = []
    if study_kind == "confirmatory":
        missing = {
            PRIMARY_METHOD, *PRIMARY_BASELINES
        } - set(data["methods"])
        require(not missing,
                f"confirmatory analysis is missing prespecified methods {sorted(missing)}")
    for first_method, second_method in itertools.combinations(data["methods"], 2):
        pair = {first_method, second_method}
        if PRIMARY_METHOD in pair and len(pair & set(PRIMARY_BASELINES)) == 1:
            left_method = PRIMARY_METHOD
            right_method = next(iter(pair - {PRIMARY_METHOD}))
        else:
            left_method, right_method = first_method, second_method
        for metric, definition in METRIC_DEFINITIONS.items():
            left = contribution_cache[(left_method, metric)]
            right = contribution_cache[(right_method, metric)]
            prespecified = (
                metric == PRIMARY_METRIC
                and left_method == PRIMARY_METHOD
                and right_method in PRIMARY_BASELINES
            )
            hypothesis_role = (
                "primary_confirmatory"
                if prespecified and study_kind == "confirmatory"
                else (
                    "prespecified_primary_endpoint_descriptive"
                    if prespecified else "secondary_exploratory_descriptive"
                )
            )
            if metric == "conditional_retention":
                left_estimate = ratio_estimate(left)
                right_estimate = ratio_estimate(right)
                difference = (
                    left_estimate - right_estimate
                    if left_estimate is not None and right_estimate is not None else None
                )
                interval = empty_interval(
                    level, repetitions,
                    "method-specific conditioning sets differ; descriptive only",
                )
                sign_summary = empty_sign_summary()
                support_definition = "method_specific_clear_success_denominators"
                eligible_cells = None
                pair_contributions = None
            else:
                paired = paired_metric_contributions(
                    data, left_method, right_method, metric
                )
                pair_contributions = paired["differences"]
                left_estimate = ratio_estimate(paired["left"])
                right_estimate = ratio_estimate(paired["right"])
                difference = ratio_estimate(pair_contributions)
                support_definition = paired["support_definition"]
                eligible_cells = paired["eligible_cells"]
                sign_summary = paired_stream_sign_summary(
                    pair_contributions, data["streams"]
                )
                interval = bootstrap_difference_ci(
                    pair_contributions, data["streams"], repetitions, level,
                    derived_seed(seed, "pair", left_method, right_method, metric),
                )
            comparison = {
                "left_method": left_method,
                "right_method": right_method,
                "metric": metric,
                "definition": definition["label"],
                "unit": definition["unit"],
                "effect_definition": (
                    "paired within-cell left minus right"
                    if metric != "conditional_retention"
                    else "difference of method-specific conditional rates"
                ),
                "left_estimate": left_estimate,
                "right_estimate": right_estimate,
                "paired_effect": difference,
                "paired_cluster_bootstrap_ci": interval,
                "paired_stream_sign_summary": sign_summary,
                "support_definition": support_definition,
                "eligible_common_cells": eligible_cells,
                "hypothesis_role": hypothesis_role,
                "inferential_use": hypothesis_role == "primary_confirmatory",
                "fixed_catalog_estimand": (
                    "mean_over_roadmap_streams_of_within_stream_dynamic_"
                    "exact_valid_risk_difference_over_the_frozen_scene_catalog"
                    if prespecified else None
                ),
                "primary_permutation_test": None,
                "scene_generalization_sensitivity": None,
            }
            if prespecified:
                stream_differences = paired_stream_differences(
                    data, left_method, right_method, "dynamic"
                )
                comparison["paired_effect"] = statistics.fmean(
                    stream_differences.values()
                )
                comparison["paired_cluster_bootstrap_ci"] = bootstrap_stream_mean_ci(
                    stream_differences, data["streams"], repetitions, level,
                    derived_seed(seed, "primary-ci-shared-stream-resamples"),
                )
                comparison["primary_permutation_test"] = paired_stream_permutation(
                    stream_differences, data["streams"], permutation_repetitions,
                    derived_seed(seed, "primary-permutation", left_method, right_method),
                )
                comparison["scene_generalization_sensitivity"] = (
                    two_way_stratified_base_bootstrap_ci(
                        data, left_method, right_method, repetitions, level,
                        derived_seed(seed, "two-way", left_method, right_method),
                    )
                )
            if metric in ("clear_success", "dynamic_success"):
                phase = "clear" if metric == "clear_success" else "dynamic"
                comparison["binary_pair_description"] = binary_discordance(
                    data, left_method, right_method, phase
                )
            comparisons.append(comparison)
    if study_kind == "confirmatory":
        holm_adjust_primary_family(comparisons)
    return method_results, comparisons


def finite_or_none(value):
    if value is None:
        return None
    return value if math.isfinite(value) else None


def format_number(value, digits=4):
    if value is None:
        return "NA"
    return f"{value:.{digits}f}"


def format_percent(value):
    return "NA" if value is None else f"{100.0 * value:.1f}%"


def ci_text(payload, percent=False):
    if payload["lower"] is None or payload["upper"] is None:
        return "NA"
    if percent:
        return f"[{100.0 * payload['lower']:.1f}, {100.0 * payload['upper']:.1f}]%"
    return f"[{payload['lower']:.3f}, {payload['upper']:.3f}]"


def method_csv_rows(method_results):
    rows = []
    for method, metrics in method_results.items():
        for metric, payload in metrics.items():
            distribution = payload["distribution"]
            interval = payload["cluster_bootstrap_ci"]
            rows.append({
                "method": method,
                "metric": metric,
                "unit": payload["unit"],
                "estimate": payload["estimate"],
                "numerator_or_sum": payload["numerator_or_sum"],
                "eligible_or_count": payload["eligible_or_count"],
                "ci_level": interval["level"],
                "ci_lower": interval["lower"],
                "ci_upper": interval["upper"],
                **{f"raw_{key}": value for key, value in distribution.items()},
            })
    return rows


def comparison_csv_rows(comparisons, study_kind):
    rows = []
    for payload in comparisons:
        interval = payload["paired_cluster_bootstrap_ci"]
        sign = payload["paired_stream_sign_summary"]
        primary_test = payload["primary_permutation_test"] or {}
        sensitivity = payload["scene_generalization_sensitivity"] or {}
        rows.append({
            "study_kind": study_kind,
            "inferential_use": payload["inferential_use"],
            "hypothesis_role": payload["hypothesis_role"],
            "left_method": payload["left_method"],
            "right_method": payload["right_method"],
            "metric": payload["metric"],
            "unit": payload["unit"],
            "support_definition": payload["support_definition"],
            "eligible_common_cells": payload["eligible_common_cells"],
            "left_estimate": payload["left_estimate"],
            "right_estimate": payload["right_estimate"],
            "paired_effect_left_minus_right": payload["paired_effect"],
            "ci_level": interval["level"],
            "ci_lower": interval["lower"],
            "ci_upper": interval["upper"],
            "ci_resampling_unit": interval["resampling_unit"],
            "paired_streams": sign["paired_streams"],
            "positive_stream_differences": sign["positive"],
            "negative_stream_differences": sign["negative"],
            "tied_stream_differences": sign["ties"],
            "descriptive_exact_sign_p": sign["exact_sign_test_two_sided_p"],
            "sign_dominance": sign["sign_dominance"],
            "paired_standardized_mean_difference_dz": (
                sign["paired_standardized_mean_difference_dz"]
            ),
            "primary_studentized_permutation_p": primary_test.get("two_sided_p"),
            "primary_holm_adjusted_p": primary_test.get(
                "holm_adjusted_p_primary_family"
            ),
            "primary_permutation_repetitions": primary_test.get(
                "requested_permutations"
            ),
            "scene_sensitivity_ci_lower": sensitivity.get("lower"),
            "scene_sensitivity_ci_upper": sensitivity.get("upper"),
            "scene_sensitivity_resampling_unit": sensitivity.get("resampling_unit"),
        })
    return rows


def stratum_csv_rows(strata):
    rows = []
    for payload in strata:
        methods = payload["method_dynamic_exact_valid_rates"]
        contrasts = payload["prespecified_risk_differences"]
        rows.append({
            "dimension": payload["dimension"],
            "level": payload["level"],
            "scene_count": payload["scene_count"],
            "stream_scene_cells_per_method": payload[
                "stream_scene_cells_per_method"
            ],
            "gng_dynamic_exact_valid_rate": methods.get("gng"),
            "guarded_gng_dynamic_exact_valid_rate": methods.get("guarded_gng"),
            "halton_prm_dynamic_exact_valid_rate": methods.get("halton_prm"),
            "guarded_minus_gng_risk_difference": contrasts.get(
                "guarded_gng_minus_gng"
            ),
            "guarded_minus_halton_risk_difference": contrasts.get(
                "guarded_gng_minus_halton_prm"
            ),
            "analysis_role": payload["analysis_role"],
        })
    return rows


def render_markdown(result):
    study = result["study"]
    audit = result["audit"]
    methods = result["methods"]
    lines = [
        "# Multi-scene reachability benchmark summary",
        "",
    ]
    if study["kind"] == "smoke_descriptive":
        lines.extend([
            "**SMOKE / DESCRIPTIVE ONLY.** Confidence intervals and p-values "
            "are exploratory diagnostics, not confirmatory evidence.",
            "",
        ])
    else:
        lines.extend([
            "**EXPLICIT CONFIRMATORY ANALYSIS.** Inference is conditional on the fixed "
            "scene catalog and clusters deterministic roadmap streams.",
            "",
        ])
    lines.extend([
        f"Audit: **PASS** — {audit['counts']['graphs']} graph builds and "
        f"{audit['counts']['queries']} paired phase queries; no timeout or "
        "infrastructure-error rows.",
        f"Timing context: `{study['timing_context']}`.",
    ])
    if audit["counts"].get("base_trajectories") is not None:
        lines.append(
            f"Catalog pairing: {audit['counts']['base_trajectories']} base trajectories, "
            "each represented by exactly one point and one segment scene."
        )
    lines.extend([
        "",
        "## Per-method descriptive results",
        "",
        "| Method | Clear success | Dynamic success | Conditional retention | "
        "Path change on joint-success | Build ms (median [IQR]) |",
        "|---|---:|---:|---:|---:|---:|",
    ])
    for method, payload in methods.items():
        clear = payload["clear_success"]
        dynamic = payload["dynamic_success"]
        retention = payload["conditional_retention"]
        change = payload["path_change_joint_success"]
        build = payload["build_time_ms"]["distribution"]
        lines.append(
            f"| {method} | {format_percent(clear['estimate'])} "
            f"{ci_text(clear['cluster_bootstrap_ci'], True)} | "
            f"{format_percent(dynamic['estimate'])} "
            f"{ci_text(dynamic['cluster_bootstrap_ci'], True)} | "
            f"{format_percent(retention['estimate'])} | "
            f"{format_percent(change['estimate'])} | "
            f"{format_number(build['median'], 1)} "
            f"[{format_number(build['q1'], 1)}, {format_number(build['q3'], 1)}] |"
        )
    lines.extend([
        "",
        "| Method | Clear planning ms* | Dynamic planning ms* | Clear path cost* | "
        "Dynamic path cost* |",
        "|---|---:|---:|---:|---:|",
    ])
    for method, payload in methods.items():
        values = []
        for metric in (
            "planning_time_clear_ms", "planning_time_dynamic_ms",
            "path_cost_clear", "path_cost_dynamic",
        ):
            summary = payload[metric]["distribution"]
            values.append(
                f"{format_number(summary['median'], 3)} "
                f"[{format_number(summary['q1'], 3)}, {format_number(summary['q3'], 3)}]"
            )
        lines.append(f"| {method} | " + " | ".join(values) + " |")
    lines.extend([
        "",
        "*Planning-time and path-cost summaries include exact-valid paths only. "
        "Values are median [IQR].*",
        "",
        "| Method | Clear publish-to-plan ms (all outcomes) | "
        "Dynamic publish-to-plan ms (all outcomes) |",
        "|---|---:|---:|",
    ])
    for method, payload in methods.items():
        clear_latency = payload["query_latency_clear_ms"]["distribution"]
        dynamic_latency = payload["query_latency_dynamic_ms"]["distribution"]
        lines.append(
            f"| {method} | {format_number(clear_latency['median'], 3)} "
            f"[{format_number(clear_latency['q1'], 3)}, "
            f"{format_number(clear_latency['q3'], 3)}] | "
            f"{format_number(dynamic_latency['median'], 3)} "
            f"[{format_number(dynamic_latency['q1'], 3)}, "
            f"{format_number(dynamic_latency['q3'], 3)}] |"
        )
    lines.extend([
        "",
        "## Prespecified primary endpoint",
        "",
        "The endpoint is dynamic exact-valid success risk difference over the frozen "
        "catalog. Effects are guarded GNG minus baseline. The primary CI resamples "
        "whole roadmap streams only; the p-value uses a studentized paired stream-level "
        "method-label permutation. Holm correction covers exactly the two prespecified "
        "contrasts.",
        "",
        "| Comparison | Risk difference | Fixed-catalog CI | Permutation p / Holm p | "
        "Sign dominance* |",
        "|---|---:|---:|---:|---:|",
    ])
    for payload in result["pairwise_comparisons"]:
        if payload["hypothesis_role"] not in (
            "primary_confirmatory", "prespecified_primary_endpoint_descriptive"
        ):
            continue
        sign = payload["paired_stream_sign_summary"]
        test = payload["primary_permutation_test"]
        lines.append(
            f"| {payload['left_method']} − {payload['right_method']} | "
            f"{format_percent(payload['paired_effect'])} | "
            f"{ci_text(payload['paired_cluster_bootstrap_ci'], True)} | "
            f"{format_number(test['two_sided_p'], 4)} / "
            f"{format_number(test.get('holm_adjusted_p_primary_family'), 4)} | "
            f"{format_number(sign['sign_dominance'], 3)} |"
        )
    lines.extend([
        "",
        "*Sign dominance is (positive streams − negative streams) / non-tied streams; "
        "it is not a rank-biserial effect.*",
        "",
        "The two-way stream × base-trajectory bootstrap is reported only as a "
        "scene-generalization sensitivity in `pairwise.csv` and `analysis.json`; it is "
        "not the primary fixed-catalog CI.",
        "",
        "## Descriptive catalog strata",
        "",
        "These subgroup estimates are descriptive and are not multiplicity-tested.",
        "",
        "| Dimension | Level | Scenes | GNG | Guarded GNG | Halton PRM | "
        "Guarded−GNG | Guarded−Halton |",
        "|---|---|---:|---:|---:|---:|---:|---:|",
    ])
    for payload in result["stratified_dynamic_results"]:
        rates = payload["method_dynamic_exact_valid_rates"]
        contrasts = payload["prespecified_risk_differences"]
        lines.append(
            f"| {payload['dimension']} | {payload['level']} | "
            f"{payload['scene_count']} | {format_percent(rates.get('gng'))} | "
            f"{format_percent(rates.get('guarded_gng'))} | "
            f"{format_percent(rates.get('halton_prm'))} | "
            f"{format_percent(contrasts.get('guarded_gng_minus_gng'))} | "
            f"{format_percent(contrasts.get('guarded_gng_minus_halton_prm'))} |"
        )
    lines.extend([
        "",
        "All secondary effects, including build/planning time and path cost, are "
        "descriptive and retained in `pairwise.csv` and `analysis.json`. Planning and "
        "path-cost differences use only common exact-success cells; path-change uses "
        "common four-way clear/dynamic joint-success cells.",
        "",
        "## Statistical scope",
        "",
        "- The primary estimand averages within-stream dynamic exact-valid risk "
        "differences over the frozen scene catalog, then averages over roadmap streams.",
        "- The primary bootstrap resamples complete roadmap streams and never resamples "
        "catalog scenes.",
        "- Both primary contrasts use the same deterministic sequence of resampled "
        "roadmap-stream indices.",
        "- Scene-level McNemar tests are intentionally not reported because repeated "
        "scenes within a stream are not independent pairs.",
        "- Conditional retention is dynamic success divided by clear successes for "
        "the same method-stream-scene cells and remains method-specific descriptive.",
        "- Path change compares clear and dynamic `path_ids` only when both plans are "
        "exact-valid and remains descriptive.",
        "",
    ])
    return "\n".join(lines)


def write_csv(path, rows, fieldnames=None):
    fields = list(fieldnames) if fieldnames is not None else list(rows[0])
    temporary = path.with_suffix(path.suffix + ".tmp")
    with temporary.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)
        stream.flush()
        os.fsync(stream.fileno())
    os.replace(temporary, path)


def atomic_write_text(path, text):
    temporary = path.with_suffix(path.suffix + ".tmp")
    with temporary.open("w", encoding="utf-8", newline="\n") as stream:
        stream.write(text)
        stream.flush()
        os.fsync(stream.fileno())
    os.replace(temporary, path)


def run_analysis(
    input_dir, output_dir=None, study_kind="auto", repetitions=50000,
    level=0.95, seed=20260824, permutation_repetitions=100000,
    timing_context="unspecified", overwrite=False,
):
    input_dir = Path(input_dir).expanduser().resolve()
    paths = {
        "graphs": input_dir / "graphs.csv",
        "queries": input_dir / "queries.csv",
        "manifest": input_dir / "manifest.json",
    }
    for name, path in paths.items():
        if not path.is_file():
            fail(f"missing {name} input: {path}")
    input_hashes = {name: sha256_file(path) for name, path in paths.items()}
    manifest = strict_json_load(paths["manifest"])
    if (
        isinstance(manifest, dict)
        and isinstance(manifest.get("artifacts"), dict)
        and "run_config.json" in manifest["artifacts"]
    ):
        run_config_path = input_dir / "run_config.json"
        require(run_config_path.is_file(), "missing run_config input")
        paths["run_config"] = run_config_path
        input_hashes["run_config"] = sha256_file(run_config_path)
    raw_graphs = read_csv(paths["graphs"], GRAPH_REQUIRED_FIELDS)
    raw_queries = read_csv(paths["queries"], QUERY_REQUIRED_FIELDS)
    data = audit_bundle(manifest, raw_graphs, raw_queries)
    v2_provenance_verified = audit_v2_artifact_contents(
        manifest, input_dir, input_hashes
    )
    chosen_kind = choose_study_kind(
        study_kind, data["counts"]["streams"],
        data["counts"]["base_trajectories"],
    )
    if chosen_kind == "confirmatory":
        require_confirmatory_design(data, manifest, v2_provenance_verified)
        require_confirmatory_resampling(repetitions, permutation_repetitions)
        require(
            data["difficulty_strata_present"],
            "confirmatory analysis requires difficulty stratum telemetry",
        )
    method_results, comparisons = analyze(
        data, manifest, chosen_kind, repetitions, level, seed,
        permutation_repetitions,
    )
    strata = stratified_dynamic_results(data)
    result = {
        "schema": SCHEMA,
        "generated_utc": datetime.now(timezone.utc).isoformat(),
        "inputs": {
            name: {"path": str(path), "sha256": input_hashes[name]}
            for name, path in paths.items()
        },
        "study": {
            "kind": chosen_kind,
            "requested_kind": study_kind,
            "timing_context": timing_context,
            "confirmatory_thresholds": {
                "minimum_streams": CONFIRMATORY_MIN_STREAMS,
                "minimum_base_trajectories": CONFIRMATORY_MIN_BASE_TRAJECTORIES,
                "minimum_bootstrap_repetitions": (
                    CONFIRMATORY_MIN_BOOTSTRAP_REPETITIONS
                ),
                "minimum_permutation_repetitions": (
                    CONFIRMATORY_MIN_PERMUTATION_REPETITIONS
                ),
            },
            "fixed_scene_catalog": True,
            "primary_endpoint": "dynamic_exact_valid_success",
            "primary_effect": "guarded_gng_minus_baseline_risk_difference",
            "primary_estimand": (
                "mean roadmap-stream risk difference over every scene in the "
                "frozen catalog"
            ),
            "primary_contrasts": [
                "guarded_gng_minus_gng",
                "guarded_gng_minus_halton_prm",
            ],
            "multiplicity_control": "Holm across exactly two primary contrasts",
            "inference_scope": (
                "roadmap_stream_population_conditional_on_frozen_fixed_scene_catalog"
            ),
            "bootstrap_repetitions": repetitions,
            "bootstrap_seed": seed,
            "permutation_repetitions": permutation_repetitions,
            "confidence_level": level,
            "smoke_results_are_descriptive_only": chosen_kind == "smoke_descriptive",
        },
        "audit": {
            "passed": True,
            "fail_fast": True,
            "run_manifest_schema": manifest["schema"],
            "v2_provenance_verified": (
                v2_provenance_verified["verified"]
                if isinstance(v2_provenance_verified, dict) else None
            ),
            "v2_run_config_verified": (
                v2_provenance_verified["run_config_verified"]
                if isinstance(v2_provenance_verified, dict) else None
            ),
            "v2_frozen_inputs_verified": (
                v2_provenance_verified["frozen_inputs_verified"]
                if isinstance(v2_provenance_verified, dict) else None
            ),
            "counts": data["counts"],
            "method_order": data["method_order_audit"],
            "scene_order": data["scene_order_audit"],
            "phase_order_position_present": data["phase_order_position_present"],
            "phase_order": data["phase_order_audit"],
            "trajectory_pairing": data["trajectory_pair_audit"],
            "difficulty_strata_present": data["difficulty_strata_present"],
            "v2_runtime_telemetry": data["v2_runtime_telemetry"],
            "raw_inputs_unchanged": True,
            "invariants": [
                "complete stream-method graph Cartesian product",
                "complete stream-method-scene-phase query Cartesian product",
                "unique correlated query IDs and graph revisions",
                "no timeout or infrastructure-error row",
                "valid equals exact_valid",
                "start/target echoes and successful path-cost decomposition",
                "preview-only, localhost-only, and no controller publications",
                (
                    "phase order follows balanced stream+catalog parity"
                    if data["phase_order_position_present"]
                    else "legacy artifact without explicit phase-order telemetry"
                ),
                (
                    "exact point/segment pairing per base trajectory and source counter"
                    if data["trajectory_pair_audit"]["present"]
                    else "legacy artifact without base-trajectory pairing telemetry"
                ),
            ],
        },
        "metric_definitions": METRIC_DEFINITIONS,
        "source_provenance": (
            {
                "phase_order_design": manifest["phase_order_design"],
                "catalog_model": manifest["catalog_model"],
                "catalog_generator_implementation": (
                    manifest["catalog_generator_implementation"]
                ),
                "implementation": manifest["implementation"],
                "artifacts": manifest["artifacts"],
            }
            if manifest["schema"] == RUN_SCHEMA_V2 else None
        ),
        "methods": method_results,
        "pairwise_comparisons": comparisons,
        "stratified_dynamic_results": strata,
    }

    output_dir = (
        Path(output_dir).expanduser().resolve()
        if output_dir is not None else input_dir / "analysis"
    )
    require(output_dir != input_dir,
            "output directory must differ from the raw input directory")
    known_outputs = (
        output_dir / "analysis.json", output_dir / "summary.md",
        output_dir / "methods.csv", output_dir / "pairwise.csv",
        output_dir / "strata.csv",
    )
    if output_dir.exists() and any(path.exists() for path in known_outputs) and not overwrite:
        fail(f"analysis outputs already exist in {output_dir}; pass --overwrite")
    output_dir.mkdir(parents=True, exist_ok=True)

    # Verify inputs immediately before output and again after output.  This makes
    # accidental in-place raw-data changes observable and fatal.
    require(
        input_hashes == {name: sha256_file(path) for name, path in paths.items()},
        "raw input changed during analysis before output",
    )
    atomic_write_text(
        output_dir / "analysis.json",
        json.dumps(result, sort_keys=True, indent=2, allow_nan=False) + "\n",
    )
    atomic_write_text(output_dir / "summary.md", render_markdown(result))
    method_rows = method_csv_rows(method_results)
    comparison_rows = comparison_csv_rows(comparisons, chosen_kind)
    stratum_rows = stratum_csv_rows(strata)
    write_csv(
        output_dir / "methods.csv", method_rows,
        fieldnames=list(method_rows[0]),
    )
    write_csv(
        output_dir / "pairwise.csv",
        comparison_rows,
        fieldnames=(
            list(comparison_rows[0]) if comparison_rows else [
                "study_kind", "inferential_use", "left_method", "right_method",
                "metric", "unit", "left_estimate", "right_estimate",
                "paired_effect_left_minus_right", "ci_level", "ci_lower",
                "ci_upper", "paired_streams", "positive_stream_differences",
                "negative_stream_differences", "tied_stream_differences",
                "descriptive_exact_sign_p", "sign_dominance",
                "paired_standardized_mean_difference_dz",
            ]
        ),
    )
    write_csv(
        output_dir / "strata.csv", stratum_rows,
        fieldnames=(
            list(stratum_rows[0]) if stratum_rows else [
                "dimension", "level", "scene_count",
                "stream_scene_cells_per_method", "analysis_role",
            ]
        ),
    )
    require(
        input_hashes == {name: sha256_file(path) for name, path in paths.items()},
        "raw input changed while writing analysis outputs",
    )
    if manifest["schema"] == RUN_SCHEMA_V2:
        audit_v2_artifact_contents(manifest, input_dir, input_hashes)
    return result, output_dir


def main(argv=None):
    parser = argparse.ArgumentParser(
        description="Fail-fast audit and paired analysis of multi-scene benchmark outputs"
    )
    parser.add_argument("--input-dir", required=True)
    parser.add_argument("--output-dir")
    parser.add_argument(
        "--study-kind", choices=("auto", "smoke", "confirmatory"), default="auto"
    )
    parser.add_argument("--bootstrap-repetitions", type=int, default=50000)
    parser.add_argument("--confidence-level", type=float, default=0.95)
    parser.add_argument("--bootstrap-seed", type=int, default=20260824)
    parser.add_argument("--permutation-repetitions", type=int, default=100000)
    parser.add_argument("--timing-context", default="unspecified")
    parser.add_argument("--overwrite", action="store_true")
    args = parser.parse_args(argv)
    if args.bootstrap_repetitions < 200:
        parser.error("--bootstrap-repetitions must be >= 200")
    if not 0.5 < args.confidence_level < 1.0:
        parser.error("--confidence-level must be between 0.5 and 1")
    if args.bootstrap_seed < 0:
        parser.error("--bootstrap-seed must be non-negative")
    if args.permutation_repetitions < 200:
        parser.error("--permutation-repetitions must be >= 200")
    try:
        result, output_dir = run_analysis(
            args.input_dir,
            output_dir=args.output_dir,
            study_kind=args.study_kind,
            repetitions=args.bootstrap_repetitions,
            level=args.confidence_level,
            seed=args.bootstrap_seed,
            permutation_repetitions=args.permutation_repetitions,
            timing_context=args.timing_context,
            overwrite=args.overwrite,
        )
    except AuditError as exc:
        print(f"AUDIT FAILED: {exc}", file=sys.stderr)
        return 2
    print(json.dumps({
        "audit": "PASS",
        "study_kind": result["study"]["kind"],
        "output_dir": str(output_dir),
        "counts": result["audit"]["counts"],
    }, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
