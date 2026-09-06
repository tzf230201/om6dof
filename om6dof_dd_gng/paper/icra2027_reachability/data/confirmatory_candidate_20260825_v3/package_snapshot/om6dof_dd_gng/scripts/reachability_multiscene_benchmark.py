#!/usr/bin/env python3
"""Preview-only paired multi-scene reachability benchmark.

Each roadmap process receives atomic ReachabilityQuery messages containing the
complete start state, target, and environment. The script never publishes to a
controller topic and never creates a controller action client.
"""

import argparse
import csv
import hashlib
import itertools
import json
import math
import os
import platform
import shutil
import signal
import subprocess
import sys
import time
from datetime import datetime, timezone
from pathlib import Path


SUPPORTED_METHODS = ("gng", "guarded_gng", "halton_prm")
UINT32_MAX = 2**32 - 1
CATALOG_SCHEMA_VERSION = 2
DIFFICULTIES = ("low", "medium", "high")
REQUIRED_MODEL_HASHES = (
    "expanded_urdf_sha256", "srdf_sha256", "reachability_parameters_sha256"
)
REQUIRED_ORACLE_TRUE = (
    "clear_evaluated", "clear_start_self_valid", "clear_target_self_valid",
    "clear_detour_self_valid", "clear_capsule_direct_valid",
    "clear_exact_direct_valid", "dynamic_evaluated",
    "dynamic_capsule_direct_blocked", "dynamic_exact_direct_blocked",
    "dynamic_capsule_detour_valid", "dynamic_exact_detour_valid",
    "dynamic_start_valid", "dynamic_target_valid", "dynamic_detour_state_valid",
)
CONFIRMATORY_PROTOCOL_ID = "icra_confirmatory_v3"
CONFIRMATORY_STREAMS = tuple(range(100, 160))
CONFIRMATORY_SCENES = 60
CONFIRMATORY_BASE_TRAJECTORIES = 30
CONFIRMATORY_MODEL_HASHES = {
    "expanded_urdf_sha256": "daf37611724f4c8efd69b3b470bf505cfce4732353ea985ec54fbbb01f6d412d",
    "srdf_sha256": "730e590951a205ec639ff20613ce3305e18290f0c9a599130d38ce7c86d3d424",
    "reachability_parameters_sha256": "abf60d6f21bbb0e9b77558316dfb746dacf3af37814a349c60308f7dcfd22175",
}
CONFIRMATORY_MASTER_KEY_HEX = (
    "600025f316f133ef34d1baf8bb9107b3aed500e069247baa4ebb5d6af45ad92f"
)
FROZEN_INPUT_ARGUMENTS = {
    "catalog.json": "catalog",
    "catalog_generation.log": "catalog_generation_log",
    "source_snapshot.tar.gz": "source_snapshot",
    "confirmatory_protocol.md": "protocol_document",
    "analyze_reachability_multiscene.py": "analyzer_script",
}
CONFIRMATORY_REQUIRED_FROZEN_INPUTS = frozenset(FROZEN_INPUT_ARGUMENTS)


def file_sha256(path):
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def resolve_frozen_input_sources(args):
    """Resolve immutable inputs that will be copied into the result bundle."""
    resolved = {}
    for bundle_name, argument_name in FROZEN_INPUT_ARGUMENTS.items():
        value = getattr(args, argument_name, "")
        if not value:
            continue
        path = Path(value).expanduser().resolve(strict=True)
        if not path.is_file():
            raise ValueError(f"--{argument_name.replace('_', '-')} must be a file")
        resolved[bundle_name] = path
    if "catalog.json" not in resolved:
        raise ValueError("--catalog must resolve to a regular file")
    return resolved


def freeze_input_bundle(output_dir, sources, resume):
    """Atomically copy or verify every frozen source under ``frozen_inputs``."""
    target_dir = Path(output_dir) / "frozen_inputs"
    target_dir.mkdir(exist_ok=True)
    hashes = {}
    for name, source in sources.items():
        expected = file_sha256(source)
        target = target_dir / name
        if target.exists():
            if not resume:
                raise ValueError(f"unexpected pre-existing frozen input: {target}")
            if not target.is_file() or file_sha256(target) != expected:
                raise ValueError(f"resume frozen input mismatch: {name}")
        else:
            if resume:
                raise ValueError(f"resume frozen input is missing: {name}")
            temporary = target.with_name(target.name + ".tmp")
            with source.open("rb") as input_stream, temporary.open("wb") as output_stream:
                shutil.copyfileobj(input_stream, output_stream, length=1024 * 1024)
                output_stream.flush()
                os.fsync(output_stream.fileno())
            os.replace(temporary, target)
        hashes[name] = expected
    return hashes


def reachability_binary_path():
    try:
        from ament_index_python.packages import get_package_prefix
        candidate = (
            Path(get_package_prefix("om6dof_dd_gng"))
            / "lib" / "om6dof_dd_gng" / "reachability_graph_node"
        )
        if candidate.is_file():
            return str(candidate)
    except (ImportError, LookupError):
        pass
    return shutil.which("reachability_graph_node")


def validate_node_binary_provenance(catalog, current_binary_sha256):
    generated_with = catalog["generator"]["implementation"][
        "reachability_node_binary_sha256"
    ].lower()
    if generated_with != current_binary_sha256.lower():
        raise ValueError(
            "catalog collision-oracle binary differs from the current benchmark "
            "reachability_graph_node; regenerate the catalog after rebuilding"
        )


def git_provenance(path):
    def git(*arguments):
        completed = subprocess.run(
            ["git", "-C", str(Path(path).resolve().parent), *arguments],
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.DEVNULL,
            check=False,
        )
        return completed.stdout.strip() if completed.returncode == 0 else ""

    status = git("status", "--porcelain", "--untracked-files=all")
    return {
        "head": git("rev-parse", "HEAD"),
        "describe": git("describe", "--always", "--dirty"),
        "dirty": bool(status),
        "dirty_entry_count": len(status.splitlines()) if status else 0,
    }


def finite_vector(value, length, name):
    if not isinstance(value, list) or len(value) != length:
        raise ValueError(f"{name} must contain exactly {length} values")
    result = []
    for item in value:
        number = float(item)
        if not math.isfinite(number):
            raise ValueError(f"{name} contains a non-finite value")
        result.append(number)
    return result


def strict_json_load(path):
    return json.loads(
        Path(path).read_text(encoding="utf-8"),
        parse_constant=lambda value: (_ for _ in ()).throw(
            ValueError(f"non-standard JSON constant {value}")
        ),
    )


def validate_node(node, name):
    if not isinstance(node, dict):
        raise ValueError(f"{name} must be an object")
    node_id = node.get("id")
    if not isinstance(node_id, int) or not 0 <= node_id < UINT32_MAX:
        raise ValueError(f"{name}.id must be a uint32 value below the sentinel")
    class_id = node.get("class_id")
    if not isinstance(class_id, int):
        raise ValueError(f"{name}.class_id must be an integer")
    confidence = float(node.get("confidence", 1.0))
    if not math.isfinite(confidence) or not 0.0 <= confidence <= 1.0:
        raise ValueError(f"{name}.confidence must be finite and within [0, 1]")
    position = finite_vector(node.get("position"), 3, f"{name}.position")
    return {
        "id": node_id,
        "class_id": class_id,
        "confidence": confidence,
        "position": position,
    }


def validate_pose_dict(pose, name):
    if not isinstance(pose, dict):
        raise ValueError(f"{name} must be an object")
    position = finite_vector(pose.get("position"), 3, f"{name}.position")
    orientation = finite_vector(
        pose.get("orientation_xyzw"), 4, f"{name}.orientation_xyzw"
    )
    norm = math.sqrt(sum(value * value for value in orientation))
    if not math.isclose(norm, 1.0, rel_tol=0.0, abs_tol=1.0e-6):
        raise ValueError(f"{name}.orientation_xyzw must be normalized")
    return {"position": position, "orientation_xyzw": orientation}


def validate_catalog(payload):
    if not isinstance(payload, dict) or payload.get("schema_version") != CATALOG_SCHEMA_VERSION:
        raise ValueError(
            f"scene catalog schema_version must equal {CATALOG_SCHEMA_VERSION}"
        )
    catalog_id = payload.get("catalog_id")
    if not isinstance(catalog_id, str) or not catalog_id:
        raise ValueError("catalog_id must be a non-empty string")
    frame_id = payload.get("frame_id")
    if frame_id != "world":
        raise ValueError("catalog frame_id must be 'world'")
    if payload.get("group_name") != "arm":
        raise ValueError("catalog group_name must be 'arm'")
    if payload.get("end_effector_link") != "end_effector_link":
        raise ValueError("catalog end_effector_link must be 'end_effector_link'")
    joint_names = payload.get("joint_names")
    if (
        not isinstance(joint_names, list)
        or not joint_names
        or any(not isinstance(name, str) or not name for name in joint_names)
        or len(set(joint_names)) != len(joint_names)
    ):
        raise ValueError("joint_names must be a non-empty unique string list")
    scenes = payload.get("scenes")
    if not isinstance(scenes, list) or not scenes:
        raise ValueError("catalog must contain at least one scene")

    model = payload.get("model")
    if not isinstance(model, dict):
        raise ValueError("catalog model provenance must be an object")
    for key in REQUIRED_MODEL_HASHES:
        value = model.get(key)
        if (
            not isinstance(value, str) or len(value) != 64
            or any(character not in "0123456789abcdefABCDEF" for character in value)
        ):
            raise ValueError(f"model.{key} must be a SHA-256 hex digest")
    generator = payload.get("generator")
    if (
        not isinstance(generator, dict)
        or generator.get("roadmap_independent") is not True
        or generator.get("paired_obstacle_design") is not True
    ):
        raise ValueError(
            "catalog generator must declare roadmap_independent and paired_obstacle_design"
        )
    lower = finite_vector(
        generator.get("joint_lower_bounds"), len(joint_names),
        "generator.joint_lower_bounds"
    )
    upper = finite_vector(
        generator.get("joint_upper_bounds"), len(joint_names),
        "generator.joint_upper_bounds"
    )
    if any(maximum <= minimum for minimum, maximum in zip(lower, upper)):
        raise ValueError("generator joint bounds must be increasing")
    oracle_metadata = generator.get("oracle")
    if (
        not isinstance(oracle_metadata, dict)
        or oracle_metadata.get("query_mode") is not True
        or oracle_metadata.get("graph_method") != "halton_prm"
        or set(oracle_metadata.get("collision_checks", []))
        != {"conservative_capsules", "moveit_fcl"}
    ):
        raise ValueError("generator collision-oracle metadata is incomplete")
    implementation = generator.get("implementation")
    if not isinstance(implementation, dict):
        raise ValueError("generator implementation provenance must be an object")
    for key in ("generator_script_sha256", "reachability_node_binary_sha256"):
        value = implementation.get(key)
        if (
            not isinstance(value, str) or len(value) != 64
            or any(character not in "0123456789abcdefABCDEF" for character in value)
        ):
            raise ValueError(f"generator.implementation.{key} must be a SHA-256 digest")

    normalized_scenes = []
    scene_ids = set()
    environment_ids = set()
    for index, scene in enumerate(scenes):
        name = f"scenes[{index}]"
        if not isinstance(scene, dict):
            raise ValueError(f"{name} must be an object")
        scene_id = scene.get("scene_id")
        if not isinstance(scene_id, str) or not scene_id or scene_id in scene_ids:
            raise ValueError(f"{name}.scene_id must be non-empty and unique")
        scene_ids.add(scene_id)
        if scene.get("catalog_index") != index:
            raise ValueError(f"{name}.catalog_index must equal its zero-based list index")
        source_counter = scene.get("source_counter")
        if not isinstance(source_counter, int) or source_counter < 0:
            raise ValueError(f"{name}.source_counter must be a non-negative integer")
        base_trajectory_id = scene.get("base_trajectory_id")
        if base_trajectory_id != f"base_{source_counter:06d}":
            raise ValueError(
                f"{name}.base_trajectory_id must be derived from source_counter"
            )
        start_joints = finite_vector(
            scene.get("start_joint_positions"), len(joint_names),
            f"{name}.start_joint_positions"
        )
        target = validate_node(scene.get("target"), f"{name}.target")
        if target["class_id"] < 0:
            raise ValueError(f"{name}.target must have class_id >= 0")
        if target["id"] in environment_ids:
            raise ValueError(f"{name}.target.id must be globally unique")
        environment_ids.add(target["id"])
        source_joints = finite_vector(
            scene.get("target", {}).get("source_joint_positions"), len(joint_names),
            f"{name}.target.source_joint_positions"
        )
        dynamic = scene.get("dynamic")
        if not isinstance(dynamic, dict) or dynamic.get("kind") not in ("point", "segment"):
            raise ValueError(f"{name}.dynamic.kind must be 'point' or 'segment'")
        nodes = [
            validate_node(node, f"{name}.dynamic.nodes[{node_index}]")
            for node_index, node in enumerate(dynamic.get("nodes", []))
        ]
        if any(node["class_id"] >= 0 for node in nodes):
            raise ValueError(f"{name}.dynamic obstacle nodes must have class_id < 0")
        stratum = scene.get("stratum")
        if (
            not isinstance(stratum, dict)
            or stratum.get("difficulty") not in DIFFICULTIES
            or stratum.get("obstacle_kind") != dynamic["kind"]
        ):
            raise ValueError(f"{name}.stratum is inconsistent with its obstacle")
        all_ids = {target["id"]}
        for node in nodes:
            if node["id"] in all_ids:
                raise ValueError(f"{name} contains duplicate environment node IDs")
            if node["id"] in environment_ids:
                raise ValueError(f"{name} contains a globally duplicated environment node ID")
            all_ids.add(node["id"])
            environment_ids.add(node["id"])
        edges = []
        for edge_index, edge in enumerate(dynamic.get("edges", [])):
            edge_name = f"{name}.dynamic.edges[{edge_index}]"
            if not isinstance(edge, dict):
                raise ValueError(f"{edge_name} must be an object")
            source = edge.get("source_id")
            target_id = edge.get("target_id")
            cost = float(edge.get("cost", 0.0))
            if source not in all_ids or target_id not in all_ids or source == target_id:
                raise ValueError(f"{edge_name} references invalid endpoints")
            if source == target["id"] or target_id == target["id"]:
                raise ValueError(f"{edge_name} must connect obstacle endpoints only")
            if not math.isfinite(cost) or cost < 0.0:
                raise ValueError(f"{edge_name}.cost must be finite and non-negative")
            edges.append({"source_id": source, "target_id": target_id, "cost": cost})
        expected_nodes = 1 if dynamic["kind"] == "point" else 2
        expected_edges = 0 if dynamic["kind"] == "point" else 1
        if len(nodes) != expected_nodes or len(edges) != expected_edges:
            raise ValueError(
                f"{name}.dynamic {dynamic['kind']} requires "
                f"{expected_nodes} obstacle node(s) and {expected_edges} edge(s)"
            )
        if dynamic["kind"] == "segment":
            segment_length = math.dist(nodes[0]["position"], nodes[1]["position"])
            if not math.isclose(edges[0]["cost"], segment_length, rel_tol=0.0, abs_tol=1.0e-9):
                raise ValueError(f"{name}.dynamic segment edge cost must equal its length")
        oracle = scene.get("oracle")
        if not isinstance(oracle, dict):
            raise ValueError(f"{name}.oracle must be an object")
        missing_or_false = [key for key in REQUIRED_ORACLE_TRUE if oracle.get(key) is not True]
        if missing_or_false:
            raise ValueError(f"{name}.oracle evidence failed or is missing: {missing_or_false}")
        detour_joints = finite_vector(
            oracle.get("detour_joint_positions"), len(joint_names),
            f"{name}.oracle.detour_joint_positions"
        )
        hit_fraction = float(oracle.get("hit_fraction", math.nan))
        target_obstacle_distance = float(
            oracle.get("target_obstacle_distance_m", math.nan)
        )
        joint_distance = float(oracle.get("joint_distance_normalized", math.nan))
        hit_pose = validate_pose_dict(oracle.get("hit_ee_pose"), f"{name}.oracle.hit_ee_pose")
        if not 0.0 < hit_fraction < 1.0:
            raise ValueError(f"{name}.oracle.hit_fraction must lie inside (0, 1)")
        if not math.isfinite(target_obstacle_distance) or target_obstacle_distance <= 0.0:
            raise ValueError(f"{name}.oracle target-obstacle distance must be positive")
        if not math.isfinite(joint_distance) or joint_distance < 0.0:
            raise ValueError(f"{name}.oracle joint distance must be non-negative")
        if dynamic["kind"] == "point":
            obstacle_midpoint = nodes[0]["position"]
        else:
            obstacle_midpoint = [
                0.5 * (left + right)
                for left, right in zip(nodes[0]["position"], nodes[1]["position"])
            ]
        if any(
            abs(actual - expected) > 1.0e-9
            for actual, expected in zip(obstacle_midpoint, hit_pose["position"])
        ):
            raise ValueError(f"{name}.dynamic obstacle must be centered on oracle hit pose")
        for label, joints in (
            ("start", start_joints), ("target", source_joints), ("detour", detour_joints)
        ):
            if any(
                value < minimum or value > maximum
                for value, minimum, maximum in zip(joints, lower, upper)
            ):
                raise ValueError(f"{name}.{label} joints exceed generator bounds")
        normalized_scenes.append({
            "scene_id": scene_id,
            "catalog_index": int(scene.get("catalog_index", index)),
            "source_counter": source_counter,
            "base_trajectory_id": base_trajectory_id,
            "stratum": stratum,
            "start_joint_positions": start_joints,
            "target": {**target, "source_joint_positions": source_joints},
            "dynamic": {"kind": dynamic["kind"], "nodes": nodes, "edges": edges},
            "oracle": oracle,
        })

    base_groups = {}
    for scene in normalized_scenes:
        base_groups.setdefault(scene["base_trajectory_id"], []).append(scene)
    for base_id, variants in base_groups.items():
        if len(variants) != 2 or {item["dynamic"]["kind"] for item in variants} != {
            "point", "segment"
        }:
            raise ValueError(
                f"base trajectory {base_id} must have exactly point and segment variants"
            )
        first, second = variants
        if (
            first["source_counter"] != second["source_counter"]
            or first["stratum"]["difficulty"] != second["stratum"]["difficulty"]
            or any(
                abs(left - right) > 1.0e-12
                for left, right in zip(
                    first["start_joint_positions"], second["start_joint_positions"]
                )
            )
            or any(
                abs(left - right) > 1.0e-12
                for left, right in zip(
                    first["target"]["source_joint_positions"],
                    second["target"]["source_joint_positions"],
                )
            )
            or any(
                abs(left - right) > 1.0e-12
                for left, right in zip(
                    first["target"]["position"], second["target"]["position"]
                )
            )
            or abs(
                float(first["oracle"]["hit_fraction"])
                - float(second["oracle"]["hit_fraction"])
            ) > 1.0e-12
            or any(
                abs(left - right) > 1.0e-9
                for left, right in zip(
                    first["oracle"]["hit_ee_pose"]["position"],
                    second["oracle"]["hit_ee_pose"]["position"],
                )
            )
            or abs(
                float(first["oracle"]["joint_distance_normalized"])
                - float(second["oracle"]["joint_distance_normalized"])
            ) > 1.0e-12
        ):
            raise ValueError(f"base trajectory {base_id} variants are not kinematically paired")
    if generator.get("base_trajectory_count") != len(base_groups):
        raise ValueError("generator base_trajectory_count does not match catalog pairing")
    if len(normalized_scenes) % 6:
        raise ValueError("catalog scene count must be divisible by six")
    expected_cell_count = len(normalized_scenes) // 6
    for difficulty in DIFFICULTIES:
        for obstacle_kind in ("point", "segment"):
            observed = sum(
                scene["stratum"] == {
                    "difficulty": difficulty, "obstacle_kind": obstacle_kind
                }
                for scene in normalized_scenes
            )
            if observed != expected_cell_count:
                raise ValueError(
                    "catalog difficulty-by-obstacle quota mismatch for "
                    f"{difficulty}/{obstacle_kind}: {observed} != {expected_cell_count}"
                )

    return {
        "schema_version": CATALOG_SCHEMA_VERSION,
        "catalog_id": catalog_id,
        "frame_id": frame_id,
        "group_name": payload["group_name"],
        "end_effector_link": payload["end_effector_link"],
        "joint_names": list(joint_names),
        "model": model,
        "generator": generator,
        "scenes": normalized_scenes,
    }


def load_catalog(path, max_scenes=0):
    raw = Path(path).read_bytes()
    payload = json.loads(raw.decode("utf-8"), parse_constant=lambda value: (_ for _ in ()).throw(
        ValueError(f"non-standard JSON constant {value}")
    ))
    catalog = validate_catalog(payload)
    if max_scenes:
        catalog["scenes"] = catalog["scenes"][:max_scenes]
    if not catalog["scenes"]:
        raise ValueError("scene selection is empty")
    return catalog, hashlib.sha256(raw).hexdigest()


def parse_unique_nonnegative_ints(text, option_name):
    values = [int(item.strip()) for item in text.split(",") if item.strip()]
    if not values or any(value < 0 for value in values) or len(values) != len(set(values)):
        raise ValueError(f"{option_name} must contain unique non-negative integers")
    return values


def validate_methods(text):
    methods = [item.strip() for item in text.split(",") if item.strip()]
    if not methods or len(methods) != len(set(methods)):
        raise ValueError("--methods must contain unique method names")
    invalid = set(methods) - set(SUPPORTED_METHODS)
    if invalid:
        raise ValueError(f"unsupported methods: {sorted(invalid)}")
    return methods


def validate_protocol(args, catalog, catalog_sha256, methods, stream_ids):
    if args.expected_catalog_sha256 and args.expected_catalog_sha256 != catalog_sha256:
        raise ValueError("catalog SHA-256 does not match --expected-catalog-sha256")
    if args.protocol_id == "development":
        return
    errors = []
    if args.protocol_id != CONFIRMATORY_PROTOCOL_ID:
        errors.append(f"unsupported protocol_id={args.protocol_id!r}")
    if tuple(methods) != SUPPORTED_METHODS:
        errors.append(f"methods must equal {list(SUPPORTED_METHODS)} in canonical order")
    if tuple(stream_ids) != CONFIRMATORY_STREAMS:
        errors.append("roadmap streams must equal the prespecified labels 100..159")
    if len(catalog["scenes"]) != CONFIRMATORY_SCENES:
        errors.append(f"catalog must contain exactly {CONFIRMATORY_SCENES} scenes")
    base_count = len({scene["base_trajectory_id"] for scene in catalog["scenes"]})
    if base_count != CONFIRMATORY_BASE_TRAJECTORIES:
        errors.append(
            f"catalog must contain exactly {CONFIRMATORY_BASE_TRAJECTORIES} base trajectories"
        )
    if catalog["catalog_id"] != "om6dof_icra_scene_catalog_v3":
        errors.append("catalog_id must equal om6dof_icra_scene_catalog_v3")
    if catalog["model"] != CONFIRMATORY_MODEL_HASHES:
        errors.append("catalog model hashes do not match the frozen protocol")
    if catalog["generator"].get("master_key_hex") != CONFIRMATORY_MASTER_KEY_HEX:
        errors.append("catalog master key does not match the frozen protocol")
    if args.sample_count != 800:
        errors.append("sample_count must equal 800")
    if args.halton_start_index != 17:
        errors.append("halton_start_index must equal 17")
    if not math.isclose(args.gng_guard_fraction, 0.75, rel_tol=0.0, abs_tol=1.0e-12):
        errors.append("gng_guard_fraction must equal 0.75")
    if args.max_scenes:
        errors.append("--max-scenes is forbidden by the confirmatory protocol")
    if not args.source_tree_sha256:
        errors.append("--source-tree-sha256 is required by the confirmatory protocol")
    if not args.expected_catalog_sha256:
        errors.append("--expected-catalog-sha256 is required by the confirmatory protocol")
    missing_frozen_inputs = [
        name for name, argument_name in FROZEN_INPUT_ARGUMENTS.items()
        if not getattr(args, argument_name, "")
    ]
    if missing_frozen_inputs:
        errors.append(
            "confirmatory frozen-input arguments are missing for "
            + ", ".join(missing_frozen_inputs)
        )
    if args.rmw_implementation != "rmw_fastrtps_cpp":
        errors.append("RMW implementation must equal rmw_fastrtps_cpp")
    if errors:
        raise ValueError("confirmatory protocol violation: " + "; ".join(errors))


def make_schedule(stream_ids, methods, scene_count):
    permutations = list(itertools.permutations(methods))
    canonical_index = {method: index for index, method in enumerate(methods)}
    schedule = []
    query_ids = set()
    for stream_ordinal, stream_id in enumerate(stream_ids):
        method_order = permutations[stream_ordinal % len(permutations)]
        scene_order = [
            (position + stream_ordinal) % scene_count for position in range(scene_count)
        ]
        for method_order_position, method in enumerate(method_order):
            run_slot = stream_ordinal * len(methods) + canonical_index[method]
            phase_query_ids = {}
            phase_orders = {}
            for catalog_index in range(scene_count):
                base = 1 + 2 * (run_slot * scene_count + catalog_index)
                phase_query_ids[catalog_index] = {"clear": base, "dynamic": base + 1}
                phase_orders[catalog_index] = (
                    ("clear", "dynamic")
                    if (stream_ordinal + catalog_index) % 2 == 0
                    else ("dynamic", "clear")
                )
                query_ids.update((base, base + 1))
            schedule.append({
                "stream_ordinal": stream_ordinal,
                "stream_id": stream_id,
                "method": method,
                "method_order_position": method_order_position,
                "scene_order": scene_order,
                "query_ids": phase_query_ids,
                "phase_orders": phase_orders,
            })
    expected = len(stream_ids) * len(methods) * scene_count * 2
    if len(query_ids) != expected:
        raise RuntimeError("query ID construction is not globally unique")
    return schedule


def summarize_graph(message):
    return {
        "reported_method": message.graph_method,
        "graph_revision": int(message.graph_revision),
        "expanded_urdf_sha256": message.expanded_urdf_sha256,
        "srdf_sha256": message.srdf_sha256,
        "reachability_parameters_sha256": (
            message.reachability_parameters_sha256
        ),
        "requested_node_count": int(message.requested_node_count),
        "anchor_node_count": int(message.anchor_node_count),
        "prototype_budget": int(message.prototype_budget),
        "prototype_node_count": int(message.prototype_node_count),
        "requested_guard_node_count": int(message.requested_guard_node_count),
        "guard_node_count": int(message.guard_node_count),
        "fill_sample_node_count": int(message.fill_sample_node_count),
        "candidate_attempts": int(message.candidate_attempts),
        "halton_start_index": int(message.halton_start_index),
        "sample_stream_seed": int(message.sample_stream_seed),
        "sample_stream_type": message.sample_stream_type,
        "gng_training_sample_count": int(message.gng_training_sample_count),
        "effective_guard_fraction": float(message.effective_guard_fraction),
        "nodes": len(message.nodes),
        "edges": len(message.edges),
        "components": int(message.connected_components),
        "build_time_ms": float(message.build_time_ms),
        "joint_names": list(message.joint_names),
    }


def summarize_plan(message, publish_to_plan_ms):
    preview_start = []
    if message.joint_path_preview.points:
        preview_start = list(message.joint_path_preview.points[0].positions)
    return {
        "plan_graph_method": message.graph_method,
        "graph_revision": int(message.graph_revision),
        "query_id": int(message.query_id),
        "scene_id": message.scene_id,
        "requested_target_environment_node_id": int(
            message.requested_target_environment_node_id
        ),
        "requested_target_position": [
            float(message.requested_target_position.x),
            float(message.requested_target_position.y),
            float(message.requested_target_position.z),
        ],
        "valid": bool(message.valid),
        "exact_valid": bool(message.exact_collision_valid),
        "reason": message.reason,
        "blocked_nodes": int(message.blocked_node_count),
        "blocked_edges": int(message.blocked_edge_count),
        "planning_time_ms": float(message.planning_time_ms),
        "publish_to_plan_ms": publish_to_plan_ms,
        "exact_checks": int(message.exact_state_checks),
        "exact_replans": int(message.exact_replans),
        "exact_time_ms": float(message.exact_validation_time_ms),
        "start_node_id": int(message.start_node_id),
        "goal_node_id": int(message.goal_node_id),
        "selected_target_environment_node_id": int(
            message.target_environment_node_id
        ),
        "target_distance": float(message.target_distance),
        "graph_cost": float(message.graph_cost),
        "start_connection_cost": float(message.start_connection_cost),
        "total_joint_path_cost": float(message.total_joint_path_cost),
        "path_nodes": len(message.reachability_node_ids),
        "path_ids": list(message.reachability_node_ids),
        "preview_start_joints": preview_start,
        "timeout": False,
        "infrastructure_error": "",
    }


def run_probe(args):
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from om6dof_dd_gng.msg import (
        EnvironmentGraph,
        EnvironmentNode,
        ReachabilityGraph,
        ReachabilityPlan,
        ReachabilityQuery,
        TopologyEdge,
    )

    catalog, catalog_sha256 = load_catalog(args.catalog, args.max_scenes)
    methods = validate_methods(args.methods)
    if args.method not in methods:
        raise ValueError("probe method is not present in --methods")
    stream_ids = parse_unique_nonnegative_ints(args.stream_list, "--stream-list")
    schedule = make_schedule(stream_ids, methods, len(catalog["scenes"]))
    matching_runs = [
        run for run in schedule
        if run["stream_ordinal"] == args.stream_ordinal and run["method"] == args.method
    ]
    if len(matching_runs) != 1:
        raise ValueError("probe run identity is not unique")
    run = matching_runs[0]

    latched = QoSProfile(depth=1)
    latched.reliability = ReliabilityPolicy.RELIABLE
    latched.durability = DurabilityPolicy.TRANSIENT_LOCAL

    class Probe(Node):
        def __init__(self):
            super().__init__("reachability_multiscene_probe")
            self.graph = None
            self.plans = {}
            self.create_subscription(
                ReachabilityGraph,
                "/om6dof_topo_gng/reachability_graph_data",
                self.graph_callback,
                latched,
            )
            self.create_subscription(
                ReachabilityPlan,
                "/om6dof_topo_gng/reachability_plan",
                self.plan_callback,
                latched,
            )
            self.query_pub = self.create_publisher(
                ReachabilityQuery,
                "/om6dof_topo_gng/reachability_query",
                latched,
            )

        def graph_callback(self, message):
            if message.nodes:
                self.graph = message

        def plan_callback(self, message):
            if message.query_id:
                self.plans[(int(message.query_id), message.scene_id)] = message

        def encode_query(self, scene, phase, query_id):
            query = ReachabilityQuery()
            query.header.stamp = self.get_clock().now().to_msg()
            query.header.frame_id = catalog["frame_id"]
            query.query_id = query_id
            query.scene_id = scene["scene_id"]
            query.start_state.header = query.header
            query.start_state.name = list(catalog["joint_names"])
            query.start_state.position = list(scene["start_joint_positions"])
            query.target_environment_node_id = scene["target"]["id"]
            query.target_position.x, query.target_position.y, query.target_position.z = (
                scene["target"]["position"]
            )
            environment = EnvironmentGraph()
            environment.header = query.header
            target = EnvironmentNode()
            target.id = scene["target"]["id"]
            target.position.x, target.position.y, target.position.z = (
                scene["target"]["position"]
            )
            target.class_id = scene["target"]["class_id"]
            target.confidence = scene["target"]["confidence"]
            environment.nodes.append(target)
            if phase == "dynamic":
                for encoded in scene["dynamic"]["nodes"]:
                    node = EnvironmentNode()
                    node.id = encoded["id"]
                    node.position.x, node.position.y, node.position.z = encoded["position"]
                    node.class_id = encoded["class_id"]
                    node.confidence = encoded["confidence"]
                    environment.nodes.append(node)
                for encoded in scene["dynamic"]["edges"]:
                    edge = TopologyEdge()
                    edge.source_id = encoded["source_id"]
                    edge.target_id = encoded["target_id"]
                    edge.cost = encoded["cost"]
                    environment.edges.append(edge)
            query.environment = environment
            return query

    rclpy.init()
    probe = Probe()
    graph_deadline = time.monotonic() + args.graph_timeout
    try:
        while rclpy.ok() and probe.graph is None and time.monotonic() < graph_deadline:
            rclpy.spin_once(probe, timeout_sec=0.05)
        if probe.graph is None:
            print(json.dumps({"error": "graph_timeout"}), flush=True)
            return 2
        discovery_deadline = time.monotonic() + min(5.0, args.graph_timeout)
        endpoint_counts = {}
        while rclpy.ok() and time.monotonic() < discovery_deadline:
            endpoint_counts = {
                "graph_publisher_count": probe.count_publishers(
                    "/om6dof_topo_gng/reachability_graph_data"
                ),
                "plan_publisher_count": probe.count_publishers(
                    "/om6dof_topo_gng/reachability_plan"
                ),
                "query_subscriber_count": probe.query_pub.get_subscription_count(),
            }
            if all(count >= 1 for count in endpoint_counts.values()):
                break
            rclpy.spin_once(probe, timeout_sec=0.05)
        if any(count != 1 for count in endpoint_counts.values()):
            print(json.dumps({
                "error": "runtime_endpoint_count_mismatch",
                **endpoint_counts,
            }, sort_keys=True), flush=True)
            return 4

        query_rows = []
        for scene_order_position, catalog_index in enumerate(run["scene_order"]):
            scene = catalog["scenes"][catalog_index]
            for phase_order_position, phase in enumerate(
                run["phase_orders"][catalog_index]
            ):
                query_id = run["query_ids"][catalog_index][phase]
                query = probe.encode_query(scene, phase, query_id)
                published = time.monotonic()
                probe.query_pub.publish(query)
                deadline = published + args.phase_timeout
                key = (query_id, scene["scene_id"])
                while rclpy.ok() and key not in probe.plans and time.monotonic() < deadline:
                    rclpy.spin_once(probe, timeout_sec=0.02)
                common = {
                    "scene_id": scene["scene_id"],
                    "catalog_index": catalog_index,
                    "base_trajectory_id": scene["base_trajectory_id"],
                    "source_counter": scene["source_counter"],
                    "scene_order_position": scene_order_position,
                    "phase": phase,
                    "phase_order_position": phase_order_position,
                    "obstacle_kind": "none" if phase == "clear" else scene["dynamic"]["kind"],
                    "query_id": query_id,
                    "start_joint_positions": scene["start_joint_positions"],
                    "target_position": scene["target"]["position"],
                    "target_source_joint_positions": scene["target"]["source_joint_positions"],
                    "stratum": scene.get("stratum", {}),
                }
                if key not in probe.plans:
                    query_rows.append({
                        **common,
                        "valid": False,
                        "exact_valid": False,
                        "reason": "phase_timeout",
                        "timeout": True,
                        "infrastructure_error": "no_correlated_plan_before_deadline",
                    })
                    continue
                message = probe.plans.pop(key)
                elapsed_ms = (time.monotonic() - published) * 1000.0
                row = {**common, **summarize_plan(message, elapsed_ms)}
                row["infrastructure_error"] = "; ".join(query_contract_errors(
                    row,
                    scene,
                    phase,
                    int(probe.graph.graph_revision),
                    len(catalog["joint_names"]),
                    run["method"],
                ))
                query_rows.append(row)

        graph_summary = summarize_graph(probe.graph)
        graph_summary.update(endpoint_counts)
        output = {
            "catalog_sha256": catalog_sha256,
            "catalog_id": catalog["catalog_id"],
            "graph": graph_summary,
            "queries": query_rows,
        }
        print(json.dumps(output, sort_keys=True), flush=True)
        return 0
    finally:
        probe.destroy_node()
        rclpy.shutdown()


def assert_clean_ros_domain(env, timeout_sec=8.0):
    """Fail closed if an isolated benchmark domain already has visible nodes."""
    try:
        completed = subprocess.run(
            ["ros2", "node", "list", "--no-daemon", "--spin-time", "1.0"],
            env=env,
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            timeout=timeout_sec,
            check=False,
        )
    except subprocess.TimeoutExpired as error:
        raise ValueError("ROS domain cleanliness preflight timed out") from error
    if completed.returncode != 0:
        detail = (completed.stderr or completed.stdout).strip()[-500:]
        raise ValueError(f"ROS domain cleanliness preflight failed: {detail}")
    nodes = sorted({
        line.strip() for line in completed.stdout.splitlines()
        if line.strip().startswith("/")
    })
    if nodes:
        raise ValueError(
            "ROS domain is not clean before launch; visible nodes: "
            + ", ".join(nodes)
        )


def stop_launch(process):
    """Stop and reap the entire launch session, escalating only as needed."""
    def group_exists():
        try:
            os.killpg(process.pid, 0)
            return True
        except ProcessLookupError:
            return False
        except PermissionError:
            return True

    for group_signal, timeout_sec in (
        (signal.SIGINT, 8.0),
        (signal.SIGTERM, 3.0),
        (signal.SIGKILL, 3.0),
    ):
        if not group_exists():
            if process.poll() is None:
                process.wait(timeout=1.0)
            return
        try:
            os.killpg(process.pid, group_signal)
        except ProcessLookupError:
            if process.poll() is None:
                process.wait(timeout=1.0)
            return
        deadline = time.monotonic() + timeout_sec
        while group_exists() and time.monotonic() < deadline:
            if process.poll() is None:
                try:
                    process.wait(timeout=min(0.1, max(0.01, deadline - time.monotonic())))
                except subprocess.TimeoutExpired:
                    pass
            else:
                time.sleep(0.05)
        if not group_exists():
            if process.poll() is None:
                process.wait(timeout=1.0)
            return
    raise RuntimeError("failed to terminate and reap reachability launch process group")


def graph_contract_errors(graph, run, args, catalog):
    errors = []
    expected_fraction = args.gng_guard_fraction if run["method"] == "guarded_gng" else 0.0
    expected = {
        "reported_method": run["method"],
        **catalog["model"],
        "requested_node_count": args.sample_count,
        "nodes": args.sample_count,
        "halton_start_index": args.halton_start_index,
        "sample_stream_seed": run["stream_id"],
        "sample_stream_type": "digit_permuted_halton",
        "joint_names": catalog["joint_names"],
        "graph_publisher_count": 1,
        "plan_publisher_count": 1,
        "query_subscriber_count": 1,
    }
    for key, value in expected.items():
        if graph.get(key) != value:
            errors.append(f"{key}={graph.get(key)!r}, expected {value!r}")
    if graph.get("graph_revision", 0) < 1:
        errors.append("graph_revision must be positive")
    if not math.isclose(
        float(graph.get("effective_guard_fraction", math.nan)),
        expected_fraction,
        rel_tol=0.0,
        abs_tol=1.0e-12,
    ):
        errors.append("effective_guard_fraction mismatch")
    composition = sum(
        int(graph.get(key, -args.sample_count)) for key in (
            "anchor_node_count", "prototype_node_count",
            "guard_node_count", "fill_sample_node_count"
        )
    )
    if composition != args.sample_count:
        errors.append(f"node composition sums to {composition}")
    anchor_count = int(graph.get("anchor_node_count", -1))
    remaining_budget = args.sample_count - anchor_count
    if anchor_count != 2:
        errors.append(f"anchor_node_count={anchor_count}, expected 2")
    if remaining_budget < 0:
        errors.append("anchor count exceeds requested node budget")
        return errors

    if run["method"] == "gng":
        method_composition = {
            "prototype_budget": remaining_budget,
            "prototype_node_count": remaining_budget,
            "requested_guard_node_count": 0,
            "guard_node_count": 0,
            "fill_sample_node_count": 0,
        }
        if int(graph.get("gng_training_sample_count", 0)) < remaining_budget:
            errors.append("GNG training sample count is smaller than its prototype budget")
    elif run["method"] == "guarded_gng":
        requested_guards = min(
            int(math.floor(args.gng_guard_fraction * remaining_budget + 0.5)),
            max(0, remaining_budget - 2),
        )
        prototype_budget = remaining_budget - requested_guards
        method_composition = {
            "prototype_budget": prototype_budget,
            "prototype_node_count": prototype_budget,
            "requested_guard_node_count": requested_guards,
            "guard_node_count": requested_guards,
            "fill_sample_node_count": 0,
        }
        if int(graph.get("gng_training_sample_count", 0)) < remaining_budget:
            errors.append("guarded GNG training sample count is smaller than the node budget")
    else:
        method_composition = {
            "prototype_budget": 0,
            "prototype_node_count": 0,
            "requested_guard_node_count": 0,
            "guard_node_count": 0,
            "fill_sample_node_count": remaining_budget,
        }
        if int(graph.get("gng_training_sample_count", -1)) != 0:
            errors.append("Halton PRM unexpectedly reports GNG training samples")
    for key, expected_value in method_composition.items():
        if int(graph.get(key, -1)) != expected_value:
            errors.append(
                f"{key}={graph.get(key)!r}, expected {expected_value} for {run['method']}"
            )
    if int(graph.get("candidate_attempts", 0)) < remaining_budget:
        errors.append("candidate_attempts is smaller than the remaining node budget")
    if int(graph.get("components", 0)) < 1:
        errors.append("connected component count must be positive")
    if int(graph.get("edges", 0)) < 1:
        errors.append("validated edge count must be positive")
    if not math.isfinite(float(graph.get("build_time_ms", math.nan))):
        errors.append("build time must be finite")
    return errors


def query_contract_errors(
    row, scene, phase, graph_revision, joint_count, expected_method
):
    errors = []
    if row.get("plan_graph_method") != expected_method:
        errors.append("plan_graph_method_mismatch")
    if row.get("graph_revision") != graph_revision:
        errors.append("graph_revision_mismatch")
    if row.get("requested_target_environment_node_id") != scene["target"]["id"]:
        errors.append("target_id_echo_mismatch")
    requested_position = row.get("requested_target_position", [])
    if len(requested_position) != 3 or any(
        abs(actual - expected) > 1.0e-9
        for actual, expected in zip(requested_position, scene["target"]["position"])
    ):
        errors.append("target_position_echo_mismatch")
    preview_start = row.get("preview_start_joints", [])
    if len(preview_start) != joint_count or any(
        abs(actual - expected) > 1.0e-9
        for actual, expected in zip(preview_start, scene["start_joint_positions"])
    ):
        errors.append("start_state_echo_mismatch")
    if bool(row.get("valid")) != bool(row.get("exact_valid")):
        errors.append("valid_exact_valid_mismatch")
    if row.get("path_nodes") != len(row.get("path_ids", [])):
        errors.append("path_node_count_mismatch")
    if phase == "clear":
        if int(row.get("blocked_nodes", -1)) != 0:
            errors.append("clear_query_reported_blocked_nodes")
        # A clear query can legitimately reject roadmap edges during the exact
        # FCL validation pass and then reroute.  Those edges are retained in
        # the telemetry by design.  In an obstacle-free scene every blocked
        # edge must therefore be explained by exactly one exact replan.
        if int(row.get("blocked_edges", -1)) != int(row.get("exact_replans", -2)):
            errors.append("clear_blocked_edges_exact_replans_mismatch")

    if row.get("exact_valid"):
        if row.get("selected_target_environment_node_id") != scene["target"]["id"]:
            errors.append("selected_target_id_mismatch")
        if not row.get("path_ids"):
            errors.append("valid_plan_has_empty_path")
        if row.get("start_node_id") == UINT32_MAX or row.get("goal_node_id") == UINT32_MAX:
            errors.append("valid_plan_uses_invalid_node_sentinel")
        costs = [
            float(row.get("graph_cost", math.nan)),
            float(row.get("start_connection_cost", math.nan)),
            float(row.get("total_joint_path_cost", math.nan)),
            float(row.get("target_distance", math.nan)),
        ]
        if any(not math.isfinite(value) or value < 0.0 for value in costs):
            errors.append("valid_plan_has_invalid_cost_or_distance")
        elif not math.isclose(costs[0] + costs[1], costs[2], rel_tol=1.0e-9, abs_tol=1.0e-9):
            errors.append("total_joint_path_cost_mismatch")
    else:
        if row.get("selected_target_environment_node_id") != UINT32_MAX:
            errors.append("invalid_plan_did_not_use_target_sentinel")
        if row.get("goal_node_id") != UINT32_MAX:
            errors.append("invalid_plan_did_not_use_goal_sentinel")
        if row.get("path_ids"):
            errors.append("invalid_plan_has_nonempty_path")
    for key in ("planning_time_ms", "publish_to_plan_ms", "exact_time_ms"):
        value = float(row.get(key, math.nan))
        if not math.isfinite(value) or value < 0.0:
            errors.append(f"{key}_invalid")
    return errors


def encode_csv_value(value):
    if isinstance(value, (list, dict)):
        return json.dumps(value, sort_keys=True, separators=(",", ":"))
    return value


def append_rows(path, rows, fieldnames):
    exists = path.exists() and path.stat().st_size > 0
    with path.open("a", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fieldnames, extrasaction="ignore")
        if not exists:
            writer.writeheader()
        for row in rows:
            writer.writerow({key: encode_csv_value(row.get(key, "")) for key in fieldnames})
        stream.flush()
        os.fsync(stream.fileno())


def atomic_write_json(path, payload):
    temporary = path.with_name(path.name + ".tmp")
    with temporary.open("w", encoding="utf-8") as stream:
        json.dump(payload, stream, sort_keys=True, indent=2)
        stream.write("\n")
        stream.flush()
        os.fsync(stream.fileno())
    os.replace(temporary, path)


GRAPH_FIELDS = [
    "run_index", "stream_ordinal", "roadmap_stream_id", "method",
    "method_order_position", "ros_domain_id", "reported_method", "graph_revision",
    "expanded_urdf_sha256", "srdf_sha256", "reachability_parameters_sha256",
    "requested_node_count", "anchor_node_count", "prototype_budget",
    "prototype_node_count", "requested_guard_node_count", "guard_node_count",
    "fill_sample_node_count", "candidate_attempts", "halton_start_index",
    "sample_stream_seed", "sample_stream_type", "gng_training_sample_count",
    "effective_guard_fraction", "nodes", "edges", "components", "build_time_ms",
    "joint_names", "graph_publisher_count", "plan_publisher_count",
    "query_subscriber_count", "catalog_id", "catalog_sha256", "rmw_implementation",
    "ros_localhost_only", "launch_log", "infrastructure_error",
]

QUERY_FIELDS = [
    "run_index", "stream_ordinal", "roadmap_stream_id", "method",
    "method_order_position", "ros_domain_id", "plan_graph_method",
    "graph_revision", "scene_id",
    "catalog_index", "base_trajectory_id", "source_counter",
    "scene_order_position", "phase", "obstacle_kind", "stratum",
    "phase_order_position",
    "query_id", "start_joint_positions", "target_position",
    "target_source_joint_positions", "requested_target_environment_node_id",
    "requested_target_position", "selected_target_environment_node_id", "valid",
    "exact_valid", "reason", "timeout", "infrastructure_error", "blocked_nodes",
    "blocked_edges", "planning_time_ms", "publish_to_plan_ms", "exact_checks",
    "exact_replans", "exact_time_ms", "start_node_id", "goal_node_id",
    "target_distance", "graph_cost", "start_connection_cost", "total_joint_path_cost",
    "path_nodes", "path_ids", "preview_start_joints",
]


def validate_resume_rows(graph_rows, query_rows, schedule, args, catalog, catalog_sha256):
    scheduled = {
        (run["stream_ordinal"], run["method"]): run for run in schedule
    }
    graph_by_key = {}
    query_rows_by_key = {}
    for row in query_rows:
        key = (int(row["stream_ordinal"]), row["method"])
        if key not in scheduled:
            raise ValueError(f"resume query row has unknown run key {key}")
        query_rows_by_key.setdefault(key, []).append(row)
    for row in graph_rows:
        key = (int(row["stream_ordinal"]), row["method"])
        if key not in scheduled:
            raise ValueError(f"resume graph row has unknown run key {key}")
        if key in graph_by_key:
            raise ValueError(f"resume graph CSV contains duplicate run key {key}")
        graph_by_key[key] = row
    query_ids = [int(row["query_id"]) for row in query_rows]
    if len(query_ids) != len(set(query_ids)):
        raise ValueError("resume query CSV contains duplicate query IDs")

    completed = set()
    for key, run in scheduled.items():
        row = graph_by_key.get(key)
        rows = query_rows_by_key.get(key, [])
        if row is None and not rows:
            continue
        if row is None or not rows:
            raise ValueError(
                f"resume found a partial run {key}; use a new output directory"
            )
        expected_ids = {
            query_id
            for phase_ids in run["query_ids"].values()
            for query_id in phase_ids.values()
        }
        actual_ids = {int(query_row["query_id"]) for query_row in rows}
        graph_clean = (
            not row.get("infrastructure_error")
            and row.get("catalog_sha256") == catalog_sha256
            and row.get("catalog_id") == catalog["catalog_id"]
            and row.get("reported_method") == run["method"]
            and int(row.get("roadmap_stream_id", -1)) == run["stream_id"]
            and int(row.get("requested_node_count", -1)) == args.sample_count
            and int(row.get("halton_start_index", -1)) == args.halton_start_index
            and int(row.get("sample_stream_seed", -1)) == run["stream_id"]
            and all(row.get(key) == value for key, value in catalog["model"].items())
            and int(row.get("graph_publisher_count", -1)) == 1
            and int(row.get("plan_publisher_count", -1)) == 1
            and int(row.get("query_subscriber_count", -1)) == 1
        )
        queries_clean = (
            actual_ids == expected_ids
            and len(rows) == len(expected_ids)
            and all(
                not query_row.get("infrastructure_error")
                and query_row.get("timeout") == "False"
                and query_row.get("plan_graph_method") == run["method"]
                for query_row in rows
            )
        )
        if not graph_clean or not queries_clean:
            raise ValueError(
                f"resume found a corrupt or failed run {key}; use a new output directory"
            )
        completed.add(key)
    return completed


def run_controller(args):
    catalog, catalog_sha256 = load_catalog(args.catalog, args.max_scenes)
    frozen_input_sources = resolve_frozen_input_sources(args)
    frozen_input_source_hashes = {
        name: file_sha256(path) for name, path in frozen_input_sources.items()
    }
    node_binary = reachability_binary_path()
    if not node_binary:
        raise ValueError("could not resolve the installed reachability_graph_node binary")
    node_binary_sha256 = file_sha256(node_binary)
    validate_node_binary_provenance(catalog, node_binary_sha256)
    methods = validate_methods(args.methods)
    stream_ids = parse_unique_nonnegative_ints(args.stream_list, "--stream-list")
    if any(stream_id == 0 for stream_id in stream_ids):
        raise ValueError("multi-scene roadmap stream IDs must be positive")
    validate_protocol(args, catalog, catalog_sha256, methods, stream_ids)
    schedule = make_schedule(stream_ids, methods, len(catalog["scenes"]))
    last_domain = args.domain_base + args.domain_pool_size - 1
    if args.domain_base < 20 or args.domain_pool_size < 1 or last_domain > 99:
        raise ValueError("ROS domain pool must stay within 20..99")
    expected_queries = len(schedule) * len(catalog["scenes"]) * 2
    dry_run = {
        "catalog_id": catalog["catalog_id"],
        "catalog_sha256": catalog_sha256,
        "protocol_id": args.protocol_id,
        "streams": stream_ids,
        "methods": methods,
        "scene_count": len(catalog["scenes"]),
        "base_trajectory_count": len({
            scene["base_trajectory_id"] for scene in catalog["scenes"]
        }),
        "graph_builds": len(schedule),
        "query_rows": expected_queries,
        "domain_range": [args.domain_base, last_domain],
        "reachability_node_binary_sha256": node_binary_sha256,
        "frozen_input_sha256": frozen_input_source_hashes,
    }
    if args.dry_run:
        print(json.dumps(dry_run, sort_keys=True, indent=2))
        return 0

    script = str(Path(__file__).resolve())
    run_config = {
        "schema": "om6dof-reachability-multiscene-config-v2",
        **dry_run,
        "sample_count": args.sample_count,
        "halton_start_index": args.halton_start_index,
        "guarded_fraction": args.gng_guard_fraction,
        "graph_timeout_sec": args.graph_timeout,
        "phase_timeout_sec": args.phase_timeout,
        "rmw_implementation": args.rmw_implementation,
        "ros_localhost_only": True,
        "phase_order_design": (
            "paired_across_methods_counterbalanced_by_stream_ordinal_plus_catalog_index"
        ),
        "source_tree_sha256": args.source_tree_sha256,
        "expected_catalog_sha256": args.expected_catalog_sha256,
        "runner_script_sha256": file_sha256(script),
        "reachability_node_binary_sha256": node_binary_sha256,
        "catalog_model": catalog["model"],
        "catalog_generator_implementation": catalog["generator"]["implementation"],
        "graph_fields": GRAPH_FIELDS,
        "query_fields": QUERY_FIELDS,
    }
    output_dir = Path(args.output_dir).expanduser().resolve()
    if output_dir.exists() and any(output_dir.iterdir()) and not args.resume:
        raise ValueError(f"output directory is not empty: {output_dir}; pass --resume")
    output_dir.mkdir(parents=True, exist_ok=True)
    frozen_input_hashes = freeze_input_bundle(
        output_dir, frozen_input_sources, args.resume
    )
    run_config_path = output_dir / "run_config.json"
    if args.resume:
        if not run_config_path.is_file():
            raise ValueError("resume requires an existing run_config.json")
        existing_config = strict_json_load(run_config_path)
        if existing_config != run_config:
            raise ValueError("resume configuration/provenance does not match this invocation")
    else:
        atomic_write_json(run_config_path, run_config)
    log_dir = output_dir / "logs"
    log_dir.mkdir(exist_ok=True)
    graph_csv = output_dir / "graphs.csv"
    query_csv = output_dir / "queries.csv"
    completed = set()
    if args.resume and (graph_csv.exists() != query_csv.exists()):
        raise ValueError("resume found only one of graphs.csv and queries.csv")
    if args.resume and graph_csv.exists():
        with graph_csv.open(newline="", encoding="utf-8") as stream:
            graph_rows = list(csv.DictReader(stream))
        with query_csv.open(newline="", encoding="utf-8") as stream:
            query_rows = list(csv.DictReader(stream))
        completed = validate_resume_rows(
            graph_rows, query_rows, schedule, args, catalog, catalog_sha256
        )

    started_utc = datetime.now(timezone.utc).isoformat()
    had_infrastructure_error = False
    for run_index, run in enumerate(schedule):
        run_key = (run["stream_ordinal"], run["method"])
        if run_key in completed:
            continue
        domain = args.domain_base + (run_index % args.domain_pool_size)
        env = os.environ.copy()
        env["ROS_DOMAIN_ID"] = str(domain)
        env["ROS_LOCALHOST_ONLY"] = "1"
        env["RMW_IMPLEMENTATION"] = args.rmw_implementation
        assert_clean_ros_domain(env)
        launch_log_path = log_dir / (
            f"run_{run_index:04d}_stream_{run['stream_id']}_{run['method']}_domain_{domain}.log"
        )
        with launch_log_path.open("w", encoding="utf-8") as launch_log:
            launch = subprocess.Popen(
                [
                    "ros2", "launch", "om6dof_dd_gng", "reachability_graph.launch.py",
                    "launch_rviz:=false", "query_mode:=true",
                    f"graph_method:={run['method']}",
                    f"sample_count:={args.sample_count}",
                    f"halton_start_index:={args.halton_start_index}",
                    f"sample_stream_seed:={run['stream_id']}",
                    f"gng_guard_fraction:={args.gng_guard_fraction}",
                ],
                env=env,
                stdout=launch_log,
                stderr=subprocess.STDOUT,
                start_new_session=True,
            )
            try:
                command = [
                    sys.executable, script, "--probe", "--catalog", str(Path(args.catalog).resolve()),
                    "--methods", args.methods, "--stream-list", args.stream_list,
                    "--stream-ordinal", str(run["stream_ordinal"]),
                    "--method", run["method"], "--graph-timeout", str(args.graph_timeout),
                    "--phase-timeout", str(args.phase_timeout),
                ]
                if args.max_scenes:
                    command.extend(("--max-scenes", str(args.max_scenes)))
                completed_probe = subprocess.run(
                    command,
                    env=env,
                    text=True,
                    stdout=subprocess.PIPE,
                    stderr=subprocess.STDOUT,
                    timeout=args.graph_timeout + len(catalog["scenes"]) * 2 * args.phase_timeout + 15,
                    check=False,
                )
                lines = [line for line in completed_probe.stdout.splitlines() if line.startswith("{")]
                result = json.loads(lines[-1]) if lines else {
                    "error": f"probe_exit_{completed_probe.returncode}: {completed_probe.stdout[-500:]}"
                }
                if completed_probe.returncode != 0 and not result.get("error"):
                    result["error"] = f"probe_exit_{completed_probe.returncode}"
            except subprocess.TimeoutExpired:
                result = {"error": "probe_process_timeout"}
            finally:
                stop_launch(launch)

        common = {
            "run_index": run_index,
            "stream_ordinal": run["stream_ordinal"],
            "roadmap_stream_id": run["stream_id"],
            "method": run["method"],
            "method_order_position": run["method_order_position"],
            "ros_domain_id": domain,
        }
        graph = result.get("graph", {})
        errors = [result["error"]] if result.get("error") else []
        if result.get("catalog_sha256") not in (None, catalog_sha256):
            errors.append("probe_catalog_sha256_mismatch")
        if graph:
            errors.extend(graph_contract_errors(graph, run, args, catalog))
        elif not errors:
            errors.append("probe_missing_graph")
        query_rows = [
            {**common, **row} for row in result.get("queries", [])
        ]
        if len(query_rows) != len(catalog["scenes"]) * 2:
            errors.append(
                f"unexpected_query_row_count={len(query_rows)}, "
                f"expected {len(catalog['scenes']) * 2}"
            )
        graph_row = {
            **common,
            **graph,
            "catalog_id": catalog["catalog_id"],
            "catalog_sha256": catalog_sha256,
            "rmw_implementation": args.rmw_implementation,
            "ros_localhost_only": True,
            "launch_log": str(launch_log_path),
            "infrastructure_error": "; ".join(errors),
        }
        append_rows(graph_csv, [graph_row], GRAPH_FIELDS)
        if query_rows:
            append_rows(query_csv, query_rows, QUERY_FIELDS)
        had_infrastructure_error = had_infrastructure_error or bool(errors) or any(
            bool(row.get("infrastructure_error")) for row in query_rows
        )
        print(json.dumps(graph_row, sort_keys=True), flush=True)

    manifest = {
        **dry_run,
        "schema": "om6dof-reachability-multiscene-run-v2",
        "started_utc": started_utc,
        "finished_utc": datetime.now(timezone.utc).isoformat(),
        "sample_count": args.sample_count,
        "halton_start_index": args.halton_start_index,
        "guarded_fraction": args.gng_guard_fraction,
        "phase_timeout_sec": args.phase_timeout,
        "graph_timeout_sec": args.graph_timeout,
        "rmw_implementation": args.rmw_implementation,
        "ros_localhost_only": True,
        "preview_only": True,
        "controller_topics_published": [],
        "protocol_id": args.protocol_id,
        "resume_used": args.resume,
        "precompleted_graph_builds": len(completed),
        "phase_order_design": (
            "paired_across_methods_counterbalanced_by_stream_ordinal_plus_catalog_index"
        ),
        "catalog_model": catalog["model"],
        "catalog_generator_implementation": catalog["generator"]["implementation"],
        "implementation": {
            "runner_script_sha256": file_sha256(Path(__file__).resolve()),
            "reachability_node_binary_sha256": node_binary_sha256,
            "source_tree_sha256": args.source_tree_sha256,
            "git": git_provenance(Path(__file__).resolve()),
        },
        "runtime": {
            "command": [sys.executable, *sys.argv],
            "python": sys.version,
            "platform": platform.platform(),
            "machine": platform.machine(),
            "hostname": platform.node(),
            "cpu_count": os.cpu_count(),
            "load_average_at_finish": (
                list(os.getloadavg()) if hasattr(os, "getloadavg") else []
            ),
            "ros_distro": os.environ.get("ROS_DISTRO", ""),
        },
        "artifacts": {
            "run_config.json": file_sha256(run_config_path),
            "graphs.csv": file_sha256(graph_csv),
            "queries.csv": file_sha256(query_csv),
            "logs": {
                path.name: file_sha256(path)
                for path in sorted(log_dir.glob("run_*.log"))
            },
            "frozen_inputs": frozen_input_hashes,
        },
    }
    manifest_path = output_dir / "manifest.json"
    manifest_path.write_text(json.dumps(manifest, sort_keys=True, indent=2) + "\n", encoding="utf-8")
    return 1 if had_infrastructure_error else 0


def positive_finite(parser, name, value):
    if not math.isfinite(value) or value <= 0.0:
        parser.error(f"{name} must be a positive finite value")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--probe", action="store_true", help=argparse.SUPPRESS)
    parser.add_argument("--catalog", required=True)
    parser.add_argument("--methods", default=",".join(SUPPORTED_METHODS))
    parser.add_argument(
        "--protocol-id", choices=("development", CONFIRMATORY_PROTOCOL_ID),
        default="development",
    )
    parser.add_argument("--stream-list", default="9000,9001,9002,9003,9004,9005")
    parser.add_argument("--stream-ordinal", type=int, default=-1, help=argparse.SUPPRESS)
    parser.add_argument("--method", default="", help=argparse.SUPPRESS)
    parser.add_argument("--max-scenes", type=int, default=0)
    parser.add_argument("--sample-count", type=int, default=800)
    parser.add_argument("--halton-start-index", type=int, default=17)
    parser.add_argument("--gng-guard-fraction", type=float, default=0.75)
    parser.add_argument("--graph-timeout", type=float, default=60.0)
    parser.add_argument("--phase-timeout", type=float, default=5.0)
    parser.add_argument("--domain-base", type=int, default=20)
    parser.add_argument("--domain-pool-size", type=int, default=80)
    parser.add_argument("--rmw-implementation", default="rmw_fastrtps_cpp")
    parser.add_argument("--source-tree-sha256", default="")
    parser.add_argument("--expected-catalog-sha256", default="")
    parser.add_argument("--catalog-generation-log", default="")
    parser.add_argument("--source-snapshot", default="")
    parser.add_argument("--protocol-document", default="")
    parser.add_argument("--analyzer-script", default="")
    parser.add_argument("--output-dir", default="reachability_multiscene_results")
    parser.add_argument("--resume", action="store_true")
    parser.add_argument("--dry-run", action="store_true")
    args = parser.parse_args()
    positive_finite(parser, "--graph-timeout", args.graph_timeout)
    positive_finite(parser, "--phase-timeout", args.phase_timeout)
    if args.source_tree_sha256 and (
        len(args.source_tree_sha256) != 64
        or any(
            character not in "0123456789abcdefABCDEF"
            for character in args.source_tree_sha256
        )
    ):
        parser.error("--source-tree-sha256 must be empty or a SHA-256 hex digest")
    if args.expected_catalog_sha256 and (
        len(args.expected_catalog_sha256) != 64
        or any(
            character not in "0123456789abcdefABCDEF"
            for character in args.expected_catalog_sha256
        )
    ):
        parser.error("--expected-catalog-sha256 must be empty or a SHA-256 hex digest")
    if args.sample_count < 2 or args.halton_start_index < 0:
        parser.error("--sample-count must be >=2 and --halton-start-index must be >=0")
    if not 0.0 <= args.gng_guard_fraction <= 0.90:
        parser.error("--gng-guard-fraction must be within [0, 0.90]")
    if args.max_scenes < 0:
        parser.error("--max-scenes must be non-negative")
    if args.probe:
        if args.stream_ordinal < 0 or not args.method:
            parser.error("probe mode requires --stream-ordinal and --method")
        return run_probe(args)
    try:
        return run_controller(args)
    except ValueError as error:
        parser.error(str(error))


if __name__ == "__main__":
    sys.exit(main())
