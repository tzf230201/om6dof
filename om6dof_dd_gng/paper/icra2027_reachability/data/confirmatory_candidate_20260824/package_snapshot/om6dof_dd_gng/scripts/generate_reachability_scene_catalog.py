#!/usr/bin/env python3
"""Generate a deterministic, roadmap-independent reachability scene catalog.

Candidate joint states come from a SHA-256 counter stream. A preview-only
MoveIt/FCL oracle accepts a scene only when the direct path is clear without
the obstacle, blocked by both capsule and exact collision checks afterward,
and a separately sampled two-leg detour remains valid. No controller interface
is created or used.
"""

import argparse
import hashlib
import json
import math
import os
import shutil
import signal
import subprocess
import sys
import time
from pathlib import Path


NAMESPACE = b"om6dof-reachability-scene-v1"
DEFAULT_MASTER_KEY_HEX = "600025f316f133ef34d1baf8bb9107b3aed500e069247baa4ebb5d6af45ad92f"
DIFFICULTIES = ("low", "medium", "high")
OBSTACLE_KINDS = ("point", "segment")


def file_sha256(path):
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


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


def runtime_model_hashes(graph):
    return {
        "expanded_urdf_sha256": graph.expanded_urdf_sha256,
        "srdf_sha256": graph.srdf_sha256,
        "reachability_parameters_sha256": graph.reachability_parameters_sha256,
    }


def expected_model_hashes(args):
    return {
        "expanded_urdf_sha256": args.urdf_sha256.lower(),
        "srdf_sha256": args.srdf_sha256.lower(),
        "reachability_parameters_sha256": args.parameters_sha256.lower(),
    }


def counter_u53(master_key, tag, counter, component):
    if not isinstance(master_key, bytes) or len(master_key) != 32:
        raise ValueError("master key must contain exactly 32 bytes")
    if counter < 0 or component < 0:
        raise ValueError("counter and component must be non-negative")
    digest = hashlib.sha256(
        NAMESPACE
        + b"\0"
        + master_key
        + b"\0"
        + tag.encode("ascii")
        + b"\0"
        + counter.to_bytes(8, "big")
        + component.to_bytes(4, "big")
    ).digest()
    mantissa = int.from_bytes(digest[:8], "big") >> 11
    return (mantissa + 0.5) / 2**53


def sample_joints(master_key, tag, counter, lower, upper, margin_fraction, component_offset=0):
    if len(lower) != len(upper) or not lower:
        raise ValueError("joint bounds are inconsistent")
    joints = []
    for dimension, (minimum, maximum) in enumerate(zip(lower, upper)):
        if not math.isfinite(minimum) or not math.isfinite(maximum) or maximum <= minimum:
            raise ValueError("joint bounds must be finite and increasing")
        span = maximum - minimum
        interior_minimum = minimum + margin_fraction * span
        interior_span = span * (1.0 - 2.0 * margin_fraction)
        unit = counter_u53(master_key, tag, counter, component_offset + dimension)
        joints.append(interior_minimum + unit * interior_span)
    return joints


def distance(a, b):
    return math.sqrt(sum((x - y) ** 2 for x, y in zip(a, b)))


def point_segment_distance(point, start, end):
    delta = [b - a for a, b in zip(start, end)]
    length_squared = sum(value * value for value in delta)
    if length_squared <= 1.0e-16:
        return distance(point, start)
    fraction = max(
        0.0,
        min(1.0, sum((p - a) * d for p, a, d in zip(point, start, delta)) / length_squared),
    )
    closest = [a + fraction * d for a, d in zip(start, delta)]
    return distance(point, closest)


def normalized_joint_distance(start, target, lower, upper):
    return math.sqrt(sum(
        ((a - b) / (maximum - minimum)) ** 2
        for a, b, minimum, maximum in zip(start, target, lower, upper)
    ))


def segment_endpoints(hit, start_position, target_position, half_length):
    direct = [b - a for a, b in zip(start_position, target_position)]
    perpendicular = [-direct[1], direct[0], 0.0]
    norm = math.sqrt(sum(value * value for value in perpendicular))
    if norm < 1.0e-9:
        perpendicular = [0.0, 1.0, 0.0]
        norm = 1.0
    direction = [value / norm for value in perpendicular]
    return (
        [value - half_length * axis for value, axis in zip(hit, direction)],
        [value + half_length * axis for value, axis in zip(hit, direction)],
    )


def finalize_scenes(candidates_by_kind, total_scene_count, joint_names, lower, upper):
    if total_scene_count < 6 or total_scene_count % 6:
        raise ValueError("scene count must be a positive multiple of six")
    per_kind = total_scene_count // 2
    per_cell = total_scene_count // 6
    stratified = {}
    ordered_sources = {}
    for kind in OBSTACLE_KINDS:
        candidates = list(candidates_by_kind.get(kind, []))
        if len(candidates) != per_kind:
            raise ValueError(f"expected {per_kind} accepted {kind} scenes")
        candidates.sort(key=lambda scene: (
            scene["joint_distance_normalized"], scene["source_counter"]
        ))
        ordered_sources[kind] = [scene["source_counter"] for scene in candidates]
        for difficulty_index, difficulty in enumerate(DIFFICULTIES):
            start = difficulty_index * per_cell
            stratified[(difficulty, kind)] = candidates[start:start + per_cell]
    if len({tuple(values) for values in ordered_sources.values()}) != 1:
        raise ValueError("point and segment candidates must share identical base trajectories")
    point_candidates = sorted(
        candidates_by_kind["point"],
        key=lambda scene: (scene["joint_distance_normalized"], scene["source_counter"]),
    )
    segment_candidates = sorted(
        candidates_by_kind["segment"],
        key=lambda scene: (scene["joint_distance_normalized"], scene["source_counter"]),
    )
    for point, segment in zip(point_candidates, segment_candidates):
        if (
            point["base_trajectory_id"] != segment["base_trajectory_id"]
            or point["start_joint_positions"] != segment["start_joint_positions"]
            or point["target_joint_positions"] != segment["target_joint_positions"]
            or abs(point["hit_fraction"] - segment["hit_fraction"]) > 1.0e-12
            or distance(
                point["hit_ee_pose"]["position"], segment["hit_ee_pose"]["position"]
            ) > 1.0e-9
        ):
            raise ValueError("point and segment variants are not kinematically paired")

    scenes = []
    for difficulty in DIFFICULTIES:
        for kind in OBSTACLE_KINDS:
            for candidate in stratified[(difficulty, kind)]:
                index = len(scenes)
                target_id = 1_000_000 + 10 * index
                obstacle_nodes = []
                for node_index, position in enumerate(candidate["obstacle_positions"], start=1):
                    obstacle_nodes.append({
                        "id": target_id + node_index,
                        "class_id": -1,
                        "confidence": 1.0,
                        "position": position,
                    })
                edges = []
                if kind == "segment":
                    edges.append({
                        "source_id": obstacle_nodes[0]["id"],
                        "target_id": obstacle_nodes[1]["id"],
                        "cost": distance(
                            obstacle_nodes[0]["position"], obstacle_nodes[1]["position"]
                        ),
                    })
                scenes.append({
                    "scene_id": f"scene_{index:03d}",
                    "catalog_index": index,
                    "source_counter": candidate["source_counter"],
                    "base_trajectory_id": candidate["base_trajectory_id"],
                    "detour_attempt": candidate["detour_attempt"],
                    "stratum": {
                        "difficulty": difficulty,
                        "obstacle_kind": kind,
                    },
                    "start_joint_positions": candidate["start_joint_positions"],
                    "start_ee_pose": candidate["start_ee_pose"],
                    "target": {
                        "id": target_id,
                        "class_id": 1,
                        "confidence": 1.0,
                        "position": candidate["target_ee_pose"]["position"],
                        "source_joint_positions": candidate["target_joint_positions"],
                        "source_pose": candidate["target_ee_pose"],
                    },
                    "dynamic": {
                        "kind": kind,
                        "nodes": obstacle_nodes,
                        "edges": edges,
                    },
                    "oracle": {
                        "hit_fraction": candidate["hit_fraction"],
                        "hit_ee_pose": candidate["hit_ee_pose"],
                        "detour_joint_positions": candidate["detour_joint_positions"],
                        "joint_distance_normalized": candidate["joint_distance_normalized"],
                        **candidate["oracle_evidence"],
                        "target_obstacle_distance_m": candidate[
                            "target_obstacle_distance_m"
                        ],
                    },
                })
    if len(scenes) != total_scene_count:
        raise RuntimeError("final scene count is inconsistent")
    if any(len(scene["start_joint_positions"]) != len(joint_names) for scene in scenes):
        raise RuntimeError("scene joint dimensionality is inconsistent")
    return scenes


def pose_dict(pose):
    return {
        "position": [float(pose.position.x), float(pose.position.y), float(pose.position.z)],
        "orientation_xyzw": [
            float(pose.orientation.x), float(pose.orientation.y),
            float(pose.orientation.z), float(pose.orientation.w),
        ],
    }


def run_probe(args):
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from om6dof_dd_gng.msg import EnvironmentGraph, EnvironmentNode, ReachabilityGraph, TopologyEdge
    from om6dof_dd_gng.srv import ValidateReachabilityScene

    master_key = bytes.fromhex(args.master_key_hex)
    if len(master_key) != 32:
        raise ValueError("--master-key-hex must encode 32 bytes")
    per_kind = args.scene_count // 2
    latched = QoSProfile(depth=1)
    latched.reliability = ReliabilityPolicy.RELIABLE
    latched.durability = DurabilityPolicy.TRANSIENT_LOCAL

    class GeneratorProbe(Node):
        def __init__(self):
            super().__init__("reachability_scene_generator_probe")
            self.graph = None
            self.create_subscription(
                ReachabilityGraph,
                "/om6dof_topo_gng/reachability_graph_data",
                self.graph_callback,
                latched,
            )
            self.client = self.create_client(
                ValidateReachabilityScene,
                "/om6dof_topo_gng/validate_reachability_scene",
            )

        def graph_callback(self, message):
            if message.nodes:
                self.graph = message

        def environment(self, kind, positions):
            environment = EnvironmentGraph()
            environment.header.frame_id = "world"
            for index, position in enumerate(positions, start=1):
                node = EnvironmentNode()
                node.id = index
                node.class_id = -1
                node.confidence = 1.0
                node.position.x, node.position.y, node.position.z = position
                environment.nodes.append(node)
            if kind == "segment":
                edge = TopologyEdge()
                edge.source_id = 1
                edge.target_id = 2
                edge.cost = distance(positions[0], positions[1])
                environment.edges.append(edge)
            return environment

        def evaluate(self, start, target, detour, hit_fraction, environment):
            request = ValidateReachabilityScene.Request()
            request.joint_names = list(self.graph.joint_names)
            request.start_joint_positions = list(start)
            request.target_joint_positions = list(target)
            request.detour_joint_positions = list(detour)
            request.hit_fraction = hit_fraction
            request.environment = environment
            future = self.client.call_async(request)
            deadline = time.monotonic() + args.service_timeout
            while rclpy.ok() and not future.done() and time.monotonic() < deadline:
                rclpy.spin_once(self, timeout_sec=0.02)
            if not future.done():
                raise RuntimeError("scene validation service timeout")
            response = future.result()
            if response is None:
                raise RuntimeError("scene validation service returned no response")
            return response

    rclpy.init()
    probe = GeneratorProbe()
    deadline = time.monotonic() + args.graph_timeout
    try:
        while rclpy.ok() and probe.graph is None and time.monotonic() < deadline:
            rclpy.spin_once(probe, timeout_sec=0.05)
        if probe.graph is None:
            print(json.dumps({"error": "graph_timeout"}), flush=True)
            return 2
        actual_model_hashes = runtime_model_hashes(probe.graph)
        expected_hashes = expected_model_hashes(args)
        if actual_model_hashes != expected_hashes:
            print(json.dumps({
                "error": "runtime_model_provenance_mismatch",
                "expected": expected_hashes,
                "actual": actual_model_hashes,
            }, sort_keys=True), flush=True)
            return 5
        graph_publisher_count = probe.count_publishers(
            "/om6dof_topo_gng/reachability_graph_data"
        )
        if graph_publisher_count != 1:
            print(json.dumps({
                "error": "reachability_graph_publisher_count_mismatch",
                "publisher_count": graph_publisher_count,
            }, sort_keys=True), flush=True)
            return 6
        if not probe.client.wait_for_service(timeout_sec=min(10.0, args.graph_timeout)):
            print(json.dumps({"error": "scene_validation_service_timeout"}), flush=True)
            return 3

        joint_names = list(probe.graph.joint_names)
        lower = list(probe.graph.joint_lower_bounds)
        upper = list(probe.graph.joint_upper_bounds)
        if len(joint_names) != len(lower) or len(lower) != len(upper):
            raise RuntimeError("graph did not publish consistent joint bounds")
        accepted = {kind: [] for kind in OBSTACLE_KINDS}
        counters_examined = 0
        for source_counter in range(args.max_source_counters):
            if len(accepted[OBSTACLE_KINDS[0]]) >= per_kind:
                break
            counters_examined += 1
            start = sample_joints(
                master_key, "start", source_counter, lower, upper, args.joint_margin_fraction
            )
            target = sample_joints(
                master_key, "target", source_counter, lower, upper, args.joint_margin_fraction
            )
            hit_fraction = 0.35 + 0.30 * counter_u53(master_key, "hit", source_counter, 0)
            pair_candidates = {}
            source_base_invalid = False
            for kind in OBSTACLE_KINDS:
                for detour_attempt in range(args.max_detour_attempts):
                    detour = sample_joints(
                        master_key,
                        "detour",
                        source_counter,
                        lower,
                        upper,
                        args.joint_margin_fraction,
                        component_offset=detour_attempt * len(joint_names),
                    )
                    empty = probe.environment("point", [])
                    clear = probe.evaluate(start, target, detour, hit_fraction, empty)
                    if not clear.evaluated:
                        raise RuntimeError(f"oracle rejected request: {clear.reason}")
                    base_valid = (
                        clear.start_self_valid
                        and clear.target_self_valid
                        and clear.clear_capsule_direct_valid
                        and clear.clear_exact_direct_valid
                    )
                    if not base_valid:
                        source_base_invalid = True
                        break
                    if not clear.detour_self_valid:
                        continue
                    start_pose = pose_dict(clear.start_pose)
                    target_pose = pose_dict(clear.target_pose)
                    hit_pose = pose_dict(clear.hit_pose)
                    hit_position = hit_pose["position"]
                    if kind == "point":
                        obstacle_positions = [hit_position]
                        target_obstacle_distance = distance(
                            target_pose["position"], hit_position
                        )
                    else:
                        endpoints = segment_endpoints(
                            hit_position,
                            start_pose["position"],
                            target_pose["position"],
                            args.segment_half_length,
                        )
                        obstacle_positions = [list(endpoints[0]), list(endpoints[1])]
                        target_obstacle_distance = point_segment_distance(
                            target_pose["position"], *obstacle_positions
                        )
                    if target_obstacle_distance < args.minimum_target_obstacle_distance:
                        break
                    dynamic_environment = probe.environment(kind, obstacle_positions)
                    dynamic = probe.evaluate(
                        start, target, detour, hit_fraction, dynamic_environment
                    )
                    if not dynamic.evaluated:
                        raise RuntimeError(f"oracle rejected dynamic scene: {dynamic.reason}")
                    if not (
                        dynamic.dynamic_start_valid
                        and dynamic.dynamic_target_valid
                        and dynamic.dynamic_detour_state_valid
                        and not dynamic.dynamic_capsule_direct_valid
                        and not dynamic.dynamic_exact_direct_valid
                        and dynamic.dynamic_capsule_detour_valid
                        and dynamic.dynamic_exact_detour_valid
                    ):
                        continue
                    pair_candidates[kind] = {
                        "source_counter": source_counter,
                        "base_trajectory_id": f"base_{source_counter:06d}",
                        "detour_attempt": detour_attempt,
                        "start_joint_positions": start,
                        "target_joint_positions": target,
                        "detour_joint_positions": detour,
                        "start_ee_pose": start_pose,
                        "target_ee_pose": target_pose,
                        "hit_ee_pose": hit_pose,
                        "hit_fraction": hit_fraction,
                        "obstacle_positions": obstacle_positions,
                        "target_obstacle_distance_m": target_obstacle_distance,
                        "joint_distance_normalized": normalized_joint_distance(
                            start, target, lower, upper
                        ),
                        "oracle_evidence": {
                            "clear_evaluated": bool(clear.evaluated),
                            "clear_reason": clear.reason,
                            "clear_start_self_valid": bool(clear.start_self_valid),
                            "clear_target_self_valid": bool(clear.target_self_valid),
                            "clear_detour_self_valid": bool(clear.detour_self_valid),
                            "clear_capsule_direct_valid": bool(
                                clear.clear_capsule_direct_valid
                            ),
                            "clear_exact_direct_valid": bool(
                                clear.clear_exact_direct_valid
                            ),
                            "dynamic_evaluated": bool(dynamic.evaluated),
                            "dynamic_reason": dynamic.reason,
                            "dynamic_capsule_direct_blocked": not bool(
                                dynamic.dynamic_capsule_direct_valid
                            ),
                            "dynamic_exact_direct_blocked": not bool(
                                dynamic.dynamic_exact_direct_valid
                            ),
                            "dynamic_capsule_detour_valid": bool(
                                dynamic.dynamic_capsule_detour_valid
                            ),
                            "dynamic_exact_detour_valid": bool(
                                dynamic.dynamic_exact_detour_valid
                            ),
                            "dynamic_start_valid": bool(dynamic.dynamic_start_valid),
                            "dynamic_target_valid": bool(dynamic.dynamic_target_valid),
                            "dynamic_detour_state_valid": bool(
                                dynamic.dynamic_detour_state_valid
                            ),
                        },
                    }
                    break
                if source_base_invalid or kind not in pair_candidates:
                    break
            if (
                not source_base_invalid
                and all(kind in pair_candidates for kind in OBSTACLE_KINDS)
            ):
                for kind in OBSTACLE_KINDS:
                    accepted[kind].append(pair_candidates[kind])

        if any(len(accepted[kind]) != per_kind for kind in OBSTACLE_KINDS):
            print(json.dumps({
                "error": "insufficient_accepted_scenes",
                "accepted": {kind: len(values) for kind, values in accepted.items()},
                "accepted_base_trajectories": len(accepted[OBSTACLE_KINDS[0]]),
                "counters_examined": counters_examined,
            }), flush=True)
            return 4
        scenes = finalize_scenes(
            accepted, args.scene_count, joint_names, lower, upper
        )
        node_binary = reachability_binary_path()
        if not node_binary:
            raise RuntimeError("could not resolve the installed reachability_graph_node binary")
        catalog = {
            "schema_version": 2,
            "catalog_id": args.catalog_id,
            "frame_id": "world",
            "group_name": "arm",
            "end_effector_link": "end_effector_link",
            "joint_names": joint_names,
            "model": actual_model_hashes,
            "generator": {
                "algorithm": "sha256-counter-u53-v1",
                "namespace": NAMESPACE.decode("ascii"),
                "master_key_hex": args.master_key_hex.lower(),
                "joint_lower_bounds": lower,
                "joint_upper_bounds": upper,
                "joint_limit_margin_fraction": args.joint_margin_fraction,
                "validation_step_rad": 0.05,
                "target_exclusion_radius_m": 0.055,
                "obstacle_clearance_m": 0.035,
                "minimum_target_obstacle_distance_m": args.minimum_target_obstacle_distance,
                "point_radius_m": 0.012,
                "segment_radius_m": 0.006,
                "segment_half_length_m": args.segment_half_length,
                "max_source_counters": args.max_source_counters,
                "max_detour_attempts": args.max_detour_attempts,
                "counters_examined": counters_examined,
                "roadmap_independent": True,
                "paired_obstacle_design": True,
                "base_trajectory_count": len(accepted[OBSTACLE_KINDS[0]]),
                "oracle": {
                    "service": "/om6dof_topo_gng/validate_reachability_scene",
                    "collision_checks": ["conservative_capsules", "moveit_fcl"],
                    "query_mode": True,
                    "graph_revision": int(probe.graph.graph_revision),
                    "graph_method": probe.graph.graph_method,
                    "graph_node_count": len(probe.graph.nodes),
                    "sample_stream_seed": int(probe.graph.sample_stream_seed),
                    "sample_stream_type": probe.graph.sample_stream_type,
                    "graph_publisher_count": graph_publisher_count,
                },
                "implementation": {
                    "generator_script_sha256": file_sha256(Path(__file__).resolve()),
                    "reachability_node_binary_sha256": (
                        file_sha256(node_binary) if node_binary else ""
                    ),
                },
            },
            "scenes": scenes,
        }
        print(json.dumps({"catalog": catalog}, sort_keys=True), flush=True)
        return 0
    finally:
        probe.destroy_node()
        rclpy.shutdown()


def assert_clean_ros_domain(env, timeout_sec=8.0):
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


def run_controller(args):
    output = Path(args.output).expanduser().resolve()
    if output.exists():
        raise ValueError(f"refusing to overwrite existing catalog: {output}")
    output.parent.mkdir(parents=True, exist_ok=True)
    log_path = output.with_suffix(output.suffix + ".log")
    env = os.environ.copy()
    env["ROS_DOMAIN_ID"] = str(args.ros_domain_id)
    env["ROS_LOCALHOST_ONLY"] = "1"
    env["RMW_IMPLEMENTATION"] = args.rmw_implementation
    assert_clean_ros_domain(env)
    script = str(Path(__file__).resolve())
    with log_path.open("w", encoding="utf-8") as launch_log:
        launch = subprocess.Popen(
            [
                "ros2", "launch", "om6dof_dd_gng", "reachability_graph.launch.py",
                "launch_rviz:=false", "query_mode:=true", "graph_method:=halton_prm",
                f"sample_count:={args.oracle_graph_nodes}",
                f"sample_stream_seed:={args.oracle_stream_seed}",
                "halton_start_index:=17",
            ],
            env=env,
            stdout=launch_log,
            stderr=subprocess.STDOUT,
            start_new_session=True,
        )
        try:
            command = [
                sys.executable, script, "--probe",
                "--output", str(output),
                "--catalog-id", args.catalog_id,
                "--scene-count", str(args.scene_count),
                "--master-key-hex", args.master_key_hex,
                "--joint-margin-fraction", str(args.joint_margin_fraction),
                "--segment-half-length", str(args.segment_half_length),
                "--minimum-target-obstacle-distance", str(
                    args.minimum_target_obstacle_distance
                ),
                "--max-source-counters", str(args.max_source_counters),
                "--max-detour-attempts", str(args.max_detour_attempts),
                "--graph-timeout", str(args.graph_timeout),
                "--service-timeout", str(args.service_timeout),
                "--urdf-sha256", args.urdf_sha256,
                "--srdf-sha256", args.srdf_sha256,
                "--parameters-sha256", args.parameters_sha256,
            ]
            completed = subprocess.run(
                command,
                env=env,
                text=True,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                timeout=args.overall_timeout,
                check=False,
            )
            lines = [line for line in completed.stdout.splitlines() if line.startswith("{")]
            result = json.loads(lines[-1]) if lines else {
                "error": f"probe_exit_{completed.returncode}: {completed.stdout[-500:]}"
            }
            if completed.returncode != 0 or result.get("error"):
                raise RuntimeError(result.get("error", f"probe_exit_{completed.returncode}"))
            encoded = json.dumps(result["catalog"], sort_keys=True, indent=2) + "\n"
            output.write_text(encoded, encoding="utf-8")
            print(json.dumps({
                "output": str(output),
                "sha256": hashlib.sha256(encoded.encode("utf-8")).hexdigest(),
                "scene_count": len(result["catalog"]["scenes"]),
                "launch_log": str(log_path),
            }, sort_keys=True))
            return 0
        finally:
            stop_launch(launch)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--probe", action="store_true", help=argparse.SUPPRESS)
    parser.add_argument("--output", required=True)
    parser.add_argument("--catalog-id", default="om6dof_icra_scene_catalog_v1")
    parser.add_argument("--scene-count", type=int, default=6)
    parser.add_argument("--master-key-hex", default=DEFAULT_MASTER_KEY_HEX)
    parser.add_argument("--joint-margin-fraction", type=float, default=0.05)
    parser.add_argument("--segment-half-length", type=float, default=0.06)
    parser.add_argument("--minimum-target-obstacle-distance", type=float, default=0.08)
    parser.add_argument("--max-source-counters", type=int, default=500)
    parser.add_argument("--max-detour-attempts", type=int, default=24)
    parser.add_argument("--graph-timeout", type=float, default=60.0)
    parser.add_argument("--service-timeout", type=float, default=10.0)
    parser.add_argument("--overall-timeout", type=float, default=1800.0)
    parser.add_argument("--oracle-graph-nodes", type=int, default=200)
    parser.add_argument("--oracle-stream-seed", type=int, default=424242)
    parser.add_argument("--ros-domain-id", type=int, default=20)
    parser.add_argument("--rmw-implementation", default="rmw_fastrtps_cpp")
    parser.add_argument("--urdf-sha256", default="")
    parser.add_argument("--srdf-sha256", default="")
    parser.add_argument("--parameters-sha256", default="")
    args = parser.parse_args()
    if args.scene_count < 6 or args.scene_count % 6:
        parser.error("--scene-count must be a positive multiple of six")
    if not 0.0 <= args.joint_margin_fraction < 0.5:
        parser.error("--joint-margin-fraction must be within [0, 0.5)")
    if args.segment_half_length <= 0.0 or args.minimum_target_obstacle_distance <= 0.055:
        parser.error("obstacle dimensions/distances are invalid")
    if args.max_source_counters < 1 or args.max_detour_attempts < 1:
        parser.error("candidate limits must be positive")
    if not 20 <= args.ros_domain_id <= 99:
        parser.error("--ros-domain-id must stay within 20..99")
    try:
        bytes.fromhex(args.master_key_hex)
    except ValueError:
        parser.error("--master-key-hex must be hexadecimal")
    for option_name, value in (
        ("--urdf-sha256", args.urdf_sha256),
        ("--srdf-sha256", args.srdf_sha256),
        ("--parameters-sha256", args.parameters_sha256),
    ):
        if (
            len(value) != 64
            or any(character not in "0123456789abcdefABCDEF" for character in value)
        ):
            parser.error(f"{option_name} must be a SHA-256 hex digest")
    if args.probe:
        return run_probe(args)
    try:
        return run_controller(args)
    except (RuntimeError, ValueError) as error:
        parser.error(str(error))


if __name__ == "__main__":
    sys.exit(main())
