#!/usr/bin/env python3
"""Repeatable preview-only GNG, guarded-GNG, and Halton/PRM benchmark.

The controller launches one isolated reachability process per method/offset. A
probe child publishes synthetic JointState and typed EnvironmentGraph messages,
then records both a clear path and a fixed midpoint point-obstacle update.
Nothing in this script publishes to a controller topic.
"""

import argparse
import csv
import itertools
import json
import math
import os
import signal
import subprocess
import sys
import time
from pathlib import Path


def summarize_plan(message):
    return {
        "valid": bool(message.valid),
        "reason": message.reason,
        "blocked_nodes": int(message.blocked_node_count),
        "blocked_edges": int(message.blocked_edge_count),
        "planning_time_ms": float(message.planning_time_ms),
        "exact_valid": bool(message.exact_collision_valid),
        "exact_checks": int(message.exact_state_checks),
        "exact_replans": int(message.exact_replans),
        "exact_time_ms": float(message.exact_validation_time_ms),
        "path_nodes": len(message.reachability_node_ids),
        "path_ids": list(message.reachability_node_ids),
    }


def run_probe(timeout):
    import rclpy
    from geometry_msgs.msg import Point
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import JointState
    from om6dof_dd_gng.msg import (
        EnvironmentGraph,
        EnvironmentNode,
        ReachabilityGraph,
        ReachabilityPlan,
    )

    class Probe(Node):
        def __init__(self):
            super().__init__("reachability_benchmark_probe")
            latched = QoSProfile(depth=1)
            latched.reliability = ReliabilityPolicy.RELIABLE
            latched.durability = DurabilityPolicy.TRANSIENT_LOCAL
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
            self.joint_pub = self.create_publisher(JointState, "/joint_states", 10)
            self.environment_pub = self.create_publisher(
                EnvironmentGraph, "/om6dof_topo_gng/environment_graph_data", 10
            )
            self.graph = None
            self.start = None
            self.goal = None
            self.obstacle_position = None
            self.clear = None
            self.dynamic = None
            self.phase = "waiting_graph"
            self.last_publish = 0.0

        def graph_callback(self, message):
            if self.graph is not None or not message.nodes:
                return
            self.graph = message
            self.start = message.nodes[0]
            # Node 0 is the default all-zero state. The graph builder inserts
            # the SRDF home_pose immediately afterwards, before any method-
            # dependent samples, so node 1 is the same executable target for
            # GNG and Halton/PRM. This keeps paired planning queries matched.
            if len(message.nodes) < 2:
                return
            self.goal = message.nodes[1]
            self.obstacle_position = Point()
            self.obstacle_position.x = 0.5 * (
                self.start.pose.position.x + self.goal.pose.position.x
            )
            self.obstacle_position.y = 0.5 * (
                self.start.pose.position.y + self.goal.pose.position.y
            )
            self.obstacle_position.z = 0.5 * (
                self.start.pose.position.z + self.goal.pose.position.z
            )
            self.phase = "clear"

        def publish_inputs(self):
            joint_state = JointState()
            joint_state.header.stamp = self.get_clock().now().to_msg()
            joint_state.name = list(self.graph.joint_names)
            joint_state.position = list(self.start.joint_positions)
            self.joint_pub.publish(joint_state)

            environment = EnvironmentGraph()
            environment.header.stamp = joint_state.header.stamp
            environment.header.frame_id = "world"
            target = EnvironmentNode()
            target.id = 900001
            target.position = self.goal.pose.position
            target.class_id = 1
            target.confidence = 1.0
            environment.nodes.append(target)
            if self.phase == "dynamic" and self.obstacle_position is not None:
                obstacle = EnvironmentNode()
                obstacle.id = 900002
                obstacle.position = self.obstacle_position
                obstacle.class_id = -1
                obstacle.confidence = 1.0
                environment.nodes.append(obstacle)
            self.environment_pub.publish(environment)

        def plan_callback(self, message):
            if self.graph is None:
                return
            if self.phase == "clear" and message.valid and message.exact_collision_valid:
                self.clear = summarize_plan(message)
                self.phase = "dynamic"
            elif self.phase == "dynamic" and message.blocked_edge_count > 0:
                self.dynamic = summarize_plan(message)
                self.phase = "done"

    rclpy.init()
    probe = Probe()
    deadline = time.monotonic() + timeout
    try:
        while rclpy.ok() and probe.phase != "done" and time.monotonic() < deadline:
            rclpy.spin_once(probe, timeout_sec=0.05)
            now = time.monotonic()
            if probe.graph is not None and probe.phase in ("clear", "dynamic"):
                if now - probe.last_publish >= 0.2:
                    probe.publish_inputs()
                    probe.last_publish = now
        if probe.phase != "done":
            print(json.dumps({"error": "probe_timeout", "phase": probe.phase}), flush=True)
            return 2
        output = {
            "method": probe.graph.graph_method,
            "requested_node_count": int(probe.graph.requested_node_count),
            "anchor_node_count": int(probe.graph.anchor_node_count),
            "prototype_budget": int(probe.graph.prototype_budget),
            "prototype_node_count": int(probe.graph.prototype_node_count),
            "requested_guard_node_count": int(
                probe.graph.requested_guard_node_count
            ),
            "guard_node_count": int(probe.graph.guard_node_count),
            "fill_sample_node_count": int(probe.graph.fill_sample_node_count),
            "candidate_attempts": int(probe.graph.candidate_attempts),
            "effective_halton_start_index": int(probe.graph.halton_start_index),
            "effective_gng_training_samples": int(
                probe.graph.gng_training_sample_count
            ),
            "effective_guard_fraction": float(probe.graph.effective_guard_fraction),
            "nodes": len(probe.graph.nodes),
            "edges": len(probe.graph.edges),
            "components": int(probe.graph.connected_components),
            "build_time_ms": float(probe.graph.build_time_ms),
            "target_xyz": [
                probe.goal.pose.position.x,
                probe.goal.pose.position.y,
                probe.goal.pose.position.z,
            ],
            "target_joints": list(probe.goal.joint_positions),
            "obstacle_xyz": [
                probe.obstacle_position.x,
                probe.obstacle_position.y,
                probe.obstacle_position.z,
            ],
            "clear": probe.clear,
            "dynamic": probe.dynamic,
        }
        print(json.dumps(output, sort_keys=True), flush=True)
        return 0
    finally:
        probe.destroy_node()
        rclpy.shutdown()


def validate_runtime_result(result, method, sample_count, sample_offset, guard_fraction):
    if result.get("error"):
        return
    expected_guard_fraction = guard_fraction if method == "guarded_gng" else 0.0
    checks = {
        "method": (result.get("method"), method),
        "requested_node_count": (result.get("requested_node_count"), sample_count),
        "nodes": (result.get("nodes"), sample_count),
        "effective_halton_start_index": (
            result.get("effective_halton_start_index"), sample_offset
        ),
    }
    mismatches = [
        f"{name}={actual!r}, expected {expected!r}"
        for name, (actual, expected) in checks.items()
        if actual != expected
    ]
    actual_guard_fraction = result.get("effective_guard_fraction")
    if actual_guard_fraction is None or not math.isclose(
        float(actual_guard_fraction), expected_guard_fraction, rel_tol=0.0, abs_tol=1.0e-12
    ):
        mismatches.append(
            "effective_guard_fraction="
            f"{actual_guard_fraction!r}, expected {expected_guard_fraction!r}"
        )
    composition = sum(
        int(result.get(name, -sample_count))
        for name in (
            "anchor_node_count",
            "prototype_node_count",
            "guard_node_count",
            "fill_sample_node_count",
        )
    )
    if composition != result.get("nodes"):
        mismatches.append(
            f"reported node composition sums to {composition}, expected {result.get('nodes')!r}"
        )
    if method in ("gng", "guarded_gng"):
        remaining = sample_count - int(result.get("anchor_node_count", sample_count))
        allocated = int(result.get("prototype_budget", -sample_count)) + int(
            result.get("requested_guard_node_count", -sample_count)
        )
        if allocated != remaining:
            mismatches.append(
                f"GNG allocation sums to {allocated}, expected remaining budget {remaining}"
            )
    for scenario in ("clear", "dynamic"):
        values = result.get(scenario)
        if not isinstance(values, dict) or "valid" not in values or "exact_valid" not in values:
            mismatches.append(f"{scenario} result is missing valid/exact_valid fields")
            continue
        if bool(values.get("valid")) != bool(values.get("exact_valid")):
            mismatches.append(
                f"{scenario} valid={values.get('valid')!r} differs from "
                f"exact_valid={values.get('exact_valid')!r}"
            )
    if mismatches:
        result["error"] = "runtime_contract_failed: " + "; ".join(mismatches)


def flatten(result, method, seed, run_index, sample_offset, guard_fraction):
    row = {
        "run_index": run_index,
        "method": method,
        "seed": seed,
        "halton_start_index": sample_offset,
        "gng_guard_fraction": guard_fraction if method == "guarded_gng" else 0.0,
        "reported_method": result.get("method", ""),
        "requested_node_count": result.get("requested_node_count", 0),
        "anchor_node_count": result.get("anchor_node_count", 0),
        "prototype_budget": result.get("prototype_budget", 0),
        "prototype_node_count": result.get("prototype_node_count", 0),
        "requested_guard_node_count": result.get("requested_guard_node_count", 0),
        "guard_node_count": result.get("guard_node_count", 0),
        "fill_sample_node_count": result.get("fill_sample_node_count", 0),
        "candidate_attempts": result.get("candidate_attempts", 0),
        "effective_halton_start_index": result.get("effective_halton_start_index", 0),
        "effective_gng_training_samples": result.get(
            "effective_gng_training_samples", 0
        ),
        "effective_guard_fraction": result.get("effective_guard_fraction", math.nan),
        "nodes": result.get("nodes", 0),
        "edges": result.get("edges", 0),
        "components": result.get("components", 0),
        "build_time_ms": result.get("build_time_ms", math.nan),
        "target_xyz": json.dumps(result.get("target_xyz", [])),
        "target_joints": json.dumps(result.get("target_joints", [])),
        "obstacle_xyz": json.dumps(result.get("obstacle_xyz", [])),
    }
    for scenario in ("clear", "dynamic"):
        values = result.get(scenario) or {}
        for key in (
            "valid",
            "reason",
            "blocked_nodes",
            "blocked_edges",
            "planning_time_ms",
            "exact_valid",
            "exact_checks",
            "exact_replans",
            "exact_time_ms",
            "path_nodes",
        ):
            row[f"{scenario}_{key}"] = values.get(key, "")
    row["error"] = result.get("error", "")
    return row


def stop_launch(process):
    if process.poll() is not None:
        return
    try:
        os.killpg(process.pid, signal.SIGINT)
        process.wait(timeout=8)
    except ProcessLookupError:
        return
    except subprocess.TimeoutExpired:
        try:
            os.killpg(process.pid, signal.SIGTERM)
            process.wait(timeout=3)
        except (ProcessLookupError, subprocess.TimeoutExpired):
            try:
                os.killpg(process.pid, signal.SIGKILL)
            except ProcessLookupError:
                return
            process.wait(timeout=3)


def run_controller(args):
    script = str(Path(__file__).resolve())
    methods = [item.strip() for item in args.methods.split(",") if item.strip()]
    if not methods:
        raise SystemExit("--methods must contain at least one method")
    if len(set(methods)) != len(methods):
        raise SystemExit("--methods must not contain duplicates")
    invalid = set(methods) - {"gng", "guarded_gng", "halton_prm"}
    if invalid:
        raise SystemExit(f"unsupported methods: {sorted(invalid)}")
    schedule = []
    seeds = (
        [int(item.strip()) for item in args.seed_list.split(",") if item.strip()]
        if args.seed_list
        else list(range(args.seeds))
    )
    if not seeds or any(seed < 0 for seed in seeds):
        raise SystemExit("--seed-list must contain non-negative integers")
    if len(set(seeds)) != len(seeds):
        raise SystemExit("--seed-list must not contain duplicates")
    method_orders = list(itertools.permutations(methods))
    for seed_index, seed in enumerate(seeds):
        # Cycle through every method permutation. This is equivalent to the
        # previous alternating order for two methods and remains balanced when
        # guarded_gng adds a third method.
        ordered_methods = method_orders[seed_index % len(method_orders)]
        schedule.extend((method, seed) for method in ordered_methods)
    last_domain = args.domain_base + args.domain_pool_size - 1
    if args.domain_base < 20 or args.domain_pool_size < 1 or last_domain > 99:
        raise SystemExit(
            f"ROS domain pool {args.domain_base}..{last_domain} must stay within 20..99 "
            "to avoid the live domain and the host ephemeral UDP port range"
        )

    output = Path(args.output).expanduser().resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    if output.exists() and not args.force:
        raise SystemExit(f"refusing to overwrite existing output: {output}; pass --force")
    log_dir = output.parent / f"{output.stem}_logs"
    log_dir.mkdir(parents=True, exist_ok=True)
    rows = []
    with output.open("w", newline="", encoding="utf-8") as stream:
        writer = None
        for run_index, (method, seed) in enumerate(schedule):
            domain = args.domain_base + (run_index % args.domain_pool_size)
            sample_offset = 17 + seed * 7919
            env = os.environ.copy()
            env["ROS_DOMAIN_ID"] = str(domain)
            env["ROS_LOCALHOST_ONLY"] = "1"
            env["RMW_IMPLEMENTATION"] = args.rmw_implementation
            launch_log_path = log_dir / (
                f"run_{run_index:04d}_{method}_offset_{seed}_domain_{domain}.log"
            )
            with launch_log_path.open("w", encoding="utf-8") as launch_log:
                launch = subprocess.Popen(
                    [
                        "ros2",
                        "launch",
                        "om6dof_dd_gng",
                        "reachability_graph.launch.py",
                        "launch_rviz:=false",
                        f"graph_method:={method}",
                        f"sample_count:={args.sample_count}",
                        f"halton_start_index:={sample_offset}",
                        f"gng_guard_fraction:={args.gng_guard_fraction}",
                    ],
                    env=env,
                    stdout=launch_log,
                    stderr=subprocess.STDOUT,
                    start_new_session=True,
                )
                try:
                    completed = subprocess.run(
                        [sys.executable, script, "--probe", "--timeout", str(args.timeout)],
                        env=env,
                        text=True,
                        stdout=subprocess.PIPE,
                        stderr=subprocess.STDOUT,
                        timeout=args.timeout + 10,
                        check=False,
                    )
                    lines = [
                        line for line in completed.stdout.splitlines() if line.startswith("{")
                    ]
                    result = json.loads(lines[-1]) if lines else {
                        "error": f"probe_exit_{completed.returncode}: {completed.stdout[-300:]}"
                    }
                    if completed.returncode != 0 and not result.get("error"):
                        result["error"] = (
                            f"probe_exit_{completed.returncode}: {completed.stdout[-300:]}"
                        )
                except subprocess.TimeoutExpired:
                    result = {"error": "probe_process_timeout"}
                finally:
                    stop_launch(launch)
            validate_runtime_result(
                result, method, args.sample_count, sample_offset, args.gng_guard_fraction
            )
            row = flatten(
                result, method, seed, run_index, sample_offset, args.gng_guard_fraction
            )
            row["ros_domain_id"] = domain
            row["rmw_implementation"] = args.rmw_implementation
            row["ros_localhost_only"] = True
            row["launch_log"] = str(launch_log_path)
            rows.append(row)
            if writer is None:
                writer = csv.DictWriter(stream, fieldnames=list(row.keys()))
                writer.writeheader()
            writer.writerow(row)
            stream.flush()
            os.fsync(stream.fileno())
            print(json.dumps(row, sort_keys=True), flush=True)
    print(f"wrote {len(rows)} runs to {output}")
    return 0 if all(not row["error"] for row in rows) else 1


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--probe", action="store_true", help=argparse.SUPPRESS)
    parser.add_argument("--timeout", type=float, default=35.0)
    parser.add_argument("--methods", default="gng,halton_prm")
    parser.add_argument("--seeds", type=int, default=3)
    parser.add_argument("--seed-list", default="")
    parser.add_argument("--sample-count", type=int, default=800)
    parser.add_argument("--gng-guard-fraction", type=float, default=0.25)
    parser.add_argument("--domain-base", type=int, default=20)
    parser.add_argument("--domain-pool-size", type=int, default=80)
    parser.add_argument("--rmw-implementation", default="rmw_fastrtps_cpp")
    parser.add_argument("--output", default="reachability_benchmark.csv")
    parser.add_argument("--force", action="store_true")
    args = parser.parse_args()
    if args.probe:
        return run_probe(args.timeout)
    if args.seeds < 1 or args.sample_count < 2:
        parser.error("--seeds must be >=1 and --sample-count must be >=2")
    if not math.isfinite(args.timeout) or args.timeout <= 0.0:
        parser.error("--timeout must be a positive finite value")
    if not 0.0 <= args.gng_guard_fraction <= 0.90:
        parser.error("--gng-guard-fraction must be within [0.0, 0.90]")
    return run_controller(args)


if __name__ == "__main__":
    sys.exit(main())
