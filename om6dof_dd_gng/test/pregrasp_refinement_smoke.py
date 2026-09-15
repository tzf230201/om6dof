#!/usr/bin/env python3
"""Paired synthetic ROS smoke for the local pregrasp correction, domain 220.

Source ROS Humble and the workspace install, then run this file with Python.
Only the reachability planner is started. Joint measurements and the scene are
synthetic; no hardware nodes, controller clients, or robot actions are created.
Logs and a machine-readable result are retained in a new /tmp directory.
"""

import copy
import csv
import hashlib
import importlib.util
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time

import yaml


DOMAIN_ID = 220
TARGET = (0.295, 0.040, 0.150)
JOINT_NAMES = [f"joint{i}" for i in range(1, 7)]
REPO = Path(__file__).resolve().parents[2]
SOURCE = (REPO / "experiments/cartesian_workspace/results/"
          "comparison_v1_v2_25mm_20260909/v2")
TOPIC_PARAMETERS = (
    "environment_graph_topic", "joint_state_topic", "graph_data_topic",
    "training_samples_topic", "training_samples_cloud_topic", "marker_topic",
    "plan_topic", "path_topic", "query_topic", "rebuild_service", "plan_service",
    "scene_validation_service",
)


def point_tuple(point):
    return (point.x, point.y, point.z)


def distance(first, second):
    return math.sqrt(sum((a - b) ** 2 for a, b in zip(first, second)))


def tool_axis(pose):
    """Rotate the configured local TCP +Z using the planner's published FK."""
    quaternion = pose.orientation
    values = (quaternion.x, quaternion.y, quaternion.z, quaternion.w)
    norm = math.sqrt(sum(value * value for value in values))
    if not math.isfinite(norm) or norm < 1.0e-9:
        raise AssertionError("Published FK quaternion is invalid")
    x, y, z, w = (value / norm for value in values)
    return (2.0 * (x * z + w * y), 2.0 * (y * z - w * x),
            1.0 - 2.0 * (x * x + y * y))


def alignment(pose, target):
    displacement = tuple(a - b for a, b in zip(target, point_tuple(pose.position)))
    norm = math.sqrt(sum(value * value for value in displacement))
    return sum(a * b / norm for a, b in zip(tool_axis(pose), displacement)) if norm else -1.0


def make_fixture(run):
    with (SOURCE / "points.csv").open() as source:
        reader = csv.DictReader(source)
        fields = reader.fieldnames
        desired = {225, 250, 275}
        rows = [row for row in reader if any(
            abs(float(row["x_mm"]) - x) < 1.0e-6 and
            abs(float(row["y_mm"])) < 1.0e-6 and
            abs(float(row["z_mm"]) - 150.0) < 1.0e-6 for x in desired)]
    rows.sort(key=lambda row: float(row["x_mm"]))
    assert len(rows) == 3 and {round(float(row["x_mm"])) for row in rows} == desired
    assert all(row["status"] == "pose_found" and row["pose_found"] == "1" and
               row["position_found"] == "1" for row in rows), "Fixture must use only green poses"
    # (200,0,150) is orientation_unresolved in this dataset; it is deliberately
    # not admitted to the green fixture merely to obtain an extra graph node.
    subset = run / "green_subset.csv"
    with subset.open("w", newline="") as destination:
        writer = csv.DictWriter(destination, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)
    initial = [float(rows[0][f"q{i}_rad"]) for i in range(1, 7)]
    return subset, initial


def make_config(run, subset, enabled, prepare_config):
    directory = run / ("enabled" if enabled else "disabled")
    directory.mkdir()
    config = prepare_config(
        REPO / "om6dof_dd_gng/config/topo_gng_v2.yaml", subset, directory,
        final_grasp=True)
    document = yaml.safe_load(config.read_text())
    parameters = document["reachability_graph_node"]["ros__parameters"]
    parameters.update({
        "robot_description": (SOURCE / "model.urdf").read_text(),
        "robot_description_semantic":
            (REPO / "om6dof_moveit_config/config/om6dof.srdf").read_text(),
        "planning_period_sec": 0.3,
        "pregrasp_refinement_enabled": enabled,
        "exclude_selected_target_from_collision": True,
    })
    namespace = "/pregrasp_smoke_" + ("enabled" if enabled else "disabled")
    for name in TOPIC_PARAMETERS:
        parameters[name] = namespace + "/" + name
    parameters["expanded_urdf_sha256"] = hashlib.sha256(
        parameters["robot_description"].encode()).hexdigest()
    parameters["srdf_sha256"] = hashlib.sha256(
        parameters["robot_description_semantic"].encode()).hexdigest()
    parameters["reachability_parameters_sha256"] = hashlib.sha256(
        yaml.safe_dump(parameters, sort_keys=True).encode()).hexdigest()
    config.write_text(yaml.safe_dump({
        "reachability_graph_node": {"ros__parameters": parameters}}, sort_keys=False))
    return directory, config, parameters, namespace + "/validate_grasp_execution"


def run_phase(binary, directory, config, parameters, service_name, initial, enabled):
    import rclpy
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import JointState
    from om6dof_dd_gng.msg import (
        EnvironmentGraph, EnvironmentNode, ReachabilityGraph, ReachabilityPlan, TopologyEdge)
    from om6dof_dd_gng.srv import ValidateGraspExecution

    result = {"refinement_enabled": enabled, "checks": [], "passed": False}
    node = rclpy.create_node("pregrasp_smoke_" + ("enabled" if enabled else "disabled"))
    joint_publisher = node.create_publisher(JointState, parameters["joint_state_topic"], 20)
    environment_publisher = node.create_publisher(
        EnvironmentGraph, parameters["environment_graph_topic"], 10)
    plans, graphs = [], []
    latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
    node.create_subscription(ReachabilityPlan, parameters["plan_topic"], plans.append, latched)
    node.create_subscription(ReachabilityGraph, parameters["graph_data_topic"], graphs.append, latched)
    client = node.create_client(ValidateGraspExecution, service_name)
    current = list(initial)

    def publish_joints():
        message = JointState()
        message.header.stamp = node.get_clock().now().to_msg()
        message.name = JOINT_NAMES + ["gripper_left_joint", "gripper_right_joint"]
        message.position = current + [0.019, 0.019]
        joint_publisher.publish(message)

    def publish_environment():
        message = EnvironmentGraph()
        message.header.frame_id = "world"
        message.header.stamp = node.get_clock().now().to_msg()
        for identifier, z in enumerate((0.140, 0.150, 0.160), 1):
            point = EnvironmentNode()
            point.id, point.class_id, point.confidence = identifier, 39, 1.0
            point.position.x, point.position.y, point.position.z = TARGET[0], TARGET[1], z
            message.nodes.append(point)
        for source_id, target_id in ((1, 2), (2, 3)):
            edge = TopologyEdge()
            edge.source_id, edge.target_id = source_id, target_id
            message.edges.append(edge)
        environment_publisher.publish(message)

    node.create_timer(0.05, publish_joints)
    node.create_timer(0.10, publish_environment)
    process = None
    log = (directory / "planner.log").open("w")
    try:
        process = subprocess.Popen([
            str(binary), "--ros-args", "--params-file", str(config), "-r",
            "/om6dof_topo_gng_v2/validate_grasp_execution:=" + service_name,
        ], stdout=log, stderr=subprocess.STDOUT, env=os.environ.copy())
        result["own_subprocess_pid"] = process.pid

        def pump(seconds):
            deadline = time.monotonic() + seconds
            while time.monotonic() < deadline:
                if process.poll() is not None:
                    raise RuntimeError(f"Planner exited with {process.returncode}")
                rclpy.spin_once(node, timeout_sec=0.02)

        deadline = time.monotonic() + 60.0
        plan = None
        while time.monotonic() < deadline:
            pump(0.1)
            matches = [candidate for candidate in plans if
                       (candidate.valid and candidate.grasp_approach_valid if enabled else
                        not candidate.valid and candidate.reason ==
                        "pregrasp_pose_unavailable_for_target")]
            if matches and graphs:
                plan = copy.deepcopy(matches[-1])
                break
        result["seen_plan_reasons"] = sorted({candidate.reason for candidate in plans})
        if plan is None:
            raise AssertionError("Expected paired plan outcome not observed within 60 seconds")

        graph = graphs[-1]
        assert graph.graph_method == "workspace_samples" and len(graph.nodes) == 3
        anchor_checks = [{
            "node_id": item.id, "fk_position_m": point_tuple(item.pose.position),
            "world_tool_axis": tool_axis(item.pose),
            "distance_m": distance(point_tuple(item.pose.position), TARGET),
            "alignment": alignment(item.pose, TARGET),
        } for item in graph.nodes]
        eligible = [item for item in anchor_checks if
                    0.07 <= item["distance_m"] <= 0.13 and item["alignment"] >= 0.95]
        result["archived_anchor_fk_checks"] = anchor_checks
        result["direct_pregrasp_anchor_count"] = len(eligible)
        assert not eligible, "Fixture unexpectedly has an eligible archived pregrasp pose"
        assert graph.requested_node_count == 3
        if not enabled:
            assert not plan.valid and not plan.grasp_approach_valid
            assert not any(candidate.valid for candidate in plans)
            result["reason"] = plan.reason
            result["passed"] = True
            return result

        count = plan.pregrasp_waypoint_count
        points = plan.joint_path_preview.points
        poses = plan.end_effector_path.poses
        assert plan.reason == "path_ready_exact_validated_grasp_preview_only"
        assert plan.exact_collision_valid and plan.selected_target_excluded_from_collision
        assert len(points) == len(poses) and count < len(points)
        assert count > len(plan.reachability_node_ids) + 1, "Validated bridge is missing from prefix"
        pregrasp_alignment = alignment(poses[count - 1].pose, TARGET)
        endpoint_error = distance(point_tuple(poses[-1].pose.position), TARGET)
        assert pregrasp_alignment >= 0.95
        assert endpoint_error < 0.003 and plan.grasp_position_error < 0.003
        assert distance(point_tuple(plan.grasp_target_position), TARGET) < 1.0e-9
        result["plan"] = {
            "reason": plan.reason, "archived_node_ids": list(plan.reachability_node_ids),
            "pregrasp_waypoint_count": count, "trajectory_points": len(points),
            "bridge_points": count - len(plan.reachability_node_ids) - 1,
            "pregrasp_alignment": pregrasp_alignment,
            "planned_fk_endpoint_error_m": endpoint_error,
            "reported_endpoint_error_m": plan.grasp_position_error,
        }
        deadline = time.monotonic() + 5.0
        while not client.service_is_ready() and time.monotonic() < deadline:
            pump(0.1)
        assert client.service_is_ready(), "Validation service is unavailable"

        def validate(label, trajectory_points, include_closure):
            pump(0.35)
            request = ValidateGraspExecution.Request()
            request.trajectory = copy.deepcopy(plan.joint_path_preview)
            request.trajectory.points = copy.deepcopy(trajectory_points)
            request.trajectory.header.stamp = node.get_clock().now().to_msg()
            request.target_position = copy.deepcopy(plan.grasp_target_position)
            request.target_class_id = 39
            request.target_environment_node_id = plan.target_environment_node_id
            request.gripper_open_position, request.gripper_close_position = 0.019, -0.010
            request.include_closure = include_closure
            future = client.call_async(request)
            deadline = time.monotonic() + 8.0
            while not future.done() and time.monotonic() < deadline:
                pump(0.03)
            assert future.done(), f"{label}: validation service timed out"
            response = future.result()
            check = {"stage": label, "valid": response.valid, "reason": response.reason}
            result["checks"].append(check)
            print(json.dumps(check), flush=True)
            assert response.valid, f"{label}: {response.reason}"

        validate("graph_plus_pregrasp_bridge", points[:count], False)
        current = list(points[count - 1].positions)
        validate("forward_insertion_and_closure", points[count - 1:], True)
        current = list(points[-1].positions)
        validate("closure_at_simulated_endpoint", points[-1:], True)
        result["passed"] = True
    except Exception as error:
        result["error"] = str(error)
        result["seen_plan_reasons"] = sorted({candidate.reason for candidate in plans})
    finally:
        if process is not None and process.poll() is None:
            process.send_signal(signal.SIGINT)
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                process.terminate()
                process.wait(timeout=5)
        log.close()
        node.destroy_node()
        (directory / "result.json").write_text(json.dumps(result, indent=2) + "\n")
    return result


def main():
    os.environ.update(ROS_DOMAIN_ID=str(DOMAIN_ID), ROS_LOCALHOST_ONLY="1",
                      RMW_IMPLEMENTATION="rmw_fastrtps_cpp")
    run = Path(tempfile.mkdtemp(prefix="om6dof_pregrasp_refinement_smoke_"))
    os.environ["ROS_LOG_DIR"] = str(run / "roslog")
    import rclpy
    from ament_index_python.packages import get_package_prefix

    spec = importlib.util.spec_from_file_location(
        "experiment_launch", REPO / "om6dof_dd_gng/launch/ddgng_experiment_reachability.launch.py")
    launch = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(launch)
    binary = Path(get_package_prefix("om6dof_dd_gng")) / "lib/om6dof_dd_gng/reachability_graph_node"
    binary_hash = hashlib.sha256(binary.read_bytes()).hexdigest()
    result = {"domain_id": DOMAIN_ID, "run_directory": str(run),
              "binary": str(binary.resolve()), "binary_sha256": binary_hash,
              "hardware_access": False, "action_clients_created": False, "phases": []}
    try:
        subset, initial = make_fixture(run)
        result["fixture"] = {"green_xyz_mm": [[225, 0, 150], [250, 0, 150], [275, 0, 150]],
                             "target_center_m": TARGET, "initial_joint_positions": initial,
                             "subset_sha256": hashlib.sha256(subset.read_bytes()).hexdigest()}
        rclpy.init()
        for enabled in (False, True):
            assert hashlib.sha256(binary.read_bytes()).hexdigest() == binary_hash, "Binary changed between paired runs"
            phase_config = make_config(run, subset, enabled, launch.prepare_config)
            phase = run_phase(binary, *phase_config, initial, enabled)
            result["phases"].append(phase)
            if not phase["passed"]:
                raise AssertionError(f"{'Enabled' if enabled else 'Disabled'} phase failed: {phase.get('error')}")
        result["passed"] = True
    except Exception as error:
        result.update(passed=False, error=str(error))
    finally:
        if rclpy.ok():
            rclpy.shutdown()
        (run / "result.json").write_text(json.dumps(result, indent=2) + "\n")
        print(json.dumps(result, indent=2), flush=True)
        print("Smoke output:", run, flush=True)
    return 0 if result.get("passed") else 1


if __name__ == "__main__":
    raise SystemExit(main())
