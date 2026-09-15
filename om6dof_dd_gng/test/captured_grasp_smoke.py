#!/usr/bin/env python3
"""Replay the captured joint-jump failure with the installed grasp planner.

Source ROS and the workspace first. Only synthetic sensors and the planner run
on localhost ROS domain 219. There are no camera/controller nodes or motor goals.
"""
import json
import math
from pathlib import Path
import subprocess
import sys


def main():
    fixture = (Path(__file__).resolve().parents[1] / 'docs/validation/'
               'adaptive_grasp_20260915/current_scene')
    replay = subprocess.run([
        sys.executable, str(fixture / 'replay_recipe.py'),
        '--capture-dir', str(fixture / 'fixture'),
        '--base-params', str(fixture / 'params.yaml'),
        '--pregrasp-refinement-enabled', 'true', '--seconds', '35',
    ], text=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    result_paths = [line.removeprefix('Result: ').strip()
                    for line in replay.stdout.splitlines() if line.startswith('Result: ')]
    if replay.returncode or not result_paths:
        raise RuntimeError(replay.stdout)
    result_path = Path(result_paths[-1])
    result = json.loads(result_path.read_text())
    plan = result.get('frozen_validated_plan', {})
    assert result['complete'] and plan.get('valid'), (result_path, result.get('error'))
    assert plan['grasp_approach_valid'] and plan['exact_collision_valid']
    assert plan['selected_target_excluded_from_collision']
    assert plan['graph_method'] == 'workspace_samples'
    checks = result['execution_validation_checks']
    assert len(checks) == 3 and all(check['valid'] for check in checks), checks
    successful = [item for item in result['diagnostics'] if item['plan_valid']]
    assert successful and all(item['capsule_collision_veto'] is False for item in successful)
    assert all(abs(item['clearance_m'] - 0.035) < 1e-9 for item in successful)
    assert any(item['capsule_mesh_clear_goals'] > 0 for item in successful)
    target = plan['grasp_target_position']
    endpoint = plan['end_effector_path']['poses'][-1]['pose']['position']
    error = math.dist([target[axis] for axis in 'xyz'], [endpoint[axis] for axis in 'xyz'])
    assert error <= 0.003 and plan['grasp_position_error'] <= 0.003
    # Large graph transitions are timed trajectories; the new Cartesian tail
    # must retain its separate per-waypoint joint-step limit.
    points = plan['joint_path_preview']['points']
    anchor_index = len(plan['reachability_node_ids'])
    assert plan['pregrasp_waypoint_count'] > anchor_index
    for before, after in zip(points[anchor_index:], points[anchor_index + 1:]):
        assert max(abs(a - b) for a, b in zip(before['positions'], after['positions'])) <= 0.15
    print(json.dumps({'passed': True, 'result': str(result_path),
                      'waypoints': len(points), 'endpoint_error_m': error,
                      'validated_stages': [item['stage'] for item in checks]}, indent=2))


if __name__ == '__main__':
    main()
