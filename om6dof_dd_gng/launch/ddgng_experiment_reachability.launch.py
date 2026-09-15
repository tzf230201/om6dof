"""YOLO/DD-GNG environment + recorded V2 workspace nodes + RViz target GUI."""
import csv
import hashlib
import io
import json
from pathlib import Path
import shutil
import tempfile

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            OpaqueFunction, RegisterEventHandler, SetEnvironmentVariable)
from launch.event_handlers import OnShutdown
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


DEFAULT_DATASET = (Path.home() / 'ros2_ws/src/om6dof/experiments/cartesian_workspace/results/'
                   'comparison_v1_v2_25mm_20260909/v2/points.csv')


def prepare_config(base_path, dataset_path, directory, sample_selection='green',
                   center_candidates=12, exact_replan_budget=80, final_grasp=False):
    """Freeze selected witnesses for this launch; change only reachability settings."""
    if sample_selection not in ('green', 'all'):
        raise ValueError('sample_selection must be green or all')
    # Keep the argument accepted for saved commands.  Centre mode now produces
    # exactly one physical target reference for each object component.
    del center_candidates
    exact_replan_budget = int(exact_replan_budget)
    if exact_replan_budget < 1:
        raise ValueError('exact_replan_budget must be positive')
    source = Path(dataset_path).expanduser().resolve(strict=True)
    data = source.read_bytes()
    source_sha256 = hashlib.sha256(data).hexdigest()
    reader = csv.DictReader(io.StringIO(data.decode('utf-8')))
    required = {'x_mm', 'y_mm', 'z_mm', 'position_found'} | {f'q{i}_rad' for i in range(1, 7)}
    if not required.issubset(reader.fieldnames or []):
        raise ValueError('Dataset must contain XYZ and q1_rad through q6_rad')
    if sample_selection == 'green':
        if not {'status', 'pose_found'}.issubset(reader.fieldnames):
            raise ValueError('Green selection requires status and pose_found columns')
        # The experiment uses pose_found status for green; pose_near_singular
        # is blue even though its numeric pose_found flag is also 1.
        rows = [row for row in reader if row['status'] == 'pose_found'
                and row['pose_found'] == '1' and row['position_found'] == '1']
        output = io.StringIO(newline='')
        writer = csv.DictWriter(output, fieldnames=reader.fieldnames, lineterminator='\n')
        writer.writeheader()
        writer.writerows(rows)
        data = output.getvalue().encode('utf-8')
        witnesses = len(rows)
    else:
        witnesses = sum(row['position_found'] == '1' for row in reader)
    if not witnesses:
        raise ValueError(f'Dataset has no {sample_selection} joint configurations')
    directory = Path(directory)
    snapshot = directory / 'workspace_samples.csv'
    snapshot.write_bytes(data)
    snapshot_sha256 = hashlib.sha256(data).hexdigest()
    (directory / 'workspace_samples_manifest.json').write_text(json.dumps({
        'source_path': str(source), 'source_sha256': source_sha256,
        'sample_selection': sample_selection, 'selected_witnesses': witnesses,
        'snapshot_sha256': snapshot_sha256,
    }, indent=2) + '\n')
    document = yaml.safe_load(Path(base_path).read_text())
    parameters = document['reachability_graph_node']['ros__parameters']
    parameters.update({
        'graph_method': 'workspace_samples',
        'workspace_samples_file': str(snapshot),
        'workspace_samples_sha256': snapshot_sha256,
        'sample_count': witnesses,
        'target_refinement_enabled': False,
        'gng_debug_publish_training_samples': False,
        'exact_collision_enabled': True,
        'include_target_in_collision': True,
        # Keep the earlier graph-only profile. Pickup explicitly uses TCP +Z:
        # physical forward in the V2 CAD, called gripper-forward X by teleop.
        # Only pickup excludes its selected object component from robot contact
        # checks; other objects and robot self-collision remain checked.
        'pregrasp_filter_enabled': True,
        'final_grasp_enabled': bool(final_grasp),
        'pregrasp_refinement_enabled': bool(final_grasp),
        # In pickup the full robot mesh validates every graph/bridge segment,
        # matching the final insertion and execution validator. Conservative
        # capsule envelopes remain diagnostic; they do not veto mesh-clear poses.
        'capsule_collision_veto': not bool(final_grasp),
        'exclude_selected_target_from_collision': bool(final_grasp),
        'pregrasp_tool_approach_axis': [0.0, 0.0, 1.0] if final_grasp else [1.0, 0.0, 0.0],
        'pregrasp_min_standoff_m': 0.07,
        'pregrasp_max_standoff_m': 0.13,
        'pregrasp_min_alignment': 0.95 if final_grasp else 0.70,
        'grasp_gripper_open': 0.019,
        'grasp_gripper_close': -0.010,
        'grasp_tcp_to_pinch': [0.0, 0.0, 0.0],
        'grasp_joint_velocity': 0.15,
        # One physical grasp reference: the 3-D centre of the selected semantic
        # component.  Its representative node ID is retained only for tracking.
        'target_node_selection': 'component_center',
        # This only permits more failed candidates to be rejected. It never
        # accepts a collision and leaves all collision radii unchanged.
        'exact_max_replans': exact_replan_budget,
        # At the folded ready pose link2 and link6 make a verified mechanical
        # touch. FCL reports coplanar triangle contact as collision, so only
        # this explicit pair is allowed; all other non-adjacent pairs remain
        # strict collision checks.
        'allowed_self_collision_pairs': ['link2:link6'],
    })
    config = directory / 'experiment_topology.yaml'
    config.write_text(yaml.safe_dump(document, sort_keys=False))
    return config


def setup(context):
    share = Path(get_package_share_directory('om6dof_dd_gng'))
    pick_share = Path(get_package_share_directory('om6dof_pick_and_place'))
    directory = tempfile.mkdtemp(prefix='ddgng_experiment_')
    pickup = LaunchConfiguration('pickup').perform(context).lower() == 'true'
    try:
        config = prepare_config(share / 'config/topo_gng_v2.yaml',
                                LaunchConfiguration('workspace_samples_file').perform(context), directory,
                                LaunchConfiguration('sample_selection').perform(context),
                                LaunchConfiguration('component_center_candidates').perform(context),
                                LaunchConfiguration('exact_replan_budget').perform(context),
                                final_grasp=pickup)
        pick_config = pick_share / 'config/graph_pick.yaml'
        if pickup:
            pick_document = yaml.safe_load(pick_config.read_text())
            pick_document['graph_pick']['ros__parameters'].update({
                'task_mode': 'pickup', 'tool_approach_axis': [0.0, 0.0, 1.0],
                'grasp_tcp_to_pinch': [0.0, 0.0, 0.0],
                'selected_target_excluded_from_collision': True,
                'minimum_approach_alignment': 0.95,
                'max_start_joint_error_rad': 0.02,
                'gripper_open': 0.019, 'gripper_close': -0.010,
                'retreat_after_grasp': False,
            })
            pick_config = Path(directory) / 'graph_grasp.yaml'
            pick_config.write_text(yaml.safe_dump(pick_document, sort_keys=False))
    except Exception:
        shutil.rmtree(directory)
        raise

    def cleanup(_context):
        shutil.rmtree(directory, ignore_errors=True)
        return []

    return [
        RegisterEventHandler(OnShutdown(on_shutdown=[OpaqueFunction(function=cleanup)])),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(share / 'launch/topo_gng_v2.launch.py')),
            launch_arguments={
                'params_file': str(config),
                'launch_reachability': 'true',
                'launch_rviz': LaunchConfiguration('launch_rviz'),
                'camera_calibration_file': LaunchConfiguration('camera_calibration_file'),
            }.items()),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(pick_share / 'launch/graph_pick.launch.py')),
            launch_arguments={
                'config_file': str(pick_config),
                'execution_enabled': LaunchConfiguration('execution_enabled'),
                'rmw_implementation': LaunchConfiguration('rmw_implementation'),
            }.items()),
        Node(package='om6dof_dd_gng', executable='semantic_target_gui.py',
             name='semantic_target_gui', output='screen'),
        # Same RGB-only viewer as yolox_ddgng_analysis.launch.py.
        Node(package='om6dof_dd_gng', executable='yolox_viewer_node',
             name='yolox_2d_viewer', output='screen',
             parameters=[{
                 'input_topic': '/om6dof_topo_gng_v2/rgb/image/compressed',
                 'output_topic': '/om6dof_yolox/debug_image/compressed',
                 'display_window': True,
             }],
             condition=IfCondition(LaunchConfiguration('launch_yolo_2d'))),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('workspace_samples_file', default_value=str(DEFAULT_DATASET)),
        DeclareLaunchArgument('sample_selection', default_value='green', choices=['green', 'all'],
                             description='Green experiment poses only, or all position-found witnesses'),
        DeclareLaunchArgument('component_center_candidates', default_value='1',
                             description='Deprecated: component-centre mode uses one target per object'),
        DeclareLaunchArgument('exact_replan_budget', default_value='80',
                             description='Collision-safe candidate rejections allowed before reporting no route'),
        DeclareLaunchArgument('pickup', default_value='false', choices=['true', 'false'],
                             description='Append validated front insertion and explicit gripper closing'),
        DeclareLaunchArgument('camera_calibration_file', default_value=''),
        DeclareLaunchArgument('execution_enabled', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('launch_rviz', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('launch_yolo_2d', default_value='true', choices=['true', 'false'],
                             description='Show the existing C++ YOLO RGB viewer alongside RViz'),
        DeclareLaunchArgument('rmw_implementation', default_value='rmw_fastrtps_cpp'),
        SetEnvironmentVariable('RMW_IMPLEMENTATION', LaunchConfiguration('rmw_implementation')),
        OpaqueFunction(function=setup),
    ])
