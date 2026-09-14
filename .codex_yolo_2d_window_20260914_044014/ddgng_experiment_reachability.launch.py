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
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


DEFAULT_DATASET = (Path.home() / 'ros2_ws/src/om6dof/experiments/cartesian_workspace/results/'
                   'comparison_v1_v2_25mm_20260909/v2/points.csv')


def prepare_config(base_path, dataset_path, directory, sample_selection='green'):
    """Freeze selected witnesses for this launch; change only reachability settings."""
    if sample_selection not in ('green', 'all'):
        raise ValueError('sample_selection must be green or all')
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
        'target_node_selection': 'component_center',
    })
    config = directory / 'experiment_topology.yaml'
    config.write_text(yaml.safe_dump(document, sort_keys=False))
    return config


def setup(context):
    share = Path(get_package_share_directory('om6dof_dd_gng'))
    pick_share = Path(get_package_share_directory('om6dof_pick_and_place'))
    directory = tempfile.mkdtemp(prefix='ddgng_experiment_')
    try:
        config = prepare_config(share / 'config/topo_gng_v2.yaml',
                                LaunchConfiguration('workspace_samples_file').perform(context), directory,
                                LaunchConfiguration('sample_selection').perform(context))
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
                'execution_enabled': LaunchConfiguration('execution_enabled'),
                'rmw_implementation': LaunchConfiguration('rmw_implementation'),
            }.items()),
        Node(package='om6dof_dd_gng', executable='semantic_target_gui.py',
             name='semantic_target_gui', output='screen'),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('workspace_samples_file', default_value=str(DEFAULT_DATASET)),
        DeclareLaunchArgument('sample_selection', default_value='green', choices=['green', 'all'],
                             description='Green experiment poses only, or all position-found witnesses'),
        DeclareLaunchArgument('camera_calibration_file', default_value=''),
        DeclareLaunchArgument('execution_enabled', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('launch_rviz', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('rmw_implementation', default_value='rmw_fastrtps_cpp'),
        SetEnvironmentVariable('RMW_IMPLEMENTATION', LaunchConfiguration('rmw_implementation')),
        OpaqueFunction(function=setup),
    ])
