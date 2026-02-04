#!/usr/bin/env python3
"""Launch LiDAR 3D detection fusion with camera image classification."""

import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _load_calibration(calibration_file: str) -> dict:
    defaults = {
        'translation': {'x': 0.0, 'y': 0.0, 'z': 0.1},
        'rotation': {'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0},
        'parent_frame': 'os_sensor',
    'child_frame': 'zed_camera_link',
    }

    if not calibration_file or not os.path.exists(calibration_file):
        return defaults

    with open(calibration_file, 'r', encoding='utf-8') as handle:
        data = yaml.safe_load(handle) or {}

    transform = data.get('camera_lidar_transform', {})
    return {
        'translation': transform.get('translation', defaults['translation']),
        'rotation': transform.get('rotation', defaults['rotation']),
        'parent_frame': transform.get('parent_frame', defaults['parent_frame']),
        'child_frame': transform.get('child_frame', defaults['child_frame']),
    }


def _build_static_tf(context):
    calibration_file = LaunchConfiguration('calibration_file').perform(context)
    calibration = _load_calibration(calibration_file)

    translation = calibration['translation']
    rotation = calibration['rotation']

    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='lidar_to_camera_tf',
        arguments=[
            str(translation['x']),
            str(translation['y']),
            str(translation['z']),
            str(rotation['roll']),
            str(rotation['pitch']),
            str(rotation['yaw']),
            calibration['parent_frame'],
            calibration['child_frame'],
        ],
        condition=IfCondition(LaunchConfiguration('enable_static_tf')),
        output='log',
    )

    return [static_tf]


def _build_base_tf(context):
    base_frame = LaunchConfiguration('base_frame').perform(context)
    lidar_frame = LaunchConfiguration('lidar_frame').perform(context)

    base_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_lidar_tf',
        arguments=[
            '0.0', '0.0', '0.0',
            '0.0', '0.0', '0.0',
            base_frame,
            lidar_frame,
        ],
        condition=IfCondition(LaunchConfiguration('enable_base_tf')),
        output='log',
    )

    return [base_tf]


def generate_launch_description():
    params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')

    package_share = get_package_share_directory('camera_gated_clustering')
    default_calibration = os.path.join(package_share, 'config', 'camera_lidar_calibration.yaml')
    zed_share = get_package_share_directory('zed_wrapper')
    zed_launch = os.path.join(zed_share, 'launch', 'zed_camera.launch.py')
    rviz_config = os.path.abspath(
        os.path.join(os.path.dirname(__file__), '..', 'config', 'sensor_fusion.rviz')
    )

    fastdds_profile = os.path.join(package_share, 'config', 'fastdds_no_shm.xml')

    return LaunchDescription([
        SetEnvironmentVariable(
            name='RMW_FASTRTPS_SHARED_MEMORY',
            value='0',
        ),
        SetEnvironmentVariable(
            name='FASTRTPS_DEFAULT_PROFILES_FILE',
            value=fastdds_profile,
        ),
        DeclareLaunchArgument(
            'params_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('camera_gated_clustering'),
                'config',
                'fusion_params.yaml',
            ]),
            description='Path to fusion parameter YAML.',
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time',
        ),
        DeclareLaunchArgument(
            'launch_zed',
            default_value='true',
            description='Launch ZED camera driver from this launch file',
        ),
        DeclareLaunchArgument(
            'camera_model',
            default_value='zed',
            description='ZED camera model (zed, zedm, zed2, zed2i, zedx, zedxm, virtual, zedxonegs, zedxone4k)',
        ),
        DeclareLaunchArgument(
            'enable_viz',
            default_value='true',
            description='Launch RViz overlay visualization',
        ),
        DeclareLaunchArgument(
            'enable_static_tf',
            default_value='true',
            description='Publish static LiDAR->camera TF from calibration file',
        ),
        DeclareLaunchArgument(
            'enable_base_tf',
            default_value='true',
            description='Publish base_link -> lidar static TF (zero transform)',
        ),
        DeclareLaunchArgument(
            'base_frame',
            default_value='base_link',
            description='Base frame for base_link -> lidar static TF',
        ),
        DeclareLaunchArgument(
            'lidar_frame',
            default_value='os_sensor',
            description='LiDAR frame for base_link -> lidar static TF',
        ),
        DeclareLaunchArgument(
            'calibration_file',
            default_value=default_calibration,
            description='Path to camera_lidar_calibration.yaml',
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(zed_launch),
            launch_arguments={
                'camera_model': LaunchConfiguration('camera_model'),
                'publish_tf': 'false',
                'publish_map_tf': 'false',
            }.items(),
            condition=IfCondition(LaunchConfiguration('launch_zed')),
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            condition=IfCondition(LaunchConfiguration('enable_viz')),
            output='log',
        ),
        OpaqueFunction(function=_build_static_tf),
        OpaqueFunction(function=_build_base_tf),
        Node(
            package='camera_gated_clustering',
            executable='lidar_camera_fusion_node.py',
            name='lidar_camera_fusion_node',
            parameters=[params_file, {'use_sim_time': use_sim_time}],
            output='screen',
            emulate_tty=True,
        ),
    ])
