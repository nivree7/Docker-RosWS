#!/usr/bin/env python3
"""Launch camera-gated LiDAR clustering with optional Ouster driver and visualization."""

import lifecycle_msgs.msg
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, EmitEvent, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.parameter_descriptions import ParameterValue
from launch.events import matches_action
from launch_ros.actions import Node, LifecycleNode
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # Declare arguments
    args = [
        DeclareLaunchArgument(
            'sensor_hostname',
            default_value='os-992209000435.local',
            description='Ouster LiDAR hostname or IP'
        ),
        DeclareLaunchArgument(
            'lidar_mode',
            default_value='',
            description='Ouster LiDAR mode (e.g., 512x10, 1024x10, 2048x10)'
        ),
        DeclareLaunchArgument(
            'launch_ouster',
            default_value='false',
            description='Launch Ouster driver from this file'
        ),
        DeclareLaunchArgument(
            'ouster_ns',
            default_value='ouster',
            description='Namespace for Ouster nodes'
        ),
        DeclareLaunchArgument(
            'azimuth_window_start',
            default_value='135000',
            description='Azimuth window start in millidegrees'
        ),
        DeclareLaunchArgument(
            'azimuth_window_end',
            default_value='225000',
            description='Azimuth window end in millidegrees'
        ),
        DeclareLaunchArgument(
            'camera_model',
            default_value='zed2',
            description='ZED camera model: zed, zedm, zed2, zed2i, zedx, zedxm'
        ),
        DeclareLaunchArgument(
            'gating_mode',
            default_value='soft',
            description='Gating mode: hard, soft, confidence, distance_adaptive'
        ),
        DeclareLaunchArgument(
            'use_pcl_clustering',
            default_value='true',
            description='Use PCL KD-tree Euclidean clustering node'
        ),
        DeclareLaunchArgument(
            'enable_viz',
            default_value='true',
            description='Launch RViz visualization'
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time'
        ),
        DeclareLaunchArgument(
            'params_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('camera_gated_clustering'),
                'config',
                'fusion_params.yaml'
            ]),
            description='Path to fusion parameters file'
        ),
    ]

    # Optional Ouster driver with azimuth window controls
    os_driver = LifecycleNode(
        package='ouster_ros',
        executable='os_driver',
        name='os_driver',
        namespace=LaunchConfiguration('ouster_ns'),
        parameters=[{
            'sensor_hostname': LaunchConfiguration('sensor_hostname'),
            'lidar_mode': LaunchConfiguration('lidar_mode'),
            'sensor_frame': 'os_sensor',
            'lidar_frame': 'os_lidar',
            'imu_frame': 'os_imu',
            'point_type': 'original',
            'timestamp_mode': 'TIME_FROM_INTERNAL_OSC',
            'proc_mask': 'IMU|IMG|PCL|SCAN',
            'azimuth_window_start': ParameterValue(
                LaunchConfiguration('azimuth_window_start'), value_type=int
            ),
            'azimuth_window_end': ParameterValue(
                LaunchConfiguration('azimuth_window_end'), value_type=int
            ),
        }],
        output='screen',
        condition=IfCondition(LaunchConfiguration('launch_ouster')),
    )

    driver_configure = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_action(os_driver),
            transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
        ),
        condition=IfCondition(LaunchConfiguration('launch_ouster')),
    )

    driver_activate = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=os_driver,
            goal_state='inactive',
            entities=[
                LogInfo(msg='Ouster driver activating...'),
                EmitEvent(event=ChangeState(
                    lifecycle_node_matcher=matches_action(os_driver),
                    transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE,
                )),
            ],
            handle_once=True
        ),
        condition=IfCondition(LaunchConfiguration('launch_ouster')),
    )

    # Static transforms (camera-lidar calibration)
    # Load calibration from YAML
    transforms_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='lidar_to_camera_tf',
        arguments=[
            '0.0', '0.0', '0.1',  # x, y, z (placeholder values)
            '0', '0', '0',         # roll, pitch, yaw
            'os_sensor',           # parent frame
            'zed_camera_center'    # child frame
        ],
        output='log'
    )

    # Base to LiDAR transform
    base_to_lidar_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_lidar_tf',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'os_sensor'],
        output='log'
    )

    # Camera-gated clustering node
    fusion_node = Node(
        package='camera_gated_clustering',
        executable='camera_gated_clustering_node.py',
        name='camera_gated_clustering_node',
        parameters=[
            LaunchConfiguration('params_file'),
            {
                'gating.mode': LaunchConfiguration('gating_mode'),
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }
        ],
        output='screen',
        emulate_tty=True
    )

    pcl_clustering_node = Node(
        package='camera_gated_clustering',
        executable='pcl_euclidean_clustering_node',
        name='pcl_euclidean_clustering_node',
        parameters=[LaunchConfiguration('params_file')],
        condition=IfCondition(LaunchConfiguration('use_pcl_clustering')),
        output='screen'
    )

    # RViz (optional)
    rviz_config = PathJoinSubstitution([
        FindPackageShare('camera_gated_clustering'),
        'config',
        'sensor_fusion.rviz'
    ])

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        condition=IfCondition(LaunchConfiguration('enable_viz')),
        output='log'
    )
    
    return LaunchDescription([
        *args,
        os_driver,
        driver_configure,
        driver_activate,
        base_to_lidar_tf,
        transforms_node,
        pcl_clustering_node,
        fusion_node,
        rviz_node,
    ])
