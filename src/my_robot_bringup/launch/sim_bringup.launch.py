import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node

def generate_launch_description():

    # --- 1. Find Paths ---
    my_robot_description_share_dir = get_package_share_directory('my_robot_description')
    my_robot_gazebo_share_dir = get_package_share_directory('my_robot_gazebo')

    # --- 2. Define Launch Arguments

    # --- 3. Include Core Launch Files ---
    rsp_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            my_robot_description_share_dir, 'launch', 'rsp.launch.py'
            )]),
        launch_arguments={'use_sim_time': 'true', 'use_ros2_control': 'true'}.items()
    )

    my_robot_gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            my_robot_gazebo_share_dir, 'launch', 'gazebo.launch.py'
        )])
    )

    # --- 4. Define Nodes

    # --- 5. Return Final Launch Description ---
    return LaunchDescription([
        rsp_launch,
        my_robot_gazebo_launch
    ])