import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node

def generate_launch_description():

    # --- 1. Find Paths ---
    my_robot_description_share_dir = get_package_share_directory('my_robot_description')
    my_robot_localization_share_dir = get_package_share_directory('my_robot_localization')
    my_robot_navigation_share_dir = get_package_share_directory('my_robot_navigation')

    # --- 2. Define Launch Arguments

    # --- 3. Include Core Launch Files ---
    rsp_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            my_robot_description_share_dir, 'launch', 'rsp.launch.py'
        )]),
        launch_arguments={'use_sim_time': 'false', 'use_ros2_control': 'true'}.items()
    )

    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            my_robot_localization_share_dir, 'launch', 'gps_localization_launch.py'
        )]),
        launch_arguments = {'use_sim_time': 'false'}.items()
    )

    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            my_robot_navigation_share_dir, 'launch', 'nav2.launch.py'
        )]),
        launch_arguments = {'use_sim_time': 'false'}.items()
    )
    

    # --- 4. Define Nodes

    # --- 5. Return Final Launch Description ---
    return LaunchDescription([
        rsp_launch,
        localization_launch,
        navigation_launch
    ])