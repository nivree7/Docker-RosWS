import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, Command
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

import xacro

def generate_launch_description():

    # --- 1. Define Launch Configurations ---
    # Check if we're told to use sim time.
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_ros2_control = LaunchConfiguration('use_ros2_control')
    
    # --- 2. Find Paths ---
    my_robot_description_share_dir = get_package_share_directory('my_robot_description')

    xacro_file = os.path.join(
        my_robot_description_share_dir, 'xacro', 'robot.urdf.xacro'  # Constructs full path to robots urdf file
        )

    # robot_description_config = xacro.process_file(xacro_file).toxml()
    robot_description_config = Command(['xacro ', xacro_file, ' use_ros2_control:=', use_ros2_control, ' sim_mode:=', use_sim_time])

    # --- 3. Define Nodes ---
    # --- Robot State Publisher Node ---
    params = {'robot_description': robot_description_config, 'use_sim_time': use_sim_time}
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    # --- 4. Return Final Launch Description ---
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use sim time if true'
        ),
        DeclareLaunchArgument(
            'use_ros2_control',
            default_value='true',
            description='Use ros2_control if true'
        ),
        rsp_node
    ])
