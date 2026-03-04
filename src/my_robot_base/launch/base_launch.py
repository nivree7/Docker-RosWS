import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    # --- 1. Find Paths ---
    package_share_dir = get_package_share_directory('my_robot_base')
    config_file_path = os.path.join(package_share_dir, 'config', 'base_config.yaml')

    # --- 2. Define Nodes ---
    arduino_node = Node(
        package='my_robot_base',
        executable='arduino_node', # match entry point in setup.py
        name='arduino_node_ros', # match node name in py script & YAML
        output='screen',
        parameters=[config_file_path]
    )

    # --- 3. Return Final Launch Description ---
    return LaunchDescription([
        arduino_node
    ])