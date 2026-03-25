import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    # --- 1. Find Paths ---
    package_share_dir = get_package_share_directory('my_robot_teleop')
    xbox_config_file_path = os.path.join(package_share_dir, 'config', 'xbox_teleop_config.yaml')

    # --- 2. Define Nodes ---

    # Define teleop node for keyboard
    # Don't run in launch, run only as ros2 run teleop_twist_keyboard teleop_twist_keyboard
    #keyboard_node = Node(
    #    package='teleop_twist_keyboard',
    #    executable='teleop_twist_keyboard',
    #    name='teleop_keyboard',
    #    parameters=[{
    #        'speed': 0.5,      # Initial linear speed
    #        'turn': 0.5,       # Initial angular speed
    #        'repeat_rate': 0.0 # 0.0 means only send when key is pressed
    #    }],
    #    output='screen',
    #    #prefix='xterm -e' # Open in a new terminal window so it can capture keys
    #)

    # Define Joy node (hardware driver for joystick/controller)
    joy_driver_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        parameters=[{
            'dev': '/dev/input/js0',  # Std Linux Joystick port TODO: Double check this is correct port
            'deadzone': 0.1,          # Prevent minor stick drift registering
            'autorepeat_rate': 20.0,
        }]
    )

    # Define teleop node for joystick
    xbox_node = Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        name='teleop_twist_joy_node',
        parameters=[xbox_config_file_path]
    )

    # --- 3. Return Final Launch Description ---
    return LaunchDescription([
        #keyboard_node,
        joy_driver_node,
        xbox_node,
    ])