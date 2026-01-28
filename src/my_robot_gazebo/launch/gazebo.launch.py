import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node

def generate_launch_description():
    

    # --- 1. Find Paths ---
    my_robot_gazebo_share_dir = get_package_share_directory('my_robot_gazebo')

    world_file_path = os.path.expanduser('~/.gazebo/models/terrain/model.sdf')

    gazebo_params_file = os.path.join(
        my_robot_gazebo_share_dir, 'config', 'gazebo_params.yaml'
        )
        
    # --- 2. Define Launch Arguments ---
    # use_ros2_control = LaunchConfiguration('use_ros2_control')

    # --- 3. Launch Gazebo ---
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py'
            )]),
        launch_arguments={
        	'extra_gazebo_args': '--ros-args --params-file ' + gazebo_params_file,
        	'world': world_file_path	
        }.items()
    )

    # --- 4. Define Nodes ---
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'my_bot'],
        output='screen'
    )

    # spawn_world = Node(
    #    package='gazebo_ros',
    #    executable='spawn_entity.py',
    #    arguments=[
    #        '-file', os.path.expanduser('~/.gazebo/models/terrain/model.sdf'), 
    #        '-entity', 'terrain', 
    #        '-x', '0', 
    #        '-y', '0', 
    #        '-z', '0'
    #        ],
    #    output='screen'
    #)

    diff_drive_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['diff_cont'],
    )

    joint_broad_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_broad'],
    )

    twist_stamper = Node(
        package='twist_stamper',
        executable='twist_stamper',
        parameters=[{'use_sim_time': True}],
        remappings=[('/cmd_vel_in', '/cmd_vel'), 
                    ('/cmd_vel_out', '/diff_cont/cmd_vel')]
    )

# --- 5. Return Final Launch Description ---
    return LaunchDescription([
        gazebo_launch, 
        spawn_entity,
        #spawn_world,
        diff_drive_spawner,
        joint_broad_spawner,
        twist_stamper
    ])
