import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_name = "wheeled_robot"
    robot_name = "differential_drive_robot"

    # Launch arguments
    world_arg = DeclareLaunchArgument('world', default_value='empty.sdf')
    x_arg = DeclareLaunchArgument('x', default_value='0.0')
    y_arg = DeclareLaunchArgument('y', default_value='0.0')
    z_arg = DeclareLaunchArgument('z', default_value='0.5')
    roll_arg  = DeclareLaunchArgument('roll',  default_value='0.0')
    pitch_arg = DeclareLaunchArgument('pitch', default_value='0.0')
    yaw_arg   = DeclareLaunchArgument('yaw',   default_value='0.0')

    world = LaunchConfiguration('world')
    x = LaunchConfiguration('x')
    y = LaunchConfiguration('y')
    z = LaunchConfiguration('z')
    roll = LaunchConfiguration('roll')
    pitch = LaunchConfiguration('pitch')
    yaw = LaunchConfiguration('yaw')

    # Paths
    xacro_path = os.path.join(get_package_share_directory(pkg_name), 'urdf', 'robot.xacro')
    robot_description = xacro.process_file(xacro_path).toxml()
    bridge_yaml = os.path.join(get_package_share_directory(pkg_name), 'config', 'gz_bridge.yaml')

    # CORRECT: Use PathJoinSubstitution for world path
    world_path = PathJoinSubstitution([
        FindPackageShare(pkg_name),
        'worlds',
        world
    ])

    # Gazebo
    gz_launch = PythonLaunchDescriptionSource(
        os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
    )
    gazebo = IncludeLaunchDescription(
        gz_launch,
        launch_arguments={'gz_args': ['-r -v 4 ', world_path]}.items()
    )

    # Robot state publisher
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description, 'use_sim_time': True}],
        output='screen'
    )

    # Spawn
    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', robot_name,
            '-topic', 'robot_description',
            '-x', x, '-y', y, '-z', z,
            '-R', roll, '-P', pitch, '-Y', yaw
        ],
        output='screen'
    )
    delayed_spawn = TimerAction(period=3.0, actions=[spawn])

    # Bridge
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['--ros-args', '--params-file', bridge_yaml],
        output='screen'
    )

    return LaunchDescription([
        world_arg, x_arg, y_arg, z_arg, roll_arg, pitch_arg, yaw_arg,
        gazebo, rsp, delayed_spawn, bridge
    ])
