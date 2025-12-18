from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Minimal “system is alive” node. Replace later with real composition.
    heartbeat_node = Node(
        package="heartbeat",
        executable="heartbeat",
        name="heartbeat",
        output="screen",
        parameters=[{
            "rate_hz": 1.0,
            "topic": "/system/heartbeat",
        }],
    )

    return LaunchDescription([heartbeat_node])
