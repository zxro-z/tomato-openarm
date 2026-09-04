from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            Node(
                package="openarm_baseline",
                executable="cube_pnp_scene.py",
                output="screen",
            )
        ]
    )
