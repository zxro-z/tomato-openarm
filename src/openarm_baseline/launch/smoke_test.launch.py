from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="openarm_baseline",
                executable="openarm_moveit_smoke_test.py",
                output="screen",
            )
        ]
    )
