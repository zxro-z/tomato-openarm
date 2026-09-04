import os
import sys

from launch import LaunchDescription
from launch_ros.actions import Node

sys.path.append(os.path.dirname(__file__))

from _moveit_config import robot_description


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                output="screen",
                parameters=[robot_description()],
            ),
        ]
    )
