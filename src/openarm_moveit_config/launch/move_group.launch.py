import os
import sys

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

sys.path.append(os.path.dirname(__file__))

from _moveit_config import (
    planning_pipelines,
    planning_scene_monitor_parameters,
    robot_description,
    robot_description_kinematics,
    robot_description_planning,
    robot_description_semantic,
    trajectory_execution,
)


def generate_launch_description():
    allow_trajectory_execution = LaunchConfiguration("allow_trajectory_execution")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "allow_trajectory_execution",
                default_value="false",
                description="Enable trajectory execution through MoveIt controllers.",
            ),
            Node(
                package="moveit_ros_move_group",
                executable="move_group",
                name="move_group",
                output="screen",
                parameters=[
                    robot_description(),
                    robot_description_semantic(),
                    robot_description_kinematics(),
                    robot_description_planning(),
                    planning_pipelines(),
                    trajectory_execution(),
                    planning_scene_monitor_parameters(),
                    {"allow_trajectory_execution": allow_trajectory_execution},
                ],
            ),
        ]
    )
