import os
import sys

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

sys.path.append(os.path.dirname(__file__))

from _moveit_config import (
    planning_pipelines,
    robot_description,
    robot_description_kinematics,
    robot_description_planning,
    robot_description_semantic,
    rviz_config_path,
)


def generate_launch_description():
    use_rviz = LaunchConfiguration("use_rviz")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_rviz",
                default_value="true",
                description="Start RViz with the MoveIt MotionPlanning panel.",
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="log",
                arguments=["-d", rviz_config_path()],
                parameters=[
                    robot_description(),
                    robot_description_semantic(),
                    robot_description_kinematics(),
                    robot_description_planning(),
                    planning_pipelines(),
                ],
                condition=IfCondition(use_rviz),
            ),
        ]
    )
