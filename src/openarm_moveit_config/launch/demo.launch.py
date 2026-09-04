import os
import sys

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
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
    rviz_config_path,
    trajectory_execution,
)


def generate_launch_description():
    use_rviz = LaunchConfiguration("use_rviz")
    use_gui = LaunchConfiguration("use_gui")
    allow_trajectory_execution = LaunchConfiguration("allow_trajectory_execution")

    description_parameters = robot_description()
    semantic_parameters = robot_description_semantic()
    kinematics_parameters = robot_description_kinematics()
    planning_parameters = robot_description_planning()
    pipeline_parameters = planning_pipelines()
    joint_state_defaults = {
        "zeros.openarm_left_joint1": 0.349065850399,
        "zeros.openarm_left_joint2": 0.0,
        "zeros.openarm_left_joint3": 0.0,
        "zeros.openarm_left_joint4": 1.047197551197,
        "zeros.openarm_left_joint5": 0.0,
        "zeros.openarm_left_joint6": 0.0,
        "zeros.openarm_left_joint7": -0.767944870878,
        "zeros.openarm_left_finger_joint1": 0.044,
        "zeros.openarm_left_finger_joint2": 0.044,
        "zeros.openarm_right_joint1": -0.349065850399,
        "zeros.openarm_right_joint2": 0.0,
        "zeros.openarm_right_joint3": 0.0,
        "zeros.openarm_right_joint4": 1.047197551197,
        "zeros.openarm_right_joint5": 0.0,
        "zeros.openarm_right_joint6": 0.0,
        "zeros.openarm_right_joint7": 0.767944870878,
        "zeros.openarm_right_finger_joint1": 0.044,
        "zeros.openarm_right_finger_joint2": 0.044,
    }

    move_group_parameters = [
        description_parameters,
        semantic_parameters,
        kinematics_parameters,
        planning_parameters,
        pipeline_parameters,
        trajectory_execution(),
        planning_scene_monitor_parameters(),
        {"allow_trajectory_execution": allow_trajectory_execution},
    ]

    rviz_parameters = [
        description_parameters,
        semantic_parameters,
        kinematics_parameters,
        planning_parameters,
        pipeline_parameters,
    ]

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_rviz",
                default_value="true",
                description="Start RViz with the MoveIt MotionPlanning panel.",
            ),
            DeclareLaunchArgument(
                "use_gui",
                default_value="false",
                description="Start joint_state_publisher_gui instead of joint_state_publisher.",
            ),
            DeclareLaunchArgument(
                "allow_trajectory_execution",
                default_value="false",
                description="Enable trajectory execution through MoveIt controllers.",
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                output="screen",
                parameters=[description_parameters],
            ),
            Node(
                package="joint_state_publisher",
                executable="joint_state_publisher",
                name="joint_state_publisher",
                output="screen",
                condition=UnlessCondition(use_gui),
                parameters=[description_parameters, {"publish_rate": 30.0}, joint_state_defaults],
            ),
            Node(
                package="joint_state_publisher_gui",
                executable="joint_state_publisher_gui",
                name="joint_state_publisher_gui",
                output="screen",
                condition=IfCondition(use_gui),
                parameters=[description_parameters, {"publish_rate": 30.0}, joint_state_defaults],
            ),
            Node(
                package="moveit_ros_move_group",
                executable="move_group",
                name="move_group",
                output="screen",
                parameters=move_group_parameters,
            ),
            Node(
                package="openarm_baseline",
                executable="cube_pnp_scene.py",
                name="table_collision_scene",
                output="screen",
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="log",
                arguments=["-d", rviz_config_path()],
                parameters=rviz_parameters,
                condition=IfCondition(use_rviz),
            ),
        ]
    )
