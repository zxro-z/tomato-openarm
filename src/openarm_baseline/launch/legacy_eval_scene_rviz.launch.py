"""RViz reference scene matching the legacy LeRobot OpenArm evaluator."""

import math
from pathlib import Path

import yaml
from launch import LaunchDescription
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    baseline_share = FindPackageShare("openarm_baseline")
    description_share = FindPackageShare("openarm_description")
    config_path = Path(__file__).resolve().parents[1] / "config" / "legacy_eval_scene.yaml"
    with config_path.open(encoding="utf-8") as stream:
        config = yaml.safe_load(stream)

    canonical_xacro = PathJoinSubstitution(
        [description_share, "urdf", "openarm_eval_bimanual_right_tesollo.urdf.xacro"]
    )
    filter_script = PathJoinSubstitution(
        [description_share, "launch", "openarm_legacy_eval_robot_description.py"]
    )
    robot_description = {
        "robot_description": ParameterValue(
            Command([FindExecutable(name="python3"), " ", filter_script, " ", canonical_xacro]),
            value_type=str,
        )
    }
    zeros = {}
    for side, values in (("left", config["robot"]["initial_left_joints_deg"]), ("right", config["robot"]["initial_right_joints_deg"])):
        zeros.update({f"zeros.openarm_{side}_joint{index}": math.radians(value) for index, value in enumerate(values, 1)})

    rviz_config = PathJoinSubstitution([baseline_share, "rviz", "legacy_eval_scene.rviz"])
    installed_config = PathJoinSubstitution([baseline_share, "config", "legacy_eval_scene.yaml"])
    return LaunchDescription(
        [
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                output="screen",
                parameters=[robot_description],
            ),
            Node(
                package="joint_state_publisher_gui",
                executable="joint_state_publisher_gui",
                name="joint_state_publisher_gui",
                output="screen",
                parameters=[robot_description, zeros],
            ),
            Node(
                package="openarm_baseline",
                executable="legacy_eval_scene.py",
                name="legacy_eval_scene",
                output="screen",
                parameters=[{"config_file": installed_config}],
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="screen",
                arguments=["-d", rviz_config],
                parameters=[robot_description],
            ),
        ]
    )
