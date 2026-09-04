"""OpenArm-only RViz smoke test: body, two arms, TF, and joint GUI."""

from launch import LaunchDescription
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_share = FindPackageShare("openarm_description")
    canonical_xacro = PathJoinSubstitution(
        [package_share, "urdf", "openarm_eval_bimanual_right_tesollo.urdf.xacro"]
    )
    filter_script = PathJoinSubstitution(
        [package_share, "launch", "openarm_only_robot_description.py"]
    )
    rviz_config = PathJoinSubstitution(
        [package_share, "rviz", "openarm_only_smoke_test.rviz"]
    )
    robot_description = {
        "robot_description": ParameterValue(
            Command([FindExecutable(name="python3"), " ", filter_script, " ", canonical_xacro]),
            value_type=str,
        )
    }

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
                parameters=[robot_description],
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="screen",
                arguments=["-d", rviz_config],
            ),
        ]
    )
