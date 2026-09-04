from launch import LaunchDescription
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    description_file = PathJoinSubstitution(
        [FindPackageShare("openarm_description"), "urdf", "openarm_eval_bimanual_right_tesollo.urdf.xacro"]
    )
    robot_description_content = Command(["xacro", " ", description_file])
    robot_description = {
        "robot_description": ParameterValue(
            robot_description_content,
            value_type=str,
        )
    }
    joint_state_defaults = {
        "zeros.openarm_left_joint1": 0.349065850399,
        "zeros.openarm_left_joint2": 0.0,
        "zeros.openarm_left_joint3": 0.0,
        "zeros.openarm_left_joint4": 1.047197551197,
        "zeros.openarm_left_joint5": 0.0,
        "zeros.openarm_left_joint6": 0.0,
        "zeros.openarm_left_joint7": -0.767944870878,
        "zeros.openarm_right_joint1": -0.349065850399,
        "zeros.openarm_right_joint2": 0.698131700798,
        "zeros.openarm_right_joint3": -0.174532925199,
        "zeros.openarm_right_joint4": 1.047197551197,
        "zeros.openarm_right_joint5": 0.523598775598,
        "zeros.openarm_right_joint6": 0.261799387799,
        "zeros.openarm_right_joint7": 0.767944870878,
    }

    return LaunchDescription(
        [
            Node(package="robot_state_publisher", executable="robot_state_publisher", output="screen", parameters=[robot_description]),
            Node(
                package="joint_state_publisher",
                executable="joint_state_publisher",
                output="screen",
                parameters=[robot_description, joint_state_defaults],
            ),
            Node(package="rviz2", executable="rviz2", output="screen"),
        ]
    )
