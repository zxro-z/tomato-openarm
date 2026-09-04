import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch.substitutions import Command
from launch_ros.parameter_descriptions import ParameterValue


def _load_file(package_name: str, relative_path: str) -> str:
    package_share = get_package_share_directory(package_name)
    absolute_path = os.path.join(package_share, relative_path)
    with open(absolute_path, "r", encoding="utf-8") as file:
        return file.read()


def _load_yaml(package_name: str, relative_path: str):
    return yaml.safe_load(_load_file(package_name, relative_path))


def robot_description(use_mock_hardware: bool = False):
    description_share = get_package_share_directory("openarm_description")
    xacro_path = os.path.join(description_share, "urdf", "openarm_eval_bimanual_right_tesollo.urdf.xacro")
    return {
        "robot_description": ParameterValue(
            Command(["xacro", " ", xacro_path] + ([" ", "use_mock_hardware:=true"] if use_mock_hardware else [])),
            value_type=str,
        )
    }


def robot_description_semantic():
    return {
        "robot_description_semantic": _load_file("openarm_moveit_config", "config/openarm.srdf")
    }


def robot_description_kinematics():
    return {
        "robot_description_kinematics": _load_yaml("openarm_moveit_config", "config/kinematics.yaml")
    }


def robot_description_planning():
    return {
        "robot_description_planning": _load_yaml("openarm_moveit_config", "config/joint_limits.yaml")
    }


def planning_pipelines():
    return {
        "default_planning_pipeline": "ompl",
        "planning_pipelines": ["ompl"],
        "ompl": _load_yaml("openarm_moveit_config", "config/ompl_planning.yaml"),
    }


def trajectory_execution():
    return {
        "moveit_manage_controllers": False,
        "trajectory_execution.allowed_execution_duration_scaling": 1.2,
        "trajectory_execution.allowed_goal_duration_margin": 0.5,
        "trajectory_execution.allowed_start_tolerance": 0.01,
        **_load_yaml("openarm_moveit_config", "config/moveit_controllers.yaml"),
    }


def planning_scene_monitor_parameters():
    return {
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
        "publish_robot_description": True,
        "publish_robot_description_semantic": True,
        "monitor_dynamics": False,
    }


def rviz_config_path() -> str:
    config_share = get_package_share_directory("openarm_moveit_config")
    return os.path.join(config_share, "config", "moveit.rviz")
