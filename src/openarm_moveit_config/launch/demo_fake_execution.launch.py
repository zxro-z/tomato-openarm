"""Demo-only RViz execution using mock_components/GenericSystem; no hardware I/O."""
import os
import sys
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

sys.path.append(os.path.dirname(__file__))

from _moveit_config import (planning_pipelines, planning_scene_monitor_parameters,
                            robot_description, robot_description_kinematics,
                            robot_description_planning, robot_description_semantic,
                            rviz_config_path, trajectory_execution)


def generate_launch_description():
    use_rviz = LaunchConfiguration('use_rviz')
    rviz_config = LaunchConfiguration('rviz_config')
    description = robot_description(use_mock_hardware=True)
    moveit_params = [description, robot_description_semantic(), robot_description_kinematics(),
                     robot_description_planning(), planning_pipelines(), trajectory_execution(),
                     planning_scene_monitor_parameters(), {'allow_trajectory_execution': True}]
    controllers = PathJoinSubstitution([FindPackageShare('openarm_moveit_config'), 'config', 'ros2_controllers_fake.yaml'])
    return LaunchDescription([
        DeclareLaunchArgument('use_rviz', default_value='true'),
        # Keep the general MoveIt configuration as the default.  Scenario
        # launches can opt into a dedicated RViz display-only configuration.
        DeclareLaunchArgument('rviz_config', default_value=rviz_config_path()),
        Node(package='robot_state_publisher', executable='robot_state_publisher', name='robot_state_publisher', output='screen', parameters=[description]),
        # Do not remap this executable's node name.  A process-wide __node remap
        # would also rename the dynamically loaded controller lifecycle nodes,
        # preventing their left_arm_controller/right_arm_controller YAML sections
        # from being applied.
        Node(package='controller_manager', executable='ros2_control_node', output='screen', parameters=[description, controllers]),
        Node(package='controller_manager', executable='spawner', arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'], output='screen'),
        Node(package='controller_manager', executable='spawner', arguments=['left_arm_controller', '--controller-manager', '/controller_manager'], output='screen'),
        Node(package='controller_manager', executable='spawner', arguments=['right_arm_controller', '--controller-manager', '/controller_manager'], output='screen'),
        Node(package='controller_manager', executable='spawner', arguments=['left_gripper_controller', '--controller-manager', '/controller_manager'], output='screen'),
        Node(package='controller_manager', executable='spawner', arguments=['right_gripper_controller', '--controller-manager', '/controller_manager'], output='screen'),
        Node(package='moveit_ros_move_group', executable='move_group', name='move_group', output='screen', parameters=moveit_params),
        Node(package='rviz2', executable='rviz2', name='rviz2', output='log', arguments=['-d', rviz_config], parameters=[description, robot_description_semantic(), robot_description_kinematics(), robot_description_planning(), planning_pipelines()], condition=IfCondition(use_rviz)),
    ])
