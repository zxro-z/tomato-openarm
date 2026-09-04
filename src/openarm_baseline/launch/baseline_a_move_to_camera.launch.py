"""Fake Baseline-A Move: attached cube top face -> head-camera optical centre."""
import os
import sys

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription, LogInfo,
                            RegisterEventHandler, TimerAction)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessIO
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


# Reuse the same robot-description factories as the fake MoveIt launch.  The
# Baseline-A launch owns RViz startup so it can defer RobotModel creation until
# the audit has received the first complete /joint_states message.
_moveit_launch_dir = os.path.join(get_package_share_directory('openarm_moveit_config'), 'launch')
if _moveit_launch_dir not in sys.path:
    sys.path.append(_moveit_launch_dir)
from _moveit_config import (planning_pipelines, robot_description,
                            robot_description_kinematics,
                            robot_description_planning,
                            robot_description_semantic)


def generate_launch_description():
    baseline = FindPackageShare('openarm_baseline')
    config = PathJoinSubstitution([baseline, 'config', 'baseline_a.yaml'])
    rviz_config = PathJoinSubstitution([baseline, 'rviz', 'baseline_a_move.rviz'])
    fake = PathJoinSubstitution([FindPackageShare('openarm_moveit_config'), 'launch', 'demo_fake_execution.launch.py'])
    description = robot_description(use_mock_hardware=True)
    rviz = Node(
        package='rviz2', executable='rviz2', name='rviz2', output='log',
        arguments=['-d', LaunchConfiguration('rviz_config')],
        parameters=[description, robot_description_semantic(),
                    robot_description_kinematics(), robot_description_planning(),
                    planning_pipelines()],
        condition=IfCondition(LaunchConfiguration('use_rviz')))
    audit = Node(package='openarm_baseline', executable='baseline_a_startup_audit.py', output='screen')
    rviz_started = {'value': False}

    def start_rviz_after_first_joint_state(event):
        # The audit line is emitted exactly once, only after all 14 arm and
        # four gripper joints are present.  This avoids the zero-pose RViz
        # flash without delaying or commanding the robot.
        if (not rviz_started['value'] and
                b'STARTUP_AUDIT_FIRST_JOINT_STATE' in event.text):
            rviz_started['value'] = True
            return [LogInfo(msg='BASELINE_A_RVIZ_GATE: first complete /joint_states received; starting RViz'), rviz]
        return []

    return LaunchDescription([
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('return_home', default_value='false'),
        DeclareLaunchArgument('rviz_config', default_value=rviz_config,
                              description='Baseline-A RViz display configuration.'),
        # Passive recorder starts before controller startup.  It never sends
        # motion commands; its log is the evidence for the initial-state gate.
        audit,
        IncludeLaunchDescription(PythonLaunchDescriptionSource(fake), launch_arguments={
            # RViz is launched by the OnProcessIO gate below, after the first
            # complete HOME joint state; the generic fake launch keeps its
            # normal immediate-RViz behavior for every other launch.
            'use_rviz': 'false'}.items()),
        RegisterEventHandler(OnProcessIO(
            # rclpy INFO logs normally use stderr; observe both streams so
            # the gate is independent of the user's ROS logging settings.
            target_action=audit, on_stdout=start_rviz_after_first_joint_state,
            on_stderr=start_rviz_after_first_joint_state)),
        Node(package='openarm_baseline', executable='head_camera_tf.py', parameters=[{'config_file': config}]),
        Node(package='openarm_baseline', executable='camera_visual_markers.py', parameters=[
            {'config_file': config, 'publish_virtual_cube': False,
             'publish_head_optical_axis': False,
             'publish_wrist_optical_axis': False}]),
        Node(package='openarm_baseline', executable='baseline_a_grasp_transform_calibration.py', output='screen', parameters=[{'config_file': config}]),
        TimerAction(period=8.0, actions=[Node(package='openarm_baseline', executable='baseline_a_calibration_home.py', output='screen', parameters=[{'config_file': config}])]),
        TimerAction(period=9.0, actions=[Node(package='openarm_baseline', executable='baseline_a_contact_grasp.py', output='screen', parameters=[{'config_file': config}])]),
        # The fake controller starts at canonical HOME.  Add the table only
        # after HOME validation/grasp setup, then require it for the
        # attached-cube planning run below.
        TimerAction(period=25.0, actions=[Node(package='openarm_baseline', executable='cube_pnp_scene.py', output='screen')]),
        # The Move node also waits for the durable attached-cube-ready signal.
        TimerAction(period=27.0, actions=[Node(package='openarm_baseline', executable='baseline_a_move_to_camera.py', output='screen', parameters=[
            {'config_file': config, 'return_home': LaunchConfiguration('return_home')}])]),
    ])
