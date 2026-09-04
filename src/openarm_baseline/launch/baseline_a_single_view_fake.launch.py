"""Start A0 against MoveIt's fake GenericSystem and show it in RViz.

Usage:
  ros2 launch openarm_baseline baseline_a_single_view_fake.launch.py
  ros2 launch openarm_baseline baseline_a_single_view_fake.launch.py observation_distance_m:=0.30
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config = PathJoinSubstitution([FindPackageShare("openarm_baseline"), "config", "baseline_a.yaml"])
    fake = PathJoinSubstitution([FindPackageShare("openarm_moveit_config"), "launch", "demo_fake_execution.launch.py"])
    distance = LaunchConfiguration("observation_distance_m")
    hold = LaunchConfiguration("observation_hold_sec")
    use_rviz = LaunchConfiguration("use_rviz")
    diagnostic_distance = LaunchConfiguration("diagnostic_distance_m")
    return LaunchDescription([
        DeclareLaunchArgument("observation_distance_m", default_value="0.25"),
        DeclareLaunchArgument("observation_hold_sec", default_value="2.0"),
        DeclareLaunchArgument("use_rviz", default_value="true"),
        DeclareLaunchArgument("diagnostic_distance_m", default_value="-1.0"),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(fake), launch_arguments={"use_rviz": use_rviz}.items()),
        Node(package="openarm_baseline", executable="head_camera_tf.py", output="screen",
             parameters=[{"config_file": config}]),
        Node(package="openarm_baseline", executable="camera_visual_markers.py", output="screen",
             parameters=[{"config_file": config}]),
        # Wait for controller spawners, MoveIt services, and /joint_states.
        TimerAction(period=8.0, actions=[Node(
            package="openarm_baseline", executable="baseline_a_single_view.py", output="screen",
            parameters=[{"config_file": config,
                         "observation_distance_m": distance,
                         "observation_hold_sec": hold,
                         "diagnostic_distance_m": diagnostic_distance}])]),
    ])
