"""MoveIt/TF diagnostic only: no trajectory planning or execution node is started."""
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    baseline = FindPackageShare("openarm_baseline")
    config = PathJoinSubstitution([baseline, "config", "baseline_a.yaml"])
    fake = PathJoinSubstitution([FindPackageShare("openarm_moveit_config"), "launch", "demo_fake_execution.launch.py"])
    return LaunchDescription([
        IncludeLaunchDescription(PythonLaunchDescriptionSource(fake), launch_arguments={"use_rviz": "false"}.items()),
        Node(package="openarm_baseline", executable="head_camera_tf.py", parameters=[{"config_file": config}]),
        Node(package="openarm_baseline", executable="camera_visual_markers.py", parameters=[{"config_file": config}]),
        TimerAction(period=8.0, actions=[Node(package="openarm_baseline", executable="baseline_a_free_orientation_diagnostic.py", output="screen", parameters=[{"config_file": config}])]),
    ])
