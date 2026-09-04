from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
def generate_launch_description():
 b=FindPackageShare('openarm_baseline'); c=PathJoinSubstitution([b,'config','baseline_a.yaml']); f=PathJoinSubstitution([FindPackageShare('openarm_moveit_config'),'launch','demo_fake_execution.launch.py'])
 names=['grasp_debug_dx','grasp_debug_dy','grasp_debug_dz','grasp_debug_roll_deg','grasp_debug_pitch_deg','grasp_debug_yaw_deg']
 params={'config_file': c, **{n: LaunchConfiguration(n) for n in names}}
 actions=[DeclareLaunchArgument('use_rviz', default_value='true')]
 actions += [DeclareLaunchArgument(n,default_value='0.0') for n in names]
 actions += [IncludeLaunchDescription(
   PythonLaunchDescriptionSource(f),
   launch_arguments={'use_rviz': LaunchConfiguration('use_rviz')}.items()),
  Node(package='openarm_baseline',executable='head_camera_tf.py',parameters=[{'config_file':c}]),
  # The calibration node owns the grasp cube; do not show its marker before HOME passes.
  Node(package='openarm_baseline',executable='camera_visual_markers.py',parameters=[{'config_file':c, 'publish_virtual_cube':False}]),
  Node(package='openarm_baseline',executable='baseline_a_grasp_transform_calibration.py',output='screen',parameters=[params]),
  TimerAction(period=8.0,actions=[Node(package='openarm_baseline',executable='baseline_a_calibration_home.py',output='screen',parameters=[{'config_file':c}])]),
  TimerAction(period=9.0,actions=[Node(package='openarm_baseline',executable='baseline_a_contact_grasp.py',output='screen',parameters=[{'config_file':c}])])]
 return LaunchDescription(actions)
