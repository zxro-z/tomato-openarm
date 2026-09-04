import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

os.environ['RCUTILS_COLORIZED_OUTPUT'] = '1'

def generate_launch_description():
    use_rviz = LaunchConfiguration('use_rviz')

    # MoveIt 데모 및 RViz 환경 로드 (가상 제어기 및 액션 서버 내장)
    moveit_demo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('piper_tomato_one_arm_moveit_config'),
                'launch',
                'demo.launch.py',
            ])
        ),
        launch_arguments={
            'use_rviz': use_rviz,
            'use_gui': 'false',
            'allow_trajectory_execution': 'true',
            'fake_execution': 'true',  # 가상 실행 모드 강제 활성화
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            description='Start RViz with the MoveIt MotionPlanning panel.',
        ),
        moveit_demo,
    ])
