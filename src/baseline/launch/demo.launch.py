import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def _moveit_config():
    description_share = get_package_share_directory('piper_tomato_one_arm')
    urdf_path = os.path.join(description_share, 'urdf', 'one_piper_tomato.urdf')

    return (
        MoveItConfigsBuilder(
            'right_piper_tomato',
            package_name='baseline',
        )
        .robot_description(file_path=urdf_path)
        .robot_description_semantic(file_path='config/one_piper_tomato.srdf')
        .robot_description_kinematics(file_path='config/kinematics.yaml')
        .joint_limits(file_path='config/joint_limits.yaml')
        .trajectory_execution(file_path='config/moveit_controllers.yaml')
        .planning_pipelines(default_planning_pipeline='ompl', pipelines=['ompl'])
        .planning_scene_monitor(
            publish_planning_scene=True,
            publish_geometry_updates=True,
            publish_state_updates=True,
            publish_transforms_updates=True,
            publish_robot_description=True,
            publish_robot_description_semantic=True,
        )
        .sensors_3d(file_path='config/sensors_3d.yaml')
        .to_moveit_configs()
    )


def _launch_setup(context, *args, **kwargs):
    moveit_config = _moveit_config()
    config_share = get_package_share_directory('baseline')
    rviz_config = os.path.join(config_share, 'config', 'moveit.rviz')
    use_rviz = LaunchConfiguration('use_rviz')
    use_gui = LaunchConfiguration('use_gui')
    allow_execution = LaunchConfiguration('allow_trajectory_execution')

    move_group_parameters = [
        moveit_config.to_dict(),
        {
            'allow_trajectory_execution': allow_execution,
            'publish_planning_scene': True,
            'publish_geometry_updates': True,
            'publish_state_updates': True,
            'publish_transforms_updates': True,
            'publish_robot_description': True,
            'publish_robot_description_semantic': True,
            'monitor_dynamics': False,
        },
    ]

    return [
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[moveit_config.robot_description],
        ),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            condition=IfCondition(use_gui),
            parameters=[moveit_config.robot_description, {'publish_rate': 50.0}],
        ),
        Node(
            package='moveit_ros_move_group',
            executable='move_group',
            name='move_group',
            output='screen',
            parameters=move_group_parameters,
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='log',
            arguments=['-d', rviz_config],
            parameters=[
                moveit_config.robot_description,
                moveit_config.robot_description_semantic,
                moveit_config.robot_description_kinematics,
                moveit_config.planning_pipelines,
                moveit_config.joint_limits,
            ],
            condition=IfCondition(use_rviz),
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            description='Start RViz with the MoveIt MotionPlanning panel.',
        ),
        DeclareLaunchArgument(
            'use_gui',
            default_value='true',
            description='Start joint_state_publisher_gui for manual state checks.',
        ),
        DeclareLaunchArgument(
            'allow_trajectory_execution',
            default_value='false',
            description='Enable trajectory execution through MoveIt controllers.',
        ),
        OpaqueFunction(function=_launch_setup),
    ])
