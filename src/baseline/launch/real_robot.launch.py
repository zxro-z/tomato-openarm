import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


os.environ['RCUTILS_COLORIZED_OUTPUT'] = '1'


def generate_launch_description():
    can_port = LaunchConfiguration('can_port')
    auto_enable = LaunchConfiguration('auto_enable')
    gripper_exist = LaunchConfiguration('gripper_exist')
    gripper_val_mutiple = LaunchConfiguration('gripper_val_mutiple')
    start_driver = LaunchConfiguration('start_driver')
    use_rviz = LaunchConfiguration('use_rviz')
    command_speed_percent = LaunchConfiguration('command_speed_percent')
    command_rate_hz = LaunchConfiguration('command_rate_hz')
    joint_signs = LaunchConfiguration('joint_signs')
    log_level = LaunchConfiguration('log_level')

    piper_driver = Node(
        package='piper',
        executable='piper_single_ctrl',
        name='piper_ctrl_single_node',
        output='screen',
        ros_arguments=['--log-level', log_level],
        parameters=[{
            'can_port': can_port,
            'auto_enable': auto_enable,
            'gripper_exist': gripper_exist,
            'gripper_val_mutiple': gripper_val_mutiple,
        }],
        remappings=[
            ('joint_ctrl_single', '/joint_ctrl_cmd'),
            ('joint_states_single', '/joint_states_piper'),
            ('joint_states_feedback', '/joint_feedback_piper'),
            ('joint_ctrl', '/joint_states_ctrl_piper'),
        ],
        condition=IfCondition(start_driver),
    )

    moveit_demo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('baseline'),
                'launch',
                'demo.launch.py',
            ])
        ),
        launch_arguments={
            'use_rviz': use_rviz,
            'use_gui': 'false',
            'allow_trajectory_execution': 'true',
        }.items(),
    )

    bridge = Node(
        package='piper_dual_arm_moveit_bridge',
        executable='single_arm_moveit_bridge',
        name='piper_single_arm_moveit_bridge',
        output='screen',
        parameters=[{
            'state_topic': '/joint_states_piper',
            'command_topic': '/joint_ctrl_cmd',
            'moveit_joint_prefix': 'right_',
            'joint_signs': PythonExpression(['[float(x) for x in "', joint_signs, '".split(",")]']),
            'command_speed_percent': command_speed_percent,
            'command_rate_hz': command_rate_hz,
            'require_feedback': True,
            'goal_tolerance_rad': 0.05,
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'can_port',
            default_value='can0',
            description='CAN interface for the real Piper arm.',
        ),
        DeclareLaunchArgument(
            'auto_enable',
            default_value='false',
            description='Automatically enable the Piper arm when the driver starts.',
        ),
        DeclareLaunchArgument(
            'gripper_exist',
            default_value='true',
            description='Whether the real Piper arm has a gripper.',
        ),
        DeclareLaunchArgument(
            'gripper_val_mutiple',
            default_value='1',
            description='Piper driver gripper multiplier parameter.',
        ),
        DeclareLaunchArgument(
            'start_driver',
            default_value='true',
            description='Start the real Piper CAN driver in this launch.',
        ),
        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            description='Start RViz with the MoveIt MotionPlanning panel.',
        ),
        DeclareLaunchArgument(
            'command_speed_percent',
            default_value='10.0',
            description='Piper joint command speed percentage sent with MoveIt waypoints.',
        ),
        DeclareLaunchArgument(
            'command_rate_hz',
            default_value='50.0',
            description='Interpolated Piper joint command publish rate.',
        ),
        DeclareLaunchArgument(
            'joint_signs',
            default_value='1,1,1,1,1,1',
            description='Signs mapping driver joints to MoveIt joints: joint1,...,joint6.',
        ),
        DeclareLaunchArgument(
            'log_level',
            default_value='info',
            description='Logging level for the Piper driver.',
        ),
        piper_driver,
        bridge,
        moveit_demo,
    ])
