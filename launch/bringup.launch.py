import os

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('test_dd_bot')
    xacro_file = os.path.join(pkg_share, 'description', 'robot.urdf.xacro')
    robot_description = xacro.process_file(xacro_file).toxml()

    controllers_yaml = os.path.join(pkg_share, 'config', 'controllers.yaml')
    hardware_yaml = os.path.join(pkg_share, 'config', 'hardware.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time')

    controller_manager_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        name='controller_manager',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            controllers_yaml,
            hardware_yaml,
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation clock if true.'
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description, 'use_sim_time': use_sim_time}],
        ),
        controller_manager_node,
        RegisterEventHandler(
            OnProcessStart(
                target_action=controller_manager_node,
                on_start=[
                    Node(
                        package='controller_manager',
                        executable='spawner',
                        name='spawner_joint_state_broadcaster',
                        arguments=['joint_state_broadcaster'],
                        output='screen',
                    ),
                    Node(
                        package='controller_manager',
                        executable='spawner',
                        name='spawner_diff_drive_controller',
                        arguments=['diff_drive_controller'],
                        output='screen',
                    ),
                ],
            )
        ),
        Node(
            package='test_dd_bot',
            executable='demo_motion_node',
            name='demo_motion_node',
            output='screen',
        ),
    ])
