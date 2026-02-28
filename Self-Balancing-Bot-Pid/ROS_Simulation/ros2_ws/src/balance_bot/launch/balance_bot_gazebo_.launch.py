from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, TimerAction
from ament_index_python.packages import get_package_share_directory
import os
import xacro

def generate_launch_description():
    pkg_path = get_package_share_directory('balance_bot')
    xacro_file = os.path.join(pkg_path, 'urdf', 'balance_bot.urdf.xacro')
    gazebo_world_file = os.path.join(pkg_path, 'gazebo', 'plane.world')
    controller_config_file = os.path.join(pkg_path, 'gazebo', 'config', 'controller_config.yaml')

    # Process xacro via Python API
    doc = xacro.process_file(xacro_file, mappings={'config_file': controller_config_file})
    robot_desc = doc.toxml()

    # gazebo_ros2_control (Humble) internally re-spawns a node and passes
    # robot_description as a --param CLI argument.  rcl's argument parser
    # breaks on both the <?xml header AND on newlines (it treats them as
    # argument boundaries).  Fix both by stripping the header and collapsing
    # to a single line.
    if robot_desc.startswith('<?xml'):
        robot_desc = robot_desc[robot_desc.index('?>') + 2:]
    robot_desc = robot_desc.replace('\n', '').replace('\r', '').strip()

    spawn_z = '0.45'

    return LaunchDescription([

        ExecuteProcess(
            cmd=[
                'gazebo', '--verbose',
                '-s', 'libgazebo_ros_factory.so',
                '-s', 'libgazebo_ros_init.so',
                gazebo_world_file
            ],
            output='screen',
            shell=True
        ),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{'robot_description': robot_desc}],
            output='screen'
        ),

        TimerAction(
            period=5.0,
            actions=[
                Node(
                    package='gazebo_ros',
                    executable='spawn_entity.py',
                    arguments=[
                        '-entity', 'balance_bot',
                        '-topic', 'robot_description',
                        '-z', spawn_z
                    ],
                    output='screen'
                )
            ]
        ),

        TimerAction(
            period=8.0,
            actions=[
                Node(
                    package='controller_manager',
                    executable='spawner',
                    arguments=['joint_state_broadcaster',
                               '--controller-manager-timeout', '60'],
                    output='screen'
                ),
                Node(
                    package='controller_manager',
                    executable='spawner',
                    arguments=['left_wheel_controller',
                               '--controller-manager-timeout', '60'],
                    output='screen'
                ),
                Node(
                    package='controller_manager',
                    executable='spawner',
                    arguments=['right_wheel_controller',
                               '--controller-manager-timeout', '60'],
                    output='screen'
                )
            ]
        ),

        TimerAction(
            period=20.0,
            actions=[
                Node(
                    package='balance_bot',
                    executable='balance_bot',
                    name='balance_controller',
                    parameters=[{'use_sim_time': True}],
                    output='screen'
                )
            ]
        ),
    ])