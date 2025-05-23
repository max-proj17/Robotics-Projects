from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, TimerAction
from ament_index_python.packages import get_package_share_directory
import os
from launch.substitutions import Command

def generate_launch_description():
    pkg_path = get_package_share_directory('balance_bot')
    xacro_file = os.path.join(pkg_path, 'urdf', 'balance_bot.urdf.xacro')
    gazebo_world_file = os.path.join(pkg_path, 'gazebo', 'plane.world')
    controller_config_file = os.path.join(pkg_path, 'gazebo', 'config', 'controller_config.yaml')

    return LaunchDescription([
        # Start Gazebo paused
        ExecuteProcess(
            cmd=['ros2', 'service', 'call', '/gazebo/pause_physics', 'std_srvs/srv/Empty'],
            output='screen'
        ),
        
        # Launch Gazebo with the specified world
        ExecuteProcess(
            cmd=[
                'gazebo', '--verbose', 
                 '-s', 'libgazebo_ros_factory.so', 
                 '-s', 'libgazebo_ros_init.so',
                 gazebo_world_file],
            output='screen',
            shell=True
        ),

        # Publish the robot description
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[
                {'robot_description': Command([
                'xacro ', xacro_file, ' config_file:=', controller_config_file
                ])}
            ],
            output='screen'
        ),

        # Spawn the robot in Gazebo
        TimerAction(
            period=5.0,
            actions=[
                Node(
                    package='gazebo_ros',
                    executable='spawn_entity.py',
                    arguments=[
                        '-entity', 'balance_bot',
                        '-topic', 'robot_description'
                    ],
                    output='screen'
                )
            ]
        ),

        # Delay the ros2_control_node to ensure robot_description is available
        TimerAction(
            period=8.0,
            actions=[
                Node(
                    package='controller_manager',
                    executable='ros2_control_node',
                    parameters=[controller_config_file],
                    output='screen',
                    emulate_tty=True
                )
            ]
        ),

        # Spawn the joint state broadcaster
        TimerAction(
            period=10.0,
            actions=[
                Node(
                    package='controller_manager',
                    executable='spawner',
                    arguments=['joint_state_broadcaster'],
                    output='screen'
                ),
                Node(
                    package='controller_manager',
                    executable='spawner',
                    arguments=['left_wheel_controller'],
                    output='screen'
                ),
                Node(
                    package='controller_manager',
                    executable='spawner',
                    arguments=['right_wheel_controller'],
                    output='screen'
                )
            ]
        ),
        
        # Launch balance controller node
        TimerAction(
            period=12.0,
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
        
        # Unpause physics once everything is ready
        TimerAction(
            period=20.0,
            actions=[
                ExecuteProcess(
                    cmd=['ros2', 'service', 'call', '/gazebo/unpause_physics', 'std_srvs/srv/Empty'],
                    output='screen'
                )
            ]
        )
    ])