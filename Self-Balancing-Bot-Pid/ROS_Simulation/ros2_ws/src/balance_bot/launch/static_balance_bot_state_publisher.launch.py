from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node
from launch.substitutions import Command
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_path = get_package_share_directory('balance_bot')
    xacro_file = os.path.join(pkg_path, 'urdf', 'balance_bot.urdf.xacro')

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{'robot_description': Command(['xacro ', xacro_file])}],
        output='screen'
    )

    joint_state_publisher_node = TimerAction(
        period=2.0,
        actions=[
            Node(
                package='joint_state_publisher_gui',
                executable='joint_state_publisher_gui',
                name='joint_state_publisher',
                output='screen'
            )
        ]
    )

    # rviz_config_file = os.path.join(pkg_path, 'rviz', 'balance_bot.rviz')
    # rviz_node = Node(
    #     package='rviz2',
    #     executable='rviz2',
    #     name='rviz2',
    #     arguments=['-d', rviz_config_file],
    #     output='screen'
    # )

    return LaunchDescription([
        robot_state_publisher_node,
        joint_state_publisher_node,
        # rviz_node
    ])

