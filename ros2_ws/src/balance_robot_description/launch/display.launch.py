"""
display.launch.py
Launch robot_state_publisher + rviz2 to visualize the URDF.
Usage:
    ros2 launch balance_robot_description display.launch.py
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('balance_robot_description')
    xacro_file = os.path.join(pkg, 'urdf', 'balance_robot.xacro')

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    robot_description = Command(['xacro ', xacro_file])

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),

        # Publish TF from URDF
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': robot_description,
                'use_sim_time': use_sim_time,
            }],
        ),

        # Joint state publisher GUI (manually move joints)
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
        ),

        # RViz2
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', os.path.join(pkg, 'rviz', 'robot.rviz')],
            output='screen',
        ),
    ])
