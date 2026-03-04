"""
sim.launch.py  — One-shot launch for full simulation

Starts:
  1. Gazebo (world + robot spawn + ros2_control)
  2. Balance controller node (with PID params)

Usage:
    ros2 launch balance_robot_bringup sim.launch.py
    ros2 launch balance_robot_bringup sim.launch.py use_rviz:=true
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_bringup = get_package_share_directory('balance_robot_bringup')
    pkg_gazebo  = get_package_share_directory('balance_robot_gazebo')
    pkg_desc    = get_package_share_directory('balance_robot_description')

    pid_params  = os.path.join(pkg_bringup, 'config', 'pid_params.yaml')

    use_rviz     = LaunchConfiguration('use_rviz',     default='false')
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    return LaunchDescription([
        DeclareLaunchArgument('use_rviz',     default_value='false',
                              description='Launch RViz2 alongside Gazebo'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),

        # --- 1. Gazebo simulation ---
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_gazebo, 'launch', 'gazebo.launch.py')
            ),
        ),

        # --- 2. Balance controller (delayed 3s to let Gazebo fully start) ---
        TimerAction(
            period=3.0,
            actions=[
                Node(
                    package='balance_robot_controller',
                    executable='balance_controller_node',
                    name='balance_controller',
                    output='screen',
                    parameters=[pid_params, {'use_sim_time': use_sim_time}],
                    # Enable debug logging:
                    # arguments=['--ros-args', '--log-level', 'debug'],
                ),
            ]
        ),

        # --- 3. RViz2 (optional) ---
        TimerAction(
            period=2.0,
            actions=[
                Node(
                    condition=IfCondition(use_rviz),
                    package='rviz2',
                    executable='rviz2',
                    name='rviz2',
                    arguments=['-d', os.path.join(pkg_desc, 'rviz', 'robot.rviz')],
                    parameters=[{'use_sim_time': use_sim_time}],
                    output='screen',
                ),
            ]
        ),
    ])
