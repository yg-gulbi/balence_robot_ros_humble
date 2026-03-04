"""
sim.launch.py
1. Gazebo starts PAUSED (physics frozen)
2. Controller node starts (0.5s)
3. Gazebo unpauses → controller is ready from the first physics tick
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription,
    TimerAction, ExecuteProcess,
)
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
        DeclareLaunchArgument('use_rviz',     default_value='false'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),

        # 1. Gazebo — start PAUSED so robot doesn't fall before controller is ready
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_gazebo, 'launch', 'gazebo.launch.py')
            ),
            launch_arguments={'paused': 'true'}.items(),
        ),

        # 2. Balance controller starts first (physics still paused)
        TimerAction(
            period=2.0,
            actions=[
                Node(
                    package='balance_robot_controller',
                    executable='balance_controller_node',
                    name='balance_controller',
                    output='screen',
                    parameters=[pid_params, {'use_sim_time': use_sim_time}],
                ),
            ]
        ),

        # 3. Unpause Gazebo after controller is ready
        TimerAction(
            period=3.0,
            actions=[
                ExecuteProcess(
                    cmd=['ros2', 'service', 'call',
                         '/unpause_physics', 'std_srvs/srv/Empty', '{}'],
                    output='screen',
                ),
            ]
        ),

        # 4. RViz2 (optional)
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
