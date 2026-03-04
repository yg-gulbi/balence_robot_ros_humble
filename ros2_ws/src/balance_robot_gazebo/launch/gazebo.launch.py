"""
gazebo.launch.py
Launches Gazebo with the balance robot.
Control is handled by libgazebo_ros_diff_drive plugin (no ros2_control needed).
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_desc      = get_package_share_directory('balance_robot_description')
    pkg_gazebo    = get_package_share_directory('balance_robot_gazebo')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    world_file = os.path.join(pkg_gazebo, 'worlds', 'empty.world')
    xacro_file = os.path.join(pkg_desc,   'urdf',   'balance_robot.xacro')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    x_pose       = LaunchConfiguration('x_pose',       default='0.0')
    z_pose       = LaunchConfiguration('z_pose',       default='0.12')
    paused       = LaunchConfiguration('paused',        default='false')

    robot_description = ParameterValue(Command(['xacro ', xacro_file]), value_type=str)

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('x_pose',       default_value='0.0'),
        DeclareLaunchArgument('z_pose',       default_value='0.12'),
        DeclareLaunchArgument('paused',        default_value='false'),

        # Gazebo
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')
            ),
            launch_arguments={
                'world':  world_file,
                'paused': paused,
            }.items(),
        ),

        # robot_state_publisher
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

        # Spawn robot
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name='spawn_entity',
            output='screen',
            arguments=[
                '-topic', '/robot_description',
                '-entity', 'balance_robot',
                '-x', x_pose,
                '-z', z_pose,
            ],
        ),
    ])
