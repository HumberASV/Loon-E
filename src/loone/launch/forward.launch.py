"""Bench-test utility launch: run forward_node alone.

Run this in a second terminal ALONGSIDE an already-running bringup.launch.py or
task1.launch.py (thrust_mixer must already be subscribed to /cmd_vel) -- see
forward_node.py's docstring for why this exists.

Usage:
    ros2 launch loone forward.launch.py
    ros2 launch loone forward.launch.py linear_speed:=0.3 duration:=10.0
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    linear_speed = LaunchConfiguration('linear_speed')
    duration = LaunchConfiguration('duration')
    use_sim_time = LaunchConfiguration('use_sim_time')

    forward_node = Node(
        package='loone',
        executable='forward_node',
        name='forward_node',
        output='screen',
        parameters=[{
            'linear_speed': linear_speed,
            'duration': duration,
            'use_sim_time': use_sim_time,
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument('linear_speed', default_value='0.3',
                              description='Forward speed to drive at, m/s.'),
        DeclareLaunchArgument('duration', default_value='10.0',
                              description='Seconds to drive before stopping. <= 0 drives '
                                          'forever (until Ctrl-C).'),
        DeclareLaunchArgument('use_sim_time', default_value='false',
                              description='Use /clock simulated time.'),
        forward_node,
    ])
