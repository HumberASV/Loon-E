"""Bench-test utility launch: run spin_node alone.

Run this in a second terminal ALONGSIDE an already-running bringup.launch.py or
task1.launch.py (thrust_mixer must already be subscribed to /cmd_vel) -- see
spin_node.py's docstring for why this exists.

Usage:
    ros2 launch loone spin.launch.py
    ros2 launch loone spin.launch.py angular_speed:=0.3 duration:=20.0
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    angular_speed = LaunchConfiguration('angular_speed')
    duration = LaunchConfiguration('duration')
    use_sim_time = LaunchConfiguration('use_sim_time')

    spin_node = Node(
        package='loone',
        executable='spin_node',
        name='spin_node',
        output='screen',
        parameters=[{
            'angular_speed': angular_speed,
            'duration': duration,
            'use_sim_time': use_sim_time,
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument('angular_speed', default_value='0.3',
                              description='Yaw rate to spin at, rad/s.'),
        DeclareLaunchArgument('duration', default_value='20.0',
                              description='Seconds to spin before stopping. <= 0 spins '
                                          'forever (until Ctrl-C).'),
        DeclareLaunchArgument('use_sim_time', default_value='false',
                              description='Use /clock simulated time.'),
        spin_node,
    ])
