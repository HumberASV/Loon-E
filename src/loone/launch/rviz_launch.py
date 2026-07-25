"""Opens RViz2 pre-configured for the loone/zedx bringup stack.

Loads rviz/loone.rviz -- a copy of the stock zed_display_rviz2 stereo config
with every hardcoded '/zed/...' topic repointed at '/zedx/...' to match the
camera_name used by bringup.launch.py / slam_launch.py. Run bringup first;
this only opens the viewer, it does not start the camera itself.

Usage:
    ros2 launch loone rviz_launch.py
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    rviz_config = os.path.join(
        get_package_share_directory('loone'), 'rviz', 'loone.rviz')

    return LaunchDescription([
        Node(
            package='rviz2',
            executable='rviz2',
            name='loone_rviz2',
            output='screen',
            arguments=['-d', rviz_config],
        ),
    ])
