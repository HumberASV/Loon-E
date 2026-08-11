from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():
    
    loone_share = get_package_share_directory('loone')

    config = os.path.join(loone_share, 'config', 'config.yaml')

    task = Node(
        package='loone',
        executable='task',
        name='task',
        output='screen',
        parameters=[config, {'use_sim_time': use_sim_time}],
    )

    return LaunchDescription([ 