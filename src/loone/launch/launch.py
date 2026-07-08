from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(package='loone', executable='phone', name='phone'),
        Node(package='loone', executable='task', name='task'),
        Node(package='loone', executable='motor', name='motor'),
    ])