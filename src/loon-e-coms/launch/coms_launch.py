from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='loon-e-coms',
            namespace='loon-e',
            executable='coms',
            name='communications_node',
            output='screen'
        ),
        Node(
            package='loon-e-control',
            namespace='loon-e',
            executable='task_node',
            name='task_logic_node',
            output='screen'
        ),
        Node(
            package='loon-e-map',
            namespace='loon-e',
            executable='map_publisher',
            name='mapping_node',
            output='screen'
        ),
        Node(
            package='loon-e-planning',
            namespace='loon-e',
            executable='path_planning',
            name='path_planner_node',
            output='screen'
        ),
    ])