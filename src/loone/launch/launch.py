from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.conditions import UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

import os

def generate_launch_description():
    camera_name = LaunchConfiguration('camera_name')
    camera_model = LaunchConfiguration('camera_model')
    zed_node_name = LaunchConfiguration('zed_node_name')
    use_sim_time = LaunchConfiguration('use_sim_time')
    sim = LaunchConfiguration('sim')
     
    loone_share = get_package_share_directory('loone')
    # gps_waypoints_json = LaunchConfiguration('gps_waypoints_json')
    # gps_waypoints_file = LaunchConfiguration('gps_waypoints_file')


    # # Placeholder course near Humber College -- swap gps_waypoints_file for the real
    # # Task 1 course coordinates before running on the water.
    # default_waypoints_file = os.path.join(loone_share, 'config', 'task1_waypoints.json')

    slam_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(loone_share, 'launch', 'slam_launch.py')),
            launch_arguments={
                'camera_name': camera_name,
                'camera_model': camera_model,
                'zed_node_name': zed_node_name,
                'use_sim_time': use_sim_time,
            }.items()
        )

    phone = Node(
            package='loone',
            executable='phone',
            name='phone',
            output='screen',
            condition=UnlessCondition(sim),
        )

    battery_node = Node(
            package='loone',
            executable='battery_node',
            name='battery_node',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            condition=UnlessCondition(sim),
        )

    task = Node(
        package='loone',
        executable='task',
        name='task',
    )

    motor =  Node(
        package='loone',
        executable='motor',
        name='motor',
    )

    path_planning = Node(
        package='loone',
        executable='path_planning',
        name='path_planning',
    )

    # list of above inorder of importance
    return LaunchDescription([
        DeclareLaunchArgument('camera_name', default_value='zedx',
                                description='ZED camera name / namespace (sets the <name>_camera_link frame).'),
        DeclareLaunchArgument('camera_model', default_value='zedx',
                                description='ZED camera model passed to the wrapper.'),
        DeclareLaunchArgument('zed_node_name', default_value='zed_node',
                                description='ZED wrapper node name inside the camera namespace.'),
        DeclareLaunchArgument('use_sim_time', default_value='false',
                                    description='Use /clock simulated time. Keep false on the real boat.'),
        DeclareLaunchArgument('sim', default_value='false',
                                    description='Simulation mode: swap busio_node for sim_state_echo '
                                                '(Isaac Sim drives the actuators).'),
        motor,
        task,
        path_planning,
        phone,
        battery_node,
        slam_launch,
    ])