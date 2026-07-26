"""Task 1 (Maneuvering and Path-Finding) bringup: bringup.launch.py + a GPS-waypoint mission.

Includes bringup.launch.py unmodified (SLAM/ZED, controller_manager, thrust_mixer,
busio_node, navsat_transform, nav2, etc. -- see that file's docstring for the full
chain) and adds one more node on top:

  11. gps_waypoint_mission - reads a JSON list of GPS points (inline or from a file),
                              converts them via navsat_transform's /fromLL service, and
                              sends them to nav2's follow_waypoints action.

This is the MVP for Task 1: it gets the boat through a fixed list of GPS points using
Nav2's stock waypoint_follower. It does NOT yet know about buoys, cardinal marks, or
the "Otter" obstacle -- that smarter, vision-aware planning is future work tracked in
TODO.md ("Later": loone_msgs + a custom Nav2 planner).

Usage:
    ros2 launch loone task1.launch.py
    ros2 launch loone task1.launch.py gps_waypoints_file:=/path/to/my_course.json
    ros2 launch loone task1.launch.py gps_waypoints_json:='[{"lat":43.65,"lon":-79.38}]'
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    loone_share = get_package_share_directory('loone')

    camera_name = LaunchConfiguration('camera_name')
    camera_model = LaunchConfiguration('camera_model')
    zed_node_name = LaunchConfiguration('zed_node_name')
    use_sim_time = LaunchConfiguration('use_sim_time')
    sim = LaunchConfiguration('sim')
    gps_waypoints_json = LaunchConfiguration('gps_waypoints_json')
    gps_waypoints_file = LaunchConfiguration('gps_waypoints_file')

    # Placeholder course near Humber College -- swap gps_waypoints_file for the real
    # Task 1 course coordinates before running on the water.
    default_waypoints_file = os.path.join(loone_share, 'config', 'task1_waypoints.json')

    bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(loone_share, 'launch', 'bringup.launch.py')),
        launch_arguments={
            'camera_name': camera_name,
            'camera_model': camera_model,
            'zed_node_name': zed_node_name,
            'use_sim_time': use_sim_time,
            'sim': sim,
        }.items()
    )

    # 11. GPS-waypoint mission runner (see gps_waypoint_mission.py). gps_waypoints_json
    #     takes precedence over gps_waypoints_file when non-empty.
    gps_waypoint_mission = Node(
        package='loone',
        executable='gps_waypoint_mission',
        name='gps_waypoint_mission',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'gps_waypoints_json': gps_waypoints_json,
            'gps_waypoints_file': gps_waypoints_file,
        }],
    )

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
        DeclareLaunchArgument('gps_waypoints_json', default_value='',
                              description='Inline JSON list of GPS points, e.g. '
                                          '[{"lat":43.65,"lon":-79.38}]. Takes precedence over '
                                          'gps_waypoints_file when non-empty.'),
        DeclareLaunchArgument('gps_waypoints_file', default_value=default_waypoints_file,
                              description='Path to a JSON file of GPS points (same schema as '
                                          'gps_waypoints_json). Defaults to the placeholder '
                                          'course in config/task1_waypoints.json.'),
        bringup_launch,
        gps_waypoint_mission,
    ])
