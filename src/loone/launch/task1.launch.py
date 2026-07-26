"""Task 1 (Maneuvering and Path-Finding) bringup: bringup.launch.py + a spin-in-place
warmup + a GPS-waypoint mission.

Includes bringup.launch.py unmodified (SLAM/ZED, controller_manager, thrust_mixer,
busio_node, navsat_transform, nav2, etc. -- see that file's docstring for the full
chain) and adds two more steps on top, run in sequence:

  11. spin_node (delayed `spin_delay` seconds, see spin_node.py) - spins the boat in
      place for `spin_duration` seconds so SLAM Toolbox's map fills in all the way
      around the boat's starting position before any goal is sent. Without this, the
      ZED's forward-facing depth camera only ever maps a narrow wedge, the robot's own
      start pose can end up outside that map ("Robot is out of bounds of the
      costmap!"), and nav2's planner then fails ComputePathToPose forever without ever
      publishing /cmd_vel. Skip with spin_before_mission:=false (e.g. if the map is
      already built from a previous run).
  12. gps_waypoint_mission (started once spin_node exits) - reads a JSON list of GPS
      points (inline or from a file), converts them via navsat_transform's /fromLL
      service, and sends them to nav2's follow_waypoints action.

This is the MVP for Task 1: it gets the boat through a fixed list of GPS points using
Nav2's stock waypoint_follower. It does NOT yet know about buoys, cardinal marks, or
the "Otter" obstacle -- that smarter, vision-aware planning is future work tracked in
TODO.md ("Later": loone_msgs + a custom Nav2 planner).

Usage:
    ros2 launch loone task1.launch.py
    ros2 launch loone task1.launch.py gps_waypoints_file:=/path/to/my_course.json
    ros2 launch loone task1.launch.py gps_waypoints_json:='[{"lat":43.65,"lon":-79.38}]'
    ros2 launch loone task1.launch.py spin_before_mission:=false
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
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
    spin_before_mission = LaunchConfiguration('spin_before_mission')
    spin_delay = LaunchConfiguration('spin_delay')
    spin_duration = LaunchConfiguration('spin_duration')
    spin_angular_speed = LaunchConfiguration('spin_angular_speed')

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

    # 11a. Spin-in-place warmup (see spin_node.py / spin.launch.py). Delayed so the
    #      ZED + SLAM Toolbox stack from bringup_launch has time to come up first --
    #      spinning before SLAM is even receiving scans wouldn't build any map.
    spin_node = Node(
        package='loone',
        executable='spin_node',
        name='spin_node',
        output='screen',
        parameters=[{
            'angular_speed': spin_angular_speed,
            'duration': spin_duration,
            'use_sim_time': use_sim_time,
        }],
        condition=IfCondition(spin_before_mission),
    )
    delayed_spin = TimerAction(period=spin_delay, actions=[spin_node])

    # Start the mission once spin_node exits (spin_node self-terminates after
    # `spin_duration`, see spin_node.py's main()).
    mission_after_spin = RegisterEventHandler(
        OnProcessExit(target_action=spin_node, on_exit=[gps_waypoint_mission]),
        condition=IfCondition(spin_before_mission),
    )
    # spin_before_mission:=false skips the warmup and starts the mission immediately,
    # same as before this launch file added the spin step.
    mission_immediately = TimerAction(
        period=0.0, actions=[gps_waypoint_mission], condition=UnlessCondition(spin_before_mission))

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
        DeclareLaunchArgument('spin_before_mission', default_value='true',
                              description='Spin in place before sending the mission so SLAM '
                                          "builds a map around the boat's start pose. Set false "
                                          'to send the mission immediately instead (e.g. the map '
                                          'is already built from a previous run).'),
        DeclareLaunchArgument('spin_delay', default_value='15.0',
                              description='Seconds to wait after bringup before spinning, to '
                                          'give the ZED + SLAM Toolbox stack time to come up.'),
        DeclareLaunchArgument('spin_duration', default_value='20.0',
                              description='Seconds to spin in place for.'),
        DeclareLaunchArgument('spin_angular_speed', default_value='0.3',
                              description='Yaw rate to spin at, rad/s.'),
        bringup_launch,
        delayed_spin,
        mission_after_spin,
        mission_immediately,
    ])
