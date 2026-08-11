from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.conditions import UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command, LaunchConfiguration

import os

def generate_launch_description():
    camera_name = LaunchConfiguration('camera_name')
    camera_model = LaunchConfiguration('camera_model')
    zed_node_name = LaunchConfiguration('zed_node_name')
    use_sim_time = LaunchConfiguration('use_sim_time')
    sim = LaunchConfiguration('sim')
     
    loone_share = get_package_share_directory('loone')

    # Node parameters. Nothing previously passed this file to task/motor/
    # path_planning, so every tuned value in it was ignored and the in-code
    # defaults were used instead.
    config = os.path.join(loone_share, 'config', 'config.yaml')

    # Robot description lives in the le1000_urdf_t4 package (src/loone_urdf --
    # the directory name and the package name differ).
    urdf_share = get_package_share_directory('le1000_urdf_t4')
    xacro_path = os.path.join(urdf_share, 'urdf', 'loone_asv.urdf.xacro')
    robot_description = {
        'robot_description': ParameterValue(
            Command(['xacro ', xacro_path]), value_type=str)
    }

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
            parameters=[config],
            condition=UnlessCondition(sim),
        )

    battery_node = Node(
            package='loone',
            executable='battery_node',
            name='battery_node',
            output='screen',
            parameters=[config, {'use_sim_time': use_sim_time}],
            condition=UnlessCondition(sim),
        )

    # Publishes base_link, gps_link and the prop/rudder frames from the URDF.
    # gps_link is the frame phone.py stamps its NavSatFix messages with, and
    # navsat_transform_node needs it in TF to remove the antenna offset from the
    # robot's origin -- without this node that lookup fails on every fix.
    # Only the fixed joints (gps_link) actually reach TF: nothing here publishes
    # /joint_states, so the prop/rudder frames stay absent. That is harmless.
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': use_sim_time}],
    )

    # Bridge the ZED camera frame to base_link. ZED publishes odom -> <cam>_camera_link
    # and this wrapper version has no base_frame parameter, so base_link is defined
    # as a child of the camera frame.
    # TODO(team): measure the real mount. These are the camera->base_link offsets
    # (i.e. the negative of where the camera sits relative to the boat center), in
    # metres/radians -- REP-103: x forward, y left, z up.
    static_tf_cam_to_base = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='cam_to_base_link',
        output='screen',
        arguments=[
            '--x', '0.0', '--y', '0.0', '--z', '0.0',
            '--yaw', '0.0', '--pitch', '0.0', '--roll', '0.0',
            '--frame-id', [camera_name, '_camera_link'],
            '--child-frame-id', 'base_link',
        ],
    )

    # parameters for navsat; it's the gps localization transform
    navsat_transform_params = os.path.join(loone_share, 'config', 'navsat_transform.yaml')

    navsat_transform = Node(
        package='robot_localization',
        executable='navsat_transform_node',
        name='navsat_transform',
        output='screen',
        parameters=[navsat_transform_params, {'use_sim_time': use_sim_time}],
        remappings=[
            ('gps/fix', '/navsatfix'),
            # TODO(team): verify against the ZED wrapper's actual topic names if you
            # change the camera_name/zed_node_name launch args (defaults: zedx/zed_node).
            ('imu/data', ['/', camera_name, '/', zed_node_name, '/imu/data']),
            ('odometry/filtered', ['/', camera_name, '/', zed_node_name, '/odom']),
        ],
    )

    # The name= values must match the top-level keys in config.yaml (/task,
    # /motor, /path_planning) for the parameters to be applied.
    # this is commented out because we can just call the node,
    # we want to send data to the GUI without running task.py,
    # it's not a blocking node or anything, it's just better to run on separate terminal
    # so we don't shut down the entire system on tests and hot fixing.
    # task = Node(
    #     package='loone',
    #     executable='task',
    #     name='task',
    #     output='screen',
    #     parameters=[config, {'use_sim_time': use_sim_time}],
    # )

    motor =  Node(
        package='loone',
        executable='motor',
        name='motor',
        output='screen',
        parameters=[config, {'use_sim_time': use_sim_time}],
        condition=UnlessCondition(sim),
    )

    path_planning = Node(
        package='loone',
        executable='path_planning',
        name='path_planning',
        output='screen',
        parameters=[config, {'use_sim_time': use_sim_time}],
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
        path_planning,
        phone,
        battery_node,
        slam_launch,
        robot_state_publisher,
        static_tf_cam_to_base,
        navsat_transform,
    ])