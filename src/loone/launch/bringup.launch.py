"""Top-level bringup for the nav2 + ros2_control "chained controls" stack.

Starts, in order:
  1. slam_launch.py         - ZED wrapper + depth->laserscan + SLAM Toolbox
                              (provides map->odom->zedx_camera_link TF, /odom, /scan).
  2. robot_state_publisher  - publishes base_link + prop/rudder frames from the URDF.
  3. static_transform_pub   - connects zedx_camera_link -> base_link (the camera is the
                              tracked frame; see loone_asv.urdf.xacro for why base_link
                              hangs below it).
  4. controller_manager     - ros2_control node hosting the hardware + controllers.
  5. spawners               - joint_state_broadcaster + asv_forward_controller.
  6. thrust_mixer           - /cmd_vel -> /asv_forward_controller/commands.
  7. pca9685_driver         - /asv/joint_commands -> PCA9685 over I2C.
  8. phone                  - phone GPS (ADB) -> /navsatfix, fused into the ZED's
                              pos_tracking via gnss_fusion (config/common_stereo.yaml).
  9. navigation_launch.py   - nav2 planner/controller/costmaps -> /cmd_vel.

The old task/motor/path_planning nodes are intentionally NOT started here.
Send goals with RViz "2D Goal Pose" or a NavigateToPose action client.

Usage:
    ros2 launch loone bringup.launch.py
    # bench test without a camera/hardware: see the plan's verification section.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    loone_share = get_package_share_directory('loone')
    nav2_bringup_share = get_package_share_directory('nav2_bringup')

    camera_name = LaunchConfiguration('camera_name')
    camera_model = LaunchConfiguration('camera_model')
    zed_node_name = LaunchConfiguration('zed_node_name')
    use_sim_time = LaunchConfiguration('use_sim_time')
    sim = LaunchConfiguration('sim')

    # Expand the xacro once and share the result with rsp + controller_manager.
    xacro_path = os.path.join(loone_share, 'urdf', 'loone_asv.urdf.xacro')
    robot_description = {
        'robot_description': ParameterValue(Command(['xacro ', xacro_path]), value_type=str)
    }

    ros2_control_params = os.path.join(loone_share, 'config', 'ros2_control.yaml')
    nav2_params = os.path.join(loone_share, 'config', 'nav2_params.yaml')

    # 1. Perception / localization (already working on hardware).
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

    # 2. Robot description -> TF for base_link and the prop/rudder links.
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': use_sim_time}],
    )

    # 3. Bridge the ZED camera frame to base_link. ZED publishes odom -> <cam>_camera_link,
    #    so base_link is defined as a child of the camera frame here.
    #    TODO(team): measure the real mount. These are the camera->base_link offsets
    #    (i.e. the negative of where the camera sits relative to the boat center),
    #    in metres/radians. Order: x y z yaw pitch roll.
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

    # 4. controller_manager: hosts topic_based_ros2_control hardware + the controllers.
    #    NOTE (Humble): robot_description is passed as a parameter here. On Iron/Jazzy
    #    the controller_manager instead reads it from the /robot_description topic.
    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        output='screen',
        parameters=[robot_description, ros2_control_params],
    )

    # 5. Spawn the broadcaster + forward controller into the controller_manager.
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen',
    )
    asv_forward_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['asv_forward_controller', '--controller-manager', '/controller_manager'],
        output='screen',
    )

    # 6. Control mixing (cmd_vel -> servo fractions).
    thrust_mixer = Node(
        package='loone',
        executable='thrust_mixer',
        name='thrust_mixer',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # 7. Hardware driver (fractions -> PCA9685). Only node touching I2C.
    #    Excluded in sim: it imports board/busio at module scope, so it cannot even
    #    start on a machine without an I2C bus.
    pca9685_driver = Node(
        package='loone',
        executable='pca9685_driver',
        name='pca9685_driver',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        condition=UnlessCondition(sim),
    )

    # 7b. Simulation stand-in for the driver's open-loop state echo. Isaac Sim
    #     consumes /asv/joint_commands (see isaac_sim/.../ros2_bridge.py), but
    #     nothing would publish /asv/joint_states back to topic_based_ros2_control
    #     without this -- the state interfaces would sit at NaN.
    sim_state_echo = Node(
        package='loone',
        executable='sim_state_echo',
        name='sim_state_echo',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(sim),
    )

    # 8. Phone GPS bridge: publishes NavSatFix on /navsatfix, which the ZED wrapper's
    #    gnss_fusion (config/common_stereo.yaml) subscribes to and fuses into pos_tracking.
    phone = Node(
        package='loone',
        executable='phone',
        name='phone',
        output='screen',
    )

    # 9. nav2 (produces /cmd_vel). No AMCL/map_server -- SLAM Toolbox owns those.
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_share, 'launch', 'navigation_launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': nav2_params,
            'autostart': 'true',
        }.items()
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
                              description='Simulation mode: swap pca9685_driver for sim_state_echo '
                                          '(Isaac Sim drives the actuators). Usually set together '
                                          'with use_sim_time:=true.'),
        slam_launch,
        robot_state_publisher,
        static_tf_cam_to_base,
        controller_manager,
        joint_state_broadcaster_spawner,
        asv_forward_controller_spawner,
        thrust_mixer,
        pca9685_driver,
        sim_state_echo,
        phone,
        nav2_launch,
    ])
