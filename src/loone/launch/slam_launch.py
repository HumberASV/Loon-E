import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

default_slam_params_file = os.path.join(
    get_package_share_directory('loone'),
    'config',
    'mapper_params_online_async.yaml'
)

default_depth_to_laserscan_params_file = os.path.join(
    get_package_share_directory('loone'),
    'config',
    'depth_to_laserscan.yaml'
)

default_custom_object_detection_config_path = os.path.join(
    get_package_share_directory('loone'),
    'config',
    'model.yaml'
)

# custom_onnx_file in model.yaml is a relative path; override it here with the
# absolute path of the ONNX file as installed into the loone package share
# directory, since the ZED SDK opens the path directly without resolving it
# against the package share directory.
default_model_onnx_path = os.path.join(
    get_package_share_directory('loone'),
    'model',
    'loone.onnx'
)


def generate_launch_description():
    camera_name = LaunchConfiguration('camera_name')
    camera_model = LaunchConfiguration('camera_model')
    zed_node_name = LaunchConfiguration('zed_node_name')
    slam_params_file = LaunchConfiguration('slam_params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')

    zed_wrapper_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('zed_wrapper'),
                'launch',
                'zed_camera.launch.py'
            )
        ),
        launch_arguments={
            'camera_name': camera_name,
            'camera_model': camera_model,
            'node_name': zed_node_name,
            'publish_map_tf': 'false',  # SLAM Toolbox publishes map -> odom instead
            'use_sim_time': use_sim_time,
            'custom_object_detection_config_path': default_custom_object_detection_config_path,
            'param_overrides': 'object_detection.custom_onnx_file:=' + default_model_onnx_path,
        }.items()
    )

    # Depth image -> LaserScan, subscribing to the ZED node's registered
    # depth image and re-publishing it as a 2D scan for SLAM Toolbox.
    depth_to_laserscan_node = Node(
        package='depthimage_to_laserscan',
        executable='depthimage_to_laserscan_node',
        name='depthimage_to_laserscan',
        output='screen',
        parameters=[
            default_depth_to_laserscan_params_file,
            {
                'output_frame': [camera_name, '_camera_link'],
                'use_sim_time': use_sim_time,
            }
        ],
        remappings=[
            ('depth', ['/', camera_name, '/', zed_node_name, '/depth/depth_registered']),
            ('depth_camera_info', ['/', camera_name, '/', zed_node_name, '/depth/camera_info']),
            ('scan', '/scan'),
        ]
    )

    slam_toolbox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('slam_toolbox'),
                'launch',
                'online_async_launch.py'
            )
        ),
        launch_arguments={
            'slam_params_file': slam_params_file,
            'use_sim_time': use_sim_time,
        }.items()
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'camera_name',
            default_value='zedx',
            description='Name of the ZED X camera, used as its node namespace.'),
        DeclareLaunchArgument(
            'camera_model',
            default_value='zedx',
            description='Model of the ZED camera passed to the zed_wrapper launch file.'),
        DeclareLaunchArgument(
            'zed_node_name',
            default_value='zed_node',
            description='Name of the zed_wrapper node inside the camera_name namespace.'),
        DeclareLaunchArgument(
            'slam_params_file',
            default_value=default_slam_params_file,
            description='Path to the SLAM Toolbox parameters YAML file.'),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulated /clock time instead of the system clock.'),
        zed_wrapper_launch,
        depth_to_laserscan_node,
        slam_toolbox_launch,
    ])
