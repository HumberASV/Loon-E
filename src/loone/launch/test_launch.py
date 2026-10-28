from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    # One-shot demo; stop the motor node first (same I2C bus).
    hardware_test = Node(
        package='loone',
        executable='hardware_test',
        name='hardware_test',
        output='screen',
    )

    return LaunchDescription([
        hardware_test
    ])
