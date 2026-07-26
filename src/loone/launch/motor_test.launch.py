"""Bench-test launch: bring up ONLY the control chain (controller_manager,
thrust_mixer, busio_node) and toggle the thrusters on/off -- no ZED, SLAM, or nav2.

Purpose: verify the props/rudder actually respond to /cmd_vel before trusting the
full nav2 stack, without waiting on ZED camera init or debugging SLAM/costmap issues.
See motor_test_node.py's docstring for the on/off duty-cycle parameters.

Usage:
    ros2 launch loone motor_test.launch.py
    ros2 launch loone motor_test.launch.py thrust:=0.1 on_duration:=2.0 off_duration:=2.0 cycles:=5
    ros2 launch loone motor_test.launch.py sim:=true use_sim_time:=true
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    loone_share = get_package_share_directory('loone')
    urdf_share = get_package_share_directory('le1000_urdf_t4')

    use_sim_time = LaunchConfiguration('use_sim_time')
    sim = LaunchConfiguration('sim')
    thrust = LaunchConfiguration('thrust')
    on_duration = LaunchConfiguration('on_duration')
    off_duration = LaunchConfiguration('off_duration')
    cycles = LaunchConfiguration('cycles')

    # Same robot_description + controller_manager setup as bringup.launch.py, minus
    # everything perception/nav related (see that file's docstring for why).
    xacro_path = os.path.join(urdf_share, 'urdf', 'loone_asv.urdf.xacro')
    robot_description = {
        'robot_description': ParameterValue(Command(['xacro ', xacro_path]), value_type=str)
    }
    ros2_control_params = os.path.join(loone_share, 'config', 'ros2_control.yaml')

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': use_sim_time}],
    )

    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        output='screen',
        parameters=[robot_description, ros2_control_params],
    )

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

    thrust_mixer = Node(
        package='loone',
        executable='thrust_mixer',
        name='thrust_mixer',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    busio_node = Node(
        package='loone',
        executable='busio_node',
        name='busio_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        condition=UnlessCondition(sim),
    )
    sim_state_echo = Node(
        package='loone',
        executable='sim_state_echo',
        name='sim_state_echo',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(sim),
    )

    motor_test_node = Node(
        package='loone',
        executable='motor_test_node',
        name='motor_test_node',
        output='screen',
        parameters=[{
            'thrust': thrust,
            'on_duration': on_duration,
            'off_duration': off_duration,
            'cycles': cycles,
            'use_sim_time': use_sim_time,
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false',
                              description='Use /clock simulated time.'),
        DeclareLaunchArgument('sim', default_value='false',
                              description='Simulation mode: swap busio_node for sim_state_echo.'),
        DeclareLaunchArgument('thrust', default_value='0.1',
                              description='linear.x fraction sent to thrust_mixer while "on" '
                                          '(kept small -- see thrust_mixer.py prop_limit).'),
        DeclareLaunchArgument('on_duration', default_value='2.0',
                              description='Seconds thrust is applied per cycle.'),
        DeclareLaunchArgument('off_duration', default_value='2.0',
                              description='Seconds held neutral between cycles.'),
        DeclareLaunchArgument('cycles', default_value='3',
                              description='Number of on/off cycles. <= 0 repeats forever '
                                          '(until Ctrl-C).'),
        robot_state_publisher,
        controller_manager,
        joint_state_broadcaster_spawner,
        asv_forward_controller_spawner,
        thrust_mixer,
        busio_node,
        sim_state_echo,
        motor_test_node,
    ])
