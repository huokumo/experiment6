"""Opt-in SDK velocity adapter only; does not launch a simulator or auto-stand."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('network_interface',default_value=''),
        DeclareLaunchArgument('enable_hardware',default_value='false'),
        Node(package='experiment6_navigation',executable='r1_sdk_adapter',output='screen',
             parameters=[{'network_interface':LaunchConfiguration('network_interface'),
                          'enable_hardware':ParameterValue(LaunchConfiguration('enable_hardware'),value_type=bool)}]),
    ])
