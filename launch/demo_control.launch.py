import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'mode',
            default_value='sweep',
            description='Control mode: position, twist, or sweep'
        ),

        Node(
            package='binosense_ros2',
            executable='binosense_driver',
            name='binosense_driver',
            output='screen',
            parameters=[{
                'frame_prefix': 'binosense',
                'connection_mode': 'ic',
                'publish_rate': 30.0,
                'publish_tf': True,
                'stereo_enabled': True,
            }]
        ),

        Node(
            package='binosense_ros2',
            executable='binosense_control_example',
            name='binosense_control_example',
            output='screen',
            parameters=[{
                'mode': LaunchConfiguration('mode'),
            }]
        )
    ])
