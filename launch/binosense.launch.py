import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_dir = get_package_share_directory('binosense_ros2')
    rviz_config = os.path.join(package_dir, 'rviz', 'binosense.rviz')

    return LaunchDescription([
        DeclareLaunchArgument(
            'frame_prefix',
            default_value='binosense',
            description='Frame ID prefix for all TF frames'
        ),
        DeclareLaunchArgument(
            'connection_mode',
            default_value='ic',
            description='Connection mode: i (image), c (control), ic (both)'
        ),
        DeclareLaunchArgument(
            'device_ip',
            default_value='',
            description='Device IP address (empty for auto-detect)'
        ),
        DeclareLaunchArgument(
            'publish_rate',
            default_value='25.0',
            description='Publish rate in Hz'
        ),
        DeclareLaunchArgument(
            'publish_tf',
            default_value='true',
            description='Whether to publish TF transforms'
        ),
        DeclareLaunchArgument(
            'stereo_enabled',
            default_value='true',
            description='Whether to enable stereo image publishing'
        ),
        DeclareLaunchArgument(
            'depth_enabled',
            default_value='false',
            description='Whether to enable depth and pointcloud publishing'
        ),
        DeclareLaunchArgument(
            'depth_precision',
            default_value='1',
            description='Depth precision: 1000=mm, 100=cm, 10=dm, 1=m'
        ),
        DeclareLaunchArgument(
            'depth_min',
            default_value='200.0',
            description='Minimum valid depth distance'
        ),
        DeclareLaunchArgument(
            'depth_max',
            default_value='5000.0',
            description='Maximum valid depth distance'
        ),
        DeclareLaunchArgument(
            'depth_sv_enabled',
            default_value='false',
            description='Whether to enable SV for depth calculation'
        ),

        Node(
            package='binosense_ros2',
            executable='binosense_driver',
            name='binosense_driver',
            output='screen',
            parameters=[{
                'frame_prefix': LaunchConfiguration('frame_prefix'),
                'connection_mode': LaunchConfiguration('connection_mode'),
                'device_ip': LaunchConfiguration('device_ip'),
                'publish_rate': LaunchConfiguration('publish_rate'),
                'publish_tf': LaunchConfiguration('publish_tf'),
                'stereo_enabled': LaunchConfiguration('stereo_enabled'),
                'depth_enabled': LaunchConfiguration('depth_enabled'),
                'depth_precision': LaunchConfiguration('depth_precision'),
                'depth_min': LaunchConfiguration('depth_min'),
                'depth_max': LaunchConfiguration('depth_max'),
                'depth_sv_enabled': LaunchConfiguration('depth_sv_enabled'),
            }]
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            output='screen'
        )
    ])
