import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    cfg = os.path.join(
        get_package_share_directory('pure_pursuit'),
        'config', 'pure_pursuit_params.yaml',
    )
    return LaunchDescription([
        DeclareLaunchArgument('waypoints_file', default_value=''),
        DeclareLaunchArgument('pose_source', default_value='pf'),
        DeclareLaunchArgument('speed', default_value='1.0'),
        Node(
            package='pure_pursuit',
            executable='pure_pursuit_node',
            name='pure_pursuit_node',
            output='screen',
            parameters=[
                cfg,
                {
                    'waypoints_file': LaunchConfiguration('waypoints_file'),
                    'pose_source': LaunchConfiguration('pose_source'),
                    'speed': LaunchConfiguration('speed'),
                },
            ],
        ),
    ])
