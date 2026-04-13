from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    config_dir = os.path.join(
        get_package_share_directory('f1tenth_stack'), 'config')
    slam_common = os.path.join(config_dir, 'slam_common.yaml')
    loc_specific = os.path.join(config_dir, 'slam_localization.yaml')

    loc_config_la = DeclareLaunchArgument(
        'loc_config',
        default_value=loc_specific,
        description='Path to localization-specific slam_toolbox config yaml')

    map_file_la = DeclareLaunchArgument(
        'map_file',
        description='Path to map file (without extension), e.g. /home/ryan/map')

    loc_node = Node(
        package='slam_toolbox',
        executable='localization_slam_toolbox_node',
        name='slam_toolbox',
        parameters=[
            slam_common,
            LaunchConfiguration('loc_config'),
            {'map_file_name': LaunchConfiguration('map_file')},
        ],
        output='screen'
    )

    return LaunchDescription([loc_config_la, map_file_la, loc_node])
