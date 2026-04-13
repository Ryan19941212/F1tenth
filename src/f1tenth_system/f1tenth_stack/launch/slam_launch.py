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
    slam_specific = os.path.join(config_dir, 'slam_online_async.yaml')

    slam_config_la = DeclareLaunchArgument(
        'slam_config',
        default_value=slam_specific,
        description='Path to mapping-specific slam_toolbox config yaml')

    slam_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        parameters=[slam_common, LaunchConfiguration('slam_config')],
        arguments=['--ros-args', '--log-level', 'WARN']
    )

    return LaunchDescription([slam_config_la, slam_node])
