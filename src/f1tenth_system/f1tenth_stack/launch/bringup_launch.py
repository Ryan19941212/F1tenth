# MIT License

# Copyright (c) 2020 Hongrui Zheng

# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:

# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.

# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    joy_teleop_config = os.path.join(
        get_package_share_directory('f1tenth_stack'),
        'config',
        'joy_teleop.yaml'
    )
    vesc_config = os.path.join(
        get_package_share_directory('f1tenth_stack'),
        'config',
        'vesc.yaml'
    )
    mux_config = os.path.join(
        get_package_share_directory('f1tenth_stack'),
        'config',
        'mux.yaml'
    )

    joy_la = DeclareLaunchArgument(
        'joy_config',
        default_value=joy_teleop_config,
        description='Descriptions for joy and joy_teleop configs')
    vesc_la = DeclareLaunchArgument(
        'vesc_config',
        default_value=vesc_config,
        description='Descriptions for vesc configs')
    mux_la = DeclareLaunchArgument(
        'mux_config',
        default_value=mux_config,
        description='Descriptions for ackermann mux configs')

    # Laser TF: base_link → laser (x y z roll pitch yaw)
    laser_x_la = DeclareLaunchArgument('laser_x', default_value='0.27')
    laser_y_la = DeclareLaunchArgument('laser_y', default_value='0.0')
    laser_z_la = DeclareLaunchArgument('laser_z', default_value='0.15')

    # LiDAR device
    lidar_port_la = DeclareLaunchArgument(
        'lidar_port', default_value='/dev/sensors/lidar',
        description='LiDAR serial port')
    lidar_baudrate_la = DeclareLaunchArgument(
        'lidar_baudrate', default_value='256000',
        description='LiDAR serial baudrate')
    lidar_scan_mode_la = DeclareLaunchArgument(
        'lidar_scan_mode', default_value='Sensitivity',
        description='LiDAR scan mode')

    # Optional status_monitor (opt-in — defaults to off to save CPU)
    status_monitor_la = DeclareLaunchArgument(
        'enable_status_monitor', default_value='false',
        description='Enable the terminal status monitor (5 Hz text dashboard)')

    ld = LaunchDescription([joy_la, vesc_la, mux_la,
                            laser_x_la, laser_y_la, laser_z_la,
                            lidar_port_la, lidar_baudrate_la, lidar_scan_mode_la,
                            status_monitor_la])

    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy',
        parameters=[LaunchConfiguration('joy_config')],
        arguments=['--ros-args', '--log-level', 'WARN']
    )
    joy_teleop_node = Node(
        package='joy_teleop',
        executable='joy_teleop',
        name='joy_teleop',
        parameters=[LaunchConfiguration('joy_config')],
        arguments=['--ros-args', '--log-level', 'WARN']
    )
    ackermann_to_vesc_node = Node(
        package='vesc_ackermann',
        executable='ackermann_to_vesc_node',
        name='ackermann_to_vesc_node',
        parameters=[LaunchConfiguration('vesc_config')],
        remappings=[
            ('commands/motor/speed', 'commands/motor/unsmoothed_speed'),
            ('commands/servo/position', 'commands/servo/unsmoothed_position'),
        ],
        arguments=['--ros-args', '--log-level', 'WARN']
    )
    vesc_to_odom_node = Node(
        package='vesc_ackermann',
        executable='vesc_to_odom_node',
        name='vesc_to_odom_node',
        parameters=[LaunchConfiguration('vesc_config')],
        arguments=['--ros-args', '--log-level', 'WARN']
    )
    vesc_driver_node = Node(
        package='vesc_driver',
        executable='vesc_driver_node',
        name='vesc_driver_node',
        parameters=[LaunchConfiguration('vesc_config')],
        arguments=['--ros-args', '--log-level', 'WARN']
    )
    throttle_interpolator_node = Node(
        package='f1tenth_stack',
        executable='throttle_interpolator',
        name='throttle_interpolator',
        parameters=[LaunchConfiguration('vesc_config')],
        arguments=['--ros-args', '--log-level', 'WARN']
    )
    sllidar_node = Node(
        package='sllidar_ros2',
        executable='sllidar_node',
        name='sllidar_node',
        parameters=[{
            'channel_type': 'serial',
            'serial_port': LaunchConfiguration('lidar_port'),
            'serial_baudrate': LaunchConfiguration('lidar_baudrate'),
            'frame_id': 'laser',
            'inverted': False,
            'angle_compensate': True,
            'scan_mode': LaunchConfiguration('lidar_scan_mode'),
        }],
        arguments=['--ros-args', '--log-level', 'WARN']
    )
    status_monitor_node = Node(
        package='f1tenth_stack',
        executable='status_monitor',
        name='status_monitor',
        parameters=[{'update_rate': 5.0}],
        condition=IfCondition(LaunchConfiguration('enable_status_monitor')),
    )
    ackermann_mux_node = Node(
        package='ackermann_mux',
        executable='ackermann_mux',
        name='ackermann_mux',
        parameters=[LaunchConfiguration('mux_config')],
        remappings=[('ackermann_cmd_out', 'ackermann_drive')],
        arguments=['--ros-args', '--log-level', 'WARN']
    )
    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_baselink_to_laser',
        arguments=['--frame-id', 'base_link', '--child-frame-id', 'laser',
                   '--x', LaunchConfiguration('laser_x'),
                   '--y', LaunchConfiguration('laser_y'),
                   '--z', LaunchConfiguration('laser_z'),
                   '--ros-args', '--log-level', 'WARN']
    )
    # finalize
    ld.add_action(joy_node)
    ld.add_action(joy_teleop_node)
    ld.add_action(ackermann_to_vesc_node)
    ld.add_action(vesc_to_odom_node)
    ld.add_action(vesc_driver_node)
    ld.add_action(throttle_interpolator_node)
    ld.add_action(sllidar_node)
    ld.add_action(ackermann_mux_node)
    ld.add_action(static_tf_node)
    ld.add_action(status_monitor_node)

    return ld