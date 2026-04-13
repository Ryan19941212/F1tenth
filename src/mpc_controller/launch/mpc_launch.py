from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg = FindPackageShare('mpc_controller')

    # ── Launch arguments ──────────────────────────────────────────────
    waypoints_la = DeclareLaunchArgument(
        'waypoints_file',
        default_value=PathJoinSubstitution(
            [pkg, 'waypoints', 'example_circle.csv']
        ),
        description='Absolute path to waypoints CSV (x,y,yaw,v)',
    )
    speed_la = DeclareLaunchArgument(
        'speed', default_value='0.8',
        description='Constant longitudinal speed [m/s]',
    )
    odom_topic_la = DeclareLaunchArgument(
        'odom_topic', default_value='/odom',
        description='Odometry topic consumed by the MPC node',
    )
    pose_source_la = DeclareLaunchArgument(
        'pose_source', default_value='odom',
        description='Pose source: odom or autodrive',
    )
    drive_topic_la = DeclareLaunchArgument(
        'drive_topic', default_value='/drive',
        description='Ackermann command topic published by the MPC node',
    )
    q_ey_la = DeclareLaunchArgument(
        'q_ey', default_value='10.0',
        description='Cross-track error cost weight',
    )
    q_epsi_la = DeclareLaunchArgument(
        'q_epsi', default_value='5.0',
        description='Heading error cost weight',
    )
    r_delta_la = DeclareLaunchArgument(
        'r_delta', default_value='0.1',
        description='Steering effort cost weight',
    )

    # ── MPC node ──────────────────────────────────────────────────────
    mpc_node = Node(
        package='mpc_controller',
        executable='mpc_node',
        name='mpc_node',
        output='screen',
        parameters=[
            PathJoinSubstitution([pkg, 'config', 'mpc_params.yaml']),
            {
                'waypoints_file': LaunchConfiguration('waypoints_file'),
                'pose_source':    LaunchConfiguration('pose_source'),
                'odom_topic':      LaunchConfiguration('odom_topic'),
                'drive_topic':     LaunchConfiguration('drive_topic'),
                'speed':          LaunchConfiguration('speed'),
                'q_ey':           LaunchConfiguration('q_ey'),
                'q_epsi':         LaunchConfiguration('q_epsi'),
                'r_delta':        LaunchConfiguration('r_delta'),
            },
        ],
    )

    return LaunchDescription([
        waypoints_la, speed_la,
        pose_source_la, odom_topic_la, drive_topic_la,
        q_ey_la, q_epsi_la, r_delta_la,
        mpc_node,
    ])
