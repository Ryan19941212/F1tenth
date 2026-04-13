import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from tf2_ros import Buffer, TransformListener
from f1tenth_utils.constants import SCAN_TOPIC, ODOM_TOPIC, ACKERMANN_DRIVE_TOPIC


class StatusMonitor(Node):
    def __init__(self):
        super().__init__('status_monitor')
        self.declare_parameter('update_rate', 5.0)
        rate = self.get_parameter('update_rate').value

        # State
        self.odom = None
        self.scan = None
        self.cmd = None
        self.map_x = None
        self.map_y = None
        self.map_yaw = None

        # Subscribers
        self.create_subscription(Odometry, ODOM_TOPIC, self._odom_cb, 10)
        self.create_subscription(LaserScan, SCAN_TOPIC, self._scan_cb, 10)
        self.create_subscription(
            AckermannDriveStamped, ACKERMANN_DRIVE_TOPIC, self._cmd_cb, 10)

        # TF for map->base_link (available when SLAM/localization runs)
        self.tf_buf = Buffer()
        self.tf_listener = TransformListener(self.tf_buf, self)

        # Timer
        self.create_timer(1.0 / rate, self._print)
        self.get_logger().info('Status monitor started (%.1f Hz)' % rate)

    def _odom_cb(self, msg):
        self.odom = msg

    def _scan_cb(self, msg):
        self.scan = msg

    def _cmd_cb(self, msg):
        self.cmd = msg

    def _quat_to_yaw(self, q):
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny, cosy)

    def _try_map_tf(self):
        try:
            t = self.tf_buf.lookup_transform('map', 'base_link', rclpy.time.Time())
            self.map_x = t.transform.translation.x
            self.map_y = t.transform.translation.y
            self.map_yaw = self._quat_to_yaw(t.transform.rotation)
        except Exception:
            self.map_x = None

    def _print(self):
        lines = ['\033[2J\033[H']  # clear screen + cursor home
        lines.append('═' * 58)
        lines.append('  F1TENTH Status Monitor')
        lines.append('═' * 58)

        # Odom
        if self.odom:
            o = self.odom
            vx = o.twist.twist.linear.x
            wz = o.twist.twist.angular.z
            ox = o.pose.pose.position.x
            oy = o.pose.pose.position.y
            yaw = self._quat_to_yaw(o.pose.pose.orientation)
            lines.append(f'  [Odom]  pos=({ox:+.3f}, {oy:+.3f})  yaw={math.degrees(yaw):+.1f}°')
            lines.append(f'          vel={vx:+.3f} m/s  omega={math.degrees(wz):+.1f} °/s')
        else:
            lines.append('  [Odom]  -- no data --')

        # Map pose (from TF, only when SLAM/localization active)
        self._try_map_tf()
        if self.map_x is not None:
            lines.append(f'  [Map]   pos=({self.map_x:+.3f}, {self.map_y:+.3f})  yaw={math.degrees(self.map_yaw):+.1f}°')

        # Command
        if self.cmd:
            spd = self.cmd.drive.speed
            steer = self.cmd.drive.steering_angle
            lines.append(f'  [Cmd]   speed={spd:+.3f} m/s  steer={math.degrees(steer):+.1f}°')
        else:
            lines.append('  [Cmd]   -- no command --')

        # LiDAR
        if self.scan:
            ranges = [r for r in self.scan.ranges if self.scan.range_min < r < self.scan.range_max]
            if ranges:
                front_idx = len(self.scan.ranges) // 2
                half_w = 30
                front_slice = self.scan.ranges[max(0, front_idx - half_w):front_idx + half_w]
                front_valid = [r for r in front_slice if self.scan.range_min < r < self.scan.range_max]
                front_min = min(front_valid) if front_valid else float('inf')
                lines.append(f'  [LiDAR] pts={len(ranges)}  min={min(ranges):.2f}m  front_min={front_min:.2f}m')
            else:
                lines.append('  [LiDAR] -- no valid ranges --')
        else:
            lines.append('  [LiDAR] -- no data --')

        lines.append('═' * 58)
        print('\n'.join(lines))


def main(args=None):
    rclpy.init(args=args)
    node = StatusMonitor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
