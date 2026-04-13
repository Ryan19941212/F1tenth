"""Pure pursuit waypoint follower for F1Tenth.

Interface:
  - subscribes: /odom (Odometry) or /pf/pose (PoseStamped) for pose
  - subscribes: /odom always (for speed, used in adaptive lookahead)
  - publishes : /drive (AckermannDriveStamped)
  - publishes : /pure_pursuit/lookahead (Marker, for RViz debug)

Waypoints CSV: columns (x, y, yaw[, speed]). yaw ignored; speed optional.
"""

import math
import os
from typing import Optional

import numpy as np
import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from visualization_msgs.msg import Marker


def yaw_from_quat(qz: float, qw: float) -> float:
    return math.atan2(2.0 * qw * qz, 1.0 - 2.0 * qz * qz)


class PurePursuitNode(Node):
    def __init__(self) -> None:
        super().__init__('pure_pursuit_node')

        self.declare_parameter('waypoints_file', '')
        self.declare_parameter('pose_source', 'pf')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('pf_pose_topic', '/pf/pose')
        self.declare_parameter('drive_topic', '/drive')
        self.declare_parameter('marker_topic', '/pure_pursuit/lookahead')
        self.declare_parameter('lookahead_gain', 0.6)
        self.declare_parameter('lookahead_min', 0.5)
        self.declare_parameter('lookahead_max', 1.8)
        self.declare_parameter('speed', 1.0)
        self.declare_parameter('speed_min', 0.4)
        self.declare_parameter('speed_max', 2.0)
        self.declare_parameter('use_waypoint_speed', True)
        self.declare_parameter('max_lateral_accel', 1.5)
        self.declare_parameter('wheelbase', 0.3302)
        self.declare_parameter('steering_max', 0.40)

        gp = self.get_parameter
        self.pose_source = gp('pose_source').value
        self.k_ld = float(gp('lookahead_gain').value)
        self.ld_min = float(gp('lookahead_min').value)
        self.ld_max = float(gp('lookahead_max').value)
        self.v_default = float(gp('speed').value)
        self.v_min = float(gp('speed_min').value)
        self.v_max = float(gp('speed_max').value)
        self.use_wp_speed = bool(gp('use_waypoint_speed').value)
        self.a_lat_max = float(gp('max_lateral_accel').value)
        self.L = float(gp('wheelbase').value)
        self.delta_max = float(gp('steering_max').value)

        wp_path = gp('waypoints_file').value
        if not wp_path or not os.path.isfile(wp_path):
            raise RuntimeError(f'waypoints_file not found: {wp_path!r}')
        self.waypoints = self._load_waypoints(wp_path)
        self.get_logger().info(
            f'Loaded {len(self.waypoints)} waypoints from {wp_path}'
            f' (speed column: {self.waypoints.shape[1] >= 4})'
        )

        self.current_speed = 0.0
        self.have_pose = False
        self.px = self.py = self.pyaw = 0.0

        self.drive_pub = self.create_publisher(
            AckermannDriveStamped, gp('drive_topic').value, 10)
        self.marker_pub = self.create_publisher(
            Marker, gp('marker_topic').value, 10)

        self.create_subscription(
            Odometry, gp('odom_topic').value, self._odom_cb, 20)
        if self.pose_source == 'pf':
            self.create_subscription(
                PoseStamped, gp('pf_pose_topic').value, self._pf_pose_cb, 20)
            self.get_logger().info('Using PF pose as localization source')
        else:
            self.get_logger().info('Using odometry pose as localization source')

        self.create_timer(0.02, self._tick)  # 50 Hz control

    @staticmethod
    def _load_waypoints(path: str) -> np.ndarray:
        data = np.loadtxt(path, delimiter=',', ndmin=2)
        if data.shape[1] < 2:
            raise RuntimeError(f'waypoints must have >=2 cols, got {data.shape}')
        return data  # (N, 2|3|4): x,y[,yaw[,v]]

    def _odom_cb(self, msg: Odometry) -> None:
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        self.current_speed = math.hypot(vx, vy)
        if self.pose_source == 'odom':
            p = msg.pose.pose
            self.px = p.position.x
            self.py = p.position.y
            self.pyaw = yaw_from_quat(p.orientation.z, p.orientation.w)
            self.have_pose = True

    def _pf_pose_cb(self, msg: PoseStamped) -> None:
        p = msg.pose
        self.px = p.position.x
        self.py = p.position.y
        self.pyaw = yaw_from_quat(p.orientation.z, p.orientation.w)
        self.have_pose = True

    def _find_target(self, ld: float) -> Optional[int]:
        wp = self.waypoints[:, :2]
        d = np.hypot(wp[:, 0] - self.px, wp[:, 1] - self.py)
        closest = int(np.argmin(d))
        n = len(wp)
        for i in range(n):
            idx = (closest + i) % n
            if d[idx] >= ld:
                return idx
        return closest

    def _tick(self) -> None:
        if not self.have_pose:
            return

        v_meas = self.current_speed
        ld = max(self.ld_min, min(self.ld_max, self.k_ld * max(v_meas, 0.1)))

        idx = self._find_target(ld)
        if idx is None:
            return
        tx, ty = self.waypoints[idx, 0], self.waypoints[idx, 1]

        dx = tx - self.px
        dy = ty - self.py
        c, s = math.cos(-self.pyaw), math.sin(-self.pyaw)
        x_body = c * dx - s * dy
        y_body = s * dx + c * dy

        ld_eff = math.hypot(x_body, y_body)
        if ld_eff < 1e-3 or x_body <= 0.0:
            # target behind us; coast straight to let geometry recover
            self._publish_drive(0.0, max(self.v_min, 0.3))
            return

        curvature = 2.0 * y_body / (ld_eff * ld_eff)
        delta = math.atan(self.L * curvature)
        delta = max(-self.delta_max, min(self.delta_max, delta))

        # Speed: waypoint column, fallback to param; then cap by lateral accel
        if self.use_wp_speed and self.waypoints.shape[1] >= 4:
            v_cmd = float(self.waypoints[idx, 3])
        else:
            v_cmd = self.v_default

        if abs(curvature) > 1e-4:
            v_cap = math.sqrt(self.a_lat_max / abs(curvature))
            v_cmd = min(v_cmd, v_cap)
        v_cmd = max(self.v_min, min(self.v_max, v_cmd))

        self._publish_drive(delta, v_cmd)
        self._publish_marker(tx, ty)

    def _publish_drive(self, delta: float, v: float) -> None:
        msg = AckermannDriveStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.drive.steering_angle = float(delta)
        msg.drive.speed = float(v)
        self.drive_pub.publish(msg)

    def _publish_marker(self, x: float, y: float) -> None:
        m = Marker()
        m.header.frame_id = 'map'
        m.header.stamp = self.get_clock().now().to_msg()
        m.type = Marker.SPHERE
        m.action = Marker.ADD
        m.pose.position.x = float(x)
        m.pose.position.y = float(y)
        m.pose.orientation.w = 1.0
        m.scale.x = m.scale.y = m.scale.z = 0.15
        m.color.r, m.color.g, m.color.b, m.color.a = 0.1, 1.0, 0.2, 0.9
        self.marker_pub.publish(m)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PurePursuitNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
