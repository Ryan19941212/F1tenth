"""
MPC node for waypoint tracking on F1Tenth.

The controller keeps the external interface simple:
  - subscribes to odometry
  - publishes AckermannDriveStamped

Internally it uses:
  - forward-window path projection for smoother error estimates
  - speed-aware linear MPC
  - steering rate penalties and limits
  - curvature/lateral-acceleration based speed scheduling
"""

import math
import os
from typing import Optional, Tuple

import numpy as np
import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Imu
from tf2_ros import Buffer, TransformListener

from mpc_controller.mpc_solver import MPCSolver


def wrap_angle(angle: float) -> float:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


class MPCNode(Node):
    """Waypoint-tracking MPC controller."""

    def __init__(self) -> None:
        super().__init__('mpc_node')

        self.declare_parameter('waypoints_file', '')
        self.declare_parameter('pose_source', 'odom')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('drive_topic', '/drive')
        self.declare_parameter('sim_speed_topic', '/autodrive/odom')
        self.declare_parameter('sim_ips_topic', '/autodrive/f1tenth_1/ips')
        self.declare_parameter('sim_imu_topic', '/autodrive/f1tenth_1/imu')
        self.declare_parameter('speed', 0.8)
        self.declare_parameter('speed_min', 0.4)
        self.declare_parameter('speed_max', 1.5)
        self.declare_parameter('use_waypoint_speed', True)
        self.declare_parameter('max_lateral_accel', 1.5)
        self.declare_parameter('wheelbase', 0.3302)
        self.declare_parameter('dt', 0.1)
        self.declare_parameter('horizon', 12)
        self.declare_parameter('delta_max', 0.40)
        self.declare_parameter('delta_rate_max', 0.08)
        self.declare_parameter('q_ey', 14.0)
        self.declare_parameter('q_epsi', 8.0)
        self.declare_parameter('r_delta', 0.08)
        self.declare_parameter('r_d_delta', 1.2)
        self.declare_parameter('search_window', 80)
        self.declare_parameter('log_period', 0.5)
        self.declare_parameter('safe_stop_on_solver_failure', True)

        wp_file = self.get_parameter('waypoints_file').value
        self._pose_source = self.get_parameter('pose_source').value
        self._odom_topic = self.get_parameter('odom_topic').value
        self._drive_topic = self.get_parameter('drive_topic').value
        self._sim_speed_topic = self.get_parameter('sim_speed_topic').value
        self._sim_ips_topic = self.get_parameter('sim_ips_topic').value
        self._sim_imu_topic = self.get_parameter('sim_imu_topic').value
        self._nominal_speed = float(self.get_parameter('speed').value)
        self._speed_min = float(self.get_parameter('speed_min').value)
        self._speed_max = float(self.get_parameter('speed_max').value)
        self._use_waypoint_speed = bool(self.get_parameter('use_waypoint_speed').value)
        self._max_lateral_accel = float(self.get_parameter('max_lateral_accel').value)
        self._wheelbase = float(self.get_parameter('wheelbase').value)
        self._dt = float(self.get_parameter('dt').value)
        horizon = int(self.get_parameter('horizon').value)
        delta_max = float(self.get_parameter('delta_max').value)
        delta_rate_max = float(self.get_parameter('delta_rate_max').value)
        q_ey = float(self.get_parameter('q_ey').value)
        q_epsi = float(self.get_parameter('q_epsi').value)
        r_delta = float(self.get_parameter('r_delta').value)
        r_d_delta = float(self.get_parameter('r_d_delta').value)
        self._search_window = int(self.get_parameter('search_window').value)
        self._log_period = float(self.get_parameter('log_period').value)
        self._safe_stop_on_solver_failure = bool(
            self.get_parameter('safe_stop_on_solver_failure').value
        )

        if not wp_file:
            self.get_logger().fatal('waypoints_file parameter is empty. Shutting down.')
            raise SystemExit(1)
        self._load_waypoints(wp_file)

        Q = np.diag([q_ey, q_epsi])
        R = np.array([[r_delta]])
        Rd = np.array([[r_d_delta]])
        self.get_logger().info('Building MPC solver...')
        self._solver = MPCSolver(
            v=self._nominal_speed,
            L=self._wheelbase,
            dt=self._dt,
            N=horizon,
            Q=Q,
            R=R,
            Rd=Rd,
            delta_max=delta_max,
            ddelta_max=delta_rate_max,
        )
        self.get_logger().info('MPC solver ready.')

        self._pose: Optional[Tuple[float, float, float]] = None
        self._odom_received = False
        self._nearest_idx = 0
        self._current_speed = 0.0
        self._last_delta = 0.0
        self._last_cmd_speed = self._nominal_speed
        self._last_log_time = 0.0
        self._pf_converged = False
        self._pf_cov_threshold = 0.15  # max std-dev (m) before we trust PF
        self._sim_x = 0.0
        self._sim_y = 0.0
        self._sim_yaw = 0.0
        self._sim_imu_received = False
        self._sim_ips_received = False

        if self._pose_source == 'autodrive':
            self._odom_sub = self.create_subscription(
                Odometry, self._sim_speed_topic, self._sim_speed_callback, 10
            )
            self._ips_sub = self.create_subscription(
                Point, self._sim_ips_topic, self._sim_ips_callback, 10
            )
            self._imu_sub = self.create_subscription(
                Imu, self._sim_imu_topic, self._sim_imu_callback, 10
            )
        elif self._pose_source == 'slam':
            self._tf_buffer = Buffer()
            self._tf_listener = TransformListener(self._tf_buffer, self)
            self._slam_timer = self.create_timer(0.02, self._slam_tf_callback)
            # Still need odom for current_speed
            self._odom_sub = self.create_subscription(
                Odometry, self._odom_topic, self._speed_only_callback, 10
            )
        elif self._pose_source == 'pf':
            self._pf_sub = self.create_subscription(
                Odometry, '/pf/pose/odom', self._pf_callback, 10
            )
            # Separate odom subscription for responsive speed readings
            self._odom_sub = self.create_subscription(
                Odometry, self._odom_topic, self._speed_only_callback, 10
            )
        else:
            self._odom_sub = self.create_subscription(
                Odometry, self._odom_topic, self._odom_callback, 10
            )
        self._drive_pub = self.create_publisher(
            AckermannDriveStamped, self._drive_topic, 10
        )
        self._timer = self.create_timer(self._dt, self._timer_callback)

        self.get_logger().info(
            f'MPC node started | pose_source={self._pose_source} | drive={self._drive_topic} '
            f'| N={horizon} | dt={self._dt:.3f} s | waypoints={self._n_wp}'
        )

    def _load_waypoints(self, path: str) -> None:
        if not os.path.isfile(path):
            self.get_logger().fatal(f'Waypoints file not found: {path}')
            raise SystemExit(1)

        data = np.loadtxt(path, delimiter=',', skiprows=1)
        if data.ndim == 1:
            data = data.reshape(1, -1)
        if data.shape[1] < 3:
            self.get_logger().fatal('Waypoints CSV must have at least columns: x, y, yaw')
            raise SystemExit(1)

        if data.shape[1] < 4:
            data = np.column_stack([data[:, :3], np.full(len(data), self._nominal_speed)])
        else:
            data = data[:, :4]

        self._waypoints = data
        self._points = data[:, :2]
        self._headings = data[:, 2]
        self._speeds = data[:, 3]
        self._n_wp = len(data)
        self.get_logger().info(
            f'Loaded {self._n_wp} waypoints from {os.path.basename(path)}'
        )

    def _odom_callback(self, msg: Odometry) -> None:
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        qx = msg.pose.pose.orientation.x
        qy = msg.pose.pose.orientation.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        yaw = math.atan2(
            2.0 * (qw * qz + qx * qy),
            1.0 - 2.0 * (qy * qy + qz * qz),
        )

        self._pose = (x, y, yaw)
        self._current_speed = float(msg.twist.twist.linear.x)
        self._odom_received = True

    def _pf_callback(self, msg: Odometry) -> None:
        """Handle particle filter pose with convergence check."""
        cov = msg.pose.covariance
        std_x = math.sqrt(max(cov[0], 0.0))
        std_y = math.sqrt(max(cov[4], 0.0))

        if not self._pf_converged:
            max_std = max(std_x, std_y)
            if max_std < self._pf_cov_threshold:
                self._pf_converged = True
                self.get_logger().info(
                    f'PF CONVERGED — std_x={std_x:.3f} std_y={std_y:.3f} — MPC enabled'
                )
            else:
                if self._should_log():
                    self.get_logger().info(
                        f'PF converging... std_x={std_x:.3f} std_y={std_y:.3f} '
                        f'(need < {self._pf_cov_threshold:.2f})'
                    )
                return

        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        qx = msg.pose.pose.orientation.x
        qy = msg.pose.pose.orientation.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        yaw = math.atan2(
            2.0 * (qw * qz + qx * qy),
            1.0 - 2.0 * (qy * qy + qz * qz),
        )

        # --- Heading sanity check against waypoints ---
        # PF on small maps can converge to yaw ± π ambiguity.
        # Pick whichever orientation is closer to the nearest waypoint heading.
        pos = np.array([x, y])
        dists = np.linalg.norm(self._points - pos, axis=1)
        nearest = int(np.argmin(dists))
        wp_heading = float(self._headings[nearest])
        if abs(wrap_angle(yaw - wp_heading)) > math.pi / 2:
            yaw = wrap_angle(yaw + math.pi)

        self._pose = (x, y, yaw)
        self._odom_received = True

    def _slam_tf_callback(self) -> None:
        try:
            t = self._tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
        except Exception:
            return
        x = t.transform.translation.x
        y = t.transform.translation.y
        q = t.transform.rotation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self._pose = (x, y, yaw)
        self._odom_received = True

    def _speed_only_callback(self, msg: Odometry) -> None:
        self._current_speed = float(msg.twist.twist.linear.x)

    def _sim_speed_callback(self, msg: Odometry) -> None:
        self._current_speed = float(msg.twist.twist.linear.x)
        self._update_sim_pose()

    def _sim_ips_callback(self, msg: Point) -> None:
        self._sim_x = float(msg.x)
        self._sim_y = float(msg.y)
        self._sim_ips_received = True
        self._update_sim_pose()

    def _sim_imu_callback(self, msg: Imu) -> None:
        q = msg.orientation
        self._sim_yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )
        self._sim_imu_received = True
        self._update_sim_pose()

    def _update_sim_pose(self) -> None:
        if not (self._sim_ips_received and self._sim_imu_received):
            return
        self._pose = (self._sim_x, self._sim_y, self._sim_yaw)
        self._odom_received = True

    def _path_curvature(self, idx: int) -> float:
        p0 = self._points[(idx - 1) % self._n_wp]
        p1 = self._points[idx % self._n_wp]
        p2 = self._points[(idx + 1) % self._n_wp]

        a = np.linalg.norm(p1 - p0)
        b = np.linalg.norm(p2 - p1)
        c = np.linalg.norm(p2 - p0)
        if min(a, b, c) < 1e-6:
            return 0.0

        twice_area = abs(
            (p1[0] - p0[0]) * (p2[1] - p0[1]) - (p1[1] - p0[1]) * (p2[0] - p0[0])
        )
        if twice_area < 1e-9:
            return 0.0

        return 2.0 * twice_area / (a * b * c)

    def _project_to_path(self, x: float, y: float) -> Tuple[float, float, int, float]:
        window = min(self._search_window, self._n_wp)
        pos = np.array([x, y], dtype=float)

        best_dist2 = float('inf')
        best_proj = self._points[self._nearest_idx]
        best_idx = self._nearest_idx
        best_heading = float(self._headings[self._nearest_idx])

        for offset in range(window):
            idx = (self._nearest_idx + offset) % self._n_wp
            nxt = (idx + 1) % self._n_wp
            p0 = self._points[idx]
            p1 = self._points[nxt]
            seg = p1 - p0
            seg_norm2 = float(np.dot(seg, seg))

            if seg_norm2 < 1e-9:
                proj = p0
                heading = float(self._headings[idx])
            else:
                t = float(np.clip(np.dot(pos - p0, seg) / seg_norm2, 0.0, 1.0))
                proj = p0 + t * seg
                heading = math.atan2(seg[1], seg[0])

            diff = pos - proj
            dist2 = float(np.dot(diff, diff))
            if dist2 < best_dist2:
                best_dist2 = dist2
                best_proj = proj
                best_idx = idx
                best_heading = heading

        return float(best_proj[0]), float(best_proj[1]), best_idx, best_heading

    def compute_tracking_state(self) -> Tuple[float, float, int, float, float]:
        cx, cy, yaw = self._pose
        px, py, idx, path_heading = self._project_to_path(cx, cy)
        self._nearest_idx = idx

        dx = cx - px
        dy = cy - py
        e_y = -math.sin(path_heading) * dx + math.cos(path_heading) * dy
        e_psi = wrap_angle(yaw - path_heading)

        ref_speed = float(self._speeds[idx]) if self._use_waypoint_speed else self._nominal_speed
        curvature = self._path_curvature(idx)
        return e_y, e_psi, idx, ref_speed, curvature

    def _compute_target_speed(self, ref_speed: float, curvature: float, delta: float) -> float:
        target_speed = float(np.clip(ref_speed, self._speed_min, self._speed_max))

        if curvature > 1e-6:
            curvature_limited = math.sqrt(self._max_lateral_accel / curvature)
            target_speed = min(target_speed, curvature_limited)

        steer_ratio = min(abs(delta) / max(self._solver.delta_max, 1e-6), 1.0)
        steering_limited = self._speed_max * (1.0 - 0.45 * steer_ratio)
        target_speed = min(target_speed, steering_limited)

        return float(np.clip(target_speed, 0.0, self._speed_max))

    def solve_mpc(
        self,
        e_y: float,
        e_psi: float,
        target_speed: float,
    ) -> Tuple[float, str, float]:
        x0 = np.array([e_y, e_psi])
        return self._solver.solve(
            x0=x0,
            delta_prev=self._last_delta,
            speed=target_speed,
        )

    def publish_control(self, delta: float, speed: float) -> None:
        msg = AckermannDriveStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.drive.steering_angle = float(delta)
        msg.drive.speed = float(speed)
        self._drive_pub.publish(msg)

    def _should_log(self) -> bool:
        now = self.get_clock().now().nanoseconds * 1e-9
        if (now - self._last_log_time) >= self._log_period:
            self._last_log_time = now
            return True
        return False

    def _timer_callback(self) -> None:
        if not self._odom_received:
            return

        e_y, e_psi, idx, ref_speed, curvature = self.compute_tracking_state()
        pre_target_speed = float(np.clip(ref_speed, self._speed_min, self._speed_max))
        delta, status, solve_ms = self.solve_mpc(e_y, e_psi, pre_target_speed)

        if status in ('optimal', 'optimal_inaccurate'):
            cmd_speed = self._compute_target_speed(ref_speed, curvature, delta)
            self.publish_control(delta, cmd_speed)
            self._last_delta = delta
            self._last_cmd_speed = cmd_speed
        else:
            fallback_speed = 0.0 if self._safe_stop_on_solver_failure else self._last_cmd_speed
            fallback_delta = 0.0 if self._safe_stop_on_solver_failure else self._last_delta
            self.publish_control(fallback_delta, fallback_speed)
            cmd_speed = fallback_speed
            delta = fallback_delta

        if self._should_log():
            warn = ''
            if status not in ('optimal', 'optimal_inaccurate'):
                warn = f' | solver={status}'
            elif solve_ms > 30.0:
                warn = f' | slow={solve_ms:.1f}ms'

            self.get_logger().info(
                f'wp={idx:04d} | ey={e_y:+.3f} m | epsi={math.degrees(e_psi):+.2f} deg '
                f'| delta={math.degrees(delta):+.2f} deg | vcmd={cmd_speed:.2f} m/s '
                f'| vmeas={self._current_speed:.2f} m/s | kappa={curvature:.3f}{warn}'
            )


def main(args=None) -> None:
    rclpy.init(args=args)
    try:
        node = MPCNode()
        rclpy.spin(node)
    except SystemExit:
        pass
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
