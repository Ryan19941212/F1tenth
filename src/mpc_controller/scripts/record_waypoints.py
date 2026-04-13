#!/usr/bin/env python3
"""
Record waypoints from SLAM pose (map → base_link TF) while driving with joystick.

Usage:
  python3 record_waypoints.py -o waypoints.csv [--dist 0.1] [--speed 0.5]

Requires: bringup + slam_toolbox running.
Drive with joystick, Ctrl+C to stop and save.
"""

import argparse
import math
import signal
import sys

import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener


class WaypointRecorder(Node):
    def __init__(self, output_file: str, min_dist: float, speed_fill: float):
        super().__init__('waypoint_recorder')
        self.output_file = output_file
        self.min_dist = min_dist
        self.speed_fill = speed_fill
        self.waypoints = []
        self.last_x = None
        self.last_y = None

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.timer = self.create_timer(0.05, self.timer_cb)  # 20 Hz
        self.get_logger().info(f'Recording waypoints (SLAM pose) → {output_file}  (min_dist={min_dist}m)')
        self.get_logger().info('Drive with joystick. Ctrl+C to stop and save.')

    def timer_cb(self):
        try:
            t = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
        except Exception:
            return

        x = t.transform.translation.x
        y = t.transform.translation.y
        q = t.transform.rotation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))

        if self.last_x is not None:
            if math.hypot(x - self.last_x, y - self.last_y) < self.min_dist:
                return

        self.waypoints.append((x, y, yaw, self.speed_fill))
        self.last_x = x
        self.last_y = y

        n = len(self.waypoints)
        if n % 20 == 0:
            self.get_logger().info(f'{n} points recorded')

    def save(self):
        n = len(self.waypoints)
        if n == 0:
            self.get_logger().warn('No waypoints recorded!')
            return
        with open(self.output_file, 'w') as f:
            for wp in self.waypoints:
                f.write(f'{wp[0]:.4f},{wp[1]:.4f},{wp[2]:.4f},{wp[3]:.2f}\n')
        self.get_logger().info(f'Saved {n} waypoints to {self.output_file}')


def main():
    parser = argparse.ArgumentParser(description='Record waypoints from SLAM pose')
    parser.add_argument('-o', '--output', default='waypoints.csv',
                        help='Output CSV file (default: waypoints.csv)')
    parser.add_argument('--dist', type=float, default=0.05,
                        help='Min distance between recorded points in meters (default: 0.05)')
    parser.add_argument('--speed', type=float, default=0.5,
                        help='Speed value to fill in CSV (default: 0.5)')
    argv = [a for a in sys.argv[1:] if not a.startswith('--ros-args') and a != '-r']
    args = parser.parse_args(argv)

    rclpy.init()
    node = WaypointRecorder(args.output, args.dist, args.speed)

    def shutdown_handler(sig, frame):
        node.save()
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(0)

    signal.signal(signal.SIGINT, shutdown_handler)

    rclpy.spin(node)


if __name__ == '__main__':
    main()
