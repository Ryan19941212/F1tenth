#!/usr/bin/env python3
"""
LiDAR orientation verification tool.

Usage:
  1. Launch bringup_launch.py
  2. Place an obstacle (hand / box) ~30 cm in front of the car
  3. Run: python3 verify_lidar.py
  4. Check if the reported direction matches "Front"
"""
import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class LidarVerifier(Node):
    def __init__(self):
        super().__init__('lidar_verifier')
        self.count = 0
        self.create_subscription(LaserScan, '/scan', self.cb, 10)

    def cb(self, msg):
        self.count += 1
        if self.count < 3:
            return  # skip first few frames
        if self.count > 5:
            raise SystemExit(0)

        n = len(msg.ranges)
        print(f'\n{"="*60}')
        print(f'  LiDAR orientation check ({n} points)')
        print(f'  angle_min={math.degrees(msg.angle_min):.1f} deg  '
              f'angle_max={math.degrees(msg.angle_max):.1f} deg')
        print(f'{"="*60}')

        # Direction sectors (+/- 15 deg each)
        sectors = {
            'Front': 0.0,
            'Left':  90.0,
            'Rear':  180.0,
            'Right': -90.0,
        }

        half_width = 15.0  # degrees

        for name, center_deg in sectors.items():
            center_rad = math.radians(center_deg)
            dists = []
            for i in range(n):
                angle = msg.angle_min + i * msg.angle_increment
                diff = abs(math.atan2(
                    math.sin(angle - center_rad),
                    math.cos(angle - center_rad)))
                if diff < math.radians(half_width):
                    r = msg.ranges[i]
                    if msg.range_min < r < msg.range_max:
                        dists.append(r)

            if dists:
                min_d = min(dists)
                avg_d = sum(dists) / len(dists)
                print(f'  {name:10s}  min={min_d:.3f}m  avg={avg_d:.3f}m  pts={len(dists)}')
            else:
                print(f'  {name:10s}  -- no valid data --')

        # Overall closest point
        min_r = float('inf')
        min_angle = 0.0
        for i in range(n):
            r = msg.ranges[i]
            if msg.range_min < r < msg.range_max and r < min_r:
                min_r = r
                min_angle = msg.angle_min + i * msg.angle_increment

        if min_r < float('inf'):
            deg = math.degrees(min_angle)
            if -45 < deg < 45:
                direction = 'Front'
            elif 45 <= deg < 135:
                direction = 'Left'
            elif deg >= 135 or deg <= -135:
                direction = 'Rear'
            else:
                direction = 'Right'
            print(f'\n  * Closest obstacle: {min_r:.3f}m @ {deg:+.1f} deg -> {direction}')
            print(f'    Obstacle is in front and shows "Front" -> LiDAR OK')
            print(f'    Any other direction -> LiDAR needs calibration')
        print()


def main():
    rclpy.init()
    node = LidarVerifier()
    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
