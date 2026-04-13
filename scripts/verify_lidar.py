#!/usr/bin/env python3
"""
LiDAR 方向驗證工具
用法：
  1. 啟動 bringup_launch.py
  2. 在車頭正前方 ~30cm 處放一個障礙物（手或箱子）
  3. 執行此腳本：python3 verify_lidar.py
  4. 看結果是否正確
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
        print(f'  LiDAR 方向驗證（共 {n} 點）')
        print(f'  angle_min={math.degrees(msg.angle_min):.1f}°  '
              f'angle_max={math.degrees(msg.angle_max):.1f}°')
        print(f'{"="*60}')

        # Define direction sectors (±15° each)
        sectors = {
            '車頭 (Front)': 0.0,
            '車左 (Left)':  90.0,
            '車尾 (Rear)':  180.0,
            '車右 (Right)': -90.0,
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
                print(f'  {name:20s}  min={min_d:.3f}m  avg={avg_d:.3f}m  pts={len(dists)}')
            else:
                print(f'  {name:20s}  -- 無有效資料 --')

        # Find overall closest point
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
                direction = '車頭 ✅' if min_r < 0.5 else '車頭'
            elif 45 <= deg < 135:
                direction = '車左'
            elif deg >= 135 or deg <= -135:
                direction = '車尾'
            else:
                direction = '車右'
            print(f'\n  ★ 最近障礙物: {min_r:.3f}m @ {deg:+.1f}° → {direction}')
            print(f'    如果障礙物在車頭且顯示「車頭」→ LiDAR 方向正確 ✅')
            print(f'    如果顯示其他方向 → LiDAR 需要校正 ❌')
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
