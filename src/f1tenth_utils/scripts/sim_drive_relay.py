#!/usr/bin/env python3
"""
Relay node for simulation: converts /drive (AckermannDriveStamped)
to /autodrive/f1tenth_1/throttle_command and steering_command (Float32).
"""
import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import Float32
from f1tenth_utils.constants import MAX_SPEED, MAX_STEERING_ANGLE as MAX_STEERING, DRIVE_TOPIC


class SimDriveRelay(Node):
    def __init__(self):
        super().__init__('sim_drive_relay')
        self.pub_throttle = self.create_publisher(Float32, '/autodrive/f1tenth_1/throttle_command', 10)
        self.pub_steering = self.create_publisher(Float32, '/autodrive/f1tenth_1/steering_command', 10)
        self.create_subscription(AckermannDriveStamped, DRIVE_TOPIC, self.drive_cb, 10)
        self.get_logger().info('SimDriveRelay ready')

    def drive_cb(self, msg):
        throttle = Float32()
        throttle.data = float(max(-1.0, min(1.0, msg.drive.speed / MAX_SPEED)))

        steering = Float32()
        steering.data = float(max(-1.0, min(1.0, msg.drive.steering_angle / MAX_STEERING)))

        self.pub_throttle.publish(throttle)
        self.pub_steering.publish(steering)


def main(args=None):
    rclpy.init(args=args)
    node = SimDriveRelay()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
