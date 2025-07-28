#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import math

class SimpleScanFilter(Node):
    def __init__(self):
        super().__init__('simple_scan_filter')
        self.sub = self.create_subscription(LaserScan, '/scan_raw', self.callback, 10)
        self.pub = self.create_publisher(LaserScan, '/scan', 10)

    def callback(self, msg):
        new_msg = LaserScan()
        new_msg.header = msg.header
        new_msg.angle_min = msg.angle_min
        new_msg.angle_max = msg.angle_max
        new_msg.angle_increment = msg.angle_increment
        new_msg.time_increment = msg.time_increment
        new_msg.scan_time = msg.scan_time
        new_msg.range_min = msg.range_min
        new_msg.range_max = msg.range_max
        new_msg.ranges = list(msg.ranges)
        new_msg.intensities = list(msg.intensities)

        for i in range(len(new_msg.ranges)):
            angle = msg.angle_min + i * msg.angle_increment
            if angle > 1.57 or angle < -1.57:
                new_msg.ranges[i] = float('nan')

        self.pub.publish(new_msg)

def main(args=None):
    rclpy.init(args=args)
    node = SimpleScanFilter()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

