#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import math
import numpy as np

class LidarSelfFilterNode(Node):
    def __init__(self):
        super().__init__('lidar_self_filter_node')

        # Parameters for the filter box (in robot base frame)
        self.min_x = -0.63
        self.max_x =  0.05
        self.min_y = -0.25
        self.max_y =  0.25

        self.subscription = self.create_subscription(
            LaserScan,
            '/scan_raw',   # original LiDAR topic
            self.scan_callback,
            10
        )
        self.publisher = self.create_publisher(
            LaserScan,
            '/scan',       # filtered topic
            10
        )

    def scan_callback(self, msg: LaserScan):
        ranges = np.array(msg.ranges)
        angle = msg.angle_min

        for i in range(len(ranges)):
            r = ranges[i]
            if math.isinf(r) or math.isnan(r):
                angle += msg.angle_increment
                continue

            x = r * math.cos(angle)
            y = r * math.sin(angle)

            if self.min_x <= x <= self.max_x and self.min_y <= y <= self.max_y:
                ranges[i] = float('nan')  # or set to 0 or inf to indicate filtered
            angle += msg.angle_increment

        filtered_msg = LaserScan()
        filtered_msg.header = msg.header
        filtered_msg.angle_min = msg.angle_min
        filtered_msg.angle_max = msg.angle_max
        filtered_msg.angle_increment = msg.angle_increment
        filtered_msg.time_increment = msg.time_increment
        filtered_msg.scan_time = msg.scan_time
        filtered_msg.range_min = msg.range_min
        filtered_msg.range_max = msg.range_max
        filtered_msg.ranges = ranges.tolist()
        filtered_msg.intensities = msg.intensities

        self.publisher.publish(filtered_msg)


def main(args=None):
    rclpy.init(args=args)
    node = LidarSelfFilterNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
