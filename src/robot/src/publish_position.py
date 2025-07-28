#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg._pose_with_covariance_stamped import PoseWithCovarianceStamped
import math
import paho.mqtt.client as mqtt
import json




class RobotPoseMQTTPublisher(Node):
    def __init__(self):
    # ...
        self.last_pose = None

        super().__init__('mqtt_turtle_node')

        # MQTT setup
        self.mqtt_broker = "192.168.1.4"
        self.mqtt_port = 1883
        self.mqtt_topic = "/robot/pose"

        self.mqtt_client = mqtt.Client()
        self.mqtt_client.on_connect = self.on_connect

        try:
            self.mqtt_client.connect(self.mqtt_broker, self.mqtt_port, 60)
            self.mqtt_client.loop_start()
            self.get_logger().info(f"Connecting to MQTT broker at {self.mqtt_broker}:{self.mqtt_port}")
        except Exception as e:
            self.get_logger().error(f"MQTT connection failed: {e}")

        # ROS2 subscriber
        self.subscription = self.create_subscription(
            PoseWithCovarianceStamped,
            '/pose',
            self.pose_callback,
            10
        )

    def on_connect(self, client, userdata, flags, rc):
        if rc == 0:
            self.get_logger().info("Connected to MQTT broker successfully.")
        else:
            self.get_logger().error(f"Failed to connect to MQTT broker with code {rc}.")

    
    def pose_callback(self, msg):
        self.get_logger().info(f"Received pose: ")

        # Extract yaw (theta) from quaternion
        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        theta = math.atan2(siny_cosp, cosy_cosp)

        pose_data = {'x': msg.pose.pose.position.x, 'y': msg.pose.pose.position.y, 'theta': theta}

        if self.last_pose:
            dx = abs(pose_data['x'] - self.last_pose['x'])
            dy = abs(pose_data['y'] - self.last_pose['y'])
            dtheta = abs(pose_data['theta'] - self.last_pose['theta'])

            if dx < 0.01 and dy < 0.01 and dtheta < 0.01:
                return  # Ignore minor/no changes

        self.last_pose = pose_data
        pose_json = json.dumps(pose_data)
        self.mqtt_client.publish(self.mqtt_topic, pose_json,retain=True)
        self.get_logger().info(f"Published pose to MQTT: {pose_json}")

    def destroy_node(self):
        self.mqtt_client.loop_stop()
        self.mqtt_client.disconnect()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = RobotPoseMQTTPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down...")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()