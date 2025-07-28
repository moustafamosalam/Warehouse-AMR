#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from rclpy.action import ActionClient
from std_msgs.msg import String
import tf_transformations
import paho.mqtt.client as mqtt
import threading

class Nav2SerialNotifier(Node):
    def __init__(self):
        super().__init__('nav2_serial_notifier')

        # Nav2 Action Client
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # Serial Publisher
        self.serial_pub = self.create_publisher(String, '/custom_serial_topic', 10)

        # RViz goal subscriber
        self.goal_sub = self.create_subscription(
            PoseStamped,
            '/goal_pose',
            self.goal_callback,
            10
        )

        # MQTT Setup
        self.mqtt_client = mqtt.Client()
        self.mqtt_client.on_connect = self.on_mqtt_connect
        self.mqtt_client.on_message = self.on_mqtt_message
        self.mqtt_client.connect("localhost", 1883, 60)
        threading.Thread(target=self.mqtt_client.loop_forever, daemon=True).start()

        self.get_logger().info('Waiting for goals from RViz or MQTT...')

    def on_mqtt_connect(self, client, userdata, flags, rc):
        self.get_logger().info("Connected to MQTT broker")
        client.subscribe("robot/target_pose")

    def on_mqtt_message(self, client, userdata, msg):
        try:
            self.get_logger().info(f"MQTT received: {msg.payload.decode()}")
            x, y, theta = map(float, msg.payload.decode().split(","))
            self.send_nav_goal(x, y, theta)
        except Exception as e:
            self.get_logger().error(f"Error parsing MQTT msg: {e}")

    def goal_callback(self, pose_msg):
        # Extract yaw (theta) from quaternion
        orientation_q = pose_msg.pose.orientation
        _, _, yaw = tf_transformations.euler_from_quaternion([
            orientation_q.x,
            orientation_q.y,
            orientation_q.z,
            orientation_q.w
        ])
        self.get_logger().info(f"Received goal from RViz: {pose_msg.pose.position.x}, {pose_msg.pose.position.y}, theta={yaw} rad")
        self.send_nav_goal_pose(pose_msg)

    def send_nav_goal_pose(self, pose_msg):
        goal = NavigateToPose.Goal()
        goal.pose = pose_msg

        self.nav_client.wait_for_server()
        self.send_goal_future = self.nav_client.send_goal_async(goal)
        self.send_goal_future.add_done_callback(self.goal_response_callback)

    def send_nav_goal(self, x, y, theta):
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y

        q = tf_transformations.quaternion_from_euler(0, 0, theta)
        pose.pose.orientation.x = q[0]
        pose.pose.orientation.y = q[1]
        pose.pose.orientation.z = q[2]
        pose.pose.orientation.w = q[3]

        self.send_nav_goal_pose(pose)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn("Goal was rejected.")
            return
        self.get_logger().info("Goal accepted.")
        self.result_future = goal_handle.get_result_async()
        self.result_future.add_done_callback(self.goal_result_callback)

    def goal_result_callback(self, future):
        result = future.result().result
        self.get_logger().info("Goal reached. Sending serial message.")
        msg = String()
        msg.data = "M"
        self.serial_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = Nav2SerialNotifier()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
