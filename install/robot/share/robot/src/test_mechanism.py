#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from rclpy.action import ActionClient
from std_msgs.msg import String

class Nav2SerialNotifier(Node):
    def __init__(self):
        super().__init__('nav2_serial_notifier')

        # Create ActionClient for Nav2
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # Serial publisher
        self.serial_pub = self.create_publisher(String, '/custom_serial_topic', 10)

        # Subscribe to RViz goal topic
        self.goal_sub = self.create_subscription(
            PoseStamped,
            '/goal_pose',  # This is what RViz publishes
            self.goal_callback,
            10
        )

        self.get_logger().info('Waiting for goals from RViz...')

    def goal_callback(self, pose_msg):
        self.get_logger().info(f"Received goal from RViz: {pose_msg.pose.position.x}, {pose_msg.pose.position.y}")
        goal = NavigateToPose.Goal()
        goal.pose = pose_msg

        self.nav_client.wait_for_server()
        self.send_goal_future = self.nav_client.send_goal_async(goal)
        self.send_goal_future.add_done_callback(self.goal_response_callback)

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
