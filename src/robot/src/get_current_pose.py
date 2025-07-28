#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import tf2_ros
from tf_transformations import euler_from_quaternion
from builtin_interfaces.msg import Time


class CurrentPoseGetter(Node):
    def __init__(self):
        super().__init__('current_pose_getter')
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.get_current_pose()

    def get_current_pose(self):
        try:
            # Lookup transform from map -> base_link
            now = rclpy.time.Time()
            trans = self.tf_buffer.lookup_transform(
                'map',
                'base_link',
                now,
                timeout=rclpy.duration.Duration(seconds=1.0)
            )

            pose = PoseStamped()
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.header.frame_id = "map"
            pose.pose.position.x = trans.transform.translation.x
            pose.pose.position.y = trans.transform.translation.y
            pose.pose.position.z = trans.transform.translation.z
            pose.pose.orientation = trans.transform.rotation

            self.get_logger().info(f"Current pose:\n{pose}")
            # Save or return it from here depending on your use case
        except Exception as e:
            self.get_logger().warn(f"Could not get transform: {e}")


def main():
    rclpy.init()
    node = CurrentPoseGetter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()


if __name__ == '__main__':
    main()
