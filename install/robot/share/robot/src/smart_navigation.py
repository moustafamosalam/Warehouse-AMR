#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from std_msgs.msg import String
from geometry_msgs.msg import Twist, PoseStamped
import time
import tf_transformations
import json
import paho.mqtt.client as mqtt
import threading

class SmartNavNode(Node):
    def __init__(self):
        super().__init__('smart_nav_node')

        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.serial_pub = self.create_publisher(String, '/custom_serial_topic', 10)
        self.movePublisher = self.create_publisher(Twist, '/diff_cont/cmd_vel_unstamped', 10)

        # For tracking if we need to go home next
        self.delivery_pose = None
        self.poseData = None
        self.await_serial_response = False
        self.packageLevel = 0
        self.goal_handle = None

        # MQTT setup
        self.mqtt_client = mqtt.Client()

        # Set Last Will Message
        self.mqtt_client.will_set(
            topic="/robot/connection",
            payload="Disconnected",
            qos=1,
            retain=True
        )
        self.mqtt_client.on_connect = self.on_mqtt_connect
        self.mqtt_client.on_message = self.on_mqtt_message
        self.mqtt_client.connect("localhost", 1883, 60)
        threading.Thread(target=self.mqtt_client.loop_forever, daemon=True).start()

        # Serial listener (optional, if you want to wait for Arduino response)
        self.serial_sub = self.create_subscription(
            String,
            '/arduino_serial_response',  # your Arduino should publish here
            self.serial_response_callback,
            10
        )

        self.get_logger().info('Smart Nav Node ready.')

    def on_mqtt_connect(self, client, userdata, flags, rc):
        self.get_logger().info('Connected to MQTT broker')
        client.publish("/robot/connection", "Connected", qos=1, retain=True)

        client.subscribe("/robot/goal")
        client.subscribe("/robot/lift")
        client.subscribe("/robot/gripper")
        client.subscribe("/robot/grippermove")
        client.subscribe("/robot/move")
        client.subscribe("/robot/emergency")
        client.subscribe("/robot/t")

    def on_mqtt_disconnect(self, client, userdata, rc):
        self.get_logger().warn('Disconnected from MQTT broker')
        client.publish("/robot/connection", "Disconnected", qos=1, retain=True)

    def on_mqtt_message(self, client, userdata, msg):
        try:
            topic = msg.topic
            payload = msg.payload.decode()
            self.get_logger().info(topic)
            self.get_logger().info(payload)
            if topic == "/robot/goal":
                data = json.loads(payload)
                self.poseData = data
                self.delivery_pose = data.get("delivery", None)
                self.get_logger().info(f"Delivery pose: {self.delivery_pose}")

                if "target" in data:
                    self.get_logger().info("Received target pose from MQTT")
                    self.await_serial_response = True
                    self.send_nav_goal(data["target"])
                elif "home" in data:
                    self.get_logger().info("Received home-only pose from MQTT")
                    self.await_serial_response = False
                    self.send_nav_goal(data["home"])

            elif topic == "/robot/t":
                self.serial_pub.publish(String(data="M 1"))  # Or your actual message

            elif topic == "/robot/move":
                data = json.loads(payload)
                moveMsg = Twist()
                moveMsg.linear.x =  data["x"]
                moveMsg.angular.z = -data["z"]
                self.movePublisher.publish(moveMsg)

            elif topic == "/robot/lift":
                if(payload == '"liftup"'):
                    self.serial_pub.publish(String(data="L 1"))
                    self.mqtt_client.publish("/robot/status", "Lifting Up")
                elif(payload == '"liftdown"'):
                    self.serial_pub.publish(String(data="L 2"))
                    self.mqtt_client.publish("/robot/status", "Lifting Down")
                elif(payload == '"liftstop"') :
                    self.serial_pub.publish(String(data="L 0"))
                    self.mqtt_client.publish("/robot/status", "Lifting Stopped")

            elif topic == "/robot/gripper":
                if(payload == '"gripperon"'):
                    self.serial_pub.publish(String(data="S 1"))
                    self.mqtt_client.publish("/robot/status", "Suction On")
                elif(payload == '"gripperoff"'):
                    self.serial_pub.publish(String(data="S 0"))
                    self.mqtt_client.publish("/robot/status", "Suction Off")

            elif topic == "/robot/grippermove":
                if(payload == '"gripperfront"'):
                    self.serial_pub.publish(String(data="G 1"))
                    self.mqtt_client.publish("/robot/status", "Gripping Forward")
                elif(payload == '"gripperback"'):
                    self.serial_pub.publish(String(data="G 0"))
                    self.mqtt_client.publish("/robot/status", "Gripping Backward")
            elif topic == "/robot/emergency":
                self.serial_pub.publish(String(data="X"))
                self.mqtt_client.publish("/robot/status", "Emergency")
                self.cancel_goal()
                

        except Exception as e:
            self.get_logger().error(f"Invalid MQTT JSON: {e}")

    def send_nav_goal(self, pose_dict):
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = pose_dict["x"]
        pose.pose.position.y = pose_dict["y"]
        if(pose_dict.get("level", None) != None):
            self.packageLevel = pose_dict["level"]
        else:
            self.packageLevel = 0

        q = tf_transformations.quaternion_from_euler(0, 0, pose_dict["theta"])
        pose.pose.orientation.x = q[0]
        pose.pose.orientation.y = q[1]
        pose.pose.orientation.z = q[2]
        pose.pose.orientation.w = q[3]

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose

        self.nav_client.wait_for_server()
        self.send_goal_future = self.nav_client.send_goal_async(goal_msg)
        self.send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        self.goal_handle = future.result()
        if not self.goal_handle.accepted:
            self.get_logger().warn("Goal was rejected.")
            return
        self.get_logger().info("Goal accepted.")
        if self.await_serial_response:
            self.mqtt_client.publish("/robot/status", "Getting Package...")
        elif self.delivery_pose:    
            self.mqtt_client.publish("/robot/status", "Delivering Package...")
        else:
            self.mqtt_client.publish("/robot/status", "Going Home...")
        self.result_future = self.goal_handle.get_result_async()
        self.result_future.add_done_callback(self.goal_result_callback)

    def goal_result_callback(self, future):
        result = future.result().result
        self.get_logger().info("Goal reached.")
        if self.await_serial_response:
            time.sleep(2)  # Delay for 2 seconds
            self.get_logger().info("Sending serial command to Arduino...")
            self.serial_pub.publish(String(data=f"M {self.packageLevel}"))  # Or your actual message
            # wait for serial response before going home
        elif self.delivery_pose:
            self.mqtt_client.publish("/robot/status", "Package Delivered")
            self.serial_pub.publish(String(data="S 0"))  # Turn off Suction
            self.get_logger().info("Package Delivered")
        else:
            self.mqtt_client.publish("/robot/status", "Home Reached")
            self.get_logger().info("Home Reached")

    def cancel_goal(self):
        if self.goal_handle:
            self.get_logger().info('Canceling goal...')
            cancel_future = self.goal_handle.cancel_goal_async()
            cancel_future.add_done_callback(self.cancel_done)

    def cancel_done(self, future):
        cancel_response = future.result()
        if len(cancel_response.goals_canceling) > 0:
            self.get_logger().info('Goal successfully canceled.')
        else:
            self.get_logger().info('Goal failed to cancel.')

    def serial_response_callback(self, msg):
        self.get_logger().info(f"Arduino: {msg.data.strip()}")
        if self.await_serial_response and "packagePicked" in msg.data.strip():
            self.get_logger().info("Received packagePicked from Arduino. Now Delivering.")
            self.await_serial_response = False  # Avoid resending
            self.send_nav_goal(self.poseData["delivery"])
        elif "liftDone" in msg.data:
            self.get_logger().info("Operation Done")
            self.mqtt_client.publish("/robot/status", "Idle")
        elif msg.data.strip() == "suctionDone":
            self.get_logger().info("Operation Done")
            self.mqtt_client.publish("/robot/status", "Suction Done")
        elif "grippingDone" in msg.data:
            self.get_logger().info("Operation Done")
            self.mqtt_client.publish("/robot/status", "Idle")

def main(args=None):
    rclpy.init(args=args)
    node = SmartNavNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
