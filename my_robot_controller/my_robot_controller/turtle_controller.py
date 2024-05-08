#!/usr/bin/python3.10
from functools import partial
import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
from turtlesim.srv import SetPen


class TurtleControllerNode(Node):
    def __init__(self):
        super().__init__("turtle_controller")
        self.get_logger().info("Turtle Controller Node has been started!")

        self.previous_x = 5.5
        self.pose_subscriber_ = self.create_subscription(Pose, "turtle1/pose", self.pose_callback, 10)
        self.cmd_vel_publisher_ = self.create_publisher(Twist, "turtle1/cmd_vel", 10)

    def pose_callback(self, pose: Pose):
        msg = Twist()

        if pose.x > 9.0 or pose.x < 2 or pose.y > 9.0 or pose.y < 2.0:
            msg.linear.x = 1.0
            msg.angular.z = 0.9
        else:
            msg.linear.x = 5.0
            msg.angular.z = 0.0
        self.cmd_vel_publisher_.publish(msg)

        if pose.x < 5.5 and self.previous_x >= 5.5:
            self.get_logger().info("Turn the pen to Red!!")
            self.call_set_pen_service(255, 0, 0, 3, 0)
        elif pose.x > 5.5 and self.previous_x <= 5.5:
            self.get_logger().info("Turn the pen to Green!!")
            self.call_set_pen_service(0, 255, 0, 3, 0)
        
        self.previous_x = pose.x
    
    def call_set_pen_service(self, r, g, b, width, off):
        set_pen_client = self.create_client(SetPen, "/turtle1/set_pen")
        while not set_pen_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn("Set Pen service not available, waiting again...")
        
        set_pen_request = SetPen.Request()
        set_pen_request.r = r
        set_pen_request.g = g
        set_pen_request.b = b
        set_pen_request.width = width
        set_pen_request.off = off

        future = set_pen_client.call_async(set_pen_request)
        future.add_done_callback(partial(self.set_pen_callback))

    def set_pen_callback(self, future):
        try:
            response = future.result()
            self.get_logger().info(f"Set Pen service response: {response}")
        except Exception as e:
            self.get_logger().error(f"Set Pen service call failed with error: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = TurtleControllerNode()
    rclpy.spin(node)
    rclpy.shutdown()