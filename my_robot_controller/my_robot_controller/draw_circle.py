#!/usr/bin/python3.10
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class DrawCircleNode(Node):
    def __init__(self):
        super().__init__("draw_circle")
        #Create the publisher
        self.cmd_vel_pub_ = self.create_publisher(Twist, "turtle1/cmd_vel", 10) #10 is the buffer size
        self.timer_ = self.create_timer(0.1, self.send_cmd_velocity)
        self.get_logger().info("Drawing a circle Node has started!")
    
    def send_cmd_velocity(self):
        msg = Twist()
        msg.linear.x = 2.0
        msg.angular.z = 1.0
        self.cmd_vel_pub_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = DrawCircleNode()
    rclpy.spin(node)
    rclpy.shutdown()