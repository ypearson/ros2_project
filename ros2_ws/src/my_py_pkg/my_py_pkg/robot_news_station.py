#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from example_interfaces.msg import String

class RobotNewsStationNode(Node):
    def __init__(self):
        super().__init__("robot_news_station")
        self.robot_name = 'C3PO'
        self.publisher_ = self.create_publisher(String, "robot_news", 10)
        self.timer = self.create_timer(1, self.publish_news)
        self.get_logger().info("robot_news_station has started...")

    def publish_news(self):
        msg = String()
        msg.data = f'msg.data being published by {self.robot_name}'
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = RobotNewsStationNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()