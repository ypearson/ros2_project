#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from example_interfaces.msg import String

node_name = 'smartphone'

class SmartphoneNode(Node):
    def __init__(self):
        super().__init__(f'{node_name}')
        self.subscriber_ = self.create_subscription(String, 'robot_news', self.callback_robot_news, 10)
        self.get_logger().info(f'{node_name} has started...')

    def callback_robot_news(self, msg):
        self.get_logger().info(f'New msg: {msg.data}')

def main(args=None):
    rclpy.init(args=args)
    node = SmartphoneNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()