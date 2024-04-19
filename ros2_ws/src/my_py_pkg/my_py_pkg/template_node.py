#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

node_name = 'node_name'  # MODIFY NAME

class TemplateNode(Node):
    def __init__(self):
        super().__init__(f'{node_name}')
        self.get_logger().info(f'{node_name} has started...')

def main(args=None):
    rclpy.init(args=args)
    node = TemplateNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()