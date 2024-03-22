#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

class MyNode(Node):
    def __init__(self, name):
        super().__init__(name)
        self.counter = 0
        self.get_logger().info("Node __init__")
        self.create_timer(0.5, self.timer_callback)
    def timer_callback(self):
        self.counter += 1
        self.get_logger().info(f'timer_callback {self.counter}')

def main(args=None):
    rclpy.init(args=args)
    node = MyNode('py_test')

    node.get_logger().info("ROS2 Node")

    rclpy.spin(node)
    rclpy.shutdown() # Last line

if __name__ == '__main__':
    main()