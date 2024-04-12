#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int64

node_name = 'number_counter'

class NumberCounterNode(Node):
    def __init__(self):
        super().__init__(f'{node_name}')
        self.get_logger().info(f'{node_name} has started...')
        self.publisher = self.create_publisher(Int64, "number_count", 10)
        self.subscription = self.create_subscription(Int64, "number", self.getMsg, 10)
    def getMsg(self, msg):
        self.publisher.publish(msg)
def main(args=None):
    rclpy.init(args=args)
    node = NumberCounterNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()