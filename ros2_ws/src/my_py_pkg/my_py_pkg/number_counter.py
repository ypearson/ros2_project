#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int64
from example_interfaces.srv import SetBool

node_name = 'number_counter'

class NumberCounterNode(Node):
    def __init__(self):
        super().__init__(f'{node_name}')
        self.get_logger().info(f'{node_name} has started...')
        self.publisher = self.create_publisher(Int64, "number_count", 10)
        self.subscription = self.create_subscription(Int64, "number", self.getMsg, 10)
        self.service = self.create_service(SetBool, "reset_counter", self.service_cb)
        self.counter = 0
    def getMsg(self, msg):
        self.counter += msg.data
        self.publisher.publish(msg)
    def service_cb(self, request, response):
        if request.data:
            self.counter = 0
            response.success = True
            response.message = "py ok"
        else:
            response.success = False
            response.message = "py nope"
        self.get_logger().info(f'{response.message}')
        return response
def main(args=None):
    rclpy.init(args=args)
    node = NumberCounterNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()