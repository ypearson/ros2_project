#!/usr/bin/env python3
from functools import partial
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

node_name = 'add_two_ints_client'

class AddTwoIntsClientNode(Node):
    def __init__(self):
        super().__init__(f'{node_name}')
        self.get_logger().info(f'{node_name} has started...')
        self.client_ = self.create_client(AddTwoInts, 'add_two_ints')
        self.call_add_two_ints_server(6,7)
        self.call_add_two_ints_server(6,8)
        self.call_add_two_ints_server(6,9)
    def call_add_two_ints_server(self, a, b):
        client = self.create_client(AddTwoInts, 'add_two_ints')
        # Check if server is up
        while not client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for Server...")
        request = AddTwoInts.Request()
        request.a = a
        request.b = b
        future = client.call_async(request)
        future.add_done_callback(partial(self.callback_add_two_ints, a=a, b=b))
    def callback_add_two_ints(self, future, a, b):
        try:
            response = future.result()
            self.get_logger().info(f'{a} + {b} = {response.sum}')
        except Exception as e:
            self.get_logger().error('Service called failed: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = AddTwoIntsClientNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()