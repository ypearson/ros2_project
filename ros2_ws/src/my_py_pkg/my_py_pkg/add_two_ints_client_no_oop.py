#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from example_interfaces.srv import AddTwoInts

node_name = 'add_two_ints_client'

def main(args=None):
    rclpy.init(args=args)
    node = Node(node_name)
    client = node.create_client(AddTwoInts, 'add_two_ints')
    # Check if server is up
    while not client.wait_for_service(1.0):
        node.get_logger().warn("Waiting for Server...")
    request = AddTwoInts.Request()

    request.a = 3
    request.b = 8

    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future)

    try:
        response = future.result()
        node.get_logger().info(f'{response.sum=}')
    except Exception as e:
        node.get_logger().error(f'Service call failed {e}')

    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()