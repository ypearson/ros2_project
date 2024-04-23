#!/usr/bin/env python3
from functools import partial
import rclpy
from rclpy.node import Node

from my_robot_interfaces.srv import SetLed

node_name = 'battery'

class BatteryNode(Node):
    def __init__(self):
        super().__init__(f'{node_name}')
        self.get_logger().info(f'{node_name} has started...')
        self.state = 10 # full battery
        self.create_timer(1.0, self.statemachine)
    def statemachine(self):
        if self.state == 0:
                self.state = 10

        if self.state < 6:
            self.service_call(1, True)
        else:
            self.service_call(1, False)

        print(f'battery={self.state}')
        self.state -= 1

    def service_call(self, led_num, led_state):
        request = SetLed.Request()
        request.led_number = led_num
        request.state = led_state
        client = self.create_client(SetLed, 'set_led')
        while not client.wait_for_service(2):
            self.get_logger().warn("Waiting for Server...")
        future = client.call_async(request)
        future.add_done_callback(partial(self.callback))
    def callback(self, future):
        try:
            response = future.result()
            self.get_logger().info(f'{response.success=}')
        except Exception as e:
            self.get_logger().error('Service called failed: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = BatteryNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()