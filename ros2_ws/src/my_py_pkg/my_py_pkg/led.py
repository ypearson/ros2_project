#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from my_robot_interfaces.srv import SetLed
from my_robot_interfaces.msg import LedPanelStates

node_name = 'led'

class LedNode(Node):
    def __init__(self):
        super().__init__(f'{node_name}')
        self.get_logger().info(f'{node_name} has started...')
        self.service = self.create_service(SetLed, "set_led", self.callback)
        self.leds=[0,0,0]
        self.publisher = self.create_publisher(LedPanelStates, "led_panel_states", 10)
        self.create_timer(1, self.timer_callback)
    def timer_callback(self):
        msg = LedPanelStates()
        msg.data = self.leds
        self.publisher.publish(msg)
        self.get_logger().info(f'Publishing: {msg.data}')
    def callback(self, request, response):
        if request.led_number < 3:
            response.success = True
        self.leds[request.led_number] = 1 if request.state else 0
        print(self.leds)
        return response

def main(args=None):
    rclpy.init(args=args)
    node = LedNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()