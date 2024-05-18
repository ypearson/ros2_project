import rclpy
from rclpy.node import Node
from my_robot_interfaces.msg import GraphNodeData
from cv_bridge import CvBridge
import cv2
import numpy as np

class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('image_subscriber')
        self.subscription = self.create_subscription(GraphNodeData,'graph_node_data', self.listener_callback, 10)
        self.bridge = CvBridge()
        self.get_logger().info('Image Subscriber node has been started.')

    def listener_callback(self, msg):
        self.get_logger().info(f'Received image with gnode_name: {msg.gnode_name}')
        self.get_logger().info(f'Received gnode_edges: {msg.gnode_edges}')

        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg.image, desired_encoding='bgr8')
            cv2.imshow("Received Image", cv_image)
            cv2.waitKey(1)
        except CvBridgeError as e:
            self.get_logger().error(f'CvBridge Error: {e}')

def main(args=None):
    rclpy.init(args=args)
    image_subscriber = ImageSubscriber()
    rclpy.spin(image_subscriber)
    image_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
