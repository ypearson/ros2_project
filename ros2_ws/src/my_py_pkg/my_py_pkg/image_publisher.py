import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class ImagePublisher(Node):
    def __init__(self):
        super().__init__('image_publisher')
        self.publisher_ = self.create_publisher(Image, 'image_topic', 10)
        self.timer = self.create_timer(0.5, self.timer_callback)
        self.bridge = CvBridge()
        import os
        self.get_logger().info(f'Current working directory: {os.getcwd()}')
        self.image_path = 'src/my_py_pkg/my_py_pkg/bike.jpg'
        self.get_logger().info('Image Publisher node has been started.')

    def timer_callback(self):
        img = cv2.imread(self.image_path)
        if img is not None:
            msg = self.bridge.cv2_to_imgmsg(img, encoding='bgr8')
            self.publisher_.publish(msg)
            self.get_logger().info('Publishing image.')
        else:
            self.get_logger().error('Failed to read image.')

def main(args=None):
    rclpy.init(args=args)
    image_publisher = ImagePublisher()
    rclpy.spin(image_publisher)
    image_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
