import rclpy
from rclpy.node import Node
from my_robot_interfaces.msg import CustomImage
from sensor_msgs.msg import Image
from std_msgs.msg import Header
from cv_bridge import CvBridge
import cv2

class ImagePublisher(Node):
    def __init__(self):
        super().__init__('image_publisher')
        self.publisher_ = self.create_publisher(CustomImage, 'image_topic', 10)
        self.timer = self.create_timer(0.5, self.timer_callback)
        self.bridge = CvBridge()
        self.image_path = 'src/my_py_pkg/my_py_pkg/bike.jpg'
        self.get_logger().info('Image Publisher node has been started.')

    def timer_callback(self):
        self.get_logger().info(f'Reading image from {self.image_path}')
        img = cv2.imread(self.image_path)
        if img is not None:
            img_msg = self.bridge.cv2_to_imgmsg(img, encoding='bgr8')

            custom_msg = CustomImage()
            custom_msg.header = Header()
            custom_msg.header.stamp = self.get_clock().now().to_msg()
            custom_msg.image = img_msg
            custom_msg.gnode_name = 'example_node_name'
            custom_msg.gnode_edges = ['edge1', 'edge2', 'edge3']

            self.publisher_.publish(custom_msg)
            self.get_logger().info('Publishing custom image message.')
        else:
            self.get_logger().error(f'Failed to read image from {self.image_path}')

def main(args=None):
    rclpy.init(args=args)
    image_publisher = ImagePublisher()
    rclpy.spin(image_publisher)
    image_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()