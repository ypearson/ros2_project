import rclpy
from rclpy.node import Node
from my_robot_interfaces.msg import GraphNodeData
from sensor_msgs.msg import Image
from std_msgs.msg import Header
from cv_bridge import CvBridge
import cv2
import random

# Assuming 'msg' is a sensor_msgs/Image message
# cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
# Assuming 'cv_image' is a NumPy array representing an image
# ros_image_msg = bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')

gnode_name = 'node'
graph_node_count = 0
graph_node_list = []
random_graph_node = lambda: random.choice(graph_node_list)

node_name = 'graph_node_image_generator'
class ImagePublisher(Node):
    def __init__(self):
        super().__init__(f'{node_name}')
        self.get_logger().info(f'{node_name} has started...')
        self.publisher_ = self.create_publisher(GraphNodeData, 'graph_node_data', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.bridge = CvBridge()
        self.image_path = 'src/my_py_pkg/my_py_pkg/bike.jpg'
        self.get_logger().info('Image Publisher node has been started.')

    def timer_callback(self):
        self.get_logger().info(f'Reading image from {self.image_path}')
        img = cv2.imread(self.image_path)
        if img is not None:
            img_msg = self.bridge.cv2_to_imgmsg(img, encoding='bgr8')

            graph_node_data_msg = GraphNodeData()
            graph_node_data_msg.header = Header()
            graph_node_data_msg.header.stamp = self.get_clock().now().to_msg()
            graph_node_data_msg.image = img_msg
            graph_node_data_msg.gnode_name = 'example_node_name'
            graph_node_data_msg.gnode_edges = ['edge1', 'edge2', 'edge3']

            self.publisher_.publish(graph_node_data_msg)
            self.get_logger().info('Publishing graph_node_data image message.')
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