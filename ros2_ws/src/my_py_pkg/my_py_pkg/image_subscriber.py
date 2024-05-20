import rclpy
from rclpy.node import Node
from my_robot_interfaces.msg import GraphNodeData
from cv_bridge import CvBridge
import cv2
import numpy as np
import networkx as nx
import matplotlib
import matplotlib.pyplot as plt
from matplotlib.offsetbox import OffsetImage, AnnotationBbox
import numpy as np


class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('image_subscriber')
        self.subscription = self.create_subscription(GraphNodeData,'graph_node_data', self.listener_callback, 10)
        self.bridge = CvBridge()
        self.get_logger().info('Image Subscriber node has been started.')
        # Create an empty graph
        self.G = nx.Graph()
        self.G.add_node(0)

        # Initialize Matplotlib figure and axis
        plt.ion()  # Enable interactive mode
        self.fig, self.ax = plt.subplots()
        # self.fig, self.ax = plt.subplots(figsize=(12, 12))  # Set figure size to 12x12 inches
        self.pos = nx.spring_layout(self.G)
        self.image_dict = {}  # Dictionary to store images for nodes
        self.cnt = 0

    def update_graph(self, new_node, edges, image):
        self.G.add_node(new_node)

        # Add edges
        for edge in edges:
            self.G.add_edge(new_node, edge)
        
        # Update positions and redraw the graph
        self.pos = nx.spring_layout(self.G, pos=self.pos, fixed=list(self.pos.keys()))
        self.image_dict[self.cnt] = image  # Store the image in the dictionary
        self.draw_graph_with_images(self.G, self.pos, self.ax)
        self.fig.canvas.draw_idle()

    def draw_graph_with_images(self, G, pos, ax):
        ax.clear()
        for node, (x, y) in pos.items():
            image = self.image_dict.get(node)
            if image is not None:
                imagebox = OffsetImage(image, zoom=0.2)  # Adjust the zoom level as needed
                ab = AnnotationBbox(imagebox, (x, y), frameon=False)
                ax.add_artist(ab)
        nx.draw_networkx_edges(G, pos, ax=ax)
        nx.draw_networkx_labels(G, pos, ax=ax, font_size=10, font_color='black')
        plt.draw()
        plt.pause(0.1)

    def listener_callback(self, msg):
        self.get_logger().info(f'Received image with gnode_name: {msg.gnode_name}')
        self.get_logger().info(f'Received gnode_edges: {msg.gnode_edges}')

        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg.image, desired_encoding='bgr8')
            # Convert BGR which is OpenCV default to RGB
            image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
            self.update_graph(msg.gnode_name, msg.gnode_edges, image)

        except CvBridgeError as e:
            self.get_logger().error(f'CvBridge Error: {e}')

def main(args=None):
    matplotlib.use('TkAgg')
    rclpy.init(args=args)
    image_subscriber = ImageSubscriber()
    rclpy.spin(image_subscriber)
    image_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
