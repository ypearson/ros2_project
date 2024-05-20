import networkx as nx
import matplotlib.pyplot as plt
from matplotlib.offsetbox import OffsetImage, AnnotationBbox
import numpy as np
import threading
import queue
import time

# Create an empty graph
G = nx.Graph()

# Add an initial node to avoid the empty sequence error
G.add_node(1)

# Initialize the plot
fig, ax = plt.subplots()

# Initialize pos as a global variable
pos = nx.spring_layout(G)

# Dictionary to store images associated with nodes
images = {1: np.random.rand(256, 256, 3)}

# Queue to hold streaming images
image_queue = queue.Queue()

# Function to generate random 256x256 RGB images as NumPy arrays (for demonstration)
def generate_image():
    return np.random.rand(256, 256, 3)

# Function to simulate asynchronous image streaming
def image_streaming():
    while True:
        time.sleep(1)  # Simulate delay in receiving images
        new_image = generate_image()
        image_queue.put(new_image)

# Function to draw nodes with images
def draw_graph_with_images(G, pos, ax):
    ax.clear()
    for node, (x, y) in pos.items():
        image = images.get(node)
        if image is not None:
            imagebox = OffsetImage(image, zoom=0.2)  # Adjust the zoom level as needed
            ab = AnnotationBbox(imagebox, (x, y), frameon=False)
            ax.add_artist(ab)
    nx.draw_networkx_edges(G, pos, ax=ax)
    nx.draw_networkx_labels(G, pos, ax=ax, font_size=10, font_color='black')
    plt.draw()

# Function to update the graph
def update_graph():
    global pos  # Declare pos as global to modify it within the function

    # Check if there are new images in the queue
    if not image_queue.empty():
        new_image = image_queue.get()
        new_node = max(G.nodes) + 1
        G.add_node(new_node)
        if len(G.nodes) > 1:
            G.add_edge(new_node, np.random.choice(list(G.nodes)[:-1]))

        # Add the new image to the images dictionary
        images[new_node] = new_image

        # Update positions and redraw the graph
        pos = nx.spring_layout(G, pos=pos, fixed=list(pos.keys()))
        draw_graph_with_images(G, pos, ax)
        fig.canvas.draw_idle()

# Start the image streaming thread
streaming_thread = threading.Thread(target=image_streaming, daemon=True)
streaming_thread.start()

# Function to continuously check the queue and update the graph
def event_loop():
    while True:
        update_graph()
        time.sleep(0.1)  # Adjust the sleep time as needed to balance responsiveness and CPU usage

# Start the event loop in a separate thread
event_loop_thread = threading.Thread(target=event_loop, daemon=True)
event_loop_thread.start()

# Show the plot
plt.show()
