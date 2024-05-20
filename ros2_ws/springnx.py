import networkx as nx
import matplotlib.pyplot as plt

# Create a graph
G = nx.erdos_renyi_graph(10, 0.2)

# Compute the spring layout
# The spring_layout function is used to position nodes in a graph using the
# Fruchterman-Reingold force-directed algorithm.
# The Fruchterman-Reingold algorithm is designed to position the nodes of a graph in such a way that
# all the edges are of more or less equal length and there are as few crossing edges as possible. It
# simulates a physical system where nodes repel each other like charged particles and edges act like
# springs that attract the connected nodes.
pos = nx.spring_layout(G)

# Draw the graph
nx.draw(G, pos, with_labels=True, node_color='lightblue', edge_color='gray', node_size=500, font_size=10)

# Show the plot
plt.show()
