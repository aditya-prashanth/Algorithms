import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import numpy as np
from collections import deque

class GraphSubscriber(Node):
    """
    A class representing a graph subscriber node.

    This class subscribes to a graph topic and performs a breadth-first search (BFS)
    algorithm to find a target node in the graph.

    Attributes:
        visited (set): A set to keep track of visited nodes during BFS.
        queue (deque): A queue to store nodes to be visited during BFS.
        target_found (bool): A flag indicating whether the target node has been found.

    Methods:
        listener_callback: Callback function for the graph topic subscription.
        bfs: Performs the breadth-first search algorithm on the graph.
        get_neighbors: Returns the neighboring nodes of a given node in the graph.
    """

    def __init__(self):
        super().__init__('graph_subscriber')
        self.subscription = self.create_subscription(
            String,
            'graph_topic',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.visited = set()
        self.queue = deque()
        self.target_found = False

    def listener_callback(self, msg):
        """
        Callback function for the graph topic subscription.

        This function is called whenever a new message is received on the graph topic.
        It converts the message data into a graph and target node, and then calls the
        BFS algorithm to search for the target node in the graph.

        Args:
            msg (String): The received message containing the graph and target node data.
        """
        if not self.target_found:
            data = json.loads(msg.data)
            graph = np.array(data['graph'])
            target = tuple(data['target'])
            self.bfs(graph, target)

    def bfs(self, graph, target):
        """
        Performs the breadth-first search algorithm on the graph.

        This function starts the BFS algorithm from the top-left corner of the graph
        and continues until the target node is found or all nodes have been visited.

        Args:
            graph (numpy.ndarray): The graph represented as a 2D numpy array.
            target (tuple): The target node coordinates in the graph.

        Returns:
            None
        """
        self.queue.append((0, 0))  # Start from top-left corner (0,0)
        self.visited.add((0, 0))

        while self.queue:
            current = self.queue.popleft()
            print(f"Current Position: {current}")

            if current == target:
                print(f"\nTarget found at: {current}\n")
                self.target_found = True
                return

            for neighbor in self.get_neighbors(graph, current):
                if neighbor not in self.visited:
                    self.visited.add(neighbor)
                    self.queue.append(neighbor)

    def get_neighbors(self, graph, node):
        """
        Returns the neighboring nodes of a given node in the graph.

        This function returns the neighboring nodes of a given node in the graph,
        excluding nodes that are out of bounds or blocked.

        Args:
            graph (numpy.ndarray): The graph represented as a 2D numpy array.
            node (tuple): The coordinates of the node in the graph.

        Returns:
            list: A list of neighboring nodes.
        """
        neighbors = []
        directions = [(0, 1), (1, 0), (0, -1), (-1, 0)]  # Right, Down, Left, Up
        for dx, dy in directions:
            x, y = node[0] + dx, node[1] + dy
            if 0 <= x < graph.shape[0] and 0 <= y < graph.shape[1] and graph[x, y] != 1:
                neighbors.append((x, y))
        return neighbors

def main(args=None):
    rclpy.init(args=args)
    graph_subscriber = GraphSubscriber()
    rclpy.spin(graph_subscriber)
    graph_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
