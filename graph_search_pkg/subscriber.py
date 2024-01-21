import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import numpy as np
from collections import deque

class GraphSubscriber(Node):

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
        if not self.target_found:
            data = json.loads(msg.data)
            graph = np.array(data['graph'])
            target = tuple(data['target'])
            self.bfs(graph, target)

    def bfs(self, graph, target):
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
