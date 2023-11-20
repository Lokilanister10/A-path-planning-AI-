import random
import numpy as np
import math
import copy
import matplotlib.pyplot as plt

def is_point_inside_circle(point, center, radius):
    dx = point[0] - center[0]
    dy = point[1] - center[1]
    distance_squared = dx**2 + dy**2
    return distance_squared <= radius**2

def is_line_circle_intersecting(start, end, center, radius):
    line_length_squared = (end[0] - start[0])**2 + (end[1] - start[1])**2
    if line_length_squared == 0:
        return is_point_inside_circle(start, center, radius)
    
    t = max(0, min(1, ((center[0] - start[0]) * (end[0] - start[0]) + (center[1] - start[1]) * (end[1] - start[1])) / line_length_squared))
    
    closest_point = ((1 - t) * start[0] + t * end[0], (1 - t) * start[1] + t * end[1])
    
    return is_point_inside_circle(closest_point, center, radius)

def find_path_in_graph(graph, start, goal):
    visited = set()
    parent = {}
    queue = []
    queue.append(start)

    while len(queue) != 0:
        current = queue.pop(0)
        visited.add(current)

        if current == goal:
            break

        for neighbor in graph.get(current, []):
            if neighbor not in visited:
                parent[neighbor] = current
                queue.append(neighbor)

    if goal not in visited:
        return None

    path = []
    current = goal
    while current != start:
        path.append(current)
        current = parent[current]
    path.reverse()
    return path

class Node:
    def __init__(self, x, y):
        self.x = x
        self.y = y

class PRMPlanner:
    def __init__(self, n, dist, x_limit, y_limit):
        self.number_of_nodes = n
        self.distance = dist
        self.node = dict(parent=[], child=[])
        self.graph = {}
        self.x_limit = x_limit
        self.y_limit = y_limit
        self.sample = set()

    def addGraph(self, graph, new_child_nodes, new_parent_node):
        if graph.get(new_parent_node) is None:
            graph[new_parent_node] = new_child_nodes
        else:
            values = graph.get(new_parent_node)
            values.extend(new_child_nodes)
            updated_val = set(values)
            graph[new_parent_node] = list(updated_val)

        for child in new_child_nodes:
            if graph.get(child) is None:
                graph[child] = [new_parent_node]
            else:
                if new_parent_node not in set(graph.get(child)):
                    graph.get(child).extend([new_parent_node])

    def random_position(self, obstacle_list):
        random.seed(1)
        x_lo = self.x_limit[0]
        x_hi = self.x_limit[1]
        y_lo = self.y_limit[0]
        y_hi = self.y_limit[1]

        n = 0
        while n < self.number_of_nodes:
            px, py = random.uniform(x_lo, x_hi), random.uniform(y_lo, y_hi)
            px, py = round(px, 1), round(py, 1)

            if (
                self.within_range(px, py)
                and (not self.is_in_obstacles((px, py), obstacle_list))
                and (px, py) not in self.sample
            ):  
                self.sample.add((px, py))
                n = n + 1
        return self.sample

    def within_range(self, px, py):
        x_lo = self.x_limit[0]
        x_hi = self.x_limit[1]
        y_lo = self.y_limit[0]
        y_hi = self.y_limit[1]
        return (x_lo < px < x_hi and y_lo < py < y_hi)

    def is_in_obstacles(self, point, obstacle_list):
        for obstacle in obstacle_list:
            if is_point_inside_circle(point, obstacle[1], obstacle[0]):
                return True
        return False

    def is_edge_intersecting(self, p1, p2, obstacle_list):
        for obstacle in obstacle_list:
            if is_line_circle_intersecting(p1, p2, obstacle[1], obstacle[0]):
                return True
        return False

    def get_euclidean_distance(self, current_position, next_position):
        x1, y1 = current_position[0], current_position[1]
        x2, y2 = next_position[0], next_position[1]
        distance = np.sqrt((math.pow(x2 - x1, 2)) + (math.pow(y2 - y1, 2)))
        return distance

    def within_distance(self, current_position, next_position):
        required_distance = self.distance
        distance = self.get_euclidean_distance(current_position, next_position)
        return distance <= required_distance

    def generate_roadmap(self, sample_nodes, graph, obstacle_list, max_num_edges=5):
        samples = set(sample_nodes)
        if len(graph.keys()) != 0:
            existing_nodes = graph.keys()
        else:
            existing_nodes = samples

        for current_node in samples:
            if current_node not in graph.keys():
                child_nodes = []
                distance_map = []
                parent_node = current_node
                for sample in existing_nodes:
                    if (
                        sample != current_node
                        and self.within_distance(current_node, sample)
                        and not self.is_edge_intersecting(sample, current_node, obstacle_list)
                    ):
                        distance_map.append((self.get_euclidean_distance(sample, current_node), sample))
                distance_map = sorted(distance_map, key=lambda x: x[0])
                count = 0
                for pair in distance_map:
                    if count >= max_num_edges:
                        break
                    child_nodes.append(pair[1])
                    count += 1

                self.sample.update([parent_node])
                self.addGraph(graph, child_nodes, parent_node)
            else:
                pass

        self.graph = graph
        roadmap = graph
        return roadmap

    def visualize_roadmap(self, obstacle_list):
        plt.figure(figsize=(8, 8))

        for obstacle in obstacle_list:
            circle = plt.Circle(obstacle[1], obstacle[0], color='gray', alpha=0.7)
            plt.gca().add_patch(circle)

        sampled_nodes = list(self.sample)
        sampled_x, sampled_y = zip(*sampled_nodes)
        plt.scatter(sampled_x, sampled_y, c='blue', marker='o', label='Sampled Nodes')

        for parent, children in self.graph.items():
            parent_x, parent_y = parent
            for child in children:
                child_x, child_y = child
                plt.plot([parent_x, child_x], [parent_y, child_y], c='green', alpha=0.5)

        path = find_path_in_graph(self.graph, start, goal)
        if path is not None:
            for node in path:
                node_x, node_y = node
                node_x, node_y = node
                plt.plot([node_x], [node_y], c='red', marker='s')

        plt.scatter(start.x, start.y, c='orange', marker='s', label='Start')
        plt.scatter(goal.x, goal.y, c='red', marker='s', label='Goal')

        plt.xlabel('X-axis')
        plt.ylabel('Y-axis')
        plt.title('PRM Roadmap Visualization')
        plt.legend()
        plt.grid(True)
        plt.show()


if __name__ == "__main__":
    grid = [
        [0, 0, 0, 0, 0],
        [0, 1, 1, 0, 0],
        [0, 0, 0, 0, 0],
        [0, 0, 1, 1, 0],
        [0, 0, 0, 0, 0]
    ]

    start = Node(0, 0)
    goal = Node(4, 4)

    roadmap = PRMPlanner(100, 2, x_limit=(-5.0, 5.0), y_limit=(-5.0, 5.0))

    sample_nodes = roadmap.random_position([])
    roadmap.generate_roadmap(sample_nodes, roadmap.graph, [])

    roadmap.visualize_roadmap([(1, (2, 2)), (1, (3, 3))]) 
