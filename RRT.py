import matplotlib.pyplot as plt
import random
import math

class Node:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.parent = None

def rrt(grid, start, goal, grid_height, max_iters=1000, step_size=1.0):
    grid_width = len(grid[0])
    nodes = [start]

    def distance(node1, node2):
        return math.sqrt((node1.x - node2.x)**2 + (node1.y - node2.y)**2)

    def move(from_node, to_node, step_size, grid_width, grid_height):
        angle = math.atan2(to_node.y - from_node.y, to_node.x - from_node.x)
        new_x = from_node.x + step_size * math.cos(angle)
        new_y = from_node.y + step_size * math.sin(angle)

        new_x = max(0, min(new_x, grid_width - 1))
        new_y = max(0, min(new_y, grid_height - 1))

        return Node(new_x, new_y)

    def valid(grid, from_node, to_node, grid_width, grid_height):
        x1, y1 = int(from_node.x), int(from_node.y)
        x2, y2 = int(to_node.x), int(to_node.y)

        if grid[y2][x2] == 1:
            return False

        for x, y in interpolate_path(x1, y1, x2, y2):
            if grid[y][x] == 1:
                return False

        return True

    def interpolate_path(x1, y1, x2, y2):
        path = []
        dx = abs(x2 - x1)
        dy = abs(y2 - y1)
        x, y = x1, y1
        n = 1 + dx + dy

        x_inc = 1 if x2 > x1 else -1
        y_inc = 1 if y2 > y1 else -1

        error = dx - dy

        dx *= 2
        dy *= 2

        for _ in range(n):
            path.append((x, y))
            if error > 0:
                x += x_inc
                error -= dy
            else:
                y += y_inc
                error += dx

        return path

    for _ in range(max_iters):
        random_node = Node(random.uniform(0, grid_width), random.uniform(0, grid_height))
        nearest_node = min(nodes, key=lambda n: distance(n, random_node))

        new_node = move(nearest_node, random_node, step_size, grid_width, grid_height)
        if valid(grid, nearest_node, new_node, grid_width, grid_height):
            nodes.append(new_node)
            new_node.parent = nearest_node

            if distance(new_node, goal) < step_size:
                goal.parent = new_node
                nodes.append(goal)
                return extracting_path(goal)

    return None

def extracting_path(goal_node):
    path = []
    current_node = goal_node
    while current_node:
        path.insert(0, (current_node.x, current_node.y))
        current_node = current_node.parent
    return path

def vis_path(grid, path, start, goal):
    plt.imshow(grid, cmap='Greys', origin='upper')

    for i in range(1, len(path)):
        plt.plot([path[i-1][0] + 0.5, path[i][0] + 0.5], [grid_height - path[i-1][1] - 0.5, grid_height - path[i][1] - 0.5], color='blue')

    plt.scatter(start.x + 0.5, grid_height - start.y - 0.5, color='green', marker='o', s=100, label='Start')
    plt.scatter(goal.x + 0.5, grid_height - goal.y - 0.5, color='red', marker='x', s=100, label='Goal')

    plt.legend()
    plt.show()

if __name__ == "__main__":
    grid = [
        [0, 0, 0, 0, 0],
        [0, 1, 1, 0, 0],
        [0, 0, 0, 0, 0],
        [0, 0, 1, 1, 0],
        [0, 0, 0, 0, 0]
    ]

    grid_height = len(grid)
    
    start = Node(0, 0)
    goal = Node(4, 4)

    path = rrt(grid, start, goal, grid_height, max_iters=5000)

    if path is not None:
        
        vis_path(grid, path, start, goal)
    else:
        print("No valid path found.")
