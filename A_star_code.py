import heapq
import numpy as np
import matplotlib.pyplot as plt

class Node:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.g = float('inf')  
        self.h = 0  
        self.parent = None

    def __lt__(self, other):
        return (self.g + self.h) < (other.g + other.h)

def heuristic(node, goal):
    # A simple Euclidean distance heuristic
    return ((node.x - goal.x) ** 2 + (node.y - goal.y) ** 2) ** 0.5

def astar(grid, start, goal):
    open_set = []
    closed_set = set()

    start_node = Node(start[0], start[1])
    goal_node = Node(goal[0], goal[1])

    start_node.g = 0
    start_node.h = heuristic(start_node, goal_node)
    heapq.heappush(open_set, start_node)

    while open_set:
        current_node = heapq.heappop(open_set)

        if current_node.x == goal_node.x and current_node.y == goal_node.y:
            path = []
            while current_node:
                path.insert(0, (current_node.x, current_node.y))
                current_node = current_node.parent
            return path

        closed_set.add((current_node.x, current_node.y))

        for neighbor in neighbors(grid, current_node):
            if (neighbor.x, neighbor.y) in closed_set:
                continue

            tentative_g = current_node.g + 1  # Assuming a cost of 1 to move to a neighboring cell

            if tentative_g < neighbor.g:
                neighbor.g = tentative_g
                neighbor.h = heuristic(neighbor, goal_node)
                neighbor.parent = current_node
                heapq.heappush(open_set, neighbor)

    return None  # No path found

def neighbors(grid, node):
    # Get neighboring nodes that are within the grid
    neighbors = []
    for dx in [-1, 0, 1]:
        for dy in [-1, 0, 1]:
            if dx == 0 and dy == 0:
                continue

            x, y = node.x + dx, node.y + dy
            if 0 <= x < len(grid) and 0 <= y < len(grid[0]) and grid[x][y] == 0:
                neighbors.append(Node(x, y))

    return neighbors

def visualize_astar(grid, path, start_point, goal_point):
    # Create a colormap for the grid values
    cmap = plt.cm.get_cmap('coolwarm')

    # Create a figure and an axis
    fig, ax = plt.subplots()

    # Create a heatmap of the grid values
    ax.imshow(grid, cmap=cmap)

    # Add labels for the rows and columns
    ax.set_xticks(np.arange(len(grid[0])))
    ax.set_yticks(np.arange(len(grid)))
    ax.set_xticklabels(np.arange(1, len(grid[0]) + 1))
    ax.set_yticklabels(np.arange(1, len(grid) + 1))

    # Set the title of the plot
    ax.set_title('A* Pathfinding on the Grid')

    # Highlight the optimal path by plotting a red line between nodes
    x_coords, y_coords = zip(*path)  # Unpack the path coordinates
    ax.plot(y_coords, x_coords, '-o', color='red', linewidth=2)  # Note: x and y are swapped for plotting

    # Mark the start and goal points in green and magenta, respectively
    ax.plot(start_point[1], start_point[0], 'go', label='Start')  # Note: x and y are swapped for plotting
    ax.plot(goal_point[1], goal_point[0], 'mo', label='Goal')      # Note: x and y are swapped for plotting

    # Display the legend
    ax.legend()


    plt.show()

grid = [
    [0, 0, 0, 0, 0],
    [0, 1, 1, 0, 0],
    [0, 0, 0, 0, 0],
    [0, 0, 1, 1, 0],
    [0, 0, 0, 0, 0]
]

start_point = (0, 0)
goal_point = (4, 4)

path = astar(grid, start_point, goal_point)

if path:
    print("Path found:", path)
else:
    print("No path found.")

visualize_astar(grid, path, start_point, goal_point)
