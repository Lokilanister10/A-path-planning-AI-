import math
import numpy as np
import matplotlib.pyplot as plt

GRID_SIZE = 5
OBSTACLE_VALUE = 1

def create_custom_grid():
    # Define the custom grid
    custom_grid = [
        [0, 0, 0, 0, 0],
        [0, 1, 1, 0, 0],
        [0, 0, 0, 0, 0],
        [0, 0, 1, 1, 0],
        [0, 0, 0, 0, 0]
    ]
    return np.array(custom_grid)

def within_boundaries(point):
    return 0 <= point[0] < GRID_SIZE and 0 <= point[1] < GRID_SIZE

def loss_function(x, grid):
    z = (x[0][0] - START[0]) * 2 + (x[0][1] - START[1]) * 2
    for i in range(DIM - 1):
        z = z + ((x[i][0] - x[i + 1][0]) * 2 + (x[i][1] - x[i + 1][1]) * 2)
    z = z + (x[DIM - 1][0] - END[0]) * 2 + (x[DIM - 1][1] - END[1]) * 2
    for point in x:
        point_x, point_y = int(point[0]), int(point[1])
        if within_boundaries((point_x, point_y)) and grid[point_x, point_y] == OBSTACLE_VALUE:
            z += 1000
    return math.sqrt(z)

def particle_swarm_optimization(max_iterations, swarm_size, max_vel, step_size, inertia, c1, c2, grid):
    particles_loc = np.random.rand(swarm_size, DIM, 2) * (GRID_SIZE - 1)
    particles_vel = np.random.rand(swarm_size, DIM, 2)
    particles_lowest_loss = [loss_function(particles_loc[i], grid) for i in range(len(particles_loc))]
    particles_best_location = np.copy(particles_loc)

    global_lowest_loss = np.min(particles_lowest_loss)
    global_best_location = particles_loc[np.argmin(particles_lowest_loss)].copy()

    for iteration_i in range(max_iterations):
        for particle_i in range(swarm_size):
            particle_loc = particles_loc[particle_i]
            # We Update particle locations and velocities

            # We Update local and global best
            particle_error = loss_function(particle_loc, grid)
            if particle_error < particles_lowest_loss[particle_i]:
                particles_lowest_loss[particle_i] = particle_error
                particles_best_location[particle_i] = particle_loc.copy()
            if particle_error < global_lowest_loss:
                global_lowest_loss = particle_error
                global_best_location = particle_loc.copy()

    return global_best_location

def main():
    global GRID_SIZE, START, END, DIM
    GRID_SIZE = 5
    START = (0, 0) # We define the start state and the goal state
    END = (4, 4)
    DIM = 11

    grid = create_custom_grid()  
    best_location = particle_swarm_optimization(
        max_iterations=10,
        swarm_size=5,
        max_vel=3,
        step_size=1,
        inertia=0.9,
        c1=2.05,
        c2=2.05,
        grid=grid  # Finally we Pass the provided grid to the optimization function
    )


    plt.figure(figsize=(8, 8))
    plt.imshow(grid.T, cmap='Greys', origin='lower')
    plt.plot(START[0], START[1], 'go', markersize=10, label='Start')
    plt.plot(END[0], END[1], 'ro', markersize=10, label='End')
    x = [point[0] for point in best_location]
    y = [point[1] for point in best_location]
    plt.plot(x, y, marker='o', color='blue', label='Best Path')
    plt.legend()
    plt.title("Particle Swarm Optimization - Best Path")
    plt.xlabel("X-axis")
    plt.ylabel("Y-axis")
    plt.grid(visible=True)
    plt.show()

main()