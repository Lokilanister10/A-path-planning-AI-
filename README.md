# Mars Rover Path Planning Algorithms
This repository contains Python implementations of path planning algorithms for a Mars Rover navigating through a grid-based environment. The implemented algorithms include Probabilistic Roadmap (PRM), Rapidly-exploring Random Trees (RRT), Particle Swarm Optimization (PSO), and A* (A-star). Each algorithm provides a unique approach to find the optimal path for the Mars Rover.
# Table of Contents
* Introduction
* Implemented Algorithms
* Usage
* Visualization
* Sample Runs
* Dependencies
* Contributing
# Introduction
Path planning is a crucial aspect of Mars Rover missions, enabling efficient navigation through various terrains while avoiding obstacles. This repository offers implementations of four different path planning algorithms to address this challenge.
Implemented Algorithms
* 		Probabilistic Roadmap (PRM)
    * File: PRM_code.py
    * Description: PRM is a sampling-based algorithm that builds a roadmap of the environment. It samples random configurations and connects feasible configurations to form a network.
* 		Rapidly-exploring Random Trees (RRT)
    * File: RRT.py
    * Description: RRT is a tree-based algorithm that rapidly explores the configuration space. It incrementally builds a tree connecting random samples with the existing tree structure.
* 		Particle Swarm Optimization (PSO)
    * File: PSO_code.py
    * Description: PSO is a nature-inspired optimization algorithm. In the context of path planning, it optimizes the trajectory by considering a swarm of particles exploring the search space.
* 		A (A-star) Pathfinding*
    * File: A_star_code.py
    * Description: A* is a widely-used pathfinding algorithm known for its efficiency and optimality. It uses a heuristic approach to find the optimal path from a start to a goal point in a grid-based environment.
# Usage
Each algorithm is implemented in a separate Python script. To use a specific algorithm, run the corresponding script. Modify the grid, start, and goal points based on your specific scenario within each script.
# Visualization
For better understanding and visualization of the path planning results, each algorithm includes a visualization component. This can be adjusted based on your visualization preferences or integrated into a larger project.
# Sample Runs
Sample runs for each algorithm are provided within their respective scripts. Feel free to explore the provided examples to understand how each algorithm performs in different scenarios.
# Dependencies
The implementation utilizes common Python libraries such as NumPy and Matplotlib. Ensure you have these dependencies installed before running the scripts.
# Contributing
Contributions are welcome! If you have improvements or additional algorithms to share, feel free to create a pull request. For major changes, please open an issue to discuss the changes beforehand.

