# Occupancy Maze Simulator

This package allows you to test your path planning node by providing a maze-based occupancy grid message and an simulated robot pose response. It includes random maze generation, start and goal positioning, and interactive feedback with Twist commands for movement.

## Usage

```bash
rviz2 -d occupancy_maze_simulator/rviz/config.rviz 

ros2 run occupancy_maze_simulator occupancy_maze_simulator_node 
```
