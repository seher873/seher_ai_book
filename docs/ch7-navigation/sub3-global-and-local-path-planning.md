---
sidebar_position: 3
---

# Global and Local Path Planning

## Overview

Path planning is a critical component of mobile robot navigation, determining how a robot moves from its current location to a goal location. This section covers the two primary levels of path planning: global planning (finding a path across the entire environment) and local planning (executing the path while avoiding immediate obstacles).

## Global Path Planning

### Grid-Based Planning

Grid-based approaches discretize the environment into a grid of cells:

**A* Algorithm**
A* is a popular graph search algorithm that uses a heuristic function to guide the search toward the goal:
```
f(n) = g(n) + h(n)
```
Where:
- g(n) is the actual cost from start to node n
- h(n) is the heuristic estimate from node n to the goal
- f(n) is the estimated total cost of the path through node n

**Dijkstra's Algorithm**
Dijkstra's algorithm finds the shortest path without using heuristics, guaranteeing optimality but potentially requiring more computation.

**Jump Point Search (JPS)**
JPS is an optimization of A* that prunes symmetric paths, significantly speeding up search in uniform-cost grids.

### Sampling-Based Planning

**Probabilistic Roadmap (PRM)**
PRM pre-computes a roadmap of the environment by randomly sampling the configuration space and connecting collision-free configurations.

**Rapidly-exploring Random Trees (RRT)**
RRT builds a tree of possible paths by incrementally growing from the start configuration toward randomly sampled points in the space.

### Topological Planning

Topological approaches represent the environment as a graph of topological places connected by paths:
- **Visibility graphs**: Connects visible points in polygonal environments
- **Roadmaps**: Pre-computed networks of feasible paths
- **Topological maps**: Represents connectivity rather than geometry

## Local Path Planning

### Reactive Approaches

**Vector Field Histogram (VFH)**
VFH uses local sensor data to select motion directions based on a histogram of possible directions, considering obstacle density and goal direction.

**Dynamic Window Approach (DWA)**
DWA selects velocities from a feasible set (the "dynamic window") that considers robot dynamics, obstacle avoidance, and goal reaching.

### Trajectory-Based Methods

**Timed Elastic Bands (TEB)**
TEB optimizes trajectories considering:
- Kinodynamic constraints
- Obstacle avoidance
- Goal reaching
- Smoothness requirements

**D* and D* Lite**
These algorithms are designed for dynamic environments, replanning efficiently as new information becomes available.

## Path Planning in Complex Environments

### Dynamic Environments

Planning for environments with moving obstacles requires:
- **Predictive models**: Estimating future positions of moving obstacles
- **Reactive replanning**: Adjusting paths as obstacles move
- **Time-parameterized paths**: Planning trajectories that account for time

### Multi-Modal Planning

For robots that can move in different ways:
- **Legged robots**: Planning for different gaits and foot placements
- **Wheeled-legged robots**: Switching between wheeled and legged locomotion
- **Flying robots**: 3D path planning with altitude considerations

## Integration of Global and Local Planning

### Hierarchical Planning

Global and local planners work together in a hierarchical structure:
- **Global planner**: Provides a high-level path
- **Local planner**: Executes the path while avoiding immediate obstacles
- **Replanning triggers**: When to call the global planner again

### Consistency Requirements

The local planner must be consistent with the global plan:
- **Goal convergence**: Eventually reach the global goal
- **Obstacle avoidance**: Not violate global obstacle constraints
- **Efficiency**: Make progress toward the goal

## Planning with Robot Dynamics

### Kinodynamic Planning

For robots with non-holonomic constraints:
- **Car-like robots**: Cannot move sideways
- **Differential drive**: Limited turning radius
- **Ackermann steering**: Different wheel speeds for turning

### Motion Primitives

Pre-computed motion segments that satisfy robot dynamics:
- **Reeds-Shepp curves**: Optimal paths for car-like robots
- **Dubins curves**: Shortest paths with bounded curvature
- **Steering functions**: Methods to connect configurations while satisfying constraints

## Multi-Robot Path Planning

### Decentralized Approaches

Each robot plans independently with coordination mechanisms:
- **Conflict-based search**: Resolves conflicts between robot paths
- **Prioritized planning**: Plans for robots in sequence with higher-priority paths as constraints

### Centralized Approaches

A central planner coordinates all robot paths:
- **Multi-Agent Path Finding (MAPF)**: Finds collision-free paths for all agents
- **Communication requirements**: Coordination overhead

## Planning for Different Robot Types

### Ground Vehicles

- **Differential drive**: Planning for minimum turning radius
- **Ackermann steering**: Planning for car-like kinematics
- **Tracked vehicles**: Planning for skid-steering dynamics

### Aerial Vehicles

- **3D planning**: Accounting for altitude changes
- **Dynamics constraints**: Acceleration and velocity limits
- **Wind effects**: Planning with environmental disturbances

### Legged Robots

- **Footstep planning**: Planning where to place feet
- **Balance constraints**: Maintaining stability during locomotion
- **Terrain analysis**: Planning for traversability

## Planning Quality Metrics

### Optimality

- **Path length**: Distance of the planned path
- **Execution time**: Time to compute the path
- **Clearance**: Distance from obstacles

### Feasibility

- **Kinodynamic feasibility**: Path follows robot constraints
- **Collision-free**: No collisions with obstacles
- **Dynamic feasibility**: Robot can execute the path

## Real-Time Considerations

### Computational Efficiency

Path planners must operate within real-time constraints:
- **Anytime algorithms**: Improve solutions over time
- **Hierarchical approaches**: Coarse-to-fine planning
- **Pre-computation**: Pre-computing parts of the solution

### Replanning Strategies

When the environment changes or new obstacles are detected:
- **Incremental replanning**: Efficiently update existing plans
- **Windowed planning**: Plan only for a local window
- **Recovery behaviors**: When planning fails

## Summary

Global and local path planning work together to enable robots to navigate complex environments. The choice of planning algorithm depends on the environment characteristics, robot constraints, and real-time requirements.

In the next section, we'll explore obstacle detection and collision avoidance, which is crucial for safe navigation.