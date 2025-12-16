---
title: "Chapter 2.1: Path Planning in Python"
description: "Implementing foundational path planning algorithms like A*, Dijkstra's, and RRT from scratch."
sidebar_position: 1
---


import Tabs from '@theme/Tabs';
import TabItem from '@theme/TabItem';

# Chapter 2.1: Path Planning in Python

Welcome to the first chapter of Module 2. Having established a solid foundation in ROS 2, we now turn to a fundamental problem in autonomous robotics: **Path Planning**. This chapter will guide you through the theory and implementation of classic and modern path planning algorithms. You will learn how to find the most efficient path for a robot on a map, implementing these algorithms from scratch in Python to gain a deep understanding of how they work.

:::tip[Learning Objectives]
*   Understand the theory of graph-based path planning.
*   Implement Dijkstra’s algorithm and A* for optimal path finding on a grid.
*   Implement the Rapidly-exploring Random Tree (RRT) algorithm for high-dimensional spaces.
*   Integrate a path planner into a ROS 2 service.
:::

## 2.1.1 Graph Search for Robotics

At its heart, path planning on a known map is a **graph search problem**. We can represent the environment as a graph where:
*   **Nodes** are discrete locations on the map (e.g., cells in a grid).
*   **Edges** are the connections between adjacent, traversable locations. Each edge has a **cost** associated with it, which is typically the distance between the nodes.

Our goal is to find the sequence of nodes (the path) from a given start node to a goal node with the minimum possible total cost.

### Dijkstra's Algorithm
Dijkstra's algorithm is a fundamental graph search algorithm that guarantees finding the shortest path from a single source node to all other nodes in a graph with non-negative edge weights.

**How it works:**
1.  Initialize distances to all nodes as infinite, except for the start node, which is 0.
2.  Maintain a set of visited nodes and a priority queue of nodes to visit, prioritized by their distance from the start.
3.  While the priority queue is not empty, extract the node with the smallest distance.
4.  For the current node, consider all its unvisited neighbors.
5.  For each neighbor, calculate the distance from the start node through the current node. If this path is shorter than the previously known distance to the neighbor, update the distance and add the neighbor to the priority queue.
6.  Once the goal node is reached (or all nodes have been visited), the shortest path can be reconstructed.

Dijkstra's is robust but can be inefficient as it explores equally in all directions, like ripples in a pond.

### A* Algorithm: Intelligent Search
The **A* (A-star)** algorithm is a powerful and widely used extension of Dijkstra's. It improves efficiency by using a **heuristic** to guide its search toward the goal.

The heuristic, $h(n)$, is an *estimated* cost from a node $n$ to the goal. A common heuristic for grid maps is the **Euclidean distance** or **Manhattan distance**.

A* prioritizes nodes based on the function:
$$ f(n) = g(n) + h(n) $$
*   **$g(n)$**: The actual cost (path length) from the start node to the current node $n$.
*   **$h(n)$**: The estimated heuristic cost from node $n$ to the goal.

By considering both the cost already traveled and the estimated cost remaining, A* explores the most promising paths first, making it significantly faster than Dijkstra's for many path planning problems.

### Code Example: A* Implementation in Python

Here is a simplified, from-scratch implementation of the A* algorithm for a grid-based map.

```python
# T013: A* algorithm implementation
# src/module2_examples/path_planning/a_star.py
import heapq

class AStarPlanner:
    def __init__(self, grid):
        """
        grid: A 2D list or numpy array representing the map. 0=free, 1=obstacle.
        """
        self.grid = grid
        self.rows = len(grid)
        self.cols = len(grid[0])

    def heuristic(self, a, b):
        # Manhattan distance heuristic
        return abs(a[0] - b[0]) + abs(a[1] - b[1])

    def find_path(self, start, goal):
        open_set = []
        heapq.heappush(open_set, (0, start))
        
        came_from = {}
        g_score = { (r, c): float('inf') for r in range(self.rows) for c in range(self.cols) }
        g_score[start] = 0
        
        f_score = { (r, c): float('inf') for r in range(self.rows) for c in range(self.cols) }
        f_score[start] = self.heuristic(start, goal)

        open_set_hash = {start}

        while open_set:
            _, current = heapq.heappop(open_set)
            open_set_hash.remove(current)

            if current == goal:
                path = []
                while current in came_from:
                    path.append(current)
                    current = came_from[current]
                path.append(start)
                return path[::-1]

            for dr, dc in [(0, 1), (0, -1), (1, 0), (-1, 0)]: # 4-way connectivity
                neighbor = (current[0] + dr, current[1] + dc)

                if 0 <= neighbor[0] < self.rows and 0 <= neighbor[1] < self.cols and self.grid[neighbor[0]][neighbor[1]] == 0:
                    tentative_g_score = g_score[current] + 1
                    if tentative_g_score < g_score[neighbor]:
                        came_from[neighbor] = current
                        g_score[neighbor] = tentative_g_score
                        f_score[neighbor] = tentative_g_score + self.heuristic(neighbor, goal)
                        if neighbor not in open_set_hash:
                            heapq.heappush(open_set, (f_score[neighbor], neighbor))
                            open_set_hash.add(neighbor)
        
        return None # No path found
```

## 2.1.2 Sampling-Based Planners: RRT

For high-dimensional configuration spaces (e.g., a 7-DOF robot arm), grid-based methods like A* become computationally intractable due to the "curse of dimensionality". **Sampling-based planners** provide a more efficient alternative.

The **Rapidly-exploring Random Tree (RRT)** algorithm works by building a tree of valid configurations that grows from the start state and incrementally explores the space.

**How it works:**
1.  Initialize a tree with the start configuration as the root.
2.  **Randomly sample** a point in the configuration space.
3.  Find the **nearest node** in the tree to the sampled point.
4.  **Steer** from the nearest node towards the random point by a small, fixed distance.
5.  If the path from the nearest node to the new point is collision-free, add the new point and the connecting edge to the tree.
6.  Repeat until a node is added that is within a certain distance of the goal.

RRT is not optimal (it doesn't guarantee the shortest path), but it is **probabilistically complete** (it will find a path if one exists, given enough time) and is very effective at quickly exploring large, complex spaces.

## Hands-on Exercises

1.  **Dijkstra's vs. A***:
    *   Implement Dijkstra's algorithm.
    *   Create a simple grid map with obstacles.
    *   Run both Dijkstra's and A* on the same map with the same start and end points.
    *   Visualize the nodes visited by each algorithm. You should see that A* explores a much smaller, more focused area of the map.
2.  **RRT for a Point Robot**:
    *   Implement a basic RRT algorithm for a 2D point robot in an environment with rectangular obstacles.
    *   Visualize the growth of the tree and the final path found.
    *   Experiment with different step sizes and see how it affects the path and the planning time.

## Capstone Project: A ROS 2 Path Planning Service

**Goal**: Create a ROS 2 service that can compute a path on a static 2D map using your A* implementation.

**System Components**:
1.  **Map Representation**:
    *   Create a simple 2D map as a text file or image that your node can load into a grid structure.
2.  **`path_planning_service` Node**:
    *   A ROS 2 node that starts a service (e.g., named `/plan_path`).
    *   The service definition should take a `start` pose and a `goal` pose (you can use `geometry_msgs/Pose`) and return a `nav_msgs/Path`.
3.  **Service Logic**:
    *   When the service is called, it should:
        *   Convert the start and goal poses into grid coordinates.
        *   Instantiate your `AStarPlanner` with the loaded map.
        *   Call `find_path()` to compute the path.
        *   Convert the list of grid coordinates back into a `nav_msgs/Path` message.
        *   Return the path to the client.
4.  **Client and Visualization**:
    *   Create a simple client node that calls the service with a hardcoded start and goal.
    *   Create a separate visualization node that subscribes to the returned path and publishes it as a visualization marker for RViz to display.

This project will test your ability to encapsulate a core algorithm within a ROS 2 service, a common and powerful pattern for building modular robotic systems.

---
**End of Chapter 2.1**
