---
### 2. 📄 Chapter 2.2 File Content

**Target File:** `chapter-2-2-slam.md`

```markdown
---
title: Chapter 2.2: Simultaneous Localization and Mapping (SLAM)
description: Solving the chicken-and-egg problem of mapping an unknown environment while tracking robot position.
sidebar_position: 2
---
import Tabs from '@theme/Tabs';
import TabItem from '@theme/TabItem';



## Chapter 2.2: Simultaneous Localization and Mapping (SLAM)

**Intermediate** | **Prerequisites**: Chapter 2.1

How does a robot build a map of a new environment and know where it is at the same time? This is the classic "chicken-and-egg" problem of robotics, known as **SLAM**. This chapter provides a theoretical overview of the main SLAM paradigms.

:::tip[Learning Objectives]
*   Understand the probabilistic nature of the SLAM problem.
*   Differentiate between filter-based and graph-based SLAM.
*   Explain the concepts of prediction, measurement update, and loop closure.
*   Use a standard ROS 2 SLAM package (`slam_toolbox`) to map an environment.
:::

### 2.2.1 The SLAM Problem

SLAM is difficult because of uncertainty. Sensor measurements are noisy, and the robot's motion model (odometry) has errors that accumulate over time. SLAM is a process of estimating two things simultaneously: the robot's pose (localization) and the map of the environment.

### 2.2.2 Filter-Based SLAM (EKF SLAM)

Early approaches to SLAM used filtering techniques like the **Extended Kalman Filter (EKF)**.
*   **State**: The state vector includes the robot's pose and the position of all observed landmarks.
*   **Prediction**: The robot moves, and the filter predicts the new state based on odometry. Uncertainty grows.
*   **Update**: The robot observes a landmark. The filter uses this measurement to correct both the robot's pose and the landmark's position, reducing uncertainty.

EKF SLAM is computationally intensive and has challenges with data association (knowing which landmark you're seeing) and is less common today.

### 2.2.3 Graph-Based SLAM

Modern SLAM systems use a technique called **GraphSLAM**. The problem is modeled as a graph:
*   **Nodes**: Represent robot poses at different points in time.
*   **Edges**: Represent constraints between poses. An odometry reading is an edge between two consecutive poses. A landmark observation is an edge between a pose and a landmark.
*   **Loop Closure**: The most important concept in GraphSLAM. When the robot recognizes a place it has been before, it adds a "loop closure" edge to the graph. This edge provides a powerful constraint that drastically reduces accumulated error.

The goal of GraphSLAM is to find the configuration of nodes (poses) that minimizes the error in all the constraints (edges). This is a large-scale optimization problem that is solved by a backend optimizer.

![GraphSLAM Diagram](../assets/graph_slam.svg)
*A GraphSLAM representation, showing robot poses (nodes) and constraints (edges), including a critical loop closure.*

### Practical SLAM in ROS 2: `slam_toolbox`

`slam_toolbox` is a powerful and configurable GraphSLAM implementation for ROS 2. In the exercises, you will learn how to launch and tune it to map a simulated environment.

