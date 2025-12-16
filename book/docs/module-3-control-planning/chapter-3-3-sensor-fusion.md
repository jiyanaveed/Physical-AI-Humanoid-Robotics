---
title: "Chapter 3.3: Sensor Fusion"
description: "Combining data from multiple sensors using probabilistic techniques like the Kalman Filter (KF) and Extended Kalman Filter (EKF)."
sidebar_position: 3
---

import Tabs from '@theme/Tabs';
import TabItem from '@theme/TabItem';

# Chapter 3.3: Sensor Fusion

You've learned how to acquire data from individual sensors, but what happens when you have multiple sensors, each with its own strengths and weaknesses? How do you combine their data to get a single, more accurate estimate of the robot's state? This is the domain of **Sensor Fusion**. In this professional-level chapter, we will explore the powerful probabilistic techniques that allow robots to navigate and understand their world with much greater confidence than any single sensor could provide.

:::tip[Learning Objectives]
*   Understand the mathematical foundations of Bayesian filtering for state estimation.
*   Implement a linear Kalman Filter from scratch in Python to understand its core principles.
*   Grasp the theory of the Extended Kalman Filter (EKF) for handling non-linear systems.
*   Learn about Particle Filters as a non-parametric alternative for complex problems.
*   Build a complete sensor fusion node in ROS 2 to fuse wheel odometry and IMU data for robust state estimation.
:::

## 3.3.1 Why Sensor Fusion? The Problem of Uncertainty

Every sensor is imperfect.
*   **Wheel odometry** is precise over short distances but its error accumulates unboundedly over time (drift).
*   **IMUs** provide excellent short-term orientation data but drift over time and are noisy.
*   **GPS** gives a global position but can be inaccurate (several meters of error) and unavailable indoors.
*   **LiDAR** provides accurate distance measurements but doesn't directly measure velocity or orientation.

**Sensor fusion** is the process of combining data from these disparate and imperfect sources to produce a state estimate that is more accurate, reliable, and robust than the information from any individual sensor.

The core idea is to use a **probabilistic approach**. We don't just estimate the robot's state (e.g., position `x = 5.0`); we estimate a probability distribution over its state (e.g., "position is likely around 5.0, with a standard deviation of 0.1").

## 3.3.2 The Foundation: Bayesian Filtering

The mathematical heart of sensor fusion is **Bayesian filtering**. It's a two-step recursive process:

1.  **Prediction**: We use a motion model to predict the robot's next state based on its previous state and any control inputs (e.g., motor commands). As we predict, our uncertainty about the robot's state grows.
2.  **Update**: We use a sensor measurement to update our belief about the robot's state. The measurement provides new information that reduces our uncertainty.

This "predict-update" cycle is the essence of almost all modern sensor fusion algorithms.

<!--
![Bayesian Filtering Cycle](./assets/bayesian_filtering.svg)
*The continuous predict-update cycle of Bayesian filtering.*
-->

## 3.3.3 The Workhorse: The Kalman Filter (KF)

The **Kalman Filter** is arguably the most famous and widely used sensor fusion algorithm. It is an optimal estimator for **linear systems with Gaussian noise**.

Let's break that down:
*   **Linear System**: The system's behavior can be described by linear equations. For example, `new_position = old_position + velocity * dt`.
*   **Gaussian Noise**: The noise in both the motion model and the sensor measurements is assumed to follow a bell-shaped Gaussian distribution.

The Kalman Filter represents the robot's state as a Gaussian distribution with a mean (the state estimate) and a covariance matrix (the uncertainty).

### Code Example: A Simple 1D Kalman Filter from Scratch

To truly understand how a Kalman Filter works, let's implement one in Python for a simple 1D scenario: tracking an object moving with constant velocity.

```python
# T072: A from-scratch implementation of a 1D Kalman Filter
# src/module3_examples/fusion/kalman_filter_1d.py
import numpy as np

class KalmanFilter1D:
    def __init__(self, x0, P0, Q, R):
        self.x = x0  # Initial state estimate (e.g., [position, velocity])
        self.P = P0  # Initial state covariance (uncertainty)
        self.Q = Q  # Process noise covariance
        self.R = R  # Measurement noise covariance
        self.A = np.array([[1, 1], [0, 1]])  # State transition matrix
        self.H = np.array([[1, 0]])         # Measurement matrix

    def predict(self):
        # Predict state forward
        self.x = self.A @ self.x
        # Predict state covariance forward
        self.P = self.A @ self.P @ self.A.T + self.Q

    def update(self, z):
        # z is the measurement (e.g., a position reading)
        y = z - self.H @ self.x  # Measurement residual
        S = self.H @ self.P @ self.H.T + self.R # Residual covariance
        K = self.P @ self.H.T @ np.linalg.inv(S) # Kalman gain

        # Update state estimate
        self.x = self.x + K @ y
        # Update state covariance
        I = np.eye(len(self.x))
        self.P = (I - K @ self.H) @ self.P

# Example Usage
if __name__ == '__main__':
    # Initial state: position=0, velocity=0
    x0 = np.array([0, 0])
    # Initial uncertainty: large uncertainty
    P0 = np.array([[1000, 0], [0, 1000]])
    # Process noise: assume some noise in our motion model
    Q = np.array([[0.1, 0], [0, 0.1]])
    # Measurement noise: assume our sensor has a known noise level
    R = np.array([[5.0]])

    kf = KalmanFilter1D(x0, P0, Q, R)

    # Simulate some measurements
    measurements = [1, 2.2, 2.9, 4.1, 5.0]
    
    print(f'Initial state: {kf.x}')
    for z in measurements:
        kf.predict()
        kf.update(z)
        print(f'Measurement={z}, State estimate: pos={kf.x[0]:.2f}, vel={kf.x[1]:.2f}')

```

## 3.3.4 Handling Non-Linearity: The Extended Kalman Filter (EKF)

The real world is rarely linear. A robot's motion often involves rotation, which is a non-linear operation (full of sines and cosines). The standard Kalman Filter cannot handle this.

The **Extended Kalman Filter (EKF)** is an adaptation of the KF for non-linear systems. It works by **linearizing** the non-linear motion and measurement models at the current state estimate. It uses the **Jacobian** (the matrix of all first-order partial derivatives) to perform this linearization.

The EKF was the workhorse of robotics for decades and is still widely used, especially for state estimation in ROS 2 through packages like `robot_localization`.

## 3.3.5 The Non-Parametric Approach: Particle Filters

What if the noise isn't Gaussian? Or what if the probability distribution is multi-modal (e.g., a robot is lost and could be in one of two hallways)? In these cases, the EKF can fail.

**Particle Filters** offer a more flexible, non-parametric approach. Instead of representing the belief as a single Gaussian, a particle filter represents the probability distribution as a set of thousands of weighted **particles**. Each particle is a hypothesis of the robot's state.

The predict-update cycle for a particle filter:
1.  **Predict**: Move all particles according to the motion model, adding some random noise.
2.  **Update**: For each particle, calculate its **weight** based on how well its hypothetical state matches the actual sensor measurement. Particles that match the measurement well get a high weight.
3.  **Resample**: A new set of particles is drawn from the old set, where particles with higher weights are more likely to be chosen. This focuses the particles in the more probable regions of the state space.

Particle filters are excellent for global localization (the "kidnapped robot problem") but can be computationally expensive.

## Hands-on Exercises

1.  **Kalman Filter Tuning**:
    *   Take the 1D Kalman Filter example.
    *   Experiment with changing the `Q` (process noise) and `R` (measurement noise) values.
    *   Observe how the filter's output changes. What happens if you trust your measurements more (low `R`)? What if you trust your motion model more (low `Q`)?
2.  **IMU Bias Estimation**:
    *   An IMU gyroscope often has a bias (a constant offset).
    *   Let the IMU sit still on a table. The angular velocity should be zero, but it will report a small, non-zero value.
    *   Write a ROS 2 node that subscribes to the IMU topic, collects 100 samples, and calculates the average bias for the gyroscope. You can then subtract this bias from future measurements to improve accuracy.
3.  **Comparing Raw vs. Fused Data**:
    *   Find a public ROS 2 bag file that contains both `/odom` (from wheel encoders) and `/imu` topics.
    *   Play the bag file and use `ros2 topic echo` to observe the data. Notice the noise in the IMU data and the steady drift of the odometry. This will motivate the need for the capstone project.

## Capstone Project: Fusing Odometry and IMU with an EKF

**Goal**: Implement a robust state estimation node that fuses noisy wheel odometry with IMU data to produce a clean, filtered odometry estimate for a robot.

**System Components**:
1.  **A Simulated Robot**: Use a simulated robot in Gazebo (like TurtleBot3) that publishes `/odom` (from a differential drive plugin) and `/imu` topics. You can add noise to these sensors in the Gazebo plugin settings to make the problem more realistic.
2.  **`robot_localization` Package**: You will not implement the EKF from scratch. Instead, you will learn to use and configure the highly optimized and widely used `robot_localization` package in ROS 2.
3.  **Configuration File**: Create a YAML configuration file for the `ekf_filter_node` from `robot_localization`.
    *   Specify which sensors to use (`odom0`, `imu0`).
    *   Configure which parts of each sensor message to use (e.g., use Yaw from the IMU, but not X/Y position; use X/Y velocity from odometry, but not orientation).
    *   Tune the process noise and sensor noise covariance matrices.
4.  **Launch File**: Write a launch file that starts the `ekf_filter_node` with your configuration.
5.  **Visualization and Analysis**:
    *   In RViz, visualize three things:
        *   The raw `/odom` topic.
        *   The output of your EKF node (`/odometry/filtered`).
        *   The robot's ground truth position from Gazebo (if available).
    *   Drive the robot around. You should be able to see that the filtered odometry is smoother and drifts less than the raw wheel odometry.

This capstone project is extremely practical and mirrors the exact process used in countless real-world robotic systems to achieve reliable navigation and localization.

---
**End of Chapter 3.3**
