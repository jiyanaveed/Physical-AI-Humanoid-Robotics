---
title: "Chapter 4.2: Classic Control Theory (PID)"
description: "Implementation and tuning of PID controllers and generation of smooth trajectories."
sidebar_position: 2
---

import Tabs from '@theme/Tabs';
import TabItem from '@theme/TabItem';

# Chapter 4.2: Classic Control Theory (PID) and Trajectory Generation

You've mastered kinematics, the art of knowing *where* your robot is and *where* it should be. Now, how do you get it there? This is the domain of **Control Theory**. In this chapter, you will learn to implement and tune the single most important algorithm in control engineering: the **Proportional-Integral-Derivative (PID) controller**. We will also cover how to generate smooth trajectories for your robot to follow, ensuring its movements are not just accurate, but also graceful and efficient.

:::tip[Learning Objectives]
*   Understand the intuitive and mathematical roles of the Proportional (P), Integral (I), and Derivative (D) terms in a PID controller.
*   Implement a robust, reusable PID controller class from scratch in Python.
*   Apply a systematic method (like Ziegler-Nichols) to tune PID gains for optimal performance.
*   Generate smooth, time-parameterized trajectories using polynomial interpolation.
*   Build and tune a complete control system for a simulated robot joint to make it follow a desired trajectory.
:::

## 4.2.1 The Heart of Control: The PID Controller

A PID controller is a **closed-loop control** mechanism. This means it continuously uses feedback (a sensor reading of the current state) to calculate an error and apply a corrective action.

The goal is simple: make the **process variable** (the system's actual state, e.g., current joint angle) equal to the **setpoint** (the desired state, e.g., target joint angle). The difference between these two is the **error**.

$$ e(t) = \text{setpoint} - \text{process\_variable}(t) $$

The PID controller calculates an output (a control command, like motor torque) by summing three terms:

$$ \text{output}(t) = K_p e(t) + K_i \int_0^t e(\tau)d\tau + K_d \frac{de(t)}{dt} $$

Let's break down each component.

### The P-Term: Proportional (The Present)
The **Proportional** term is the primary driving force. It generates a control output that is directly proportional to the current error.

$$ P_{out} = K_p e(t) $$
*   **$K_p$**: The proportional gain.
*   **Behavior**: A large $K_p$ results in a fast response, but too large a value can lead to instability and oscillation. It acts like a spring pulling the system towards the setpoint.

### The I-Term: Integral (The Past)
The **Integral** term looks at the accumulated error over time. Its job is to eliminate **steady-state error**—the small, persistent error that the P-term alone might not be able to correct.

$$ I_{out} = K_i \int_0^t e(\tau)d\tau $$
*   **$K_i$**: The integral gain.
*   **Behavior**: The longer an error persists, the larger the integral term becomes, increasing the control output until the error is eliminated. However, too much integral action can cause significant overshoot (the system shooting past the setpoint). This is known as "integral windup".

### The D-Term: Derivative (The Future)
The **Derivative** term looks at the rate of change of the error. It acts as a damper, predicting future error and counteracting it to reduce overshoot and oscillations.

$$ D_{out} = K_d \frac{de(t)}{dt} $$
*   **$K_d$**: The derivative gain.
*   **Behavior**: If the error is changing rapidly, the D-term applies a braking force to slow the system down as it approaches the setpoint. This helps to prevent overshoot and stabilize the system. It is, however, very sensitive to noise in the sensor measurements.

![PID Controller Diagram](../assets/pid_controller.svg)
*The three components of a PID controller work together to minimize error over time.*

### Code Example: A Python PID Controller Class
Here is a from-scratch implementation of a PID controller.

```python
# T019: PID controller implementation
# src/module4_examples/control/pid_controller.py
import time

class PIDController:
    def __init__(self, Kp, Ki, Kd, setpoint):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.setpoint = setpoint
        
        self.last_error = 0.0
        self.integral = 0.0
        self.last_time = time.time()

    def update(self, process_variable):
        current_time = time.time()
        dt = current_time - self.last_time
        
        if dt == 0:
            return 0.0 # Avoid division by zero

        error = self.setpoint - process_variable
        
        # Proportional term
        p_term = self.Kp * error
        
        # Integral term (with anti-windup)
        self.integral += error * dt
        # Basic anti-windup: clamp the integral term
        # In a real system, you might have more sophisticated anti-windup logic
        self.integral = max(min(self.integral, 10.0), -10.0) 
        i_term = self.Ki * self.integral
        
        # Derivative term
        derivative = (error - self.last_error) / dt
        d_term = self.Kd * derivative
        
        # Update for next iteration
        self.last_error = error
        self.last_time = current_time
        
        # The PID control output
        output = p_term + i_term + d_term
        return output

    def reset(self):
        self.last_error = 0.0
        self.integral = 0.0
        self.last_time = time.time()
```

## 4.2.2 PID Tuning: A Practical Art

Choosing the right values for $K_p$, $K_i$, and $K_d$ is called **tuning**. Poorly tuned gains can lead to a system that is slow, unstable, or oscillates wildly.

**General Effects of Increasing Each Gain:**
*   **$K_p$**: Decreases rise time, increases overshoot, reduces steady-state error.
*   **$K_i$**: Eliminates steady-state error, but increases overshoot and settling time.
*   **$K_d$**: Decreases overshoot and settling time, improves stability.

### The Ziegler-Nichols Tuning Method
This is a classic, heuristic method for finding a starting point for your PID gains.
1.  Set $K_i$ and $K_d$ to zero.
2.  Gradually increase $K_p$ until the system begins to oscillate at a constant amplitude. This is the **ultimate gain**, $K_u$.
3.  Measure the period of this oscillation, $T_u$.
4.  Use the following table to calculate your initial gains:

| Controller | $K_p$       | $K_i$             | $K_d$           |
|------------|-------------|-------------------|-----------------|
| P          | $0.5 K_u$   | -                 | -               |
| PI         | $0.45 K_u$  | $1.2 K_p / T_u$   | -               |
| **PID**    | **$0.6 K_u$**| **$2 K_p / T_u$** | **$K_p T_u / 8$**|

These values are a starting point. You will almost always need to fine-tune them manually to achieve the desired performance.

## 4.2.3 Trajectory Generation

A PID controller needs a setpoint. But what if we want the robot to move smoothly from point A to point B over a specific time? Simply changing the setpoint from A to B instantly would demand infinite acceleration, which is impossible.

Instead, we generate a **trajectory**, which is a time-parameterized path for the setpoint to follow.

### Polynomial Trajectories
A common method is to use a **polynomial** function of time. A **cubic polynomial** allows us to specify the start and end positions and velocities. A **quintic polynomial** allows us to also specify the start and end accelerations, resulting in even smoother motion.

For a cubic polynomial: $q(t) = a_0 + a_1t + a_2t^2 + a_3t^3$

We can solve for the coefficients $a_0, a_1, a_2, a_3$ given four constraints: the start position, start velocity, end position, and end velocity.

### Code Example: Cubic Polynomial Trajectory Generation
```python
# T020: Trajectory generation algorithms
# src/module4_examples/control/trajectory_generation.py
import numpy as np

def generate_cubic_trajectory(q0, qf, v0, vf, tf):
    """
    Generates coefficients for a cubic polynomial trajectory.
    q0: start position, qf: final position
    v0: start velocity, vf: final velocity
    tf: total time for trajectory
    """
    # System of linear equations: M * a = b
    M = np.array([
        [1, 0, 0, 0],
        [0, 1, 0, 0],
        [1, tf, tf**2, tf**3],
        [0, 1, 2*tf, 3*tf**2]
    ])
    b = np.array([q0, v0, qf, vf])
    
    # Solve for coefficients a = [a0, a1, a2, a3]
    a = np.linalg.solve(M, b)
    
    # Return a function that evaluates the trajectory at time t
    def trajectory(t):
        if t > tf: t = tf
        pos = a[0] + a[1]*t + a[2]*t**2 + a[3]*t**3
        vel = a[1] + 2*a[2]*t + 3*a[3]*t**2
        return pos, vel
        
    return trajectory

# Example usage
if __name__ == '__main__':
    traj_func = generate_cubic_trajectory(q0=0, qf=10, v0=0, vf=0, tf=2)
    
    # Sample the trajectory
    for t_step in np.linspace(0, 2, 11):
        pos, vel = traj_func(t_step)
        print(f'Time: {t_step:.1f}s, Position: {pos:.2f}, Velocity: {vel:.2f}')
```

## Hands-on Exercises

1.  **PID Controller for a Single Joint**:
    *   Create a simple simulation of a single robot joint (e.g., a point mass with inertia).
    *   Implement your PID controller to drive this joint to a target angle (e.g., 90 degrees).
    *   Manually tune the $K_p$, $K_i$, and $K_d$ gains to achieve a response with less than 5% overshoot and a settling time of under 1 second.
2.  **Trajectory Follower**:
    *   Using your PID-controlled joint from the first exercise, generate a cubic trajectory to move it from 0 to 180 degrees in 3 seconds.
    *   In your control loop, feed the time-varying setpoint from the trajectory function into your PID controller.
    *   Plot the desired position vs. the actual position of the joint over time to see how well it tracks the trajectory.
3.  **Anti-Windup**:
    *   In your PID controller, add a saturation limit to the output (e.g., clamp it between -10 and 10).
    *   Observe the "integral windup" problem when the controller is saturated.
    *   Implement a more advanced anti-windup scheme, such as resetting the integral term when the output is saturated.

## Capstone Project: Trajectory Tracking for a 2-DOF Arm

**Goal**: Make a simulated 2-DOF planar robot arm follow a specified trajectory in Cartesian space.

**System Components**:
1.  **2-DOF Arm Model**: Use the URDF and kinematics functions (FK and IK) from Chapter 4.1.
2.  **Trajectory Generation Node**:
    *   A ROS 2 node that generates a trajectory for the end-effector (e.g., a straight line from point A to point B in XY space).
    *   This node should publish the desired end-effector pose at a regular interval (e.g., 50 Hz) to a `/target_pose` topic.
3.  **IK and Control Node**:
    *   This is your main control node.
    *   It subscribes to the `/target_pose` topic.
    *   For each incoming target pose, it uses your IK solver from Chapter 4.1 to calculate the required joint angles ($q_{desired}$).
    *   It subscribes to the `/joint_states` topic to get the current joint angles ($q_{actual}$).
    *   It runs **two separate PID controllers**, one for each joint. The setpoint for each PID controller is the desired joint angle from the IK solver.
    *   The output of the PID controllers (joint efforts or velocities) is published to the robot's joint controllers in the simulation.
4.  **Simulation and Visualization**:
    *   Launch your robot arm in Gazebo.
    *   In RViz, visualize the desired trajectory and the actual path of the robot's end-effector.
    *   Use a tool like `rqt_plot` to graph the desired vs. actual joint angles over time.

This capstone project is a major milestone. It integrates kinematics, control theory, and trajectory generation into a complete system that makes a robot perform a precise, controlled motion—a core capability in robotics.

---
**End of Chapter 4.2**