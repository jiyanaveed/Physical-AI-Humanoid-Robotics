---
title: "Chapter 4.3: Advanced Dynamics and Non-Linear Control"
description: "Introduction to Lagrangian dynamics, non-linear control, and bipedal balance (ZMP)."
sidebar_position: 3
---

import Tabs from '@theme/Tabs';
import TabItem from '@theme/TabItem';
import Latex from '@site/src/components/ReactLatex';

# Chapter 4.3: Advanced Dynamics and Non-Linear Control

Welcome to the final, and most advanced, chapter of Module 4. You have mastered kinematics (the geometry of motion) and linear control (PID). Now, we will explore **Dynamics**—the study of motion in relation to the forces and torques that cause it. We will also introduce **Non-Linear Control**, a set of powerful techniques required to control the complex, high-performance, and often unstable systems that define the cutting edge of robotics, such as humanoid balancing. 

:::tip[Learning Objectives]
* Understand the fundamentals of robot dynamics using the Lagrangian formulation.
* Derive the equations of motion for a simple robotic system.
* Recognize the limitations of linear controllers (like PID) for non-linear systems.
* Grasp the core concepts of at least one non-linear control technique, such as Computed Torque Control.
* Learn about the Zero Moment Point (ZMP) and its critical role in bipedal locomotion and balance.
* Implement an advanced controller for a non-linear system, such as balancing an inverted pendulum.
:::

## 4.3.1 Robot Dynamics: The Physics of Motion

While kinematics describes the *motion* of a robot, dynamics describes the *forces* required to create that motion. The dynamic model of a robot relates the forces and torques applied by its motors to the resulting positions, velocities, and accelerations of its joints.



### The Lagrangian Formulation
One elegant way to derive these equations of motion is through **Lagrangian Dynamics**. The Lagrangian, is defined as the difference between the kinetic energy ($K$) and the potential energy ($P$) of the system.

The equations of motion are then found using the **Euler-Lagrange equation**:

The equations of motion are found using the Euler-Lagrange equation, which relates the change in the Lagrangian with respect to time to the forces applied.
Where $\tau_i$ is the external force (motor torque) applied to the $i$-th joint.


## 4.3.2 Non-Linear Control: Beyond PID

Linear controllers like PID work well when the system is "close enough" to linear, or when performance requirements are not strict. For high-speed, high-precision, or unstable systems, we need controllers that explicitly account for the non-linear dynamics.

### Computed Torque Control (or Feedback Linearization)

The idea behind **Computed Torque Control** is to "cancel out" the non-linear dynamics of the robot so that a simple linear controller can do its job.


This combined approach is incredibly powerful. It uses a perfect model of the robot's dynamics to achieve high-performance control. The main challenge is that it requires a very accurate model of $M$, $C$, and $G$.

### Code Example: Conceptual Computed Torque Control
```python
# T023: Conceptual example of non-linear control
# src/module4_examples/dynamics/non_linear_control.py

class ComputedTorqueController:
    def __init__(self, Kp, Kd):
        self.Kp = Kp
        self.Kd = Kd

    def update(self, robot_model, q, q_dot, q_d, q_d_dot, q_d_ddot):
        """
        Calculates the required torque.
        robot_model: An object that can compute M, C, and G for a given state.
        q, q_dot: Current joint positions and velocities.
        q_d, q_d_dot, q_d_ddot: Desired joint position, velocity, and acceleration.
        """ 
        
        # Get the dynamic model parameters from the robot model
        M = robot_model.get_mass_matrix(q)
        C = robot_model.get_coriolis_vector(q, q_dot)
        G = robot_model.get_gravity_vector(q)
        
        # PD control for the "linearized" system
        error = q_d - q
        error_dot = q_d_dot - q_dot
        
        u = q_d_ddot + self.Kd * error_dot + self.Kp * error
        
        # The final torque command
        # This part cancels the non-linear dynamics
        torque = M @ u + C + G
        
        return torque

# This is a conceptual example. A real implementation would require a `robot_model`
# class with methods to calculate the dynamic matrices. For a simple pendulum,
# M = mL^2, C = 0, G = mgL*sin(q).


**4.3.3 A Cornerstone of Humanoid Robotics: The Zero Moment Point (ZMP)**
How does a bipedal robot walk without falling over? The key concept is the Zero Moment Point (ZMP).

The ZMP is the point on the ground where the net moment of the inertial forces and gravity forces is zero. In simpler terms, it's the point where the "tipping-over" moment is zero. As long as the robot can keep its ZMP within the support polygon (the area enclosed by its feet), it will remain stable.

To walk, a humanoid robot must:

Plan a trajectory for its ZMP.

Plan a trajectory for its feet (footsteps).

Calculate the required body motion (joint angles) that will produce the desired ZMP trajectory while also moving the feet to the next step.

This is a highly complex control problem that combines trajectory generation, inverse kinematics, and whole-body dynamics. The ZMP is a fundamental concept used in almost all modern walking robots, from ASIMO to Atlas.

Hands-on Exercises:
Pendulum Dynamics in Python:
CCreate a Python class for the simple pendulum, as derived in the example.Write a function that takes the current angle theta and returns the gravity term $G(\theta)$.
Simulate the pendulum's motion by integrating its equation of motion over time, assuming zero torque is applied. Plot the resulting angle over time.
PID vs. Computed Torque:
Using your pendulum simulation, implement both a simple PID controller and a Computed Torque controller to drive the pendulum to a horizontal position ($\pi/2$).Compare their performance. Can the PID controller hold the pendulum at horizontal without error? How does the Computed Torque controller perform?
ZMP and the Support Polygon:
Imagine a robot standing on two feet. Each foot is a rectangle. Write a Python function that takes the coordinates of the four corners of the two feet and calculates the support polygon.

Write another function that takes a ZMP coordinate and determines if it is inside the calculated support polygon.

Capstone Project: Balancing an Inverted Pendulum:
Goal: Implement a controller to balance a classic non-linear system: the inverted pendulum on a cart.

System Components:
Simulation Environment:

Use a pre-built simulation environment for the cart-pole problem. The Gymnasium library (formerly OpenAI Gym) is perfect for this. The CartPole-v1 environment provides the simulation, state observations, and reward signals.

Controller Implementation:

Your task is to implement a controller that takes the state of the cart-pole (cart position, cart velocity, pole angle, pole angular velocity) and outputs a force to apply to the cart (left or right).

Controller Types to Implement and Compare:

Simple PID Controller: Try to balance the pole using a PID controller where the setpoint is a pole angle of 0 (upright). What are the limitations?

Advanced Controller (LQR or RL):

LQR (Linear-Quadratic Regulator): A more advanced linear control technique. You will need to linearize the system dynamics around the upright position and then use an LQR solver (e.g., from a library like scipy.signal) to find the optimal control gains.

Reinforcement Learning: As an alternative, you can use a simple RL algorithm (like Q-learning or a basic policy gradient method from the previous module) to learn a control policy that maximizes the reward (time spent balanced).

Analysis:

Compare the performance of your controllers. How long can each controller keep the pole balanced? How robust are they to disturbances?

Plot the pole angle and cart position over time for each controller.

This capstone project provides a hands-on introduction to the challenges of controlling non-linear, unstable systems, a critical skill for advanced robotics and humanoid control.

End of Chapter 4.3
