---
title: "Chapter 4.1: Kinematics and Rigid Body Transformations"
description: "Mastering forward and inverse kinematics using homogeneous transformation matrices."
sidebar_position: 1
---
import Tabs from '@theme/Tabs';
import TabItem from '@theme/TabItem';

# Chapter 4.1: Kinematics and Rigid Body Transformations

Welcome to Module 4, where we delve into the mathematical heart of robot motion: **Kinematics**. Before a robot can execute a trajectory or interact with its environment, we must precisely understand how its physical structure moves in space. This chapter lays the foundational mathematics for describing robot motion, focusing on rigid body transformations and their application to both forward and inverse kinematics. Mastering these concepts is crucial for programming any robot that interacts with the physical world. 

:::tip[Learning Objectives]
* Understand and apply 2D and 3D rigid body transformations using rotation matrices and homogeneous transformation matrices.
* Implement Forward Kinematics (FK) for a robot arm using the Denavit-Hartenberg (DH) convention.
* Grasp the theoretical concept of the Jacobian matrix in velocity kinematics.
* Implement a numerical Inverse Kinematics (IK) solver (e.g., using Jacobian Transpose or Damped Least Squares).
* Develop a ROS 2 kinematics service that provides both FK and IK solutions for a robot arm.
:::

## 4.1.1 Rigid Body Transformations: The Language of Robot Pose

A robot's **pose** refers to its position and orientation in space. To describe how different parts of a robot relate to each other, or how a robot relates to its environment, we use **rigid body transformations**. These transformations are composed of rotations and translations.

### Rotation Matrices

A **rotation matrix** describes the orientation of one coordinate frame relative to another. For a 3D rotation, it's a 3x3 matrix.



### Homogeneous Transformation Matrices

The Homogeneous Transformation Matrix (T) combines the rotation (R) and position (P) information between two frames. R is the 3x3 rotation sub-matrix, P is the 3x1 position vector, and the last row is the perspective row [0 0 0 1].

The Homogeneous Transformation Matrix (T) combines the rotation (R) and position (P) information between two frames. R is the 3x3 rotation sub-matrix, P is the 3x1 position vector, and the last row is the perspective row [0 0 0 1].



## 4.1.2 Forward Kinematics (FK)

**Forward Kinematics** calculates the end-effector's pose (position and orientation) given the joint angles of the robot arm.  

The **Denavit-Hartenberg (DH) convention** standardizes frame assignments and transformations between links. 

The transformation matrix between two consecutive frames $i$ and $i-1$ using DH parameters:


### Code Example: Forward Kinematics for a 2-DOF Planar Arm

```python
# T015: Forward kinematics algorithms
import numpy as np

def dh_matrix(alpha, a, d, theta):
    ct = np.cos(theta)
    st = np.sin(theta)
    ca = np.cos(alpha)
    sa = np.sin(alpha)
    return np.array([
        [ct, -st*ca,  st*sa,  a*ct],
        [st,  ct*ca, -ct*sa,  a*st],
        [0,      sa,     ca,     d],
        [0,       0,      0,     1]
    ])

def forward_kinematics_2dof_planar(l1, l2, q1, q2):
    dh_params = [
        [0, l1, 0, q1],
        [0, l2, 0, q2]
    ]
    T_0_1 = dh_matrix(*dh_params[0])
    T_1_2 = dh_matrix(*dh_params[1])
    T_0_2 = T_0_1 @ T_1_2
    return T_0_2[:3, 3].tolist()

**Hands-on Exercises**
**3D Homogeneous Transformations:**
Write a Python function transform_point(point, H) that takes a 3D point and a 4x4 homogeneous transformation matrix $H$, and returns the transformed 3D point.
Create two transformation matrices: one for a translation along X by 1 unit, and one for a rotation around Z by 90 degrees.
Apply them sequentially to a point $[1, 0, 0]$ and verify the result. 
Then apply them in reverse order and observe the difference.

**3-DOF Cylindrical Robot FK:**

Derive the DH parameters for a simple 3-DOF cylindrical robot (a base rotation, a prismatic joint for height, and another rotation for gripper orientation).

Implement the forward kinematics in Python, similar to the 2-DOF planar arm example.

Calculate the end-effector pose for a given set of joint values.

**Jacobian for a 3-DOF Arm:**
Extend the jacobian_2dof_planar function to a 3-DOF arm.

Numerically verify your Jacobian by making small changes to joint angles and comparing the calculated end-effector velocity with the actual change in position.

Capstone Project: ROS 2 Kinematics Service for a Custom Arm
Goal: Develop a ROS 2 service that provides both Forward Kinematics (FK) and Inverse Kinematics (IK) solutions for a 3-DOF robot arm.

**Requirements:**
1. Define a Custom 3-DOF Arm:

Design a simple 3-DOF robot arm (e.g., 2 revolute joints, 1 prismatic joint, or 3 revolute joints with a simple geometry).

Determine its Denavit-Hartenberg (DH) parameters.

2. Implement FK and IK Functions:

Create a Python module (src/module4_examples/kinematics/) containing forward_kinematics.py and inverse_kinematics.py.

Implement your custom arm's FK function in forward_kinematics.py.

Implement a numerical IK solver (e.g., Damped Least Squares) for your custom arm in inverse_kinematics.py.

3. Create ROS 2 Service:

Define a custom ROS 2 service message (e.g., KinematicsSolver.srv) that takes joint angles as input for FK and a target pose for IK. The response should provide the corresponding end-effector pose or joint angles.

Create a ROS 2 node (kinematics_service.py) that acts as a service server, utilizing your FK and IK functions.

4. Create ROS 2 Client:

Create a simple ROS 2 client node (kinematics_client.py) that can call your service with test data and print the results.

This capstone project will allow you to apply the theoretical knowledge of kinematics to a practical ROS 2 implementation, providing a crucial building block for robot control.

**End of Chapter 4.1**