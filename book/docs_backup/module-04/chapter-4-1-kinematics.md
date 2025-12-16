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
*   Understand and apply 2D and 3D rigid body transformations using rotation matrices and homogeneous transformation matrices.
*   Implement Forward Kinematics (FK) for a robot arm using the Denavit-Hartenberg (DH) convention.
*   Grasp the theoretical concept of the Jacobian matrix in velocity kinematics.
*   Implement a numerical Inverse Kinematics (IK) solver (e.g., using Jacobian Transpose or Damped Least Squares).
*   Develop a ROS 2 kinematics service that provides both FK and IK solutions for a robot arm.
:::

## 4.1.1 Rigid Body Transformations: The Language of Robot Pose

A robot's **pose** refers to its position and orientation in space. To describe how different parts of a robot relate to each other, or how a robot relates to its environment, we use **rigid body transformations**. These transformations are composed of rotations and translations.

### Rotation Matrices
A **rotation matrix** describes the orientation of one coordinate frame relative to another. For a 3D rotation, it's a 3x3 matrix.

For example, a rotation around the Z-axis by an angle $\theta$ is given by:
$$ R_z(\theta) = \begin{pmatrix} \cos\theta & -\sin\theta & 0 \\ \sin\theta & \cos\theta & 0 \\ 0 & 0 & 1 \end{pmatrix} $$

### Homogeneous Transformation Matrices
To combine rotations and translations into a single matrix operation, we use **homogeneous transformation matrices**. A 4x4 homogeneous matrix $T$ describes the pose of a frame {B} relative to a frame {A}.

$$ ^A T_B = \begin{pmatrix} R_{3x3} & P_{3x1} \\ 0_{1x3} & 1 \end{pmatrix} $$
Where $R$ is the 3x3 rotation matrix and $P$ is the 3x1 translation vector.

Applying this matrix to a point $^B p$ in frame {B} (represented as $[x, y, z, 1]^T$) transforms it to frame {A}:
$$ ^A p = ^A T_B \cdot ^B p $$

Homogeneous transformation matrices allow us to chain multiple transformations: if we know $^A T_B$ and $^B T_C$, then $^A T_C = ^A T_B \cdot ^B T_C$.

## 4.1.2 Forward Kinematics (FK)

**Forward Kinematics** is the calculation of the end-effector's pose (position and orientation) given the joint angles of the robot arm. This is a fundamental problem in robotics.

The most common method for systematically deriving the FK equations for an open-chain manipulator is the **Denavit-Hartenberg (DH) convention**. It provides a standardized way to assign coordinate frames to each link of a robot and define the transformation between successive frames.

Each joint is associated with a DH parameter table containing four parameters:
*   $a_i$: link length
*   $\alpha_i$: link twist
*   $d_i$: joint offset
*   $\theta_i$: joint angle (the variable for revolute joints)

The transformation matrix between two consecutive frames $i$ and $i-1$ using DH parameters is:
$$ ^{i-1} T_i = \begin{pmatrix} \cos\theta_i & -\sin\theta_i\cos\alpha_i & \sin\theta_i\sin\alpha_i & a_i\cos\theta_i \\ \sin\theta_i & \cos\theta_i\cos\alpha_i & -\cos\theta_i\sin\alpha_i & a_i\sin\theta_i \\ 0 & \sin\alpha_i & \cos\alpha_i & d_i \\ 0 & 0 & 0 & 1 \end{pmatrix} $$

By multiplying these transformation matrices from the base to the end-effector, we can find the end-effector's pose relative to the base.

### Code Example: Forward Kinematics for a 2-DOF Planar Arm

Let's implement FK for a simple 2-DOF planar arm using Python and NumPy.

```python
# T015: Forward kinematics algorithms
# src/module4_examples/kinematics/forward_kinematics.py
import numpy as np

def dh_matrix(alpha, a, d, theta):
    """
    Calculates the Denavit-Hartenberg transformation matrix.
    alpha: link twist
    a: link length
    d: joint offset
    theta: joint angle
    """
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
    """
    Calculates the FK for a 2-DOF planar arm with lengths l1, l2 and joint angles q1, q2.
    Assumes:
    - Base at (0,0)
    - Joint 1 rotates around Z, Link 1 along X
    - Joint 2 rotates around Z, Link 2 along X
    """
    # DH parameters for a 2-DOF planar arm
    # alpha, a, d, theta
    dh_params = [
        [0, l1, 0, q1], # Joint 1
        [0, l2, 0, q2]  # Joint 2
    ]

    T_0_1 = dh_matrix(*dh_params[0])
    T_1_2 = dh_matrix(*dh_params[1])

    # End-effector transformation matrix relative to base
    T_0_2 = T_0_1 @ T_1_2

    # End-effector position is the translation part of T_0_2
    end_effector_pos = T_0_2[:3, 3]
    return end_effector_pos.tolist()

# Example usage
if __name__ == '__main__':
    l1 = 1.0 # Length of first link
    l2 = 1.0 # Length of second link
    q1 = np.pi/4 # 45 degrees
    q2 = np.pi/4 # 45 degrees

    pos = forward_kinematics_2dof_planar(l1, l2, q1, q2)
    print(f"End-effector position for q1={np.degrees(q1)}°, q2={np.degrees(q2)}°: {pos}")
    # Expected: x = l1*cos(q1) + l2*cos(q1+q2)
    #           y = l1*sin(q1) + l2*sin(q1+q2)
```

## 4.1.3 Inverse Kinematics (IK)

**Inverse Kinematics** is the inverse problem: given a desired end-effector pose, what are the required joint angles that achieve that pose?

IK is significantly harder than FK for several reasons:
*   **Multiple Solutions**: For a given end-effector pose, there might be several sets of joint angles that achieve it (e.g., "elbow up" vs. "elbow down").
*   **No Solution**: The desired pose might be unreachable by the robot (outside its workspace).
*   **Computational Complexity**: Analytical solutions (closed-form equations) exist only for simpler robot geometries. For complex robots, numerical methods are required, which involve iterative optimization.

### Numerical Inverse Kinematics using the Jacobian

Numerical IK solvers use iterative methods to find joint angles. A common approach involves the **Jacobian matrix**.

The **Jacobian matrix ($J$)** relates joint velocities to end-effector velocities:
$$ \dot{x} = J(q) \dot{q} $$
Where $\dot{x}$ is the end-effector's linear and angular velocity, and $\dot{q}$ is the vector of joint velocities.

To solve for $\dot{q}$ given a desired $\dot{x}$, we need to find the inverse of the Jacobian. However, the Jacobian is often not square or might be singular. Techniques like **Jacobian Transpose** or **Damped Least Squares (DLS)** are used to find a pseudo-inverse of the Jacobian:
$$ \dot{q} = J^T \dot{x} \quad \text{(Jacobian Transpose)} $$
$$ \dot{q} = (J^T J + \lambda^2 I)^{-1} J^T \dot{x} \quad \text{(Damped Least Squares)} $$
Where $\lambda$ is a damping factor and $I$ is the identity matrix.

These methods are used iteratively:
1.  Calculate the current end-effector error (difference between current and desired pose).
2.  Convert this error to a desired end-effector velocity.
3.  Use the pseudo-inverse Jacobian to find the required joint velocities.
4.  Update joint angles: $q_{k+1} = q_k + \Delta q_k$.
5.  Repeat until the error is below a threshold.

### Code Example: Numerical IK for a 2-DOF Planar Arm

```python
# T016: Inverse kinematics algorithms
# src/module4_examples/kinematics/inverse_kinematics.py
import numpy as np

# Assume forward_kinematics_2dof_planar and dh_matrix from forward_kinematics.py are available

def jacobian_2dof_planar(l1, l2, q1, q2):
    """
    Calculates the Jacobian for a 2-DOF planar arm.
    The end-effector position is (x, y).
    J = [dx/dq1  dx/dq2]
        [dy/dq1  dy/dq2]
    """
    J = np.array([
        [-l1*np.sin(q1) - l2*np.sin(q1+q2), -l2*np.sin(q1+q2)],
        [ l1*np.cos(q1) + l2*np.cos(q1+q2),  l2*np.cos(q1+q2)]
    ])
    return J

def inverse_kinematics_2dof_planar(l1, l2, target_pos, initial_q, max_iterations=100, tolerance=1e-3, learning_rate=0.1):
    """
    Numerical IK solver for a 2-DOF planar arm using Jacobian Transpose.
    target_pos: [x, y]
    initial_q: [q1, q2] - starting joint angles
    """
    q = np.array(initial_q, dtype=float)
    target = np.array(target_pos)

    for i in range(max_iterations):
        current_pos = np.array(forward_kinematics_2dof_planar(l1, l2, q[0], q[1]))[:2] # Only x, y
        
        error = target - current_pos
        if np.linalg.norm(error) < tolerance:
            break

        J = jacobian_2dof_planar(l1, l2, q[0], q[1])
        # Use Jacobian Transpose
        delta_q = J.T @ error * learning_rate
        q += delta_q

        # Optional: Apply joint limits if necessary
        # q[0] = np.clip(q[0], -np.pi/2, np.pi/2)
        # q[1] = np.clip(q[1], -np.pi/2, np.pi/2)

    return q.tolist() if np.linalg.norm(error) < tolerance else None

# Example usage
if __name__ == '__main__':
    l1 = 1.0
    l2 = 1.0
    target_x = 0.5
    target_y = 1.5
    initial_q = [0.1, 0.1] # Small initial angles

    solved_q = inverse_kinematics_2dof_planar(l1, l2, [target_x, target_y], initial_q)
    
    if solved_q:
        print(f"Target position: ({target_x:.2f}, {target_y:.2f})")
        print(f"Solved joint angles: q1={np.degrees(solved_q[0]):.2f}°, q2={np.degrees(solved_q[1]):.2f}°")
        final_pos = forward_kinematics_2dof_planar(l1, l2, solved_q[0], solved_q[1])
        print(f"FK with solved angles: ({final_pos[0]:.2f}, {final_pos[1]:.2f})")
    else:
        print(f"Could not reach target position ({target_x:.2f}, {target_y:.2f})")
```

## Hands-on Exercises

1.  **3D Homogeneous Transformations**:
    *   Write a Python function `transform_point(point, H)` that takes a 3D point and a 4x4 homogeneous transformation matrix $H$, and returns the transformed 3D point.
    *   Create two transformation matrices: one for a translation along X by 1 unit, and one for a rotation around Z by 90 degrees.
    *   Apply them sequentially to a point `[1, 0, 0]` and verify the result. Then apply them in reverse order and observe the difference.
2.  **3-DOF Cylindrical Robot FK**:
    *   Derive the DH parameters for a simple 3-DOF cylindrical robot (a base rotation, a prismatic joint for height, and another rotation for gripper orientation).
    *   Implement the forward kinematics in Python, similar to the 2-DOF planar arm example.
    *   Calculate the end-effector pose for a given set of joint values.
3.  **Jacobian for a 3-DOF Arm**:
    *   Extend the `jacobian_2dof_planar` function to a 3-DOF arm.
    *   Numerically verify your Jacobian by making small changes to joint angles and comparing the calculated end-effector velocity with the actual change in position.

## Capstone Project: ROS 2 Kinematics Service for a Custom Arm

**Goal**: Develop a ROS 2 service that provides both Forward Kinematics (FK) and Inverse Kinematics (IK) solutions for a 3-DOF robot arm.

**Requirements**:
1.  **Define a Custom 3-DOF Arm**:
    *   Design a simple 3-DOF robot arm (e.g., 2 revolute joints, 1 prismatic joint, or 3 revolute joints with a simple geometry).
    *   Determine its Denavit-Hartenberg (DH) parameters.
2.  **Implement FK and IK Functions**:
    *   Create a Python module (`src/module4_examples/kinematics/`) containing `forward_kinematics.py` and `inverse_kinematics.py`.
    *   Implement your custom arm's FK function in `forward_kinematics.py`.
    *   Implement a numerical IK solver (e.g., Damped Least Squares) for your custom arm in `inverse_kinematics.py`.
3.  **Create ROS 2 Service**:
    *   Define a custom ROS 2 service message (e.g., `KinematicsSolver.srv`) that takes joint angles as input for FK and a target pose for IK. The response should provide the corresponding end-effector pose or joint angles.
    *   Create a ROS 2 node (`kinematics_service.py`) that acts as a service server, utilizing your FK and IK functions.
4.  **Create ROS 2 Client**:
    *   Create a simple ROS 2 client node (`kinematics_client.py`) that can call your service with test data and print the results.

This capstone project will allow you to apply the theoretical knowledge of kinematics to a practical ROS 2 implementation, providing a crucial building block for robot control.

---
**End of Chapter 4.1**