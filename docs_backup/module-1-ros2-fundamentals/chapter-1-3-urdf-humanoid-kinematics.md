---
title: Chapter 1.3: URDF and Humanoid Kinematics
description: Defining the robot model using URDF and calculating forward and inverse kinematics.
sidebar_position: 3
---
import Tabs from '@theme/Tabs';
import TabItem from '@theme/TabItem';

# Chapter 1.3: URDF and Humanoid Kinematics

Welcome to the professional-level chapter of Module 1. Having mastered ROS 2 communication and agent-based programming, you are now ready to tackle one of the most critical aspects of robotics: defining the robot's physical structure. In this chapter, we will master the **Unified Robot Description Format (URDF)**, an XML-based standard for modeling a robot's physical properties. We will go from simple links and joints to modeling a complex humanoid robot, and touch upon the essential concepts of forward and inverse kinematics.

:::tip[Learning Objectives]
*   Understand and write URDF files from scratch.
*   Model a robot's structure using links and joints.
*   Define visual, collision, and inertial properties for realistic simulation.
*   Use **Xacro** to create modular, reusable, and parameter-driven URDF files.
*   Visualize and debug your robot model in **RViz2**.
*   Grasp the fundamental concepts of forward and inverse kinematics.
*   Design and model a complete humanoid upper body.
:::

## 1.3.1 What is URDF?

A robot is a physical entity. For any software to control it, it must have an accurate model of the robot's body, including its size, shape, mass, joint types, and movement limits. URDF is the standard way to create this model in the ROS ecosystem.

An URDF file is an XML file that describes a robot as a tree of **links** connected by **joints**.

*   **`<link>`**: Describes a rigid part of the robot (e.g., a limb segment, a gripper, a torso). It has properties like:
    *   **`<visual>`**: The shape, color, and texture of the link for rendering in simulators like Gazebo and RViz.
    *   **`<collision>`**: The geometry of the link used for collision detection by the physics engine. This is often simpler than the visual geometry to save computation.
    *   **`<inertial>`**: The mass and rotational inertia of the link, which are crucial for accurate physics simulation.
*   **`<joint>`**: Describes the kinematic and dynamic properties of the connection between two links. Key attributes include:
    *   **`type`**: The type of motion the joint allows. Common types are:
        *   `revolute`: A hinge joint that rotates around a single axis (e.g., an elbow).
        *   `continuous`: A revolute joint with no angle limits.
        *   `prismatic`: A sliding joint that moves along an axis (e.g., a piston).
        *   `fixed`: A rigid connection with no movement.
    *   **`<parent>`** and **`<child>`**: The two links that the joint connects.
    *   **`<origin>`**: The transform (position and orientation) of the child link relative to the parent link.
    *   **`<axis>`**: The axis of rotation or translation for revolute and prismatic joints.
    *   **`<limit>`**: Specifies the joint's range of motion (for revolute) and maximum velocity/effort.

### Code Example: A Simple Two-Link Arm URDF

Let's model a simple arm with two links connected by a revolute joint.

```xml
<!-- T019: An example URDF for a simple robotic arm -->
<!-- src/module1_examples/urdf/simple_arm.urdf -->

<robot name="simple_arm">

  <!-- Base Link -->
  <link name="base_link">
    <visual>
      <geometry>
        <cylinder length="0.2" radius="0.1"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 0.8 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.2" radius="0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/>
    </inertial>
  </link>

  <!-- Arm Link -->
  <link name="arm_link">
    <visual>
      <geometry>
        <box size="0.5 0.1 0.1"/>
      </geometry>
      <material name="white">
        <color rgba="1 1 1 1"/>
      </material>
    </visual>
    <collision>
       <geometry>
        <box size="0.5 0.1 0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.5"/>
      <inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.001"/>
    </inertial>
  </link>

  <!-- Joint connecting base_link and arm_link -->
  <joint name="shoulder_joint" type="revolute">
    <parent link="base_link"/>
    <child link="arm_link"/>
    <origin xyz="0 0 0.1" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="10.0" velocity="1.0"/>
  </joint>

</robot>
```
To visualize this, you can use the `display.launch.py` file provided in the `urdf_tutorial` package, or write your own launch file that starts `robot_state_publisher` and `joint_state_publisher_gui`.

## 1.3.2 Xacro: The URDF Macro Language

As robots become more complex, writing URDF files by hand gets tedious and error-prone. Imagine defining a 7-DOF arm where each link is nearly identical! This is where **Xacro (XML Macros)** comes in.

Xacro is a macro language that lets you create more readable, modular, and maintainable URDF files. Key features include:
*   **Constants**: Define properties like `PI` or `arm_length` once and reuse them.
*   **Macros**: Create reusable templates for links and joints. You can define a `create_arm_segment` macro and call it multiple times with different parameters.
*   **Mathematical Expressions**: Perform calculations directly within the XML.
*   **File Includes**: Split your robot model into multiple files (e.g., `torso.xacro`, `left_arm.xacro`) and include them in a top-level file.

### Code Example: A Parameterized Arm using Xacro

Let's refactor our simple arm to use Xacro.

<Tabs>
  <TabItem value="torso" label="src/module1_examples/urdf/torso_with_arms.xacro" default>
```xml
<!-- T020: An example xacro for a torso with arms -->
<robot name="torso_with_arms" xmlns:xacro="http://www.ros.org/wiki/xacro">

  <!-- Include the arm macro -->
  <xacro:include filename="$(find module1_examples)/urdf/macros.xacro" />

  <!-- Constants -->
  <xacro:property name="torso_length" value="0.6" />
  <xacro:property name="torso_radius" value="0.2" />

  <!-- Base Link -->
  <link name="base_link" />
  
  <joint name="base_to_torso" type="fixed">
    <parent link="base_link"/>
    <child link="torso"/>
    <origin xyz="0 0 0.05" rpy="0 0 0"/>
  </joint>

  <!-- Torso -->
  <link name="torso">
    <visual>
      <geometry>
        <cylinder length="${torso_length}" radius="${torso_radius}" />
      </geometry>
    </visual>
    <collision>
      <geometry>
        <cylinder length="${torso_length}" radius="${torso_radius}" />
      </geometry>
    </collision>
    <inertial>
      <mass value="5.0"/>
      <inertia ixx="0.1" ixy="0" ixz="0" iyy="0.1" iyz="0" izz="0.1"/>
    </inertial>
  </link>

  <!-- Instantiate the left and right arms using the macro -->
  <xacro:arm_macro side="left" parent="torso" y_reflect="1" />
  <xacro:arm_macro side="right" parent="torso" y_reflect="-1" />

</robot>
```
  </TabItem>
  <TabItem value="macros" label="src/module1_examples/urdf/macros.xacro">
```xml
<!-- Reusable macros for robot parts -->
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">

  <!-- A macro for a single arm segment -->
  <xacro:macro name="arm_macro" params="side parent y_reflect">
    <link name="${side}_arm_link">
      <visual>
        <geometry><box size="0.5 0.1 0.1"/></geometry>
        <origin xyz="0.25 0 0" rpy="0 0 0"/>
        <material name="white"><color rgba="1 1 1 1"/></material>
      </visual>
      <collision>
        <geometry><box size="0.5 0.1 0.1"/></geometry>
        <origin xyz="0.25 0 0" rpy="0 0 0"/>
      </collision>
      <inertial>
        <mass value="0.5"/>
        <inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.001"/>
      </inertial>
    </link>

    <joint name="${side}_shoulder_joint" type="revolute">
      <parent link="${parent}"/>
      <child link="${side}_arm_link"/>
      <origin xyz="0 ${y_reflect * (torso_radius + 0.05)} ${torso_length / 2}" rpy="0 0 0"/>
      <axis xyz="0 1 0"/>
      <limit lower="-1.57" upper="1.57" effort="10.0" velocity="1.0"/>
    </joint>
  </xacro:macro>

</robot>
```
  </TabItem>
</Tabs>

To convert a `.xacro` file to a `.urdf` file that ROS can use, you run the command:
`xacro my_robot.xacro > my_robot.urdf`

## 1.3.3 Introduction to Kinematics

Kinematics is the study of motion without considering the forces that cause it. In robotics, it's the mathematical relationship between the joint parameters of the robot and the position and orientation of its end-effectors (like a hand or a tool).

### Forward Kinematics (FK)
**Forward Kinematics** answers the question: "If I know the angle of each joint in the robot's arm, where is the hand located in space?"

This is a relatively straightforward calculation. ROS 2's `robot_state_publisher` node automatically does this for you! It reads the `/joint_states` topic (which contains the current angle/position of each joint), computes the forward kinematics based on your URDF, and broadcasts the resulting transforms (`tf2`) of all the links.

### Inverse Kinematics (IK)
**Inverse Kinematics** is the opposite and much harder problem: "If I want to place the robot's hand at a specific position and orientation in space, what should the angles of all the joints in the arm be?"

IK is computationally expensive and often has multiple solutions (or no solution at all). It's the foundation of motion planning. ROS 2 has powerful libraries like **MoveIt** that provide sophisticated IK solvers. While implementing an IK solver is beyond the scope of this chapter, understanding the concept is crucial for using these advanced tools.

## Hands-on Exercises

1.  **Build a Mobile Robot URDF**:
    *   Create a URDF for a simple four-wheeled robot.
    *   It should have a chassis link and four wheel links.
    *   The joints connecting the wheels to the chassis should be `continuous`.
    *   Visualize it in RViz.
2.  **Xacro All the Things**:
    *   Convert your mobile robot URDF into a `.xacro` file.
    *   Use properties for wheel radius, chassis size, etc.
    *   Create a macro for a wheel and instantiate it four times with different parameters.
3.  **Explore Forward Kinematics**:
    *   Using the `simple_arm` URDF, write a launch file that starts `robot_state_publisher` and `joint_state_publisher_gui`.
    *   Start RViz and add a TF display.
    *   Use the GUI to change the joint angle and observe how the `arm_link` transform moves in RViz.

## Capstone Project: Model a Humanoid Upper Body

**Goal**: Create a complete, well-structured Xacro model for a humanoid robot's upper body.

**System Components**:
Your model should be broken into multiple `.xacro` files for modularity:
1.  **`humanoid.xacro`**: The top-level file that includes the other parts.
2.  **`torso.xacro`**: Defines the torso, which might have a `revolute` joint at the waist.
3.  **`head.xacro`**: Defines the head, with a 2-DOF neck (pan and tilt `revolute` joints).
4.  **`arm.xacro`**: A macro that creates a complete 7-DOF arm (e.g., shoulder pitch, shoulder roll, shoulder yaw, elbow, wrist yaw, wrist pitch, wrist roll).
    *   Use this macro twice in `humanoid.xacro` to create the left and right arms.

**Requirements**:
*   All links must have `<visual>`, `<collision>`, and `<inertial>` tags.
*   All joints must have realistic `<limit>` tags.
*   The final model must load in RViz without errors.
*   You should be able to move all joints using the `joint_state_publisher_gui`.

This capstone will solidify your understanding of robot modeling and prepare you for the advanced motion planning and control topics in future modules.

---
**End of Chapter 1.3**