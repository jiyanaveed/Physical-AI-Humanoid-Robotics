# Research: Best Practices for Teaching URDF

This document outlines best practices for teaching the Unified Robot Description Format (URDF).

## 1. Start with the Basics
- Begin with a very simple, non-robotic object, like a "link" representing a single box. Show how to define its visual and collision properties.
- Introduce a second link and a simple `revolute` joint to connect them. This creates a simple pendulum, which is a great first step.

## 2. Use `xacro` Early
- Introduce `xacro` (XML Macros) as soon as the basic concepts of links and joints are covered. Teaching `xacro` from the beginning prevents students from writing repetitive and hard-to-maintain raw URDF files.
- Emphasize the use of properties and macros to create modular and reusable robot descriptions.

## 3. Visualize Everything
- Constantly encourage students to visualize their URDF/xacro files in RViz using `robot_state_publisher` and `joint_state_publisher_gui`. This provides immediate visual feedback and helps in debugging.

## 4. Gradual Complexity
- Move from a simple arm to a more complex one, then to a simple mobile robot, and finally to a humanoid. This gradual increase in complexity helps build confidence.
