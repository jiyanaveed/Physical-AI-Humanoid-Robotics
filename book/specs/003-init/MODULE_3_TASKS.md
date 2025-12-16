# Tasks: Module 3 - Perception, Vision, and Sensor Fusion

**Input**: Design documents from `.specify/memory/`
**Prerequisites**: `MODULE_3_PLAN.md`, `MODULE_3_SPEC.md`

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Project initialization and basic structure for Module 3.

- [X] T054 [M3] Create directories `book/docs/module-3-perception-vision-fusion/` and `src/module3_examples/` as defined in the implementation plan.
- [X] T055 [M3] Verify that the project's `Dockerfile` includes all necessary dependencies for Module 3 (OpenCV, PyTorch, etc.).

---

## Phase 2: User Story 1 - Understand and Use Core Robotics Sensors (Priority: P1) 🎯 MVP

**Goal**: A student can understand the principles of LiDAR, cameras, and IMUs, and can acquire and visualize their data in ROS 2.

**Independent Test**: A student can set up a simulated robot with all three sensors and write ROS 2 nodes to subscribe to and visualize the data streams in RViz.

### Implementation for User Story 1

- [ ] T056 [US1] [M3] Write the introductory content for Chapter 3.1, explaining the principles of LiDAR, cameras, and IMUs in `book/docs/module-3-perception-vision-fusion/chapter-3-1-robotics-sensors.md`.
- [ ] T057 [P] [US1] [M3] Implement a ROS 2 Python node to subscribe to and process `sensor_msgs/PointCloud2` in `src/module3_examples/sensors/lidar_subscriber.py`.
- [ ] T058 [P] [US1] [M3] Implement a ROS 2 Python node to subscribe to and process `sensor_msgs/Image` in `src/module3_examples/sensors/camera_subscriber.py`.
- [ ] T059 [P] [US1] [M3] Implement a ROS 2 Python node to subscribe to and process `sensor_msgs/Imu` in `src/module3_examples/sensors/imu_subscriber.py`.
- [ ] T060 [US1] [M3] Add a section to Chapter 3.1 with instructions for launching and configuring these sensors in a Gazebo simulation.
- [ ] T061 [US1] [M3] Add a tutorial to Chapter 3.1 on how to use RViz to visualize the output of each sensor.
- [ ] T062 [US1] [M3] Develop the capstone project for Chapter 3.1: a unified sensor data acquisition and processing node in `src/module3_examples/sensors/data_acquisition_node.py`.
- [ ] T063 [P] [US1] [M3] Write `pytest` tests for the subscriber nodes and the capstone project in `src/module3_examples/tests/test_sensors.py`.

---

## Phase 3: User Story 2 - Apply Computer Vision Fundamentals for Robotics (Priority: P2)

**Goal**: An intermediate student can process camera images using OpenCV in ROS 2 to extract meaningful information.

**Independent Test**: A student can write a ROS 2 node that subscribes to an image topic, uses OpenCV to detect a specific object, and publishes the result.

### Implementation for User Story 2

- [ ] T064 [US2] [M3] Write the introductory content for Chapter 3.2, explaining how to integrate OpenCV with ROS 2 using `cv_bridge` in `book/docs/module-3-perception-vision-fusion/chapter-3-2-computer-vision.md`.
- [ ] T065 [P] [US2] [M3] Create a code example demonstrating the use of `cv_bridge` to convert between ROS images and OpenCV formats in `src/module3_examples/vision/cv_bridge_example.py`.
- [ ] T066 [US2] [M3] Add a section to Chapter 3.2 covering fundamental image processing techniques (filtering, thresholding) with code examples.
- [ ] T067 [P] [US2] [M3] Implement and explain a classic feature detector (e.g., ORB) in `src/module3_examples/vision/orb_detector.py`.
- [ ] T068 [P] [US2] [M3] Implement and explain both a traditional (color-based) and a deep-learning-based image segmentation approach in `src/module3_examples/vision/segmentation_example.py`.
- [ ] T069 [US2] [M3] Develop the capstone project for Chapter 3.2: a real-time object detection and tracking node in `src/module3_examples/vision/object_detection_node.py`.
- [ ] T070 [P] [US2] [M3] Write `pytest` tests for all vision code examples and the capstone project in `src/module3_examples/tests/test_vision.py`.

---

## Phase 4: User Story 3 - Fuse Multiple Sensors for Robust State Estimation (Priority: P3)

**Goal**: An advanced student can combine data from multiple noisy sensors to produce a more accurate and robust estimate of a robot's state.

**Independent Test**: A student can implement a Kalman filter to fuse noisy wheel odometry and IMU data from a ROS 2 bag file, demonstrating a clear improvement in the resulting state estimate.

### Implementation for User Story 3

- [ ] T071 [US3] [M3] Write the theoretical foundations of Bayesian filtering for Chapter 3.3 in `book/docs/module-3-perception-vision-fusion/chapter-3-3-sensor-fusion.md`.
- [ ] T072 [US3] [M3] Develop a from-scratch Python implementation of a linear Kalman Filter for a simple 1D problem in `src/module3_examples/fusion/kalman_filter_1d.py`.
- [ ] T073 [US3] [M3] Add content to Chapter 3.3 explaining the theory of the Extended Kalman Filter (EKF) for non-linear systems.
- [ ] T074 [US3] [M3] Add content to Chapter 3.3 explaining the concept and use cases of Particle Filters.
- [ ] T075 [US3] [M3] Develop the capstone project for Chapter 3.3: an EKF-based node to fuse wheel odometry and IMU data in `src/module3_examples/fusion/ekf_fusion_node.py`.
- [ ] T076 [P] [US3] [M3] Write `pytest` tests for the Kalman filter implementations in `src/module3_examples/tests/test_fusion.py`.

---

## Phase 5: Polish & Cross-Cutting Concerns

**Purpose**: Final review, documentation, and validation for the entire module.

- [ ] T077 [P] [M3] Perform a peer review of all written content in `book/docs/module-3-perception-vision-fusion/` for technical accuracy and clarity.
- [ ] T078 [P] [M3] Perform a peer review of all code in `src/module3_examples/` for correctness, style, and documentation.
- [ ] T079 [M3] Finalize all diagrams and ensure they are correctly embedded and referenced in the markdown files.
- [ ] T080 [M3] **Constitution Check**: Verify all content and code in Module 3 against `.specify/memory/constitution.md`.
- [ ] T081 [M3] **Docusaurus Compliance**: Ensure all markdown files render correctly in a local Docusaurus build.
