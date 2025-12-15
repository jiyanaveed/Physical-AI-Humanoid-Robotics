# Quickstart Guide: Module 4 - Control Systems and Robot Dynamics

This guide provides instructions to quickly set up your environment and get started with the code examples and capstone projects for Module 4.

## 1. Environment Setup

The code examples for Module 4 rely on a ROS 2 Humble environment with Python 3.x, NumPy, SciPy, and potentially OpenCV (as included in the `Dockerfile`).

### Using Docker (Recommended)

The easiest way to get a consistent development environment is to use the provided `Dockerfile`.

1.  **Build the Docker image**:
    ```bash
    docker build -t ros2_control_env -f Dockerfile .
    ```
2.  **Run the Docker container**:
    ```bash
    docker run -it --rm ros2_control_env bash
    ```
    This will drop you into a bash shell inside the container where the ROS 2 environment is sourced and all dependencies are available.

### Native Installation (Advanced Users)

If you prefer a native installation, ensure you have:
*   ROS 2 Humble installed (follow official ROS 2 documentation).
*   Python 3.x.
*   Python packages: `numpy`, `scipy`, `opencv-python` (if needed for visual control examples), `colcon-common-extensions`, `rosdep`.
*   Install ROS 2 Python packages for control and robotics:
    ```bash
    sudo apt update
    sudo apt install -y ros-humble-ros-base ros-humble-gazebo-ros-pkgs python3-colcon-common-extensions
    pip install -U colcon-common-extensions
    ```

## 2. Running Code Examples

All code examples for Module 4 are located in the `src/module4_examples/` directory.

### Build the ROS 2 Workspace

After cloning the repository and setting up your environment (either Docker or native), navigate to the root of the repository and build the ROS 2 workspace:

```bash
# From the repository root
colcon build --packages-select module4_examples
source install/setup.bash
```

### Example: Forward Kinematics Solver

To run a specific example, such as the forward kinematics solver:

```bash
# Navigate to the example directory
cd src/module4_examples/kinematics

# Run the Python script directly (for non-ROS 2 specific code)
python3 forward_kinematics.py
```

### Example: ROS 2 Kinematics Service (Capstone Project)

For ROS 2-based examples like the kinematics service, you will typically run them as ROS 2 nodes.

1.  **Launch the service (in one terminal)**:
    ```bash
    ros2 run module4_examples kinematics_service_node
    ```
    (Note: The exact node name will be defined during implementation, `kinematics_service_node` is a placeholder.)

2.  **Call the service (in another terminal)**:
    ```bash
    ros2 service call /kinematics_service module4_examples/srv/SolveKinematics "{joint_angles: [0.0, 1.0, 0.5], request_type: 'FK'}"
    ```
    (Note: Service name and message type are placeholders.)

## 3. Running Tests

Tests for the code examples are located in `src/module4_examples/tests/`.

```bash
# From the repository root, after building the workspace
colcon test --packages-select module4_examples
colcon test-result --all
```
