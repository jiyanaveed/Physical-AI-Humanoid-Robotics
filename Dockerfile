# Dockerfile for Physical AI & Humanoid Robotics Textbook Development

# Use the official ROS 2 Humble base image
FROM ros:humble-ros-base

# Set shell to bash
SHELL ["/bin/bash", "-c"]

# Set up the workspace
WORKDIR /ros_ws

# Install necessary dependencies
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-colcon-common-extensions \
    ros-humble-gazebo-ros-pkgs \
    curl \
    && rm -rf /var/lib/apt/lists/*

# Install PyTorch and related libraries
RUN pip install torch torchvision torchaudio

# Install Node.js and npm
RUN curl -fsSL https://deb.nodesource.com/setup_18.x | bash -
RUN apt-get install -y nodejs

# Copy the book and source code into the workspace
COPY book /ros_ws/src/book
COPY src /ros_ws/src/src

# Install Node.js dependencies for the Docusaurus book
WORKDIR /ros_ws/src/book
RUN npm install

# Set the working directory back to the workspace root
WORKDIR /ros_ws

# Source the ROS 2 setup file
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
RUN echo "source /ros_ws/install/local_setup.bash" >> ~/.bashrc

# The workspace can be built by running:
# . /opt/ros/humble/setup.sh && colcon build
