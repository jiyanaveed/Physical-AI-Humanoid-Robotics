#!/bin/bash

# --- Configuration ---
# NOTE: The API key must be set in your terminal BEFORE running the script:
# export GEMINI_API_KEY="YOUR_KEY_HERE"

DOCS_DIR="./book/docs" 
MODEL="gemini-2.5-flash"

PRIMARY_PROMPT="You are an expert technical writer specializing in humanoid robotics and physical AI. Generate a comprehensive, detailed, and professional chapter of technical documentation, suitable for a university-level textbook. The chapter title and topic are: "

# Use a simple, non-associative array (lists) to avoid Zsh arithmetic issues.
# We list paths and titles in order.

CHAPTER_PATHS=(
    "module-1-ros2-fundamentals/chapter-1-1-nodes-topics-services"
    "module-1-ros2-fundamentals/chapter-1-2-python-agents-ros"
    "module-1-ros2-fundamentals/chapter-1-3-urdf-humanoid-kinematics"
    "module-2-simulation-sensors/chapter-2-1-path-planning"
    "module-2-simulation-sensors/chapter-2-2-slam-principles"
    "module-2-simulation-sensors/chapter-2-3-reinforcement-learning"
    "module-3-perception-vision/chapter-3-1-robotics-sensors"
    "module-3-perception-vision/chapter-3-2-computer-vision"
    "module-3-perception-vision/chapter-3-3-sensor-fusion"
    "module-4-control-dynamics/chapter-4-1-kinematics"
    "module-4-control-dynamics/chapter-4-2-control-theory"
    "module-4-control-dynamics/chapter-4-3-advanced-dynamics"
)

CHAPTER_TITLES=(
    "ROS 2 Nodes, Topics, and Services"
    "Python Agents and ROS 2 Interfaces"
    "URDF Model Setup and Humanoid Kinematics Introduction"
    "Advanced Path Planning Algorithms"
    "SLAM Principles and 3D Reconstruction"
    "Reinforcement Learning for Locomotion"
    "Advanced Robotics Sensors (Lidar, RGB-D)"
    "Real-time Computer Vision for Robotics"
    "Sensor Fusion for State Estimation (IMU/Vision)"
    "Kinematics"
    "Control Theory"
    "Advanced Dynamics"
)

echo "Starting content generation for ${#CHAPTER_PATHS[@]} chapters using $MODEL..."
echo "---"

# Iterate using a standard C-style loop for reliable indexing
for i in "${!CHAPTER_PATHS[@]}"; do
    CHAPTER_PATH="${CHAPTER_PATHS[$i]}"
    TITLE="${CHAPTER_TITLES[$i]}"
    FILE_PATH="${DOCS_DIR}/${CHAPTER_PATH}.md"

    echo "➡️ Processing Chapter: $TITLE"

    # 2. Construct the full, specific prompt for Gemini
    FULL_PROMPT="${PRIMARY_PROMPT}'$TITLE'. Include code snippets in markdown format where relevant. Ensure the output is only the markdown content, do not include the YAML front matter."

    # 3. Call the Gemini CLI to generate content
    GENERATED_CONTENT=$(gemini generate-content -m "$MODEL" -p "$FULL_PROMPT" 2> /dev/null)

    if [ -z "$GENERATED_CONTENT" ]; then
        echo "❌ ERROR: Failed to generate content for $TITLE. (API check passed, likely prompt/quota issue)"
        continue
    fi

    # 4. Append the generated content to the existing file
    echo "$GENERATED_CONTENT" >> "$FILE_PATH"

    echo "✅ Success: Content appended to $FILE_PATH"
    echo "---"
done

echo "Content generation finished. Review all files before committing."