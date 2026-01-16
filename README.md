# ROS2 Gazebo Differential Drive Control & Localization

This repository is my personal workspace for learning Differential Drive Control & Localization by following the **"Self-Driving and ROS 2 - Learn by Doing! Odometry & Control"** tutorial by [AntoBrandi](https://github.com/AntoBrandi).

<img width="242" height="304" alt="image" src="https://github.com/user-attachments/assets/686d5901-1e4f-436c-ac99-62cf9bb4ea12" />

https://github.com/user-attachments/assets/ce78ea03-a3c7-4505-a9e4-667a36cd703f

The main goal is to implement and understand the concepts of odometry and control for a mobile robot using ROS 2. Each package in this workspace serves as a module for a specific functionality, and each contains its own `README.md` with detailed notes on the concepts I've learned.

## Workspace Structure

This workspace is organized into the following ROS 2 packages:

*   **`bumperbot_msgs`**: Contains custom service and message definitions for the project.
*   **`bumperbot_description`**: Includes the robot's URDF model, meshes, and launch files for visualization in Rviz and simulation in Gazebo.
*   **`bumperbot_controller`**: Implements the robot's controllers, including a simple teleop controller and a "noisy" controller for simulation purposes.
*   **`bumperbot_py_examples`**: A package with Python-based ROS 2 examples, such as service clients/servers, lifecycle nodes, and QoS profile demonstrations.
*   **`bumperbot_cpp_examples`**: A package with C++-based ROS 2 examples, demonstrating concepts like service servers and TF kinematics.
*   **`bumperbot_localization`**: A package for robot localization, including nodes for IMU data processing and an Extended Kalman Filter (EKF) implementation.
*   **`bumperbot_utils`**: Contains utility scripts and nodes, such as a trajectory drawing tool.

## Setup and Build

1.  **Clone the repository:**
    ```bash
    git clone <repository-url> bumperbot_ws/src
    ```

2.  **Build the workspace:**
    Navigate to the root of the workspace and run `colcon build`:
    ```bash
    cd bumperbot_ws
    colcon build --symlink-install
    ```

3.  **Source the workspace:**
    After a successful build, source the `setup.bash` file to make the packages available in your ROS 2 environment:
    ```bash
    source install/setup.bash
    ```

## Running the Robot

Each package contains its own launch files and instructions. Please refer to the `README.md` file within each package directory for specific details on how to run the nodes and what concepts are being demonstrated.

For example, to launch the robot in Gazebo, you can use the launch file from the `bumperbot_description` package:
```bash
ros2 launch bumperbot_description gazebo.launch.py
```
