# Bumperbot Workspace: My ROS 2 Learning Journey

Welcome to my personal workspace for learning ROS 2! This repository is where I'm documenting my progress through the "Self-Driving and ROS 2 - Learn by Doing! Plan & Navigation" tutorial by AntoBrandi. My goal is to create a comprehensive guide for beginners, filled with personal notes, explanations, and troubleshooting tips that I discover along the way.

<img width="242" height="304" alt="image" src="https://github.com/user-attachments/assets/f0cac622-c647-46f7-9ec5-e16c21bcf405" />

## A Beginner's Guide to this Workspace
If you're new to ROS 2 and robotics, this workspace is designed to help you understand the foundational concepts of building and controlling a robot. We'll cover:

*   **Odometry**: Figuring out how far the robot has traveled.
*   **Control**: Making the robot move the way we want it to.
*   **Localization**: Knowing where the robot is in its environment.
*   **Mapping**: Creating a map of the environment.

This workspace is structured into several ROS 2 packages. Each package is a building block of our robot's software.

## Packages Overview

Here's a breakdown of the packages in this workspace and what they do:

-   **`bumperbot_msgs`**: Defines custom message and service types. This is the foundation for communication between our nodes.
-   **`bumperbot_description`**: Contains the robot's 3D model (URDF) and the files needed to simulate it in Gazebo.
-   **`bumperbot_controller`**: Implements the robot's controllers, including joystick teleoperation and inverse kinematics.
-   **`bumperbot_py_examples` & `bumperbot_cpp_examples`**: These packages contain Python and C++ examples that demonstrate various ROS 2 concepts.
-   **`bumperbot_localization`**: Focuses on localization techniques, using tools like the Extended Kalman Filter (EKF).
-   **`bumperbot_mapping`**: Implements the algorithm to create a 2D map of the environment.
-   **`bumperbot_utils`**: Contains helpful utility scripts, like a safety stop node and a trajectory visualizer.
-   **`bumperbot_bringup`**: Provides the main launch file to start the entire robot simulation.

Each package has its own `README.md` with more detailed notes. I highly recommend reading them to understand the concepts behind each part of the project.

## Troubleshooting Common Errors

Here are some issues I've encountered and how I solved them.

### Error: `Failed to find a free participant index for domain 0`

This is a common error in ROS 2 that happens when the DDS (Data Distribution Service), the middleware ROS 2 uses for communication, gets into a bad state.

**Symptoms:**
You might see errors like:
```
ros2: Failed to find a free participant index for domain 0
[ERROR] [rmw_cyclonedds_cpp]: rmw_create_node: failed to create domain, error Error
```
And you won't be able to list topics, nodes, or run new ROS 2 programs.

**Solution:**

The most reliable solution is to find and stop any lingering ROS 2 processes.

1.  **Stop the ROS 2 Daemon:**
    ```bash
    ros2 daemon stop
    ```

2.  **Find and Kill Lingering ROS Processes:**
    This command will find any process with "ros" in its name and terminate it.
    ```bash
    kill -9 $(ps aux | grep '[r]os' | awk '{print $2}')
    ```

After running these commands, you should be able to run `ros2 topic list` and other ROS 2 commands successfully.
