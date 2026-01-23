# Bumperbot Workspace

This repository is my personal workspace for learning Odometry, Control, Localization, and Mapping for a differential drive robot using ROS 2. It is based on the tutorial "Self-Driving and ROS 2 - Learn by Doing! Odometry & Control" by AntoBrandi.

<img width="242" height="304" alt="image" src="https://github.com/user-attachments/assets/f0cac622-c647-46f7-9ec5-e16c21bcf405" />



https://github.com/user-attachments/assets/acb764d9-9d00-4e52-9a3b-6921c6f79d77



## Packages

This workspace is organized into several ROS 2 packages, each with a specific purpose:

-   **`bumperbot_bringup`**: Contains the main launch file to bring up the entire robot simulation.
-   **`bumperbot_controller`**: Implements the controllers for the robot, including a simple and a noisy controller, and joystick teleoperation.
-   **`bumperbot_cpp_examples`**: Contains C++ examples for various ROS 2 concepts.
-   **`bumperbot_description`**: Contains the robot's URDF, meshes, and launch files for simulation and visualization.
-   **`bumperbot_localization`**: Implements localization for the robot, including an EKF-based approach and a from-scratch Kalman filter.
-   **`bumperbot_msgs`**: Defines custom ROS 2 messages and services.
-   **`bumperbot_py_examples`**: Contains Python examples for various ROS 2 concepts.
-   **`bumperbot_utils`**: Contains utility scripts.

Each package has its own `README.md` file with more detailed notes on the concepts learned and the implementation details.
