# Bumperbot Localization: My Journey to "Where Am I?"

This package is where I tackled one of the most fundamental questions in robotics: "Where am I?". This is **localization**, and it's all about using sensor data to figure out the robot's position and orientation. In this part of my learning journey, I explored using the powerful `robot_localization` package and even built my own simple filters to grasp the core ideas.

## My Personal Notes on Localization

The big takeaway for me was **sensor fusion**. Every sensor on a robot has flaws:
*   **Wheel Odometry**: Is great for tracking short-term movement, but wheels slip and slide. Over time, the small errors accumulate, and the robot gets hopelessly lost. This is called **drift**.
*   **IMU (Inertial Measurement Unit)**: Gives us pretty good data about rotation, but it also drifts over time. It can't be trusted on its own for long-term heading.

The solution is to **fuse** them! We combine the short-term accuracy of wheel odometry with the rotational correction from the IMU. This is the core idea behind this package.

### The Extended Kalman Filter (EKF)

I learned that the EKF is the magic behind sensor fusion. My simplified understanding is:
*   It's a "smart" algorithm that constantly maintains a *belief* about the robot's state (its pose and velocity).
*   It works in a two-step loop:
    1.  **Predict**: Based on the robot's last known state and the movement commands, the EKF predicts where the robot will be next.
    2.  **Update**: A new sensor measurement comes in (e.g., from the IMU). The EKF compares this to its prediction and corrects its belief. If the measurement is close to the prediction, the filter becomes more confident.

The `robot_localization` package provides a ready-to-use EKF node, so my main job was to configure it correctly.

### Understanding Coordinate Frames (TF)

TF (transformations) is crucial here. Getting this right was a big "a-ha!" moment.
*   `base_link` / `base_footprint`: The robot's own reference frame. It moves with the robot.
*   `odom`: The "odometry" frame. This is a fixed point in the world, usually where the robot started. The EKF's main job in this package is to publish a stable, accurate transform from `odom` -> `base_link`. This transform will still drift slowly over very long distances.
*   `map`: The "map" frame. This is a globally accurate, non-drifting frame. It's like a satellite view. Localizing in this frame (using AMCL) can have sudden "jumps" when the robot figures out exactly where it is on the map.

## What's in this Package?

### Key Nodes

*   `robot_localization/ekf_node`: The heart of our local localization system. It fuses odometry and IMU data.
*   `imu_republisher`: A small but vital helper node. It takes the raw IMU data, changes its `frame_id` to what the EKF expects, and republishes it. This is a common pattern in ROS!
*   `kalman_filter`: My own from-scratch 1D Kalman filter. A great exercise to understand the predict/update cycle on a single variable.
*   `odometry_motion_model.py`: A script to visualize a particle filter, which is the basis for AMCL. It shows how you can represent the robot's pose as a "cloud" of possibilities.

### Configuration

*   `config/ekf.yaml`: **The most important file.** This YAML file tells the EKF node which topics to listen to, how much to trust each sensor measurement, and other critical parameters. The `*_config` matrix is where you select which variables from each sensor message to use.

### Launch Files

*   `local_localization.launch.py`: Starts the EKF for a stable `odom` frame.
*   `global_localization.launch.py`: Starts AMCL to find the robot's pose on a pre-existing map.

## How to Use This Package

1.  **Build it**:
    ```bash
    colcon build --packages-select bumperbot_localization
    ```

2.  **Run Local Localization (EKF)**:
    This will give you a much more accurate odometry estimate.
    ```bash
    ros2 launch bumperbot_localization local_localization.launch.py
    ```

3.  **Run Global Localization (AMCL)**:
    You need a map for this!
    ```bash
    # This launch file conveniently starts the map_server for you
    ros2 launch bumperbot_localization global_localization.launch.py map_name:=small_warehouse
    ```
---
*This README documents my learning process. The concepts are simplified for clarity and to help fellow beginners.*