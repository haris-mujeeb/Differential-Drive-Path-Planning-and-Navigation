# Bumperbot Localization: A Beginner's Guide & Personal Notes

This package is dedicated to figuring out "Where am I?" for the Bumperbot. In robotics, this is called **localization**. We explore two main ways to do this: using the powerful, industry-standard `robot_localization` package and building our own simple filters from scratch to understand the core concepts.

## Key Concepts & My Personal Notes

Localization is all about fusing noisy sensor data to get a better estimate of the robot's position and orientation (its "pose").

### Sensor Fusion: The Core Idea

*   **Problem**: Every sensor is a little bit wrong.
    *   **Wheel Odometry**: Wheels can slip, so the distance traveled isn't always accurate. Over time, these small errors add up, and the robot gets "lost."
    *   **IMU (Inertial Measurement Unit)**: Provides good rotation data (yaw), but it "drifts" over long periods. It's good for short-term rotation, but can't be trusted by itself for heading.
*   **Solution**: We **fuse** these two data sources. We take the good parts of each (short-term accuracy from odometry, rotational correction from the IMU) and combine them.

### The Extended Kalman Filter (EKF)

*   **What it is**: Think of the EKF as a "smart" ongoing average. It's a mathematical tool that maintains a "belief" about the robot's state (its pose and velocity).
*   **How it works (in simple terms)**:
    1.  **Predict**: The filter predicts where the robot will be next, based on its last known state and the odometry motion commands. This prediction is always a bit uncertain.
    2.  **Update**: A new sensor measurement comes in (e.g., from the IMU). The filter compares this measurement to its prediction.
    3.  **Correction**: If the measurement is close to the prediction, the filter gets more confident. If it's far off, it adjusts its belief, trusting the sensor that it's configured to trust more.
*   **In this Package**: The `robot_localization` package provides a ready-to-use EKF node. We just need to configure it!

### Understanding Coordinate Frames (TF)

This is one of the most critical and often confusing parts of ROS!
*   `base_link` (or `base_footprint`): A coordinate frame attached to the robot's body. It moves with the robot.
*   `odom`: A world-fixed frame. It represents the "starting point" of the robot. The pose in this frame is continuous and good for local navigation, but it will drift over time. The EKF's primary job is to publish the transform between `odom` and `base_link`.
*   `map`: Another world-fixed frame. This frame is globally accurate and doesn't drift. It's used for long-term navigation in a known environment. It's subject to large "jumps," for example, when a GPS signal or AMCL provides a correction.

### Data Preparation: The `imu_republisher`

You can't just feed raw data into a filter. The `imu_republisher` node is a perfect example of a common task:
1.  It subscribes to the raw IMU data from the `/imu/out` topic.
2.  It **changes the `frame_id`** in the message's header to what the EKF expects (`base_footprint_ekf`). This tells the EKF how to correctly interpret the data in the context of other sensor information.
3.  It republishes the corrected data on a new topic (`/imu_ekf`) for the EKF to consume.

## Package Structure and Nodes

### Launch Files

*   `local_localization.launch.py`: This is the main launch file for running EKF-based localization without a map. It's for getting a reliable, non-drifting `odom` frame.
*   `global_localization.launch.py`: This is for localizing the robot within a pre-existing map. It uses **AMCL** (Adaptive Monte Carlo Localization).

### Configuration Files

*   `config/ekf.yaml`: **This is the most important file for the EKF**. It tells the `ekf_node` everything it needs to know:
    *   `frequency`: How often to publish the updated pose.
    *   `two_d_mode`: Set to `true` for robots that only move on a flat plane.
    *   `odom0`, `imu0`: These are the input topics.
    *   `odom0_config`, `imu0_config`: A 15-element array that specifies which parts of the message to use. The order is `[X, Y, Z, R, P, Y, VX, VY, VZ, VR, VP, VY, AX, AY, AZ]`. For our `odom0`, we only use `VX` and `VY`. For our `imu0`, we use `Yaw` and `Yaw Velocity`.
*   `config/amcl.yaml`: This configures the AMCL node for global localization. It defines things like the robot's motion model and how to use the laser scanner to match against the map.

### Nodes

*   `robot_localization/ekf_node`: The core EKF filter that fuses odometry and IMU data.
*   `imu_republisher` (C++/Python): A helper node to prepare IMU data for the EKF.
*   `kalman_filter` (C++/Python): A from-scratch, simplified 1D Kalman filter. A great learning tool to see the predict/update cycle in action for a single variable (angular velocity).
*   `odometry_motion_model.py`: A script that implements a probabilistic motion model. It shows how you can represent the robot's pose not as a single point, but as a cloud of "samples" or "particles," which is the core idea behind Particle Filters and AMCL.

## How to Use

### 1. Build the Package

```bash
colcon build --packages-select bumperbot_localization
```

### 2. Run Local Localization (EKF)

This fuses wheel odometry and IMU data to publish a more accurate `odom` -> `base_footprint_ekf` transform.

```bash
ros2 launch bumperbot_localization local_localization.launch.py
```

### 3. Run Global Localization (AMCL)

This localizes the robot within a known map. It requires a map to be running.

```bash
# This will also start the map_server
ros2 launch bumperbot_localization global_localization.launch.py map_name:=small_warehouse
```

### 4. Run the Standalone Examples

These are great for learning and visualization.

*   **Simple 1D Kalman Filter (Python):**
    ```bash
    ros2 run bumperbot_localization kalman_filter.py
    ```
*   **Odometry Motion Model (for Particle Filter visualization):**
    ```bash
    ros2 run bumperbot_localization odometry_motion_model.py
    ```
    Open RViz and visualize the `/odometry_motion_model/samples` topic (as a `PoseArray`) to see the particle cloud.

---
*This README is intended as a personal learning log and a guide for beginners. The concepts are simplified for clarity.*