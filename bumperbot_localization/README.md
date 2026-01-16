# Bumperbot Localization

This package is dedicated to robot localization for the Bumperbot project. It demonstrates two approaches:
1.  Using the standard ROS 2 `robot_localization` package to fuse sensor data with an Extended Kalman Filter (EKF).
2.  A from-scratch implementation of a simple 1D Kalman filter to fuse angular velocity.

## Concepts Covered (Notes)

*   **Sensor Fusion**: The core concept is combining data from multiple noisy sensors (wheel odometry and an IMU) to produce a more accurate and reliable estimate of the robot's state (position and velocity).
*   **`robot_localization` Package (EKF)**: This package provides a ready-to-use EKF (and UKF) implementation. This is the standard way to perform sensor fusion in ROS. The `config/ekf.yaml` file is heavily commented and shows how to configure the EKF, including:
    *   Setting the world, odom, and base frames.
    *   Enabling `two_d_mode` for planar robots.
    *   Specifying which sensor topics to use (`odom0` for noisy odometry, `imu0` for the IMU).
    *   Configuring which variables from each sensor to fuse (e.g., `vx` from odometry, `yaw` from IMU).
*   **Data Preparation**: The `imu_republisher` node highlights a common task in localization pipelines: ensuring sensor data is in the correct format and coordinate frame before being sent to the filter.
*   **Custom Kalman Filter**: The `kalman_filter` node provides a simplified, hands-on example of how a Kalman filter works. It implements the `predict` and `update` steps for a single variable (angular velocity) and shows the core logic of fusing a motion model with a measurement.

## Nodes and Launch Files

### `local_localization.launch.py`

This is the main launch file for running the EKF-based localization. It starts:
1.  **`robot_localization/ekf_node`**: The core EKF filter, configured with `config/ekf.yaml`. It fuses `/bumperbot_controller/odom_noisy` and `/imu_ekf`.
2.  **`imu_republisher`**: A helper node (in C++ or Python) that subscribes to the raw `/imu/out` topic, changes its frame ID to `base_footprint_ekf`, and republishes it on `/imu_ekf` for the EKF to use.
3.  **`tf2_ros/static_transform_publisher`**: Defines the static location of the IMU on the robot.

**To Run**:
```bash
ros2 launch bumperbot_localization local_localization.launch.py
```

### `kalman_filter` (Python/C++)

This is a standalone node demonstrating a simple 1D Kalman filter.

*   **Description**: Subscribes to `/bumperbot_controller/odom_noisy` and `/imu/out` and publishes a new `/bumperbot_controller/odom_kalman` topic where the angular `z` velocity is the result of the sensor fusion.
*   **To Run (Python)**:
    ```bash
    ros2 run bumperbot_localization kalman_filter.py
    ```
*   **To Run (C++)**:
    ```bash
    ros2 run bumperbot_localization kalman_filter
    ```

## How to Build

```bash
colcon build --packages-select bumperbot_localization
```
