# Bumperbot Controller

This package contains the ROS 2 controllers for the Bumperbot robot.

## Project Explanation and Notes

This section will detail the steps involved in setting up and understanding the bumperbot_controller package. It will also serve as a personal reference for future development.

### 1. Overview
*   **Purpose:** The `bumperbot_controller` package is responsible for controlling the various joints and actuators of the Bumperbot robot within the ROS 2 framework.
*   **Technologies:** It utilizes `ros2_control` for hardware abstraction and controller management.

### 2. Setup (Notes)
*   Ensure `ros2_control` and its dependencies are installed.
*   This package typically works in conjunction with `bumperbot_description` (for URDF/XACRO definition) and a bringup package for launching the robot.

### 3. Key Files and Directories
*   `config/`: Contains YAML configuration files for the controllers.
*   `src/`: C++ source files for custom controllers (`simple_controller.cpp`, `noisy_controller.cpp`).
*   `bumperbot_controller/`: Python source files for custom controllers (`simple_controller.py`, `noisy_controller.py`).
*   `launch/`: Python launch files for starting controllers.
*   `CMakeLists.txt`/`package.xml`: Standard ROS 2 package files.

### 4. How to Use/Launch
*   Compile the workspace: `colcon build --packages-select bumperbot_controller`
*   Source the workspace: `. install/setup.bash`
*   Launch the main controller launch file:
    ```bash
    ros2 launch bumperbot_controller controller.launch.py
    ```
    You can specify which implementation to use (C++ or Python) for the simple controller:
    *   To launch the Python version:
        ```bash
        ros2 launch bumperbot_controller controller.launch.py use_python:=true
        ```
    *   To launch the C++ version (default):
        ```bash
        ros2 launch bumperbot_controller controller.launch.py use_python:=false
        ```

### 5. Testing the Robot Setup

To quickly test if the velocity controller is working correctly, you can publish a command to its topic:

```bash
ros2 topic pub /simple_velocity_controller/commands std_msgs/msg/Float64MultiArray "{data: [5.0, 0.0]}"
```
This command sends a velocity of 5.0 units to the first joint and 0.0 to the second joint. Observe the robot's behavior in simulation or on hardware.

### 6. Controlling the Robot with Inverse Kinematics (`simple_controller`)

The `simple_controller` nodes (in both C++ and Python) implement inverse kinematics to control the robot's linear (`vx`) and angular (`wz`) velocities by converting them into individual wheel velocities. This allows for more intuitive control of the robot's motion.

To control the robot using these nodes, publish `geometry_msgs/msg/TwistStamped` messages to the `/bumperbot_controller/cmd_vel` topic:

```bash
ros2 topic pub /bumperbot_controller/cmd_vel geometry_msgs/msg/TwistStamped "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: ''}, twist: {linear: {x: 0.2, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.5}}}"
```

This command will set a linear velocity of 0.2 m/s in the x-direction and an angular velocity of 0.5 rad/s around the z-axis.

### 7. Noisy Controller (`noisy_controller`)

The `noisy_controller` nodes (in both C++ and Python) are designed for simulation purposes to test localization algorithms. They mimic the behavior of real-world wheel encoders by introducing noise into the joint state readings.

*   **Purpose**: To provide a more realistic odometry source by adding random Gaussian noise to the wheel encoder positions before calculating the robot's pose.
*   **Functionality**:
    *   Subscribes to the `/joint_states` topic.
    *   Adds noise to the left and right wheel positions.
    *   Calculates odometry based on these noisy values.
    *   Publishes the resulting odometry to the `/bumperbot_controller/odom_noisy` topic.
    *   Publishes the corresponding TF transform from `odom` to `base_footprint_noisy`.

This "noisy" odometry can then be fused with other sensor data (e.g., from an IMU) in a sensor fusion algorithm like an Extended Kalman Filter (EKF) to produce a more accurate and robust localization.

### 8. ROS 2 Control CLI Commands

These commands are useful for inspecting and managing `ros2_control` components.

*   **List all available controllers:**
    ```bash
    ros2 control list_controllers
    ```

*   **List all hardware components (e.g., Robot Hardware interfaces):**
    ```bash
    ros2 control list_hardware_components
    ```

*   **Load and start a controller (example: `joint_state_broadcaster`):**
    ```bash
    ros2 control load_controller joint_state_broadcaster
    ros2 control set_controller_state joint_state_broadcaster start
    ```

*   **Stop and unload a controller:**
    ```bash
    ros2 control set_controller_state joint_state_broadcaster stop
    ros2 control unload_controller joint_state_broadcaster
    ```

*   **List controller types:**
    ```bash
    ros2 control list_controller_types
    ```

### 9. Parameters

The `simple_controller` and `noisy_controller` nodes accept the following parameters:

*   `wheel_radius`: The radius of the robot's wheels in meters.
*   `wheel_separation`: The distance between the centers of the two wheels in meters.

These parameters can be set in the launch file or overridden via the command line.

### 10. Joystick Teleoperation with `twist_mux`

This package provides a robust teleoperation setup using `twist_mux` to safely manage multiple sources of velocity commands.

*   **`joystick_teleop.launch.py`**: This is the main launch file for teleoperation. It starts the following nodes:
    *   `joy_node`: Reads raw data from the joystick.
    *   `joy_normalizer.py`: A custom node that normalizes the joystick output, making it easier to handle different joystick types. It also provides a turbo mode.
    *   `twist_mux`: The core component that arbitrates between different topics publishing `Twist` messages. It selects one based on priority and locks.
    *   `twist_relay.py`: A custom node that relays the selected `Twist` message from `twist_mux` to the robot's controller, but only if the `safety_stop` topic is not `True`.

*   **`twist_mux` Configuration**:
    *   `config/twist_mux_topics.yaml`: Defines the input topics for `twist_mux`. For example, it can listen to `/joy_vel` (from the joystick) and `/nav_vel` (from a navigation stack).
    *   `config/twist_mux_locks.yaml`: Configures the locking mechanism. For example, if a high-priority topic like `/e_stop` receives a message, all other topics can be locked out.
    *   `config/twist_mux_joy.yaml`: Maps joystick buttons to specific `twist_mux` functionalities, like selecting the input topic.

*   **To Launch**:
    ```bash
    ros2 launch bumperbot_controller joystick_teleop.launch.py
    ```

### 11. Note on Forward Differential Kinematics for a Differential Drive Robot

Forward Differential Kinematics relates the wheel velocities to the robot's chassis velocity. For a differential drive robot, this is expressed using the Jacobian matrix.

Given:
*   `v_e`: The robot's chassis velocity vector, `[vx, vy, θ_dot]^T`, where `vx` is the linear velocity along the robot's forward direction, `vy` is the linear velocity sideways, and `θ_dot` is the angular velocity. For a standard non-holonomic differential drive robot, `vy` is typically constrained to be 0.
*   `q_dot`: The wheel angular velocity vector, `[φ_dot_left, φ_dot_right]^T`, where `φ_dot_left` and `φ_dot_right` are the angular velocities of the left and right wheels, respectively.
*   `r`: The radius of the wheels.
*   `l`: The separation distance between the two wheels.

The relationship is:

`v_e = J * q_dot`

Which expands to:

$$
\begin{bmatrix} v_x \\ v_y \\ \dot{\theta} \end{bmatrix} = \begin{bmatrix} \frac{r cos(\phi)}{2} & \frac{r cos(\phi)}{2} \\ 
    \frac{r sin(\phi)}{2} & \frac{r sin(\phi)}{2}  \\ 
    -\frac{r}{l} & \frac{r}{l} 
\end{bmatrix} 
\begin{bmatrix} \dot{\phi}_{left} \\ \dot{\phi}_{right} \end{bmatrix}
$$


The Jacobian matrix `J` for this system is:

$$
J = \begin{bmatrix} \frac{r cos(\phi)}{2} & \frac{r cos(\phi)}{2} \\ 
    \frac{r sin(\phi)}{2} & \frac{r sin(\phi)}{2}  \\ 
    -\frac{r}{l} & \frac{r}{l} 
\end{bmatrix} 
$$

This formulation is crucial for calculating the robot's movement from wheel encoder data (odometry) and for determining the required wheel velocities to achieve a desired chassis velocity (inverse kinematics).
