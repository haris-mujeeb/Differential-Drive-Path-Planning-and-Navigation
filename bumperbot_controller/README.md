# Bumperbot Controller: A Beginner's Guide & Personal Notes

This package is the "brains" of the Bumperbot's movement. It contains all the ROS 2 nodes responsible for taking high-level commands (like "move forward") and translating them into low-level actions (like "spin the left wheel at 2 rad/s").

## My Personal Notes on Robot Control

At its core, controlling a robot is about managing its actuators (motors) to achieve a desired behavior. This package explores several key concepts:

*   **Hardware Abstraction with `ros2_control`**: We use `ros2_control` to create a standard interface for our robot's hardware. This is great because it separates our control logic from the specific hardware we're using. We could swap out our simulated motors for real ones with minimal changes to our controller code.
*   **Inverse Kinematics**: This is a fancy term for figuring out how to move the robot's joints to make the robot's body move in a certain way. For our differential drive robot, it means converting a desired linear and angular velocity into individual wheel speeds.
*   **Teleoperation**: This is the term for manually controlling a robot, in our case, with a joystick.

## Key Nodes and Concepts

### `simple_controller` (C++ and Python)

This is our main controller for moving the robot.

*   **What it does**: It implements inverse kinematics. It subscribes to the `/bumperbot_controller/cmd_vel` topic, which receives `Twist` messages (linear and angular velocity). It then calculates the required left and right wheel velocities to achieve that motion and publishes them to the `/simple_velocity_controller/commands` topic.
*   **How to use it**: You can send commands to the robot with a command like this:
    ```bash
    ros2 topic pub /bumperbot_controller/cmd_vel geometry_msgs/msg/TwistStamped "{twist: {linear: {x: 0.2}, angular: {z: 0.5}}}"
    ```
    This will make the robot move forward at 0.2 m/s and rotate at 0.5 rad/s.

### `noisy_controller` (C++ and Python)

This controller is essential for testing our localization algorithms.

*   **What it does**: Real-world sensors are never perfect. This node simulates that imperfection by taking the actual joint states, adding some random noise, and then calculating odometry from that noisy data.
*   **Why it's important**: It provides a realistic "odom_noisy" topic. We can then use this, along with other sensors like an IMU, to test how well our sensor fusion algorithms (like the EKF) can filter out noise and produce a more accurate position estimate.

### Joystick Teleoperation and `twist_mux`

Driving the robot with a joystick is fun, but what happens when another part of the system (like a navigation stack) also wants to control the robot? This is where `twist_mux` comes in.

*   **What is `twist_mux`?**: It's a "multiplexer" for `Twist` messages. Think of it as a smart traffic cop for robot velocity commands. It takes in commands from multiple sources (like a joystick and a navigation algorithm) and, based on a priority system, decides which command to send to the robot.
*   **Our Setup**:
    1.  `joy_node`: Reads the raw joystick data.
    2.  `joy_normalizer.py`: A custom node to make joystick data easier to work with.
    3.  `twist_mux`: Arbitrates between different velocity command topics.
    4.  `twist_relay.py`: A final safety check. It only passes the command from `twist_mux` to the robot if our `safetly_stop.py` node says it's safe to do so.

*   **To launch teleoperation**:
    ```bash
    ros2 launch bumperbot_controller joystick_teleop.launch.py
    ```

## Useful Commands

*   **Build the package**:
    ```bash
    colcon build --packages-select bumperbot_controller
    ```
*   **Launch the main controller**:
    ```bash
    # For C++ version
    ros2 launch bumperbot_controller controller.launch.py
    # For Python version
    ros2 launch bumperbot_controller controller.launch.py use_python:=true
    ```
*   **Inspect `ros2_control`**:
    ```bash
    # See what controllers are running
    ros2 control list_controllers
    # See what hardware interfaces are available
    ros2 control list_hardware_components
    ```

## Understanding the Math: Forward Kinematics

The README previously had a great explanation of forward kinematics. Here's my simplified take on it:

Forward kinematics for our robot answers the question: "If I know how fast my wheels are spinning, how fast is the robot itself moving forward and turning?"

The formula is:
`Robot Velocity = J * Wheel Velocities`

Where `J` is the Jacobian matrix. This math is what allows us to calculate the robot's odometry (its estimated position) from the wheel encoder readings. It's the opposite of the inverse kinematics used in our `simple_controller`.
