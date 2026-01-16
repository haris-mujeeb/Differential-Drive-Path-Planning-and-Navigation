# Bumperbot C++ Examples

This package contains C++ examples for the Bumperbot ROS 2 project. These examples are developed as part of the "Self-Driving and ROS 2 - Learn by Doing! Odometry & Control" tutorial and demonstrate fundamental ROS 2 concepts using C++.

## Concepts Covered (Notes)

*   **Creating a Service Server**: The `simple_service_server.cpp` example shows how to create a ROS 2 service server. It uses a custom service type (`bumperbot_msgs/srv/AddTwoInts`) to take two integers and return their sum. This is a basic example of request/response communication in ROS 2.
*   **Broadcasting TF Transforms**: The `simple_tf_kinematics.cpp` example demonstrates how to broadcast both static and dynamic transforms to the `/tf` and `/tf_static` topics.
    *   **Static Transforms**: It uses a `tf2_ros::StaticTransformBroadcaster` to publish a fixed transform between `bumperbot_base` and `bumperbot_top`. This is efficient for transforms that never change, as they are latched and published only once.
    *   **Dynamic Transforms**: It uses a `tf2_ros::TransformBroadcaster` and a `rclcpp::Timer` to publish a moving transform between `odom` and `bumperbot_base`. This simulates the robot moving forward along the x-axis and is the standard way to publish frequently changing coordinate frame relationships.
*   **Using Timers**: The TF example also shows how to use a `create_wall_timer` to call a function at a fixed rate, which is a common pattern for periodic tasks in ROS 2.
*   **Custom Service Messages**: The service example relies on the `bumperbot_msgs` package, demonstrating how different packages in a workspace depend on each other for custom message and service definitions.

## Examples

### 1. `simple_service_server`

*   **Source**: `src/simple_service_server.cpp`
*   **Description**: A node that provides an `add_two_ints` service.
*   **To Run**:
    ```bash
    ros2 run bumperbot_cpp_examples simple_service_server
    ```
*   **To Call the Service**:
    Open another terminal and run:
    ```bash
    ros2 service call /add_two_ints bumperbot_msgs/srv/AddTwoInts "{a: 5, b: 10}"
    ```

### 2. `simple_tf_kinematic`

*   **Source**: `src/simple_tf_kinematics.cpp`
*   **Description**: A node that continuously publishes static and dynamic TF transforms. The dynamic transform simulates the robot moving.
*   **To Run**:
    ```bash
    ros2 run bumperbot_cpp_examples simple_tf_kinematic
    ```
*   **To Inspect the Transforms**:
    *   You can view the full TF tree with:
        ```bash
        ros2 run tf2_tools view_frames
        ```
    *   Or listen to a specific transform:
        ```bash
        ros2 run tf2_ros tf2_echo odom bumperbot_base
        ```

## How to Build

To build this package individually:
```bash
colcon build --packages-select bumperbot_cpp_examples
```
