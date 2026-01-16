# Bumperbot Python Examples

This package contains Python examples for the Bumperbot ROS 2 project. These examples are developed as part of the "Self-Driving and ROS 2 - Learn by Doing! Odometry & Control" tutorial and demonstrate fundamental ROS 2 concepts using Python.

## Concepts Covered (Notes)

*   **Creating a Service Server**: `simple_service_server.py` shows how to create a ROS 2 service using a custom service type (`bumperbot_msgs/srv/AddTwoInts`).
*   **Broadcasting TF Transforms**: `simple_tf_kinematics.py` demonstrates broadcasting both static and dynamic transforms to `/tf` and `/tf_static`, simulating robot movement.

## Examples

### 1. `simple_service_server.py`

*   **Description**: A node that provides an `add_two_ints` service.
*   **To Run**: `ros2 run bumperbot_py_examples simple_service_server`
*   **To Call**: `ros2 service call /add_two_ints bumperbot_msgs/srv/AddTwoInts "{a: 5, b: 10}"`

### 2. `simple_tf_kinematics.py`

*   **Description**: Publishes static and dynamic TF transforms, simulating robot motion.
*   **To Run**: `ros2 run bumperbot_py_examples simple_tf_kinematics`
*   **To Inspect**: `ros2 run tf2_ros tf2_echo odom bumperbot_base`

## How to Build

To build this package:
```bash
colcon build --packages-select bumperbot_py_examples
```
