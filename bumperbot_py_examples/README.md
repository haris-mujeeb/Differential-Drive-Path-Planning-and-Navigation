# Bumperbot Python Examples: My Learning Notes

This package is my Python playground for learning core ROS 2 concepts. It follows the "Self-Driving and ROS 2 - Learn by Doing! Odometry & Control" tutorial, and I've added my own notes here to help other beginners get started with ROS 2 in Python.

## Concepts I've Learned

### `simple_service_server.py`
*   **What it is**: A service is a request/response communication pattern. This example creates a simple `add_two_ints` service.
*   **My Notes**: This is the "Hello, World!" of ROS 2 services. It shows how to create a service, define a callback function to handle requests, and use custom service types from `bumperbot_msgs`.

### `simple_tf_kinematics.py`
*   **What it is**: This script demonstrates how to publish coordinate frame transformations (TF).
*   **My Notes**: TF is fundamental. This example was great for understanding the difference between:
    *   **Static Transforms**: For parts that don't move relative to each other. Published once, very efficient.
    *   **Dynamic Transforms**: For moving parts, like the robot's base moving in the world. Published continuously using a timer.

### QoS (Quality of Service) Examples
*   **What it is**: QoS settings give you fine-grained control over how messages are sent and received.
*   **My Notes**: This was a deep dive! The `simple_qos_publisher.py` and `simple_qos_subscriber.py` examples show how to play with two important settings:
    *   **Reliability**: `reliable` (guaranteed delivery) vs. `best_effort` (faster, but might drop messages).
    *   **Durability**: `volatile` (subscribers only get messages sent after they subscribe) vs. `transient_local` (new subscribers get the last few messages that were published).

### Lifecycle Nodes (`simple_lifecycle_node.py`)
*   **What it is**: Lifecycle nodes provide a state machine for managing your nodes. Instead of just "running," a node can be "unconfigured," "inactive," "active," etc.
*   **My Notes**: This is a more advanced, but very powerful, concept. It gives you explicit control over the startup and shutdown sequence of your nodes, which is crucial for complex robots. You can make sure a node has successfully loaded its configuration before it starts processing data.

## How to Run the Examples

### 1. Simple Service
```bash
# Run the server
ros2 run bumperbot_py_examples simple_service_server
# Call the service from another terminal
ros2 service call /add_two_ints bumperbot_msgs/srv/AddTwoInts "{a: 5, b: 10}"
```

### 2. Simple TF Kinematics
```bash
# Run the TF publisher
ros2 run bumperbot_py_examples simple_tf_kinematics
# Echo the transform in another terminal
ros2 run tf2_ros tf2_echo odom bumperbot_base
```

### 3. QoS Publisher/Subscriber
You can run these with different command-line arguments to see how QoS affects communication.
```bash
# Example: A reliable publisher and a best_effort subscriber (they won't communicate!)
ros2 run bumperbot_py_examples simple_qos_publisher --ros-args -p reliability:=reliable
ros2 run bumperbot_py_examples simple_qos_subscriber --ros-args -p reliability:=best_effort
```
Use `ros2 topic info /chatter --verbose` to inspect the QoS settings.

### 4. Lifecycle Node
```bash
# Run the node
ros2 run bumperbot_py_examples simple_lifecycle_node
# In another terminal, transition it through its states
ros2 lifecycle set /simple_lifecycle_node configure
ros2 lifecycle set /simple_lifecycle_node activate
```
You'll see the node's output change as it enters the `active` state.

## How to Build

To build just this package, run:
```bash
colcon build --packages-select bumperbot_py_examples
```
