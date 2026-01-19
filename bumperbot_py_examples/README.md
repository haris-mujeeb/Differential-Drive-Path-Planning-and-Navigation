# Bumperbot Python Examples

This package contains Python examples for the Bumperbot ROS 2 project. These examples are developed as part of the "Self-Driving and ROS 2 - Learn by Doing! Odometry & Control" tutorial and demonstrate fundamental ROS 2 concepts using Python.

## Concepts Covered (Notes)

*   **Creating a Service Server**: `simple_service_server.py` shows how to create a ROS 2 service using a custom service type (`bumperbot_msgs/srv/AddTwoInts`).
*   **Broadcasting TF Transforms**: `simple_tf_kinematics.py` demonstrates broadcasting both static and dynamic transforms to `/tf` and `/tf_static`, simulating robot movement.
*   **ROS 2 Quality of Service (QoS)**: `simple_qos_publisher.py` and `simple_qos_subscriber.py` demonstrate how to configure different QoS policies (Reliability and Durability) for ROS 2 publishers and subscribers.
*   **ROS 2 Lifecycle Management**: `simple_lifecycle_node.py` shows how to create and manage a lifecycle node, which allows for explicit control over a node's state transitions (e.g., configuring, activating, deactivating).

## Examples

### 1. `simple_service_server.py`

*   **Description**: A node that provides an `add_two_ints` service.
*   **To Run**: `ros2 run bumperbot_py_examples simple_service_server`
*   **To Call**: `ros2 service call /add_two_ints bumperbot_msgs/srv/AddTwoInts "{a: 5, b: 10}"`

### 2. `simple_tf_kinematics.py`

*   **Description**: Publishes static and dynamic TF transforms, simulating robot motion.
*   **To Run**: `ros2 run bumperbot_py_examples simple_tf_kinematics`
*   **To Inspect**: `ros2 run tf2_ros tf2_echo odom bumperbot_base`

### 3. `simple_qos_publisher.py`

*   **Description**: A node that publishes String messages on the `/chatter` topic with configurable QoS settings.
*   **To Run**:
    *   Default (system_default reliability and durability):
        `ros2 run bumperbot_py_examples simple_qos_publisher`
    *   With custom reliability (e.g., best_effort):
        `ros2 run bumperbot_py_examples simple_qos_publisher --ros-args -p reliability:=best_effort`
    *   With custom durability (e.g., transient_local):
        `ros2 run bumperbot_py_examples simple_qos_publisher --ros-args -p durability:=transient_local`
    *   With both custom reliability and durability:
        `ros2 run bumperbot_py_examples simple_qos_publisher --ros-args -p reliability:=reliable -p durability:=transient_local`

### 4. `simple_qos_subscriber.py`

*   **Description**: A node that subscribes to String messages on the `/chatter` topic with configurable QoS settings.
*   **To Run**:
    *   Default (system_default reliability and durability):
        `ros2 run bumperbot_py_examples simple_qos_subscriber`
    *   With custom reliability (e.g., best_effort):
        `ros2 run bumperbot_py_examples simple_qos_subscriber --ros-args -p reliability:=best_effort`
    *   With custom durability (e.g., transient_local):
        `ros2 run bumperbot_py_examples simple_qos_subscriber --ros-args -p durability:=transient_local`
    *   With both custom reliability and durability:
        `ros2 run bumperbot_py_examples simple_qos_subscriber --ros-args -p reliability:=reliable -p durability:=transient_local`

### 5. `simple_lifecycle_node.py`

*   **Description**: A node that demonstrates the ROS 2 lifecycle management. It subscribes to the `/chatter` topic and will only process messages when it is in the `active` state.
*   **To Run**: `ros2 run bumperbot_py_examples simple_lifecycle_node`
*   **To interact with the lifecycle node**:
    *   To see the current state: `ros2 lifecycle get /simple_lifecycle_node`
    *   To configure the node: `ros2 lifecycle set /simple_lifecycle_node configure`
    *   To activate the node: `ros2 lifecycle set /simple_lifecycle_node activate`
    *   To deactivate the node: `ros2 lifecycle set /simple_lifecycle_node deactivate`
    *   To cleanup the node: `ros2 lifecycle set /simple_lifecycle_node cleanup`
    *   To shutdown the node: `ros2 lifecycle set /simple_lifecycle_node shutdown`

### Useful ROS 2 Commands for QoS Examples

*   **To inspect the `/chatter` topic information (including QoS profiles):**
    `ros2 topic info /chatter --verbose`
*   **To see the messages being published on `/chatter`:**
    `ros2 topic echo /chatter`
*   **To list all available topics:**
    `ros2 topic list`
*   **To get information about the `simple_qos_publisher` node:**
    `ros2 node info /simple_qos_publisher`
*   **To get information about the `simple_qos_subscriber` node:**
    `ros2 node info /simple_qos_subscriber`

## How to Build

To build this package:
```bash
colcon build --packages-select bumperbot_py_examples
```
