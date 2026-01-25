# Bumperbot C++ Examples: My Learning Notes

This package is my C++ playground for learning core ROS 2 concepts. It follows the "Self-Driving and ROS 2 - Learn by Doing! Odometry & Control" tutorial, and I've added my own notes here to help other beginners.

## Concepts I've Learned

### Creating a Service Server (`simple_service_server.cpp`)

*   **What it is**: A service is a request/response form of communication in ROS 2. A "server" node provides a service, and a "client" node can call it to get a result.
*   **My Notes**: This example is a perfect introduction to services. We create a simple `add_two_ints` service that takes two numbers and returns their sum. It shows how to define a service callback function and spin up a node to wait for requests. It also demonstrates the need for custom service definitions, which live in the `bumperbot_msgs` package.

### Broadcasting TF Transforms (`simple_tf_kinematics.cpp`)

*   **What it is**: TF (Transform) is one of the most powerful and fundamental concepts in ROS. It allows us to manage the relationships between different coordinate frames (like the robot's base, its wheels, and the world).
*   **My Notes**: This example was a real eye-opener. It shows two ways to publish transforms:
    *   **Static Transforms**: For things that don't move relative to each other (like a sensor mounted on the robot's body). The `StaticTransformBroadcaster` is super efficient because it just publishes the transform once.
    *   **Dynamic Transforms**: For things that move (like the robot moving in the world). The `TransformBroadcaster` is used inside a timer callback to continuously publish the robot's changing position. This is how we can visualize the robot moving in RViz.

## How to Run the Examples

### 1. `simple_service_server`

This node provides a service to add two integers.

*   **To Run the Server**:
    ```bash
    ros2 run bumperbot_cpp_examples simple_service_server
    ```
*   **To Call the Service from another terminal**:
    ```bash
    ros2 service call /add_two_ints bumperbot_msgs/srv/AddTwoInts "{a: 5, b: 10}"
    ```
    You should see the server log that it received the request and the client will print the result.

### 2. `simple_tf_kinematics`

This node simulates the robot moving by publishing its transform relative to the "odom" frame.

*   **To Run the Node**:
    ```bash
    ros2 run bumperbot_cpp_examples simple_tf_kinematics
    ```
*   **To See the Transforms**:
    *   **Visualize in RViz**: Open RViz and set the "Fixed Frame" to "odom". You should see the `bumperbot_base` frame moving.
    *   **Use `tf2_echo`**: This command shows you the transformation between two frames in real-time.
        ```bash
        ros2 run tf2_ros tf2_echo odom bumperbot_base
        ```

## How to Build

To build just this package, run:
```bash
colcon build --packages-select bumperbot_cpp_examples
```
