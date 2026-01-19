# Bumperbot Utils

This package contains utility nodes for the Bumperbot project.

## Nodes

### `trajectory_drawer.py`

*   **Purpose**: This node visualizes the robot's path by converting odometry data into a `nav_msgs/msg/Path` message, which can be displayed in Rviz.
*   **Subscriptions**:
    *   `/bumperbot_controller/odom` (`nav_msgs/msg/Odometry`): The robot's odometry, used to get the poses for the path.
*   **Publications**:
    *   `/bumperbot_controller/trajectory` (`nav_msgs/msg/Path`): The accumulated path of the robot.

### `safetly_stop.py`

*   **Purpose**: This node acts as a safety monitor. It subscribes to a laser scan topic and, based on the distance to obstacles, it will publish a boolean to a "safety_stop" topic and also send goals to "joy_turbo" action servers to increase or decrease speed. It also visualizes the warning and danger zones as markers in Rviz.
*   **Subscriptions**:
    *   `/scan` (`sensor_msgs/msg/LaserScan`): The laser scan data.
*   **Publications**:
    *   `/safety_stop` (`std_msgs/msg/Bool`): Publishes `True` when an obstacle is within the `danger_distance`.
    *   `/zones` (`visualization_msgs/msg/MarkerArray`): Publishes markers for visualizing the warning and danger zones in Rviz.
*   **Actions**:
    *   `/joy_turbo_decrease` (`twist_mux_msgs/action/JoyTurbo`): Sends a goal to decrease the robot's speed when an obstacle is in the warning zone.
    *   `/joy_turbo_increase` (`twist_mux_msgs/action/JoyTurbo`): Sends a goal to increase the robot's speed when the robot is in the free zone.
*   **Parameters**:
    *   `warning_distance`: The distance in meters to trigger a warning and decrease speed. Default is `0.6`.
    *   `danger_distance`: The distance in meters to trigger a safety stop. Default is `0.2`.
    *   `scan_topic`: The name of the laser scan topic to subscribe to. Default is `scan`.
    *   `safety_stop_topic`: The name of the topic to publish the safety stop boolean. Default is `safety_stop`.

## How to Use

1.  **Run the Nodes**:
    ```bash
    # To run the trajectory drawer
    ros2 run bumperbot_utils trajectory_drawer.py

    # To run the safety stop node
    ros2 run bumperbot_utils safetly_stop.py
    ```

2.  **Visualize in Rviz**:
    *   Open Rviz: `rviz2`
    *   Click "Add" -> "By topic" -> and select the `/bumperbot_controller/trajectory` topic (or add a `Path` display and set the topic manually).
    *   As the robot moves, you will see a line representing its trajectory.
    *   Click "Add" -> "By topic" -> and select the `/zones` topic (or add a `MarkerArray` display and set the topic manually).

## How to Build
```bash
colcon build --packages-select bumperbot_utils
```
