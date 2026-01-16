# Bumperbot Utils

This package contains utility nodes for the Bumperbot project.

## Nodes

### `trajectory_drawer.py`

*   **Purpose**: This node visualizes the robot's path by converting odometry data into a `nav_msgs/msg/Path` message, which can be displayed in Rviz.
*   **Subscriptions**:
    *   `/bumperbot_controller/odom` (`nav_msgs/msg/Odometry`): The robot's odometry, used to get the poses for the path.
*   **Publications**:
    *   `/bumperbot_controller/trajectory` (`nav_msgs/msg/Path`): The accumulated path of the robot.

## How to Use

1.  **Run the Node**:
    ```bash
    ros2 run bumperbot_utils trajectory_drawer.py
    ```

2.  **Visualize in Rviz**:
    *   Open Rviz: `rviz2`
    *   Click "Add" -> "By topic" -> and select the `/bumperbot_controller/trajectory` topic (or add a `Path` display and set the topic manually).
    *   As the robot moves, you will see a line representing its trajectory.

## How to Build
```bash
colcon build --packages-select bumperbot_utils
```
