# Bumperbot Utils: My Collection of Helper Nodes

This package is a "toolbox" for our Bumperbot project. It contains various helper nodes that provide useful functionalities, from visualization to safety features.

## My Personal Notes on Utility Nodes

As a project grows, you'll often find you need small, single-purpose nodes that don't quite fit into other packages. A `_utils` package is the perfect home for them. It helps keep the other packages focused on their primary role.

## What's in the Toolbox?

### `trajectory_drawer.py`

*   **My Take**: This node is simple but very satisfying. It draws a "snail trail" behind the robot, showing the path it has taken.
*   **How it Works**: It listens to the robot's odometry topic (`/bumperbot_controller/odom`), collects the poses, and publishes them as a `Path` message. You can then visualize this path in RViz to see where your robot has been.

### `safety_stop.py`

*   **My Take**: This is a crucial safety feature. It acts as a lookout, stopping the robot before it crashes into something. I also fixed a typo in the original filename (`safetly_stop.py` -> `safety_stop.py`).
*   **How it Works**:
    1.  It subscribes to the `/scan` topic to get Lidar data.
    2.  It defines a "danger zone" and a "warning zone" around the robot.
    3.  If an obstacle enters the **danger zone**, it publishes `True` to the `/safety_stop` topic, which tells the `twist_relay` in our controller package to stop the robot.
    4.  If an obstacle enters the **warning zone**, it sends a goal to an action server to slow the robot down (reduce the "turbo" mode).
    5.  It also publishes visual markers for these zones, so you can see them in RViz.

## How to Use These Nodes

1.  **Build the package**:
    ```bash
    colcon build --packages-select bumperbot_utils
    ```

2.  **Run the nodes**:
    You can run these nodes while your main robot simulation is active.
    ```bash
    # To see the robot's trajectory
    ros2 run bumperbot_utils trajectory_drawer.py

    # To enable the safety stop feature
    ros2 run bumperbot_utils safety_stop.py
    ```

3.  **Visualize in RViz**:
    *   Open RViz (`rviz2`).
    *   Add a `Path` display and set the topic to `/bumperbot_controller/trajectory` to see the snail trail.
    *   Add a `MarkerArray` display and set the topic to `/zones` to see the safety zones around the robot.
