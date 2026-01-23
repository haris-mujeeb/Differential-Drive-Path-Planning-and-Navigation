# Bumperbot Mapping: A Beginner's Guide & Personal Notes

This package is responsible for creating a 2D map of the robot's environment. This process is a cornerstone of autonomous navigation, as a robot needs a map to be able to localize itself and plan paths.

## Key Concepts & My Personal Notes

### Occupancy Grid Mapping

*   **What it is**: We represent the world as a grid. Each cell in the grid holds a value that represents our "belief" about whether that cell is occupied by an obstacle or not.
    *   A value of `100` means we are very confident the cell is **occupied**.
    *   A value of `0` means we are very confident the cell is **free**.
    *   A value of `-1` means the cell is **unknown**.
    *   Values in between represent our level of certainty.

### Log-Odds Representation

*   **Problem**: If we store probabilities directly (e.g., 0.9 for occupied), updating our belief is computationally expensive and can lead to rounding errors. For example, getting to 100% or 0% certainty is difficult and numerically unstable.
*   **Solution**: We use the **log-odds** representation internally.
    *   `log(p / (1 - p))` where `p` is the probability.
    *   This converts the [0, 1] probability range to a (-inf, +inf) log-odds range.
    *   **Why?** Updating our belief becomes a simple addition! This is much faster and more stable. We only convert back to the 0-100 probability scale when we need to publish the map for visualization.

### The Inverse Sensor Model (`inverseSensorModel`)

*   This is the core logic for updating the map. For a given laser scan beam that hits an obstacle, we need to update the grid.
*   The function `inverseSensorModel` does two things:
    1.  It traces a line (using **Bresenham's line algorithm**) from the robot's current position to the point where the laser beam detected an obstacle.
    2.  All the cells along this line, *except for the last one*, are marked as **free**.
    3.  The final cell in the line, where the beam hit, is marked as **occupied**.

### The Importance of Coordinate Transforms (TF)

*   The laser scanner publishes data in its own `laser_link` frame.
*   To update the map, we need to know where that laser data is relative to the `map` frame.
*   The `tf` library allows us to do this lookup. We ask `tf`: "What is the transform from the `laser_link` to the `map` frame at this specific time?"
*   Without accurate `tf` data, the map would be a smeared, useless mess, because the robot wouldn't know how to place the laser scans correctly as it moves.

## Package Structure and Nodes

### `mapping_with_known_poses.py`

*   **Description**: This is the main node in the package. It implements the Occupancy Grid Mapping algorithm described above.
*   **"With Known Poses"**: This is a key simplification. This script assumes that the robot's pose (its position in the world) is already known and accurate. In a real-world scenario, you would run mapping and localization (like AMCL) simultaneously (this is called SLAM). Here, we rely on the `/odom` frame to provide our "known" poses.
*   **How it works**:
    1.  Subscribes to `/scan` (for laser data) and `/tf` (for poses).
    2.  For each scan, it uses the `inverseSensorModel` to calculate which cells to update.
    3.  It updates the internal log-odds probability map.
    4.  Periodically, it converts the log-odds map to a standard `OccupancyGrid` message and publishes it on the `/map` topic.

### `maps` directory
This directory stores the saved maps. A map consists of two files:
*   `.pgm`: An image file representing the map's occupancy grid.
*   `.yaml`: A metadata file containing the map's resolution, origin, and other parameters.

## How to Use

### 1. Build the Package

```bash
colcon build --packages-select bumperbot_mapping
```

### 2. Run the Mapping Node

Make sure your robot is running and publishing laser scans and odometry transforms.

```bash
ros2 launch bumperbot_description gazebo.launch.py
ros2 run bumperbot_mapping mapping_with_known_poses.py
```

### 3. Visualize the Map

1.  Open RViz: `rviz2`
2.  Add a `Map` display.
3.  Set the topic to `/map`.
4.  As you drive the robot around, you will see the map being built in real-time.

### 4. Save the Map

Once you are satisfied with your map, you can save it using the `map_saver` tool from the `nav2_map_server` package.

```bash
ros2 run nav2_map_server map_saver_cli -f ~/map
```
This will create `map.pgm` and `map.yaml` in your home directory. You can then move these into your `bumperbot_mapping/maps` directory for later use with the navigation stack.


---
*This README is intended as a personal learning log and a guide for beginners. The concepts are simplified for clarity.*