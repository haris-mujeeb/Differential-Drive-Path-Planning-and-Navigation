# Bumperbot Mapping

This package contains nodes related to mapping for the Bumperbot.

## `mapping_with_known_poses.py`

*   **Description**: This script implements a Binary Bayes Filter to create a map of the environment, assuming the robot's pose is known. It subscribes to laser scan topics and uses the occupancy grid mapping algorithm. This was initially added in commit `9a48b1e`.
*   **To Run**: `ros2 run bumperbot_mapping mapping_with_known_poses`
