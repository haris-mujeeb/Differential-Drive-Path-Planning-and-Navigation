# ROS2 Path Planning and Navigation
This repository presents a systematic study of **global path planning** and **local trajectory control** algorithms within the ROS2 Nav2 framework, with a focus on performance, optimality, and robustness.

---
## 1. Global Path Planners Comparison

This study evaluates four widely used global planners—**Dijkstra, A*, Theta*, and SMAC2D**—across 3,000 randomized navigation tasks. The objective is to analyze their performance in terms of execution time, path optimality, and robustness under consistent experimental conditions.

### Evaluation Methodology

A **paired-sample evaluation strategy** is employed to eliminate variability arising from task difficulty. Each planner is executed on identical start–goal pairs across all trials.

To ensure statistical reliability, experiments are repeated over **three independent runs (R1–R3)**, and aggregated metrics are reported. This approach enables a consistent comparison of planner behavior across varying environments.

### Quantitative Results

**Table 1. Aggregated performance across all runs**

| Algorithm | Success (%) | Mean Time (s) | Time RSD (%) | Mean Distance (m) |
| --------- | ----------- | ------------- | ------------ | ----------------- |
| Dijkstra  | 100.00      | 0.0223        | 38.54        | 13.26             |
| A*        | 100.00      | 0.0224        | 37.59        | 13.27             |
| Theta*    | 86.03       | 0.0186        | 34.07        | 12.61             |
| SMAC2D    | 90.33       | 0.0190        | 41.71        | 13.13             |

Dijkstra and A* achieve **perfect success rates (100%)**, confirming their completeness and robustness. However, both methods exhibit higher computation times and produce longer paths due to grid-constrained exploration.

Theta* demonstrates the **lowest mean computation time** and achieves the **shortest paths**, reflecting the advantage of any-angle path generation. However, its reduced success rate (~86%) indicates lower robustness in cluttered environments.

SMAC2D provides a **balanced performance profile**, maintaining a high success rate (~90%) while generating smooth, cost-aware trajectories with competitive runtime performance.

### Runtime Performance and Real-Time Feasibility

A real-time constraint of **50 ms** (corresponding to a 20 Hz control loop) is considered.

**Figure 2** presents the empirical cumulative distribution (ECDF) of planning times. All planners satisfy the real-time constraint in the majority of cases. Notably:

* **Theta*** exhibits the lowest tail latency, indicating strong worst-case performance
* **SMAC2D** achieves **>99% compliance**, making it suitable for time-critical applications

While absolute timings are hardware-dependent, the relative trends remain consistent across runs.

**Figure 1.** Qualitative comparison of generated trajectories across planners.
Theta* produces shorter, more direct paths, while SMAC2D generates smoother, cost-aware trajectories. Dijkstra paths are longer due to grid-constrained exploration.

<img width="861" height="685" alt="Trajectory comparison" src="https://github.com/user-attachments/assets/cde23379-6bd2-4ba3-a407-1feb2a90a1f6" />

**Figure 2.** ECDF and zoomed tail analysis of planning time.
The dashed line indicates the 50 ms real-time constraint. Tail behavior highlights latency variations across planners.

<img width="827" height="323" alt="ECDF analysis" src="https://github.com/user-attachments/assets/d00a9628-b67c-4d57-a0d6-d11b55ff18d6" />

**Figure 3.** Mean computation time across independent runs.
Theta* and SMAC2D consistently outperform Dijkstra and A* in terms of execution time.

<img width="608" height="368" alt="Computation time comparison" src="https://github.com/user-attachments/assets/b6594386-78d5-4b5d-a4f4-54497b72b69a" />

### Discussion

The results reveal a fundamental trade-off between **computational efficiency**, **path optimality**, and **robustness**:

* **Dijkstra and A*** prioritize reliability, achieving perfect success rates at the cost of higher computation time and longer paths
* **Theta*** improves both runtime and path optimality but sacrifices robustness in complex environments
* **SMAC2D** offers a balanced compromise, combining high success rates with smooth trajectories and real-time feasibility

### Summary of Findings

Statistical analysis (Friedman test and paired t-tests with Bonferroni correction) confirms that differences in both planning time and path optimality are **highly significant**.

Overall:

* **Theta*** achieves the best performance in terms of speed and path optimality
* **SMAC2D** provides the most practical solution for real-world deployment
* **Dijkstra and A*** remain suitable for scenarios requiring guaranteed success

These results emphasize that **planner selection should be guided by application-specific constraints**, rather than a single performance metric.

---

## 2. Local Trajectory Planners
I also tested three standard local trajectory controllers to evaluate their dynamic obstacle avoidance: Regulated Pure Pursuit (RPP), Dynamic Window Approach (DWB), and Model Predictive Path Integral (MPPI). I tested these algorithms across different obstacle densities and obstacle speeds up to 1.5 m/s.
*   **Baseline Performance:** In static or low-speed environments, all three controllers performed reliably and safely.
*   **Dynamic Environments:** When obstacle speed and density increased, performance dropped significantly for all standard planners.
*   **Regulated Pure Pursuit (RPP):** RPP maintained fast lap times but failed critically at high speeds (1.2 m/s). In these dense conditions, it reached a 100% collision rate.
*   **Dynamic Window Approach (DWB):** DWB suffered from inefficient decision-making and frequent replanning. This resulted in severe performance degradation and very high lap times (over 350 seconds) in dense environments.
*   **Model Predictive Path Integral (MPPI):** MPPI evaluates thousands of simulated trajectories to anticipate future system states. Despite this predictive capability, its collision rate still rose to nearly 100% at the highest speeds.


## A Beginner's Guide to this Workspace
If you're new to ROS 2 and robotics, this workspace is designed to help you understand the foundational concepts of building and controlling a robot. We'll cover:

*   **Odometry**: Figuring out how far the robot has traveled.
*   **Control**: Making the robot move the way we want it to.
*   **Localization**: Knowing where the robot is in its environment.
*   **Mapping**: Creating a map of the environment.

This workspace is structured into several ROS 2 packages. Each package is a building block of our robot's software.

## Packages Overview
Here's a breakdown of the packages in this workspace and what they do:
-   **`bumperbot_msgs`**: Defines custom message and service types. This is the foundation for communication between our nodes.
-   **`bumperbot_description`**: Contains the robot's 3D model (URDF) and the files needed to simulate it in Gazebo.
-   **`bumperbot_controller`**: Implements the robot's controllers, including joystick teleoperation and inverse kinematics.
-   **`bumperbot_py_examples` & `bumperbot_cpp_examples`**: These packages contain Python and C++ examples that demonstrate various ROS 2 concepts.
-   **`bumperbot_localization`**: Focuses on localization techniques, using tools like the Extended Kalman Filter (EKF).
-   **`bumperbot_mapping`**: Implements the algorithm to create a 2D map of the environment.
-   **`bumperbot_utils`**: Contains helpful utility scripts, like a safety stop node and a trajectory visualizer.
-   **`bumperbot_bringup`**: Provides the main launch file to start the entire robot simulation.

Each package has its own `README.md` with more detailed notes. I highly recommend reading them to understand the concepts behind each part of the project.

## Troubleshooting Common Errors

Here are some issues I've encountered and how I solved them.

### Error: `Failed to find a free participant index for domain 0`

This is a common error in ROS 2 that happens when the DDS (Data Distribution Service), the middleware ROS 2 uses for communication, gets into a bad state.

**Symptoms:**
You might see errors like:
```
ros2: Failed to find a free participant index for domain 0
[ERROR] [rmw_cyclonedds_cpp]: rmw_create_node: failed to create domain, error Error
```
And you won't be able to list topics, nodes, or run new ROS 2 programs.

**Solution:**

The most reliable solution is to find and stop any lingering ROS 2 processes.

1.  **Stop the ROS 2 Daemon:**
    ```bash
    ros2 daemon stop
    ```

2.  **Find and Kill Lingering ROS Processes:**
    This command will find any process with "ros" in its name and terminate it.
    ```bash
    kill -9 $(ps aux | grep '[r]os' | awk '{print $2}')
    ```

After running these commands, you should be able to run `ros2 topic list` and other ROS 2 commands successfully.

## Special Thanks:
"Self-Driving and ROS 2 - Learn by Doing! Plan & Navigation" tutorials by [AntoBrandi](https://github.com/AntoBrandi)


