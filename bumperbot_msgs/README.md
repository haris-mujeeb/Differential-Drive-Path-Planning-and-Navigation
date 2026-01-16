# Bumperbot Messages

This package contains custom message and service definitions for the Bumperbot project.

## Purpose

Centralizing message definitions in a dedicated package is a standard ROS 2 practice. It allows other packages (like `bumperbot_py_examples` and `bumperbot_cpp_examples`) to depend on this package to use the custom types.

## Contents

### Services

*   **`srv/AddTwoInts.srv`**: A simple service that takes two 64-bit integers (`a` and `b`) and returns their sum (`sum`).

    ```
    int64 a
    int64 b
    ---
    int64 sum
    ```

## How to Build

This package must be built before any packages that depend on it.
```bash
colcon build --packages-select bumperbot_msgs
```
