# Bumperbot Messages: A Beginner's Guide

This package holds the custom "Data Sheets" for our project. In ROS 2, we pass information between nodes using messages. While ROS 2 provides a lot of standard message types, sometimes you need to define your own. That's what this package is for.

## My Personal Notes on Custom Messages

I learned that it's a very good practice to keep all your custom message (`.msg`), service (`.srv`), and action (`.action`) definitions in a separate package like this one.

**Why?**

It creates a clear dependency structure. Other packages (like our C++ and Python examples) can simply depend on `bumperbot_msgs` to use these custom data types. It prevents circular dependencies and keeps the workspace organized.

## What's Inside?

### Services

*   **`srv/AddTwoInts.srv`**: This is a simple service definition for a service that adds two numbers together. It's a great "hello world" for ROS 2 services.

    The format is `Request --- Response`.

    ```
    # Request
    int64 a
    int64 b
    ---
    # Response
    int64 sum
    ```

## How to Build

Because other packages depend on the definitions in this one, you should always compile it first.

```bash
colcon build --packages-select bumperbot_msgs
```

After building, any Python or C++ node in your workspace can now `import` or `#include` these custom types.
