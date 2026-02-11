import os
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

  bumperbot_controller_pkg = get_package_share_directory("bumperbot_controller")

  use_python_arg = DeclareLaunchArgument(
    "use_python",
    default_value="False",
  )
  
  use_python = LaunchConfiguration("use_python")


  joy_node = Node(
    package="joy",
    executable="joy_node",
    name="joystick",
    parameters=[os.path.join(bumperbot_controller_pkg, "config", "joy_config.yaml")],
    remappings=[('/joy', '/joy_raw')]
  )

  joy_teleop = Node(
    package="joy_teleop",
    executable="joy_teleop",
    parameters=[os.path.join(bumperbot_controller_pkg, "config", "joy_teleop.yaml")]
  )

  twist_mux_launch = IncludeLaunchDescription(
    os.path.join(
      get_package_share_directory("twist_mux"),
      "launch",
      "twist_mux_launch.py"
    ),
    launch_arguments={
      "cmd_vel_out": "bumperbot_controller/cmd_vel_unstamped",
      "config_locks": os.path.join(bumperbot_controller_pkg, "config", "twist_mux_locks.yaml"),
      "config_topics": os.path.join(bumperbot_controller_pkg, "config", "twist_mux_topics.yaml"),
      "config_joy": os.path.join(bumperbot_controller_pkg, "config", "twist_mux_joy.yaml"),
    }.items(),
  )

  twist_relay_node_py = Node(
    package="bumperbot_controller",
    executable="twist_relay.py",
    name="twist_relay", 
    condition=IfCondition(use_python),
  )

  twist_relay_node = Node(
    package="bumperbot_controller",
    executable="twist_relay",
    name="twist_relay", 
    condition=UnlessCondition(use_python),
  )

  joy_normalizer_node = Node(
    package="bumperbot_controller",
    executable="joy_normalizer.py",
    name="joy_normalizer",
  )

  return LaunchDescription([
    use_python_arg,
    joy_teleop,
    joy_node,
    joy_normalizer_node,
    twist_mux_launch,
    twist_relay_node_py,
    twist_relay_node,
  ])