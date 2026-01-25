import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

  gazebo = IncludeLaunchDescription(
    os.path.join(
      get_package_share_directory("bumperbot_description"),
      "launch",
      "gazebo.launch.py"
    ),
  )

  controller = IncludeLaunchDescription(
    os.path.join(
      get_package_share_directory("bumperbot_controller"),
      "launch",
      "controller.launch.py"
      ),
    )

  joystick = IncludeLaunchDescription(
    os.path.join(
      get_package_share_directory("bumperbot_controller"),
      "launch",
      "joystick_teleop.launch.py"
    ),
  )

  rviz = IncludeLaunchDescription(
    os.path.join(
      get_package_share_directory("bumperbot_description"),
      "launch",
      "display.launch.py"
    ),
  )

  localization = IncludeLaunchDescription(
    os.path.join(
      get_package_share_directory("bumperbot_localization"),
      "launch",
      "global_localization.launch.py"
    ),
  )

  slam = IncludeLaunchDescription(
    os.path.join(
      get_package_share_directory("bumperbot_mapping"),
      "launch",
      "slam.launch.py"
    ),
  )

  return LaunchDescription([
    gazebo,
    controller,
    joystick,
    # localization,
    slam,
    rviz,
  ])