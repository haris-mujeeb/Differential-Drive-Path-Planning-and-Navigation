import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
  use_slam = LaunchConfiguration("use_slam")
  use_sim_time = LaunchConfiguration("use_sim_time")

  use_slam_arg = DeclareLaunchArgument(
    "use_slam",
    default_value="false"
  )

  declare_use_sim_time_arg = DeclareLaunchArgument(
    'use_sim_time',
    default_value='true',
    description='Use simulation (Gazebo) clock if true'
  )

  a_star_planner_node = Node(
        package='bumperbot_planning',
        executable='a_star_planner_optimized.py',
        name='a_star_node',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
  )

  dwb_motion_planner_node = Node(
        package='bumperbot_motion',
        executable='dwb_motion_planner.py',
        name='dwb_motion_planner_node',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
  )

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
    condition=UnlessCondition(use_slam)
  )

  slam = IncludeLaunchDescription(
    os.path.join(
      get_package_share_directory("bumperbot_mapping"),
      "launch",
      "slam.launch.py"
    ),
    condition=IfCondition(use_slam)
  )

  navigation = IncludeLaunchDescription(
    os.path.join(
      get_package_share_directory("bumperbot_navigation"),
      "launch",
      "navigation.launch.py"
    )
  )

  motion = IncludeLaunchDescription(
    os.path.join(
      get_package_share_directory("bumperbot_motion"),
      "launch",
      "motion.launch.py"
    )
  )

  return LaunchDescription([
    use_slam_arg,
    declare_use_sim_time_arg,
    gazebo,
    rviz,
    controller,
    joystick,
    localization,
    slam,
    navigation,
    a_star_planner_node,
    motion,
  ])