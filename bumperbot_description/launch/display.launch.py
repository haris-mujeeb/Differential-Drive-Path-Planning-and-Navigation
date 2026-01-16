from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
  
  model_arg = DeclareLaunchArgument(
    name='model',
    default_value=os.path.join(get_package_share_directory("bumperbot_description"), "urdf", "bumperbot.urdf.xacro"),
    description='Absolute path to robot urdf file'
  )

  robot_description = ParameterValue(Command(['xacro ', LaunchConfiguration('model')]), value_type=str)

  robot_state_publisher = Node(
    package='robot_state_publisher',
    executable='robot_state_publisher',
    name='robot_state_publisher',
    output='screen',
    parameters=[{'robot_description': robot_description}]
  )

  joint_state_publisher = Node(
    package='joint_state_publisher',
    executable='joint_state_publisher',
  )

  rviz_node = Node(
    package='rviz2',
    executable='rviz2',
    name='rviz2',
    output='screen',
    arguments=['-d', os.path.join(get_package_share_directory('bumperbot_description'), 'rviz/bumperbot.rviz')]
  )

  return LaunchDescription(
    [
      model_arg,
      robot_state_publisher,
      # joint_state_publisher, // Dont use it when running bumperbot_gazebo.py
      rviz_node
    ]
  )