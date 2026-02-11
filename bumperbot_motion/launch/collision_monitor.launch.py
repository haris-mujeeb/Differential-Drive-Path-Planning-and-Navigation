from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='bumperbot_motion',
            executable='collision_monitor.py',
            name='collision_monitor_node',
            output='screen'
        ),
    ])
