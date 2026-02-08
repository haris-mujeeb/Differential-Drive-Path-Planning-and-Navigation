import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, LifecycleNode

def generate_launch_description():
    
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    lifecycle_nodes = ["nav2_costmap_2d"]
    params_file = os.path.join(
        get_package_share_directory('bumperbot_navigation'),
        'config',
        'costmap.yaml'
    )
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'
        ),
        
        LifecycleNode(
            package='nav2_costmap_2d',
            executable='nav2_costmap_2d',
            name='costmap',
            namespace='',
            output='screen',
            parameters=[params_file, {'use_sim_time': use_sim_time}]
        ),
        
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_costmap',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time},
                        {'autostart': True},
                        {'node_names': ['costmap']}]
        )
    ])
