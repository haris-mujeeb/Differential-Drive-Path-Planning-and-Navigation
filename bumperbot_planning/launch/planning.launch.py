from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    
    use_sim_time = LaunchConfiguration("use_sim_time")

    declare_use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )

    global_planner_node = Node(
        package='bumperbot_planning',
        executable='global_planner.py',
        name='global_planner',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    navigation_manager_node = Node(
        package="bumperbot_planning",
        executable="navigation_manager.py",
        name="navigation_manager",
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    return LaunchDescription([
        declare_use_sim_time_arg,
        global_planner_node,
        navigation_manager_node,
    ])
