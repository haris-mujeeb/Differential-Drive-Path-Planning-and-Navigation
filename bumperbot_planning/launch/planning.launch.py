from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

####### REMOVE THIS AFTER TESTING
import os
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory

######

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

##### REMOVE THIS AFTER TESTING ####
    motion = IncludeLaunchDescription(
        os.path.join(
        get_package_share_directory("bumperbot_motion"),
        "launch",
        "motion.launch.py"
        )
    )

    return LaunchDescription([
        declare_use_sim_time_arg,
        global_planner_node,
        navigation_manager_node,
        motion # REMOVE THIS AFTER TESTING
    ])
