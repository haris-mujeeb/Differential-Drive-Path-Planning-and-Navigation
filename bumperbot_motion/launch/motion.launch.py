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

    collision_monitor_node = Node(
        package='bumperbot_motion',
        executable='collision_monitor.py',
        name='collision_monitor_node',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    pure_motion_planner_node = Node(
        package="bumperbot_motion",
        executable="pure_pursuit_motion_planner.py",
        name="pure_pursuit_motion_planner",
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    pd_motion_planner_node = Node(
        package="bumperbot_motion",
        executable="pd_motion_planner.py",
        name="pd_motion_planner",
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    dwb_motion_planner_node = Node(
        package="bumperbot_motion",
        executable="dwb_motion_planner.py",
        name="dwb_motion_planner",
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    return LaunchDescription([
        declare_use_sim_time_arg,
        collision_monitor_node,
        # pure_motion_planner_node,
        # pd_motion_planner_node,
        dwb_motion_planner_node,
    ])
