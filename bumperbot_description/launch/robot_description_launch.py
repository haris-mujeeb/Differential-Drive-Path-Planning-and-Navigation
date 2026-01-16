from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare arguments
    declared_arguments = [
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="Use simulation (Gazebo) clock if true",
        ),
    ]

    # Get paths to other packages
    bumperbot_description_pkg_share = FindPackageShare("bumperbot_description").find("bumperbot_description")

    # Robot Description
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [bumperbot_description_pkg_share, "urdf", "bumperbot.urdf.xacro"]
            ),
            " ",
            "use_sim_time:=",
            LaunchConfiguration("use_sim_time"),
            " ",
            PathJoinSubstitution(
                [bumperbot_description_pkg_share, "urdf", "bumperbot_ros2_control.xacro"]
            ),
        ]
    )

    return LaunchDescription(
        declared_arguments
        + [
            DeclareLaunchArgument(
                "robot_description_content",
                default_value=robot_description_content,
                description="XML content of the robot description",
            ),
        ]
    )
