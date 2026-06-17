from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    params_file = LaunchConfiguration("params_file")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("roboclaw_ros2"), "config", "params.yaml"]
                ),
                description="Path to the roboclaw_node parameter file.",
            ),
            Node(
                package="roboclaw_ros2",
                executable="roboclaw_node",
                name="roboclaw_node",
                output="screen",
                parameters=[params_file],
            ),
        ]
    )
