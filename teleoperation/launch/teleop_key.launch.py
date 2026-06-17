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
                    [FindPackageShare("teleoperation"), "config", "params.yaml"]
                ),
                description="Path to the teleop_key parameter file.",
            ),
            Node(
                package="teleoperation",
                executable="teleop_key",
                name="teleop_key",
                output="screen",
                emulate_tty=True,
                parameters=[params_file],
            ),
        ]
    )
