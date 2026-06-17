from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    front_namespace = LaunchConfiguration("front_namespace")
    rear_namespace = LaunchConfiguration("rear_namespace")
    front_port = LaunchConfiguration("front_port")
    rear_port = LaunchConfiguration("rear_port")
    baud = LaunchConfiguration("baud")
    address = LaunchConfiguration("address")
    poll_rate_hz = LaunchConfiguration("poll_rate_hz")
    control_rate_hz = LaunchConfiguration("control_rate_hz")
    max_speed = LaunchConfiguration("max_speed")
    ticks_per_revolution = LaunchConfiguration("ticks_per_revolution")
    wheel_radius = LaunchConfiguration("wheel_radius")
    base_width = LaunchConfiguration("base_width")
    cmd_vel_timeout = LaunchConfiguration("cmd_vel_timeout")
    reset_encoders_on_connect = LaunchConfiguration("reset_encoders_on_connect")
    cmd_vel_topic = LaunchConfiguration("cmd_vel_topic")

    common_parameters = {
        "baud": baud,
        "address": address,
        "poll_rate_hz": poll_rate_hz,
        "control_rate_hz": control_rate_hz,
        "max_speed": max_speed,
        "ticks_per_revolution": ticks_per_revolution,
        "wheel_radius": wheel_radius,
        "base_width": base_width,
        "cmd_vel_timeout": cmd_vel_timeout,
        "reset_encoders_on_connect": reset_encoders_on_connect,
    }

    return LaunchDescription(
        [
            DeclareLaunchArgument("front_namespace", default_value="roboclawfront"),
            DeclareLaunchArgument("rear_namespace", default_value="roboclawrear"),
            DeclareLaunchArgument("front_port", default_value="/dev/ttyACM0"),
            DeclareLaunchArgument("rear_port", default_value="/dev/ttyACM1"),
            DeclareLaunchArgument("baud", default_value="115200"),
            DeclareLaunchArgument("address", default_value="128"),
            DeclareLaunchArgument("poll_rate_hz", default_value="10.0"),
            DeclareLaunchArgument("control_rate_hz", default_value="20.0"),
            DeclareLaunchArgument("max_speed", default_value="1.0"),
            DeclareLaunchArgument("ticks_per_revolution", default_value="2048.0"),
            DeclareLaunchArgument("wheel_radius", default_value="0.05"),
            DeclareLaunchArgument("base_width", default_value="0.315"),
            DeclareLaunchArgument("cmd_vel_timeout", default_value="0.5"),
            DeclareLaunchArgument("reset_encoders_on_connect", default_value="true"),
            DeclareLaunchArgument("cmd_vel_topic", default_value="cmd_vel"),
            Node(
                package="roboclaw_ros2",
                executable="roboclaw_node",
                namespace=front_namespace,
                name="roboclaw_node",
                output="screen",
                parameters=[{**common_parameters, "port": front_port}],
                remappings=[("cmd_vel", cmd_vel_topic)],
            ),
            Node(
                package="roboclaw_ros2",
                executable="roboclaw_node",
                namespace=rear_namespace,
                name="roboclaw_node",
                output="screen",
                parameters=[{**common_parameters, "port": rear_port}],
                remappings=[("cmd_vel", cmd_vel_topic)],
            ),
        ]
    )
