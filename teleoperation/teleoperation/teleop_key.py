#!/usr/bin/env python3
"""Keyboard teleoperation node for ROS 2."""

from __future__ import annotations

import os
import select
import sys
import time

from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node

if os.name == "nt":
    import msvcrt
else:
    import termios
    import tty


MESSAGE = """
Control Your Robot!
---------------------------
Moving around:
        w
   a    s    d
        x

w/x : increase/decrease linear velocity
a/d : increase/decrease angular velocity

space key, s : force stop

CTRL-C to quit
"""

ERROR_MESSAGE = """
Communications failed
"""


class TeleopKeyNode(Node):
    """Publish cmd_vel from keyboard input."""

    def __init__(self) -> None:
        super().__init__("teleop_key")

        self.declare_parameter("cmd_vel_topic", "cmd_vel")
        self.declare_parameter("max_linear_velocity", 0.4)
        self.declare_parameter("max_angular_velocity", 1.0)
        self.declare_parameter("linear_step_size", 0.005)
        self.declare_parameter("angular_step_size", 0.005)
        self.declare_parameter("key_timeout", 0.1)

        self.cmd_vel_topic = str(self.get_parameter("cmd_vel_topic").value)
        self.max_linear_velocity = float(self.get_parameter("max_linear_velocity").value)
        self.max_angular_velocity = float(self.get_parameter("max_angular_velocity").value)
        self.linear_step_size = float(self.get_parameter("linear_step_size").value)
        self.angular_step_size = float(self.get_parameter("angular_step_size").value)
        self.key_timeout = max(float(self.get_parameter("key_timeout").value), 0.01)

        self.publisher = self.create_publisher(Twist, self.cmd_vel_topic, 10)

        self.target_linear_velocity = 0.0
        self.target_angular_velocity = 0.0
        self.control_linear_velocity = 0.0
        self.control_angular_velocity = 0.0

    def run(self) -> None:
        settings = None
        if os.name != "nt":
            settings = termios.tcgetattr(sys.stdin)

        status = 0

        try:
            print(MESSAGE)
            while rclpy.ok():
                key = self._get_key(settings)

                if key == "w":
                    self.target_linear_velocity = self._check_linear_limit(
                        self.target_linear_velocity + self.linear_step_size
                    )
                    status += 1
                    print(self._vels())
                elif key == "x":
                    self.target_linear_velocity = self._check_linear_limit(
                        self.target_linear_velocity - self.linear_step_size
                    )
                    status += 1
                    print(self._vels())
                elif key == "a":
                    self.target_angular_velocity = self._check_angular_limit(
                        self.target_angular_velocity + self.angular_step_size
                    )
                    status += 1
                    print(self._vels())
                elif key == "d":
                    self.target_angular_velocity = self._check_angular_limit(
                        self.target_angular_velocity - self.angular_step_size
                    )
                    status += 1
                    print(self._vels())
                elif key in (" ", "s"):
                    self.target_linear_velocity = 0.0
                    self.control_linear_velocity = 0.0
                    self.target_angular_velocity = 0.0
                    self.control_angular_velocity = 0.0
                    print(self._vels())
                elif key == "\x03":
                    break

                if status == 20:
                    print(MESSAGE)
                    status = 0

                twist = Twist()
                self.control_linear_velocity = self._make_simple_profile(
                    self.control_linear_velocity,
                    self.target_linear_velocity,
                    self.linear_step_size / 2.0,
                )
                self.control_angular_velocity = self._make_simple_profile(
                    self.control_angular_velocity,
                    self.target_angular_velocity,
                    self.angular_step_size / 2.0,
                )

                twist.linear.x = self.control_linear_velocity
                twist.angular.z = self.control_angular_velocity
                self.publisher.publish(twist)

        except Exception:
            print(ERROR_MESSAGE)
            raise
        finally:
            self._publish_stop()
            if os.name != "nt" and settings is not None:
                termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

    def _get_key(self, settings) -> str:
        if os.name == "nt":
            start_time = time.time()
            while True:
                if msvcrt.kbhit():
                    if sys.version_info[0] >= 3:
                        return msvcrt.getch().decode()
                    return msvcrt.getch()
                if time.time() - start_time > self.key_timeout:
                    return ""

        tty.setraw(sys.stdin.fileno())
        readable, _, _ = select.select([sys.stdin], [], [], self.key_timeout)
        key = sys.stdin.read(1) if readable else ""
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        return key

    def _vels(self) -> str:
        return (
            "currently:\tlinear vel "
            f"{self.target_linear_velocity}\t angular vel {self.target_angular_velocity}"
        )

    def _make_simple_profile(self, output: float, input_value: float, slop: float) -> float:
        if input_value > output:
            output = min(input_value, output + slop)
        elif input_value < output:
            output = max(input_value, output - slop)
        else:
            output = input_value
        return output

    def _constrain(self, value: float, low: float, high: float) -> float:
        if value < low:
            return low
        if value > high:
            return high
        return value

    def _check_linear_limit(self, value: float) -> float:
        return self._constrain(
            value,
            -self.max_linear_velocity,
            self.max_linear_velocity,
        )

    def _check_angular_limit(self, value: float) -> float:
        return self._constrain(
            value,
            -self.max_angular_velocity,
            self.max_angular_velocity,
        )

    def _publish_stop(self) -> None:
        twist = Twist()
        self.publisher.publish(twist)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = TeleopKeyNode()

    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
