#!/usr/bin/env python3
"""ROS 2 RoboClaw node with cmd_vel control and telemetry publishing."""

from __future__ import annotations

import math
import time
from typing import Any

from basicmicro import Basicmicro
from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from std_msgs.msg import Int32
from std_msgs.msg import Int64
from std_msgs.msg import UInt32


class RoboclawNode(Node):
    """Bridge a RoboClaw controller with ROS 2 topics."""

    def __init__(self) -> None:
        super().__init__("roboclaw_node")

        self.declare_parameter("port", "/dev/ttyACM0")
        self.declare_parameter("baud", 115200)
        self.declare_parameter("address", 0x80)
        self.declare_parameter("poll_rate_hz", 10.0)
        self.declare_parameter("control_rate_hz", 20.0)
        self.declare_parameter("max_speed", 1.0)
        self.declare_parameter("ticks_per_revolution", 2048.0)
        self.declare_parameter("wheel_radius", 0.05)
        self.declare_parameter("base_width", 0.315)
        self.declare_parameter("cmd_vel_timeout", 0.5)
        self.declare_parameter("reset_encoders_on_connect", True)

        self.port = str(self.get_parameter("port").value)
        self.baud = int(self.get_parameter("baud").value)
        self.address = int(self.get_parameter("address").value)
        self.poll_rate_hz = max(float(self.get_parameter("poll_rate_hz").value), 1.0)
        self.control_rate_hz = max(float(self.get_parameter("control_rate_hz").value), 1.0)
        self.max_speed = float(self.get_parameter("max_speed").value)
        self.ticks_per_revolution = float(self.get_parameter("ticks_per_revolution").value)
        self.wheel_radius = float(self.get_parameter("wheel_radius").value)
        self.base_width = float(self.get_parameter("base_width").value)
        self.cmd_vel_timeout = max(float(self.get_parameter("cmd_vel_timeout").value), 0.0)
        self.reset_encoders_on_connect = bool(
            self.get_parameter("reset_encoders_on_connect").value
        )
        self.ticks_per_meter = self._compute_ticks_per_meter()

        self.controller: Basicmicro | None = None
        self.version = "unknown"
        self._last_connection_log_ns = 0
        self._last_command_log_ns = 0
        self._latest_cmd_vel = Twist()
        self._last_cmd_vel_time = time.monotonic()
        self._last_sent_left_ticks: int | None = None
        self._last_sent_right_ticks: int | None = None

        self.cmd_vel_sub = self.create_subscription(Twist, "cmd_vel", self._cmd_vel_callback, 10)

        self.encoder_m1_pub = self.create_publisher(Int64, "roboclaw/encoder/m1", 10)
        self.encoder_m2_pub = self.create_publisher(Int64, "roboclaw/encoder/m2", 10)
        self.speed_m1_pub = self.create_publisher(Int32, "roboclaw/speed/m1", 10)
        self.speed_m2_pub = self.create_publisher(Int32, "roboclaw/speed/m2", 10)
        self.current_m1_pub = self.create_publisher(Float32, "roboclaw/current/m1", 10)
        self.current_m2_pub = self.create_publisher(Float32, "roboclaw/current/m2", 10)
        self.main_battery_pub = self.create_publisher(
            Float32, "roboclaw/voltage/main_battery", 10
        )
        self.logic_battery_pub = self.create_publisher(
            Float32, "roboclaw/voltage/logic_battery", 10
        )
        self.temperature_1_pub = self.create_publisher(
            Float32, "roboclaw/temperature/sensor1", 10
        )
        self.temperature_2_pub = self.create_publisher(
            Float32, "roboclaw/temperature/sensor2", 10
        )
        self.error_pub = self.create_publisher(UInt32, "roboclaw/error", 10)
        self.left_target_pub = self.create_publisher(Int32, "roboclaw/cmd_ticks/left", 10)
        self.right_target_pub = self.create_publisher(Int32, "roboclaw/cmd_ticks/right", 10)

        self.control_timer = self.create_timer(1.0 / self.control_rate_hz, self._control_loop)
        self.telemetry_timer = self.create_timer(1.0 / self.poll_rate_hz, self._poll_and_publish)

        self.get_logger().info(
            f"Starting RoboClaw node on {self.port} at {self.baud} baud, "
            f"address 0x{self.address:02X}"
        )
        self.get_logger().info(
            f"Using {self.ticks_per_revolution:.3f} ticks/rev, "
            f"wheel radius {self.wheel_radius:.4f} m, "
            f"which gives {self.ticks_per_meter:.3f} ticks/m"
        )

    def _compute_ticks_per_meter(self) -> float:
        if self.ticks_per_revolution <= 0.0:
            raise ValueError("ticks_per_revolution must be greater than zero")
        if self.wheel_radius <= 0.0:
            raise ValueError("wheel_radius must be greater than zero")
        return self.ticks_per_revolution / (2.0 * math.pi * self.wheel_radius)

    def _cmd_vel_callback(self, msg: Twist) -> None:
        self._latest_cmd_vel = msg
        self._last_cmd_vel_time = time.monotonic()

    def _connect(self) -> bool:
        if self.controller is not None:
            return True

        try:
            controller = Basicmicro(self.port, self.baud)
            if not controller.Open():
                raise RuntimeError("could not open serial port")

            version = controller.ReadVersion(self.address)
            if version[0]:
                self.version = str(version[1])

            controller.SpeedM1M2(self.address, 0, 0)
            if self.reset_encoders_on_connect:
                controller.ResetEncoders(self.address)

            self.controller = controller
            self._last_sent_left_ticks = None
            self._last_sent_right_ticks = None

            self.get_logger().info(f"Connected to RoboClaw firmware: {self.version}")
            return True
        except Exception as exc:
            self._throttled_connection_warning(exc)
            self._close_controller()
            return False

    def _throttled_connection_warning(self, exc: Exception) -> None:
        now_ns = self.get_clock().now().nanoseconds
        if now_ns - self._last_connection_log_ns >= 5_000_000_000:
            self.get_logger().warning(f"Waiting for RoboClaw connection: {exc}")
            self._last_connection_log_ns = now_ns

    def _close_controller(self) -> None:
        if self.controller is None:
            return

        try:
            self.controller.close()
        except Exception:
            pass
        finally:
            self.controller = None
            self._last_sent_left_ticks = None
            self._last_sent_right_ticks = None

    def _control_loop(self) -> None:
        if not self._connect():
            return

        left_ticks, right_ticks = self._compute_wheel_ticks()

        try:
            self._send_speed_command(left_ticks, right_ticks)
            self._publish_int(self.left_target_pub, left_ticks, Int32)
            self._publish_int(self.right_target_pub, right_ticks, Int32)
        except Exception as exc:
            self.get_logger().warning(f"Motor command failed: {exc}")
            self._close_controller()

    def _compute_wheel_ticks(self) -> tuple[int, int]:
        timed_out = (time.monotonic() - self._last_cmd_vel_time) > self.cmd_vel_timeout
        cmd = Twist() if timed_out else self._latest_cmd_vel

        linear_x = max(-self.max_speed, min(self.max_speed, cmd.linear.x))
        right_velocity = linear_x + cmd.angular.z * self.base_width / 2.0
        left_velocity = linear_x - cmd.angular.z * self.base_width / 2.0

        right_ticks = int(right_velocity * self.ticks_per_meter)
        left_ticks = int(left_velocity * self.ticks_per_meter)

        if timed_out:
            now_ns = self.get_clock().now().nanoseconds
            if now_ns - self._last_command_log_ns >= 5_000_000_000:
                self.get_logger().info("No recent cmd_vel received, commanding zero speed")
                self._last_command_log_ns = now_ns

        return left_ticks, right_ticks

    def _send_speed_command(self, left_ticks: int, right_ticks: int) -> None:
        controller = self.controller
        if controller is None:
            return

        left_ticks = int(left_ticks)
        right_ticks = int(right_ticks)

        if (
            left_ticks == self._last_sent_left_ticks
            and right_ticks == self._last_sent_right_ticks
        ):
            return

        controller.SpeedM1M2(self.address, left_ticks, right_ticks)

        self._last_sent_left_ticks = left_ticks
        self._last_sent_right_ticks = right_ticks

    def _poll_and_publish(self) -> None:
        if not self._connect():
            return

        try:
            controller = self.controller
            if controller is None:
                return

            enc1 = self._require_ok("encoder M1", controller.ReadEncM1(self.address))
            enc2 = self._require_ok("encoder M2", controller.ReadEncM2(self.address))
            speed1 = self._require_ok("speed M1", controller.ReadSpeedM1(self.address))
            speed2 = self._require_ok("speed M2", controller.ReadSpeedM2(self.address))
            currents = self._require_ok("currents", controller.ReadCurrents(self.address))
            main_batt = self._require_ok(
                "main battery voltage",
                controller.ReadMainBatteryVoltage(self.address),
            )
            logic_batt = self._require_ok(
                "logic battery voltage",
                controller.ReadLogicBatteryVoltage(self.address),
            )
            temp1 = self._require_ok("temperature 1", controller.ReadTemp(self.address))
            temp2 = self._require_ok("temperature 2", controller.ReadTemp2(self.address))
            error = self._require_ok("error flags", controller.ReadError(self.address))

            self._publish_int(self.encoder_m1_pub, int(enc1[1]), Int64)
            self._publish_int(self.encoder_m2_pub, int(enc2[1]), Int64)
            self._publish_int(self.speed_m1_pub, int(speed1[1]), Int32)
            self._publish_int(self.speed_m2_pub, int(speed2[1]), Int32)
            self._publish_float(self.current_m1_pub, float(currents[1]) / 1000.0)
            self._publish_float(self.current_m2_pub, float(currents[2]) / 1000.0)
            self._publish_float(
                self.main_battery_pub,
                float(main_batt[1]) / Basicmicro.VOLTAGE_SCALE,
            )
            self._publish_float(
                self.logic_battery_pub,
                float(logic_batt[1]) / Basicmicro.VOLTAGE_SCALE,
            )
            self._publish_float(
                self.temperature_1_pub,
                float(temp1[1]) / Basicmicro.TEMP_SCALE,
            )
            self._publish_float(
                self.temperature_2_pub,
                float(temp2[1]) / Basicmicro.TEMP_SCALE,
            )
            self._publish_int(self.error_pub, int(error[1]), UInt32)
        except Exception as exc:
            self.get_logger().warning(f"Telemetry read failed: {exc}")
            self._close_controller()

    def _require_ok(self, label: str, result: Any) -> Any:
        if not result[0]:
            raise RuntimeError(f"read failed for {label}")
        return result

    def _publish_float(self, publisher: Any, value: float) -> None:
        msg = Float32()
        msg.data = value
        publisher.publish(msg)

    def _publish_int(self, publisher: Any, value: int, msg_type: Any) -> None:
        msg = msg_type()
        msg.data = value
        publisher.publish(msg)

    def destroy_node(self) -> bool:
        try:
            self._send_speed_command(0, 0)
        except Exception:
            pass
        self._close_controller()
        return super().destroy_node()


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = RoboclawNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
