#!/usr/bin/env python3
"""ROS 2 odometry node for a differential mobile base driven by RoboClaw."""

from __future__ import annotations

import math
from typing import Callable

from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Int64
from tf2_ros import TransformBroadcaster


class RoboclawOdometryNode(Node):
    """Estimate base odometry from left/right encoder topics."""

    def __init__(self) -> None:
        super().__init__("roboclaw_odometry")

        self.declare_parameter("odom_topic", "odom")
        self.declare_parameter("odom_frame_id", "odom")
        self.declare_parameter("base_frame_id", "base_link")
        self.declare_parameter("publish_tf", True)
        self.declare_parameter("publish_rate_hz", 30.0)
        self.declare_parameter("ticks_per_revolution", 2048.0)
        self.declare_parameter("wheel_radius", 0.05)
        self.declare_parameter("base_width", 0.315)
        self.declare_parameter("max_encoder_jump_counts", 10000)
        self.declare_parameter("left_encoder_topics", ["roboclaw/encoder/m1"])
        self.declare_parameter("right_encoder_topics", ["roboclaw/encoder/m2"])

        self.odom_topic = str(self.get_parameter("odom_topic").value)
        self.odom_frame_id = str(self.get_parameter("odom_frame_id").value)
        self.base_frame_id = str(self.get_parameter("base_frame_id").value)
        self.publish_tf = bool(self.get_parameter("publish_tf").value)
        self.publish_rate_hz = max(float(self.get_parameter("publish_rate_hz").value), 1.0)
        self.ticks_per_revolution = float(self.get_parameter("ticks_per_revolution").value)
        self.wheel_radius = float(self.get_parameter("wheel_radius").value)
        self.base_width = float(self.get_parameter("base_width").value)
        self.max_encoder_jump_counts = int(self.get_parameter("max_encoder_jump_counts").value)
        self.left_encoder_topics = list(self.get_parameter("left_encoder_topics").value)
        self.right_encoder_topics = list(self.get_parameter("right_encoder_topics").value)

        if not self.left_encoder_topics:
            raise ValueError("left_encoder_topics must contain at least one topic")
        if not self.right_encoder_topics:
            raise ValueError("right_encoder_topics must contain at least one topic")
        if self.base_width <= 0.0:
            raise ValueError("base_width must be greater than zero")

        self.ticks_per_meter = self._compute_ticks_per_meter()

        self.odom_publisher = self.create_publisher(Odometry, self.odom_topic, 10)
        self.tf_broadcaster = TransformBroadcaster(self) if self.publish_tf else None
        self.encoder_subscriptions = []

        self.left_counts_current = {topic: None for topic in self.left_encoder_topics}
        self.right_counts_current = {topic: None for topic in self.right_encoder_topics}
        self.left_counts_previous = {topic: None for topic in self.left_encoder_topics}
        self.right_counts_previous = {topic: None for topic in self.right_encoder_topics}

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.last_update_time = None
        self._last_waiting_log_ns = 0
        self._last_jump_log_ns = 0

        self.pose_covariance = [
            0.01, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.01, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 99999.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 99999.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 99999.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.05,
        ]
        self.twist_covariance = [
            0.05, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.05, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 99999.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 99999.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 99999.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.1,
        ]

        for topic in self.left_encoder_topics:
            self.encoder_subscriptions.append(
                self.create_subscription(
                    Int64,
                    topic,
                    self._make_encoder_callback(self.left_counts_current, topic),
                    10,
                )
            )

        for topic in self.right_encoder_topics:
            self.encoder_subscriptions.append(
                self.create_subscription(
                    Int64,
                    topic,
                    self._make_encoder_callback(self.right_counts_current, topic),
                    10,
                )
            )

        self.timer = self.create_timer(1.0 / self.publish_rate_hz, self._update_odometry)

        self.get_logger().info(
            f"Starting odometry node with {self.ticks_per_meter:.3f} ticks/m, "
            f"left topics={self.left_encoder_topics}, right topics={self.right_encoder_topics}"
        )

    def _compute_ticks_per_meter(self) -> float:
        if self.ticks_per_revolution <= 0.0:
            raise ValueError("ticks_per_revolution must be greater than zero")
        if self.wheel_radius <= 0.0:
            raise ValueError("wheel_radius must be greater than zero")
        return self.ticks_per_revolution / (2.0 * math.pi * self.wheel_radius)

    def _make_encoder_callback(
        self,
        storage: dict[str, int | None],
        topic: str,
    ) -> Callable[[Int64], None]:
        def callback(msg: Int64) -> None:
            storage[topic] = int(msg.data)

        return callback

    def _update_odometry(self) -> None:
        now = self.get_clock().now()

        if not self._has_encoder_data():
            self._throttled_waiting_log()
            return

        if self.last_update_time is None:
            self._snapshot_counts_as_previous()
            self.last_update_time = now
            return

        dt = (now - self.last_update_time).nanoseconds * 1e-9
        if dt <= 0.0:
            return

        left_distance = self._mean_distance_delta(
            self.left_counts_current,
            self.left_counts_previous,
        )
        right_distance = self._mean_distance_delta(
            self.right_counts_current,
            self.right_counts_previous,
        )

        self._snapshot_counts_as_previous()
        self.last_update_time = now

        if left_distance is None or right_distance is None:
            self._throttled_waiting_log()
            return

        delta_s = 0.5 * (left_distance + right_distance)
        delta_yaw = (right_distance - left_distance) / self.base_width

        mid_yaw = self.yaw + 0.5 * delta_yaw
        self.x += delta_s * math.cos(mid_yaw)
        self.y += delta_s * math.sin(mid_yaw)
        self.yaw = self._normalize_angle(self.yaw + delta_yaw)

        linear_velocity = delta_s / dt
        angular_velocity = delta_yaw / dt

        self._publish_odometry(now, linear_velocity, angular_velocity)

    def _has_encoder_data(self) -> bool:
        return (
            any(value is not None for value in self.left_counts_current.values())
            and any(value is not None for value in self.right_counts_current.values())
        )

    def _snapshot_counts_as_previous(self) -> None:
        for topic, value in self.left_counts_current.items():
            if value is not None:
                self.left_counts_previous[topic] = value
        for topic, value in self.right_counts_current.items():
            if value is not None:
                self.right_counts_previous[topic] = value

    def _mean_distance_delta(
        self,
        current_counts: dict[str, int | None],
        previous_counts: dict[str, int | None],
    ) -> float | None:
        deltas = []

        for topic, current_value in current_counts.items():
            previous_value = previous_counts.get(topic)
            if current_value is None or previous_value is None:
                continue

            delta_counts = current_value - previous_value
            if abs(delta_counts) > self.max_encoder_jump_counts:
                self._throttled_jump_log(topic, delta_counts)
                continue

            deltas.append(delta_counts / self.ticks_per_meter)

        if not deltas:
            return None
        return sum(deltas) / len(deltas)

    def _publish_odometry(
        self,
        stamp,
        linear_velocity: float,
        angular_velocity: float,
    ) -> None:
        quaternion = self._yaw_to_quaternion(self.yaw)

        odom = Odometry()
        odom.header.stamp = stamp.to_msg()
        odom.header.frame_id = self.odom_frame_id
        odom.child_frame_id = self.base_frame_id
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = quaternion
        odom.twist.twist.linear.x = linear_velocity
        odom.twist.twist.angular.z = angular_velocity
        odom.pose.covariance = self.pose_covariance
        odom.twist.covariance = self.twist_covariance
        self.odom_publisher.publish(odom)

        if self.tf_broadcaster is None:
            return

        transform = TransformStamped()
        transform.header.stamp = stamp.to_msg()
        transform.header.frame_id = self.odom_frame_id
        transform.child_frame_id = self.base_frame_id
        transform.transform.translation.x = self.x
        transform.transform.translation.y = self.y
        transform.transform.translation.z = 0.0
        transform.transform.rotation = quaternion
        self.tf_broadcaster.sendTransform(transform)

    def _yaw_to_quaternion(self, yaw: float) -> Quaternion:
        quaternion = Quaternion()
        quaternion.z = math.sin(yaw * 0.5)
        quaternion.w = math.cos(yaw * 0.5)
        return quaternion

    def _normalize_angle(self, angle: float) -> float:
        return math.atan2(math.sin(angle), math.cos(angle))

    def _throttled_waiting_log(self) -> None:
        now_ns = self.get_clock().now().nanoseconds
        if now_ns - self._last_waiting_log_ns >= 5_000_000_000:
            self.get_logger().info("Waiting for encoder data to start odometry")
            self._last_waiting_log_ns = now_ns

    def _throttled_jump_log(self, topic: str, delta_counts: int) -> None:
        now_ns = self.get_clock().now().nanoseconds
        if now_ns - self._last_jump_log_ns >= 2_000_000_000:
            self.get_logger().warning(
                f"Ignoring encoder jump on {topic}: delta={delta_counts} counts"
            )
            self._last_jump_log_ns = now_ns


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = RoboclawOdometryNode()

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
