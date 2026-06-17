#!/usr/bin/env python3
"""Tkinter teleoperation interface for ROS 2."""

from __future__ import annotations

import tkinter as tk
from tkinter import ttk

from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node


class TeleopGuiNode(Node):
    """ROS 2 publisher node for the teleop GUI."""

    def __init__(self) -> None:
        super().__init__("teleop_gui")

        self.declare_parameter("cmd_vel_topic", "cmd_vel")
        self.declare_parameter("max_linear_velocity", 0.4)
        self.declare_parameter("max_angular_velocity", 1.0)
        self.declare_parameter("linear_step_size", 0.05)
        self.declare_parameter("angular_step_size", 0.1)
        self.declare_parameter("publish_rate_hz", 20.0)
        self.declare_parameter("window_title", "ROS 2 Teleop Interface")

        self.cmd_vel_topic = str(self.get_parameter("cmd_vel_topic").value)
        self.max_linear_velocity = float(self.get_parameter("max_linear_velocity").value)
        self.max_angular_velocity = float(self.get_parameter("max_angular_velocity").value)
        self.linear_step_size = float(self.get_parameter("linear_step_size").value)
        self.angular_step_size = float(self.get_parameter("angular_step_size").value)
        self.publish_rate_hz = max(float(self.get_parameter("publish_rate_hz").value), 1.0)
        self.window_title = str(self.get_parameter("window_title").value)

        self.publisher = self.create_publisher(Twist, self.cmd_vel_topic, 10)

    def publish_twist(self, linear: float, angular: float) -> None:
        twist = Twist()
        twist.linear.x = linear
        twist.angular.z = angular
        self.publisher.publish(twist)

    def publish_stop(self) -> None:
        self.publish_twist(0.0, 0.0)


class TeleopGuiApp:
    """Desktop interface to control cmd_vel."""

    def __init__(self, node: TeleopGuiNode) -> None:
        self.node = node
        self.root = tk.Tk()
        self.root.title(self.node.window_title)
        self.root.geometry("760x520")
        self.root.minsize(700, 460)

        self.linear_var = tk.DoubleVar(value=0.0)
        self.angular_var = tk.DoubleVar(value=0.0)
        self.status_var = tk.StringVar(value="")

        self._build_ui()
        self._bind_keys()
        self._update_status()

        self.root.protocol("WM_DELETE_WINDOW", self._on_close)
        self.root.after(25, self._spin_ros)
        self.root.after(self._publish_interval_ms, self._publish_loop)

    @property
    def _publish_interval_ms(self) -> int:
        return max(int(1000.0 / self.node.publish_rate_hz), 20)

    def _build_ui(self) -> None:
        self.root.columnconfigure(0, weight=1)
        self.root.rowconfigure(1, weight=1)

        header = ttk.Frame(self.root, padding=16)
        header.grid(row=0, column=0, sticky="ew")
        header.columnconfigure(0, weight=1)

        ttk.Label(
            header,
            text="ROS 2 Teleoperation",
            font=("TkDefaultFont", 16, "bold"),
        ).grid(row=0, column=0, sticky="w")
        ttk.Label(
            header,
            text=(
                f"Publishing to `{self.node.cmd_vel_topic}` | "
                "Keys: W/X for linear, A/D for angular, Space or S to stop"
            ),
        ).grid(row=1, column=0, sticky="w", pady=(6, 0))

        body = ttk.Frame(self.root, padding=(16, 0, 16, 8))
        body.grid(row=1, column=0, sticky="nsew")
        body.columnconfigure(0, weight=3)
        body.columnconfigure(1, weight=2)
        body.rowconfigure(0, weight=1)

        control_frame = ttk.LabelFrame(body, text="Velocity Control", padding=14)
        control_frame.grid(row=0, column=0, sticky="nsew", padx=(0, 10))
        control_frame.columnconfigure(0, weight=1)
        control_frame.columnconfigure(1, weight=1)

        self._build_scale(
            control_frame,
            column=0,
            label="Linear Velocity (m/s)",
            variable=self.linear_var,
            limit=self.node.max_linear_velocity,
            callback=self._on_linear_scale,
        )
        self._build_scale(
            control_frame,
            column=1,
            label="Angular Velocity (rad/s)",
            variable=self.angular_var,
            limit=self.node.max_angular_velocity,
            callback=self._on_angular_scale,
        )

        button_frame = ttk.LabelFrame(body, text="Teleop Pad", padding=14)
        button_frame.grid(row=0, column=1, sticky="nsew")
        for column in range(3):
            button_frame.columnconfigure(column, weight=1)

        ttk.Button(
            button_frame,
            text=f"Forward\nW (+{self.node.linear_step_size:.3f})",
            command=lambda: self._nudge_linear(self.node.linear_step_size),
        ).grid(row=0, column=1, sticky="nsew", padx=4, pady=4)

        ttk.Button(
            button_frame,
            text=f"Left\nA (+{self.node.angular_step_size:.3f})",
            command=lambda: self._nudge_angular(self.node.angular_step_size),
        ).grid(row=1, column=0, sticky="nsew", padx=4, pady=4)

        ttk.Button(
            button_frame,
            text="Stop\nSpace / S",
            command=self._stop,
        ).grid(row=1, column=1, sticky="nsew", padx=4, pady=4)

        ttk.Button(
            button_frame,
            text=f"Right\nD (-{self.node.angular_step_size:.3f})",
            command=lambda: self._nudge_angular(-self.node.angular_step_size),
        ).grid(row=1, column=2, sticky="nsew", padx=4, pady=4)

        ttk.Button(
            button_frame,
            text=f"Backward\nX (-{self.node.linear_step_size:.3f})",
            command=lambda: self._nudge_linear(-self.node.linear_step_size),
        ).grid(row=2, column=1, sticky="nsew", padx=4, pady=4)

        ttk.Button(
            button_frame,
            text="Center Angular",
            command=lambda: self._set_angular(0.0),
        ).grid(row=3, column=0, columnspan=3, sticky="ew", padx=4, pady=(12, 4))

        footer = ttk.Frame(self.root, padding=(16, 0, 16, 16))
        footer.grid(row=2, column=0, sticky="ew")
        footer.columnconfigure(0, weight=1)
        ttk.Label(footer, textvariable=self.status_var, relief="sunken", anchor="w").grid(
            row=0, column=0, sticky="ew"
        )

    def _build_scale(
        self,
        parent: ttk.LabelFrame,
        *,
        column: int,
        label: str,
        variable: tk.DoubleVar,
        limit: float,
        callback,
    ) -> None:
        frame = ttk.Frame(parent)
        frame.grid(row=0, column=column, sticky="nsew", padx=8)
        frame.columnconfigure(0, weight=1)

        ttk.Label(frame, text=label).grid(row=0, column=0, sticky="w")
        ttk.Label(
            frame,
            textvariable=variable,
            font=("TkDefaultFont", 14, "bold"),
        ).grid(row=1, column=0, sticky="w", pady=(4, 8))

        scale = ttk.Scale(
            frame,
            from_=limit,
            to=-limit,
            orient="vertical",
            command=callback,
        )
        scale.grid(row=2, column=0, sticky="ns", pady=(0, 8))
        scale.set(0.0)

        entry = ttk.Entry(frame, textvariable=variable, justify="center", width=10)
        entry.grid(row=3, column=0, sticky="ew")
        entry.bind("<Return>", lambda _event, axis=label: self._apply_entry(axis))

        setattr(self, f"scale_{column}", scale)

    def _bind_keys(self) -> None:
        self.root.bind("<KeyPress-w>", lambda _event: self._nudge_linear(self.node.linear_step_size))
        self.root.bind("<KeyPress-x>", lambda _event: self._nudge_linear(-self.node.linear_step_size))
        self.root.bind("<KeyPress-a>", lambda _event: self._nudge_angular(self.node.angular_step_size))
        self.root.bind("<KeyPress-d>", lambda _event: self._nudge_angular(-self.node.angular_step_size))
        self.root.bind("<KeyPress-s>", lambda _event: self._stop())
        self.root.bind("<space>", lambda _event: self._stop())
        self.root.bind("<Escape>", lambda _event: self._on_close())
        self.root.bind("<KeyPress-p>", lambda _event: self._on_close())
        self.root.bind("<KeyPress-P>", lambda _event: self._on_close())
        self.root.bind("<Up>", lambda _event: self._nudge_linear(self.node.linear_step_size))
        self.root.bind("<Down>", lambda _event: self._nudge_linear(-self.node.linear_step_size))
        self.root.bind("<Left>", lambda _event: self._nudge_angular(self.node.angular_step_size))
        self.root.bind("<Right>", lambda _event: self._nudge_angular(-self.node.angular_step_size))

    def _spin_ros(self) -> None:
        rclpy.spin_once(self.node, timeout_sec=0.0)
        self.root.after(25, self._spin_ros)

    def _publish_loop(self) -> None:
        self.node.publish_twist(self.linear_var.get(), self.angular_var.get())
        self.root.after(self._publish_interval_ms, self._publish_loop)

    def _apply_entry(self, axis: str) -> None:
        if axis.startswith("Linear"):
            self._set_linear(self.linear_var.get())
        else:
            self._set_angular(self.angular_var.get())

    def _on_linear_scale(self, raw_value: str) -> None:
        self.linear_var.set(round(float(raw_value), 4))
        self._update_status()

    def _on_angular_scale(self, raw_value: str) -> None:
        self.angular_var.set(round(float(raw_value), 4))
        self._update_status()

    def _set_linear(self, value: float) -> None:
        bounded = self._constrain(value, -self.node.max_linear_velocity, self.node.max_linear_velocity)
        self.linear_var.set(round(bounded, 4))
        self.scale_0.set(bounded)
        self._update_status()

    def _set_angular(self, value: float) -> None:
        bounded = self._constrain(value, -self.node.max_angular_velocity, self.node.max_angular_velocity)
        self.angular_var.set(round(bounded, 4))
        self.scale_1.set(bounded)
        self._update_status()

    def _nudge_linear(self, delta: float) -> None:
        self._set_linear(self.linear_var.get() + delta)

    def _nudge_angular(self, delta: float) -> None:
        self._set_angular(self.angular_var.get() + delta)

    def _stop(self) -> None:
        self._set_linear(0.0)
        self._set_angular(0.0)
        self.node.publish_stop()

    def _constrain(self, value: float, low: float, high: float) -> float:
        return max(low, min(high, value))

    def _update_status(self) -> None:
        self.status_var.set(
            f"cmd_vel -> linear.x={self.linear_var.get():.3f} m/s | "
            f"angular.z={self.angular_var.get():.3f} rad/s"
        )

    def _on_close(self) -> None:
        self.node.publish_stop()
        self.root.quit()
        self.root.destroy()

    def run(self) -> None:
        style = ttk.Style()
        if "clam" in style.theme_names():
            style.theme_use("clam")
        self.root.mainloop()


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = TeleopGuiNode()
    app = TeleopGuiApp(node)

    try:
        app.run()
    finally:
        node.publish_stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
