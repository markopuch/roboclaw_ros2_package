#!/usr/bin/env python3
"""
Duty-cycle dashboard for Basicmicro motor controllers.

This tool keeps the control surface focused on duty-cycle operation while
showing live controller telemetry in a small desktop interface.
"""

import argparse
import logging
import tkinter as tk
from tkinter import messagebox, ttk

from basicmicro import Basicmicro


logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s - %(name)s - %(levelname)s - %(message)s",
)
logger = logging.getLogger(__name__)


class DutyCycleDashboard:
    POLL_MS = 250

    def __init__(self, root: tk.Tk, port: str, baud: int, address: int) -> None:
        self.root = root
        self.root.title("Basicmicro Duty Cycle Dashboard")
        self.root.geometry("860x640")
        self.root.minsize(760, 560)

        self.port_var = tk.StringVar(value=port)
        self.baud_var = tk.IntVar(value=baud)
        self.address_var = tk.StringVar(value=hex(address))
        self.connection_var = tk.StringVar(value="Disconnected")
        self.version_var = tk.StringVar(value="-")
        self.status_var = tk.StringVar(value="Ready")

        self.m1_var = tk.IntVar(value=0)
        self.m2_var = tk.IntVar(value=0)

        self.telemetry_vars = {
            "enc1": tk.StringVar(value="-"),
            "enc2": tk.StringVar(value="-"),
            "speed1": tk.StringVar(value="-"),
            "speed2": tk.StringVar(value="-"),
            "pwm1": tk.StringVar(value="-"),
            "pwm2": tk.StringVar(value="-"),
            "current1": tk.StringVar(value="-"),
            "current2": tk.StringVar(value="-"),
            "main_batt": tk.StringVar(value="-"),
            "logic_batt": tk.StringVar(value="-"),
            "temp1": tk.StringVar(value="-"),
            "temp2": tk.StringVar(value="-"),
            "error": tk.StringVar(value="-"),
        }

        self.controller = None
        self.connected = False

        self._build_ui()
        self._set_controls_state(False)
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)
        self.root.after(150, self.connect)

    def _build_ui(self) -> None:
        self.root.columnconfigure(0, weight=1)
        self.root.rowconfigure(1, weight=1)

        self.connection_frame = ttk.Frame(self.root, padding=12)
        self.connection_frame.grid(row=0, column=0, sticky="ew")
        self.connection_frame.columnconfigure(1, weight=1)
        self.connection_frame.columnconfigure(3, weight=1)

        ttk.Label(self.connection_frame, text="Port").grid(row=0, column=0, sticky="w", padx=(0, 8))
        ttk.Entry(self.connection_frame, textvariable=self.port_var, width=18).grid(row=0, column=1, sticky="ew")

        ttk.Label(self.connection_frame, text="Baud").grid(row=0, column=2, sticky="w", padx=(12, 8))
        ttk.Entry(self.connection_frame, textvariable=self.baud_var, width=12).grid(row=0, column=3, sticky="ew")

        ttk.Label(self.connection_frame, text="Address").grid(row=0, column=4, sticky="w", padx=(12, 8))
        ttk.Entry(self.connection_frame, textvariable=self.address_var, width=10).grid(row=0, column=5, sticky="ew")

        ttk.Button(self.connection_frame, text="Connect", command=self.connect).grid(row=0, column=6, padx=(12, 6))
        ttk.Button(self.connection_frame, text="Disconnect", command=self.disconnect).grid(row=0, column=7)

        info = ttk.Frame(self.root, padding=(12, 0, 12, 8))
        info.grid(row=1, column=0, sticky="nsew")
        info.columnconfigure(0, weight=3)
        info.columnconfigure(1, weight=2)
        info.rowconfigure(0, weight=1)

        control_frame = ttk.LabelFrame(info, text="Duty Cycle Control", padding=12)
        control_frame.grid(row=0, column=0, sticky="nsew", padx=(0, 8))
        control_frame.columnconfigure(0, weight=1)
        control_frame.columnconfigure(1, weight=1)

        self._build_motor_panel(control_frame, 0, "Motor 1", self.m1_var, self._on_m1_slider, self.stop_m1)
        self._build_motor_panel(control_frame, 1, "Motor 2", self.m2_var, self._on_m2_slider, self.stop_m2)

        actions = ttk.Frame(control_frame)
        actions.grid(row=1, column=0, columnspan=2, sticky="ew", pady=(12, 0))
        actions.columnconfigure((0, 1, 2), weight=1)
        ttk.Button(actions, text="Stop All", command=self.stop_all).grid(row=0, column=0, padx=4, sticky="ew")
        ttk.Button(actions, text="Center Sliders", command=self.center_sliders).grid(row=0, column=1, padx=4, sticky="ew")
        ttk.Button(actions, text="Reset Encoders", command=self.reset_encoders).grid(row=0, column=2, padx=4, sticky="ew")

        telemetry_frame = ttk.LabelFrame(info, text="Live Telemetry", padding=12)
        telemetry_frame.grid(row=0, column=1, sticky="nsew")
        telemetry_frame.columnconfigure(1, weight=1)

        rows = [
            ("Connection", self.connection_var),
            ("Firmware", self.version_var),
            ("Encoder M1", self.telemetry_vars["enc1"]),
            ("Encoder M2", self.telemetry_vars["enc2"]),
            ("Speed M1", self.telemetry_vars["speed1"]),
            ("Speed M2", self.telemetry_vars["speed2"]),
            ("PWM M1", self.telemetry_vars["pwm1"]),
            ("PWM M2", self.telemetry_vars["pwm2"]),
            ("Current M1", self.telemetry_vars["current1"]),
            ("Current M2", self.telemetry_vars["current2"]),
            ("Main Batt", self.telemetry_vars["main_batt"]),
            ("Logic Batt", self.telemetry_vars["logic_batt"]),
            ("Temp 1", self.telemetry_vars["temp1"]),
            ("Temp 2", self.telemetry_vars["temp2"]),
            ("Error", self.telemetry_vars["error"]),
        ]
        for row, (label, var) in enumerate(rows):
            ttk.Label(telemetry_frame, text=label).grid(row=row, column=0, sticky="w", pady=2, padx=(0, 12))
            ttk.Label(telemetry_frame, textvariable=var).grid(row=row, column=1, sticky="w", pady=2)

        footer = ttk.Frame(self.root, padding=(12, 4, 12, 12))
        footer.grid(row=2, column=0, sticky="ew")
        footer.columnconfigure(0, weight=1)
        ttk.Label(footer, textvariable=self.status_var, relief="sunken", anchor="w").grid(row=0, column=0, sticky="ew")

    def _build_motor_panel(self, parent, column, title, variable, callback, stop_callback) -> None:
        frame = ttk.LabelFrame(parent, text=title, padding=12)
        frame.grid(row=0, column=column, sticky="nsew", padx=4)
        frame.columnconfigure(0, weight=1)

        ttk.Label(frame, textvariable=variable, font=("TkDefaultFont", 16, "bold")).grid(
            row=0, column=0, pady=(0, 8)
        )

        scale = ttk.Scale(
            frame,
            from_=Basicmicro.MIN_DUTY,
            to=Basicmicro.MAX_DUTY,
            orient="vertical",
            command=callback,
        )
        scale.grid(row=1, column=0, sticky="ns", pady=8)
        scale.set(0)

        setattr(self, f"{title.lower().replace(' ', '_')}_scale", scale)

        entry = ttk.Entry(frame, textvariable=variable, justify="center", width=10)
        entry.grid(row=2, column=0, pady=8)
        entry.bind("<Return>", lambda _event, motor=title: self._apply_entry(motor))

        ttk.Button(frame, text=f"Apply {title}", command=lambda motor=title: self._apply_entry(motor)).grid(
            row=3, column=0, sticky="ew", pady=(0, 6)
        )
        ttk.Button(frame, text=f"Stop {title}", command=stop_callback).grid(row=4, column=0, sticky="ew")

    def _set_controls_state(self, enabled: bool) -> None:
        state = "!disabled" if enabled else "disabled"
        for child in self.root.winfo_children():
            self._set_state_recursive(child, state)

        # Keep connection controls available at all times.
        for widget in self.connection_frame.winfo_children():
            widget.state(["!disabled"])

    def _set_state_recursive(self, widget, state: str) -> None:
        if isinstance(widget, ttk.Widget):
            try:
                widget.state([state])
            except tk.TclError:
                pass
        for child in widget.winfo_children():
            self._set_state_recursive(child, state)

    def _parse_address(self) -> int:
        return int(self.address_var.get(), 0)

    def connect(self) -> None:
        self.disconnect(show_status=False)
        try:
            self.controller = Basicmicro(self.port_var.get(), int(self.baud_var.get()))
            if not self.controller.Open():
                raise RuntimeError("Could not open serial port")

            address = self._parse_address()
            version = self.controller.ReadVersion(address)
            if version[0]:
                self.version_var.set(version[1])
            else:
                self.version_var.set("Unknown")

            self.connected = True
            self.connection_var.set(f"Connected to {self.port_var.get()} @ {self.baud_var.get()}")
            self.status_var.set("Connected")
            self._set_controls_state(True)
            self.center_sliders(send_command=False)
            self._poll_telemetry()
        except Exception as exc:
            if self.controller is not None:
                try:
                    self.controller.close()
                except Exception:
                    pass
                self.controller = None
            self.connected = False
            self.connection_var.set("Disconnected")
            self.status_var.set(f"Connection failed: {exc}")
            self._set_controls_state(False)
            messagebox.showerror("Connection error", str(exc))

    def disconnect(self, show_status: bool = True) -> None:
        if self.controller is not None:
            try:
                self._safe_stop()
                self.controller.close()
            except Exception as exc:
                logger.warning("Error while disconnecting: %s", exc)
        self.controller = None
        self.connected = False
        self.connection_var.set("Disconnected")
        if show_status:
            self.status_var.set("Disconnected")
        self._set_controls_state(False)

    def _safe_stop(self) -> None:
        if self.controller and self.connected:
            try:
                self.controller.DutyM1M2(self._parse_address(), 0, 0)
            except Exception as exc:
                logger.warning("Failed to stop motors cleanly: %s", exc)

    def _apply_entry(self, motor: str) -> None:
        variable = self.m1_var if motor == "Motor 1" else self.m2_var
        value = max(Basicmicro.MIN_DUTY, min(Basicmicro.MAX_DUTY, int(variable.get())))
        variable.set(value)
        scale = self.motor_1_scale if motor == "Motor 1" else self.motor_2_scale
        scale.set(value)
        if motor == "Motor 1":
            self._send_duty(1, value)
        else:
            self._send_duty(2, value)

    def _on_m1_slider(self, raw_value: str) -> None:
        value = int(float(raw_value))
        self.m1_var.set(value)
        self._send_duty(1, value)

    def _on_m2_slider(self, raw_value: str) -> None:
        value = int(float(raw_value))
        self.m2_var.set(value)
        self._send_duty(2, value)

    def _send_duty(self, motor: int, value: int) -> None:
        if not self.connected or self.controller is None:
            return

        try:
            address = self._parse_address()
            if motor == 1:
                self.controller.DutyM1(address, value)
            else:
                self.controller.DutyM2(address, value)
            self.status_var.set(f"Motor {motor} duty set to {value}")
        except Exception as exc:
            self.status_var.set(f"Failed to set Motor {motor} duty: {exc}")

    def stop_m1(self) -> None:
        self.motor_1_scale.set(0)
        self.m1_var.set(0)
        self._send_duty(1, 0)

    def stop_m2(self) -> None:
        self.motor_2_scale.set(0)
        self.m2_var.set(0)
        self._send_duty(2, 0)

    def stop_all(self) -> None:
        self.center_sliders(send_command=False)
        if self.controller and self.connected:
            try:
                self.controller.DutyM1M2(self._parse_address(), 0, 0)
                self.status_var.set("All motors stopped")
            except Exception as exc:
                self.status_var.set(f"Failed to stop motors: {exc}")

    def center_sliders(self, send_command: bool = True) -> None:
        self.m1_var.set(0)
        self.m2_var.set(0)
        self.motor_1_scale.set(0)
        self.motor_2_scale.set(0)
        if send_command:
            self.stop_all()

    def reset_encoders(self) -> None:
        if not self.controller or not self.connected:
            return
        try:
            ok = self.controller.ResetEncoders(self._parse_address())
            self.status_var.set("Encoders reset" if ok else "Encoder reset command failed")
        except Exception as exc:
            self.status_var.set(f"Failed to reset encoders: {exc}")

    def _poll_telemetry(self) -> None:
        if not self.connected or self.controller is None:
            return

        try:
            address = self._parse_address()

            enc1 = self.controller.ReadEncM1(address)
            enc2 = self.controller.ReadEncM2(address)
            speed1 = self.controller.ReadSpeedM1(address)
            speed2 = self.controller.ReadSpeedM2(address)
            pwm = self.controller.ReadPWMs(address)
            currents = self.controller.ReadCurrents(address)
            main_batt = self.controller.ReadMainBatteryVoltage(address)
            logic_batt = self.controller.ReadLogicBatteryVoltage(address)
            temp1 = self.controller.ReadTemp(address)
            temp2 = self.controller.ReadTemp2(address)
            error = self.controller.ReadError(address)

            self.telemetry_vars["enc1"].set(str(enc1[1]) if enc1[0] else "Read error")
            self.telemetry_vars["enc2"].set(str(enc2[1]) if enc2[0] else "Read error")
            self.telemetry_vars["speed1"].set(f"{speed1[1]} counts/s" if speed1[0] else "Read error")
            self.telemetry_vars["speed2"].set(f"{speed2[1]} counts/s" if speed2[0] else "Read error")
            self.telemetry_vars["pwm1"].set(
                f"{pwm[1]} ({(pwm[1] / Basicmicro.MAX_DUTY) * 100:.1f}%)" if pwm[0] else "Read error"
            )
            self.telemetry_vars["pwm2"].set(
                f"{pwm[2]} ({(pwm[2] / Basicmicro.MAX_DUTY) * 100:.1f}%)" if pwm[0] else "Read error"
            )
            self.telemetry_vars["current1"].set(f"{currents[1]} mA" if currents[0] else "Read error")
            self.telemetry_vars["current2"].set(f"{currents[2]} mA" if currents[0] else "Read error")
            self.telemetry_vars["main_batt"].set(
                f"{main_batt[1] / Basicmicro.VOLTAGE_SCALE:.1f} V" if main_batt[0] else "Read error"
            )
            self.telemetry_vars["logic_batt"].set(
                f"{logic_batt[1] / Basicmicro.VOLTAGE_SCALE:.1f} V" if logic_batt[0] else "Read error"
            )
            self.telemetry_vars["temp1"].set(f"{temp1[1] / Basicmicro.TEMP_SCALE:.1f} C" if temp1[0] else "Read error")
            self.telemetry_vars["temp2"].set(f"{temp2[1] / Basicmicro.TEMP_SCALE:.1f} C" if temp2[0] else "Read error")
            self.telemetry_vars["error"].set(f"0x{error[1]:08X}" if error[0] else "Read error")
        except Exception as exc:
            self.status_var.set(f"Telemetry read failed: {exc}")

        self.root.after(self.POLL_MS, self._poll_telemetry)

    def _on_close(self) -> None:
        self.disconnect(show_status=False)
        self.root.destroy()


def main() -> None:
    parser = argparse.ArgumentParser(description="Basicmicro duty cycle dashboard")
    parser.add_argument("-p", "--port", type=str, default="/dev/ttyACM0", help="Serial port (default: /dev/ttyACM0)")
    parser.add_argument("-b", "--baud", type=int, default=115200, help="Baud rate (default: 115200)")
    parser.add_argument(
        "-a",
        "--address",
        type=lambda x: int(x, 0),
        default=0x80,
        help="Controller address (default: 0x80)",
    )
    args = parser.parse_args()

    root = tk.Tk()
    style = ttk.Style()
    if "clam" in style.theme_names():
        style.theme_use("clam")
    DutyCycleDashboard(root, args.port, args.baud, args.address)
    root.mainloop()


if __name__ == "__main__":
    main()
