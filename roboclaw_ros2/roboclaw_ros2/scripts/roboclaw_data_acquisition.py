#!/usr/bin/env python3
"""Standalone RoboClaw data acquisition for simple system identification."""

from __future__ import annotations

import argparse
import sys
import time
from collections import deque
from dataclasses import dataclass
from pathlib import Path

import numpy as np
from basicmicro import Basicmicro
from scipy.io import savemat


TS = 0.02
DEFAULT_LEVELS = (6.0, 12.0, 18.0, 24.0)
DEFAULT_HOLD_TIME = 4.0
DEFAULT_BATTERY_VOLTAGE = 24.0


@dataclass(frozen=True)
class ExperimentConfig:
    port: str
    baud: int
    address: int
    motor: int
    levels: tuple[float, float, float, float]
    hold_time: float
    fallback_battery_voltage: float
    output_dir: Path
    reset_encoders: bool
    filter_window: int


class RoboClawDataAcquisition:
    def __init__(self, config: ExperimentConfig) -> None:
        self.config = config
        self.controller = Basicmicro(config.port, config.baud)
        self.max_duty = int(getattr(Basicmicro, "MAX_DUTY", 32767))
        self.min_duty = int(getattr(Basicmicro, "MIN_DUTY", -32767))
        self.voltage_scale = float(getattr(Basicmicro, "VOLTAGE_SCALE", 10.0))
        self.last_battery_voltage = config.fallback_battery_voltage
        self.speed_filter: deque[float] = deque(maxlen=max(1, config.filter_window))
        self._warned_battery_fallback = False
        self._warned_saturation: set[float] = set()

    def connect(self) -> str:
        if not self.controller.Open():
            raise RuntimeError(f"Could not open serial port {self.config.port}")

        version = self.controller.ReadVersion(self.config.address)
        firmware = str(version[1]) if version[0] else "unknown"
        if self.config.reset_encoders and not self.controller.ResetEncoders(self.config.address):
            print("Warning: encoder reset command was not acknowledged.", file=sys.stderr)

        self._set_motor_duty(0)
        return firmware

    def close(self) -> None:
        try:
            self._set_motor_duty(0)
        except Exception:
            pass

        try:
            self.controller.close()
        except Exception:
            pass

    def run(self) -> tuple[np.ndarray, np.ndarray]:
        samples_per_level = max(1, int(round(self.config.hold_time / TS)))
        total_samples = len(self.config.levels) * samples_per_level
        effective_hold_time = samples_per_level * TS

        print(
            f"Experiment on motor M{self.config.motor} | Ts={TS:.3f} s | "
            f"{samples_per_level} samples/level ({effective_hold_time:.2f} s)"
        )
        print(f"Voltage levels: {', '.join(f'{level:.2f} V' for level in self.config.levels)}")

        previous_ticks = self._read_encoder_ticks()
        current_duty, applied_voltage = self._voltage_to_duty(self.config.levels[0])
        self._set_motor_duty(current_duty)

        u_samples: list[float] = []
        y_samples: list[float] = []

        start_time = time.perf_counter()
        for sample_idx in range(total_samples):
            deadline = start_time + (sample_idx + 1) * TS
            sleep_time = deadline - time.perf_counter()
            if sleep_time > 0.0:
                time.sleep(sleep_time)

            # Sample y(k) at the boundary and keep u(k) as the duty held over the
            # preceding interval, so both signals share the same sample index.
            current_ticks = self._read_encoder_ticks()
            raw_speed = (current_ticks - previous_ticks) / TS
            previous_ticks = current_ticks

            self.speed_filter.append(raw_speed)
            filtered_speed = sum(self.speed_filter) / len(self.speed_filter)

            u_samples.append(applied_voltage)
            y_samples.append(filtered_speed)

            if sample_idx == total_samples - 1:
                continue

            next_level_idx = (sample_idx + 1) // samples_per_level
            next_voltage = self.config.levels[next_level_idx]
            next_duty, next_applied_voltage = self._voltage_to_duty(next_voltage)

            if next_duty != current_duty:
                self._set_motor_duty(next_duty)

            current_duty = next_duty
            applied_voltage = next_applied_voltage

            if (sample_idx + 1) % samples_per_level == 0:
                completed_level_idx = (sample_idx + 1) // samples_per_level
                completed_voltage = self.config.levels[completed_level_idx - 1]
                print(
                    f"Completed level {completed_level_idx}/{len(self.config.levels)} "
                    f"at {completed_voltage:.2f} V"
                )

        self._set_motor_duty(0)
        return self._to_column_vector(u_samples), self._to_column_vector(y_samples)

    def save(self, u: np.ndarray, y: np.ndarray) -> None:
        self.config.output_dir.mkdir(parents=True, exist_ok=True)
        savemat(self.config.output_dir / "u.mat", {"u": u})
        savemat(self.config.output_dir / "y.mat", {"y": y})

    def _to_column_vector(self, samples: list[float]) -> np.ndarray:
        return np.asarray(samples, dtype=np.float64).reshape(-1, 1)

    def _read_encoder_ticks(self) -> int:
        if self.config.motor == 1:
            result = self.controller.ReadEncM1(self.config.address)
        else:
            result = self.controller.ReadEncM2(self.config.address)

        if not result[0]:
            raise RuntimeError(f"Failed to read encoder M{self.config.motor}")

        return int(result[1])

    def _read_battery_voltage(self) -> float:
        try:
            result = self.controller.ReadMainBatteryVoltage(self.config.address)
            if result[0] and result[1] > 0:
                self.last_battery_voltage = float(result[1]) / self.voltage_scale
                return self.last_battery_voltage
        except Exception:
            pass

        if not self._warned_battery_fallback:
            print(
                f"Warning: could not read battery voltage, using {self.last_battery_voltage:.2f} V.",
                file=sys.stderr,
            )
            self._warned_battery_fallback = True

        return self.last_battery_voltage

    def _voltage_to_duty(self, target_voltage: float) -> tuple[int, float]:
        battery_voltage = self._read_battery_voltage()
        if battery_voltage <= 0.0:
            battery_voltage = self.config.fallback_battery_voltage

        duty_ratio = target_voltage / battery_voltage
        saturated = False
        if duty_ratio > 1.0:
            duty_ratio = 1.0
            saturated = True
        elif duty_ratio < -1.0:
            duty_ratio = -1.0
            saturated = True

        duty_command = int(round(duty_ratio * self.max_duty))
        duty_command = max(self.min_duty, min(self.max_duty, duty_command))
        applied_duty_ratio = duty_command / self.max_duty
        applied_voltage = battery_voltage * applied_duty_ratio

        if saturated and target_voltage not in self._warned_saturation:
            print(
                f"Warning: requested {target_voltage:.2f} V exceeds available battery "
                f"{battery_voltage:.2f} V. Command saturated.",
                file=sys.stderr,
            )
            self._warned_saturation.add(target_voltage)

        return duty_command, applied_voltage

    def _set_motor_duty(self, duty_command: int) -> None:
        duty_command = max(self.min_duty, min(self.max_duty, int(duty_command)))

        if self.config.motor == 1:
            ok = self.controller.DutyM1(self.config.address, duty_command)
        else:
            ok = self.controller.DutyM2(self.config.address, duty_command)

        if not ok:
            raise RuntimeError(
                f"Failed to send duty command {duty_command} to M{self.config.motor}"
            )


def parse_args() -> ExperimentConfig:
    parser = argparse.ArgumentParser(
        description="Acquire synchronized input/output data from a RoboClaw-driven DC motor."
    )
    parser.add_argument(
        "--port",
        default="/dev/ttyACM0",
        help="Serial port used by the RoboClaw (default: /dev/ttyACM0).",
    )
    parser.add_argument(
        "--baud",
        type=int,
        default=115200,
        help="Serial baud rate (default: 115200).",
    )
    parser.add_argument(
        "--address",
        type=lambda value: int(value, 0),
        default=0x80,
        help="RoboClaw address in decimal or hex (default: 0x80).",
    )
    parser.add_argument(
        "--motor",
        type=int,
        choices=(1, 2),
        default=1,
        help="Motor channel to test: 1 for M1 or 2 for M2 (default: 1).",
    )
    parser.add_argument(
        "--hold-time",
        type=float,
        default=DEFAULT_HOLD_TIME,
        help="Seconds to hold each voltage level (default: 4.0).",
    )
    parser.add_argument(
        "--levels",
        type=float,
        nargs=4,
        default=DEFAULT_LEVELS,
        metavar=("V1", "V2", "V3", "V4"),
        help="Exactly four voltage levels in volts. Use negative values for reverse motion.",
    )
    parser.add_argument(
        "--fallback-battery",
        type=float,
        default=DEFAULT_BATTERY_VOLTAGE,
        help="Battery voltage used when RoboClaw telemetry is unavailable (default: 24.0).",
    )
    parser.add_argument(
        "--filter-window",
        type=int,
        default=1,
        help="Moving-average window for velocity in samples (default: 1, disabled).",
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=Path.cwd(),
        help="Directory where u.mat and y.mat will be saved (default: current directory).",
    )
    parser.add_argument(
        "--no-reset-encoders",
        action="store_true",
        help="Do not reset encoders before the experiment.",
    )
    args = parser.parse_args()

    if args.hold_time <= 0.0:
        parser.error("--hold-time must be greater than zero.")
    if args.filter_window <= 0:
        parser.error("--filter-window must be greater than zero.")
    if args.fallback_battery <= 0.0:
        parser.error("--fallback-battery must be greater than zero.")

    return ExperimentConfig(
        port=args.port,
        baud=args.baud,
        address=args.address,
        motor=args.motor,
        levels=tuple(float(level) for level in args.levels),
        hold_time=float(args.hold_time),
        fallback_battery_voltage=float(args.fallback_battery),
        output_dir=args.output_dir.expanduser().resolve(),
        reset_encoders=not args.no_reset_encoders,
        filter_window=int(args.filter_window),
    )


def main() -> int:
    config = parse_args()
    acquisition = RoboClawDataAcquisition(config)

    try:
        firmware = acquisition.connect()
        print(f"Connected to RoboClaw firmware: {firmware}")
        u_data, y_data = acquisition.run()
        acquisition.save(u_data, y_data)
        print(f"Saved {u_data.shape[0]} samples to {config.output_dir / 'u.mat'}")
        print(f"Saved {y_data.shape[0]} samples to {config.output_dir / 'y.mat'}")
        return 0
    except KeyboardInterrupt:
        print("Experiment interrupted by user.", file=sys.stderr)
        return 130
    finally:
        acquisition.close()


if __name__ == "__main__":
    raise SystemExit(main())
