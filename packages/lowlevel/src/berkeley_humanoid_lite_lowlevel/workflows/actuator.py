from __future__ import annotations

import json
import math
from pathlib import Path

from berkeley_humanoid_lite_lowlevel.actuator import (
    DEFAULT_ACTUATOR_BITRATE,
    apply_actuator_parameter_overrides,
    calibrate_actuator_electrical_offset,
    create_actuator_bus,
    ping_actuator,
    read_actuator_configuration,
    resolve_angle_radians,
    run_actuator_angle_sequence,
    run_actuator_sine_motion,
)
from berkeley_humanoid_lite_lowlevel.runtime_paths import ensure_parent_directory


def export_actuator_configuration(
    *,
    channel: str,
    device_id: int,
    output_path: str | Path,
    bitrate: int = DEFAULT_ACTUATOR_BITRATE,
) -> Path:
    bus = create_actuator_bus(channel=channel, bitrate=bitrate)
    try:
        configuration = read_actuator_configuration(bus, device_id)
    finally:
        bus.stop()

    resolved_output_path = ensure_parent_directory(output_path)
    with open(resolved_output_path, "w", encoding="utf-8") as file:
        json.dump(configuration, file, indent=4)
    return resolved_output_path


def configure_actuator(
    *,
    channel: str,
    device_id: int,
    bitrate: int = DEFAULT_ACTUATOR_BITRATE,
    fast_frame_frequency: int | None = None,
    gear_ratio: float | None = None,
    position_kp: float | None = None,
    position_kd: float | None = None,
    torque_limit: float | None = None,
    phase_order: int | None = None,
    position_limit_lower: float | None = None,
    position_limit_upper: float | None = None,
    current_bandwidth_hz: float | None = None,
    phase_resistance: float | None = None,
    phase_inductance: float | None = None,
    store_to_flash: bool = False,
) -> None:
    bus = create_actuator_bus(channel=channel, bitrate=bitrate)
    try:
        apply_actuator_parameter_overrides(
            bus,
            device_id,
            fast_frame_frequency=fast_frame_frequency,
            gear_ratio=gear_ratio,
            position_kp=position_kp,
            position_kd=position_kd,
            torque_limit=torque_limit,
            phase_order=phase_order,
            position_limit_lower=position_limit_lower,
            position_limit_upper=position_limit_upper,
            current_bandwidth_hz=current_bandwidth_hz,
            phase_resistance=phase_resistance,
            phase_inductance=phase_inductance,
            store_to_flash=store_to_flash,
        )
    finally:
        bus.stop()


def check_actuator_connection(
    *,
    channel: str,
    device_id: int,
    bitrate: int = DEFAULT_ACTUATOR_BITRATE,
) -> bool:
    bus = create_actuator_bus(channel=channel, bitrate=bitrate)
    try:
        return ping_actuator(bus, device_id)
    finally:
        bus.stop()


def run_actuator_calibration(
    *,
    channel: str,
    device_id: int,
    bitrate: int = DEFAULT_ACTUATOR_BITRATE,
    wait_seconds: float = 20.0,
) -> None:
    bus = create_actuator_bus(channel=channel, bitrate=bitrate)
    try:
        calibrate_actuator_electrical_offset(bus, device_id, wait_seconds=wait_seconds)
    finally:
        bus.stop()


def run_actuator_motion_demo(
    *,
    channel: str,
    device_id: int,
    bitrate: int = DEFAULT_ACTUATOR_BITRATE,
    position_kp: float = 0.2,
    position_kd: float = 0.005,
    torque_limit: float = 0.2,
    motion_frequency_hz: float = 1.0,
    motion_amplitude_radians: float = 1.0,
    control_frequency_hz: float = 200.0,
) -> None:
    bus = create_actuator_bus(channel=channel, bitrate=bitrate)
    try:
        run_actuator_sine_motion(
            bus,
            device_id,
            position_kp=position_kp,
            position_kd=position_kd,
            torque_limit=torque_limit,
            motion_frequency_hz=motion_frequency_hz,
            motion_amplitude_radians=motion_amplitude_radians,
            control_frequency_hz=control_frequency_hz,
        )
    finally:
        bus.stop()


def run_actuator_angle_test(
    *,
    channel: str,
    device_id: int,
    bitrate: int = DEFAULT_ACTUATOR_BITRATE,
    target_angle_radians: float | None = None,
    target_angle_degrees: float | None = None,
    return_angle_radians: float | None = None,
    return_angle_degrees: float | None = None,
    position_kp: float = 0.2,
    position_kd: float = 0.005,
    torque_limit: float = 0.2,
    max_speed_degrees_per_second: float = 30.0,
    hold_seconds: float = 2.0,
    control_frequency_hz: float = 200.0,
    position_tolerance_degrees: float = 2.0,
    velocity_tolerance_degrees_per_second: float = 10.0,
    settle_timeout_seconds: float = 2.0,
    required_stable_samples: int = 20,
    cycles: int | None = None,
) -> None:
    target_angle_radians = resolve_angle_radians(target_angle_radians, target_angle_degrees)
    if target_angle_radians is None:
        raise ValueError("A target angle must be provided")

    return_angle_radians = resolve_angle_radians(return_angle_radians, return_angle_degrees)
    if cycles is None and return_angle_radians is not None:
        raise ValueError("--return-rad/--return-deg requires --cycles")

    if max_speed_degrees_per_second <= 0.0:
        raise ValueError("--max-speed-deg must be positive")
    if hold_seconds < 0.0:
        raise ValueError("--hold-seconds must be non-negative")
    if control_frequency_hz <= 0.0:
        raise ValueError("--control-rate must be positive")
    if position_tolerance_degrees <= 0.0:
        raise ValueError("--position-tolerance-deg must be positive")
    if velocity_tolerance_degrees_per_second <= 0.0:
        raise ValueError("--velocity-tolerance-deg must be positive")
    if settle_timeout_seconds <= 0.0:
        raise ValueError("--settle-timeout-seconds must be positive")
    if required_stable_samples <= 0:
        raise ValueError("--stable-samples must be positive")

    max_speed_radians_per_second = math.radians(max_speed_degrees_per_second)
    position_tolerance_radians = math.radians(position_tolerance_degrees)
    velocity_tolerance_radians_per_second = math.radians(velocity_tolerance_degrees_per_second)
    resolved_return_angle = 0.0 if return_angle_radians is None else return_angle_radians

    print(f"Actuator #{device_id} on {channel}")
    print(
        f"Target angle:  {target_angle_radians:.4f} rad "
        f"({math.degrees(target_angle_radians):.2f} deg)"
    )
    if cycles is not None:
        print(f"Cycles:        {cycles}")
        print(
            f"Return angle:  {resolved_return_angle:.4f} rad "
            f"({math.degrees(resolved_return_angle):.2f} deg)"
        )
    print(
        f"Position tol:  {position_tolerance_degrees:.2f} deg"
    )
    print(
        f"Velocity tol:  {velocity_tolerance_degrees_per_second:.2f} deg/s"
    )
    print(f"Settle limit:  {settle_timeout_seconds:.2f} s")
    print(f"Stable count:  {required_stable_samples}")

    bus = create_actuator_bus(channel=channel, bitrate=bitrate)
    try:
        run_actuator_angle_sequence(
            bus,
            device_id,
            target_angle_radians=target_angle_radians,
            return_angle_radians=return_angle_radians,
            cycles=cycles,
            position_kp=position_kp,
            position_kd=position_kd,
            torque_limit=torque_limit,
            max_speed_radians_per_second=max_speed_radians_per_second,
            hold_seconds=hold_seconds,
            control_frequency_hz=control_frequency_hz,
            position_tolerance_radians=position_tolerance_radians,
            velocity_tolerance_radians_per_second=velocity_tolerance_radians_per_second,
            settle_timeout_seconds=settle_timeout_seconds,
            required_stable_samples=required_stable_samples,
        )
    finally:
        bus.stop()
