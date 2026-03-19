from __future__ import annotations

import json
from pathlib import Path

from berkeley_humanoid_lite_lowlevel.actuator import (
    DEFAULT_ACTUATOR_BITRATE,
    apply_actuator_parameter_overrides,
    calibrate_actuator_electrical_offset,
    create_actuator_bus,
    ping_actuator,
    read_actuator_configuration,
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
