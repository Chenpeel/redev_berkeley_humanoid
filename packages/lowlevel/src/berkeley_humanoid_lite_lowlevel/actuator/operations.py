from __future__ import annotations

import math
import time

import numpy as np
from loop_rate_limiters import RateLimiter

import berkeley_humanoid_lite_lowlevel.recoil as recoil

DEFAULT_ACTUATOR_BITRATE = 1_000_000
ActuatorAngleStage = tuple[str, float]


def create_actuator_bus(channel: str, bitrate: int = DEFAULT_ACTUATOR_BITRATE) -> recoil.Bus:
    return recoil.Bus(channel=channel, bitrate=bitrate)


def ping_actuator(bus: recoil.Bus, device_id: int) -> bool:
    return bool(bus.ping(device_id))


def calibrate_actuator_electrical_offset(
    bus: recoil.Bus,
    device_id: int,
    *,
    wait_seconds: float = 20.0,
) -> None:
    bus.set_mode(device_id, recoil.Mode.CALIBRATION)
    time.sleep(wait_seconds)


def enter_actuator_position_mode(
    bus: recoil.Bus,
    device_id: int,
    *,
    position_kp: float | None = None,
    position_kd: float | None = None,
    torque_limit: float | None = None,
    parameter_apply_delay_seconds: float = 0.001,
) -> None:
    bus.set_mode(device_id, recoil.Mode.IDLE)
    time.sleep(parameter_apply_delay_seconds)

    if position_kp is not None:
        bus.write_position_kp(device_id, position_kp)
        time.sleep(parameter_apply_delay_seconds)
    if position_kd is not None:
        bus.write_position_kd(device_id, position_kd)
        time.sleep(parameter_apply_delay_seconds)
    if torque_limit is not None:
        bus.write_torque_limit(device_id, torque_limit)
        time.sleep(parameter_apply_delay_seconds)

    bus.feed(device_id)
    bus.set_mode(device_id, recoil.Mode.POSITION)


def interpolate_value(start: float, end: float, alpha: float) -> float:
    clamped_alpha = min(max(alpha, 0.0), 1.0)
    return start * (1.0 - clamped_alpha) + end * clamped_alpha


def resolve_angle_radians(
    angle_radians: float | None,
    angle_degrees: float | None,
    *,
    default: float | None = None,
) -> float | None:
    if angle_radians is not None:
        return angle_radians
    if angle_degrees is not None:
        return math.radians(angle_degrees)
    return default


def build_actuator_angle_sequence(
    target_angle_radians: float,
    return_angle_radians: float | None = None,
    cycles: int | None = None,
) -> list[ActuatorAngleStage]:
    if cycles is None:
        return [("target", target_angle_radians)]

    if cycles <= 0:
        raise ValueError("cycles must be a positive integer")

    resolved_return_angle = 0.0 if return_angle_radians is None else return_angle_radians
    sequence: list[ActuatorAngleStage] = []
    for cycle_index in range(cycles):
        cycle_name = f"cycle {cycle_index + 1}/{cycles}"
        sequence.append((f"{cycle_name}: target", target_angle_radians))
        sequence.append((f"{cycle_name}: return", resolved_return_angle))
    return sequence


def _print_motion_status(
    *,
    command_angle_radians: float,
    measured_position: float | None,
    measured_velocity: float | None,
) -> None:
    if measured_position is None or measured_velocity is None:
        return

    print(
        f"\rtarget={command_angle_radians:+.4f} rad  "
        f"measured={measured_position:+.4f} rad  "
        f"vel={measured_velocity:+.4f} rad/s",
        end="",
        flush=True,
    )


def move_actuator_to_angle(
    bus: recoil.Bus,
    rate: RateLimiter,
    device_id: int,
    *,
    target_angle_radians: float,
    max_speed_radians_per_second: float,
    hold_seconds: float,
    stage_name: str,
) -> None:
    current_angle_radians = bus.read_position_measured(device_id)
    if current_angle_radians is None:
        raise RuntimeError(f"Failed to read actuator #{device_id} position")

    ramp_duration_seconds = max(
        abs(target_angle_radians - current_angle_radians) / max_speed_radians_per_second,
        0.2,
    )

    print()
    print(f"[{stage_name}]")
    print(
        f"Current angle: {current_angle_radians:.4f} rad "
        f"({math.degrees(current_angle_radians):.2f} deg)"
    )
    print(
        f"Target angle:  {target_angle_radians:.4f} rad "
        f"({math.degrees(target_angle_radians):.2f} deg)"
    )
    print(f"Ramp duration: {ramp_duration_seconds:.2f} s")

    ramp_start_time = time.monotonic()
    while True:
        elapsed_seconds = time.monotonic() - ramp_start_time
        alpha = elapsed_seconds / ramp_duration_seconds
        command_angle_radians = interpolate_value(current_angle_radians, target_angle_radians, alpha)

        bus.feed(device_id)
        measured_position, measured_velocity = bus.write_read_pdo_2(device_id, command_angle_radians, 0.0)
        _print_motion_status(
            command_angle_radians=command_angle_radians,
            measured_position=measured_position,
            measured_velocity=measured_velocity,
        )

        if alpha >= 1.0:
            break

        rate.sleep()

    hold_start_time = time.monotonic()
    while time.monotonic() - hold_start_time < hold_seconds:
        bus.feed(device_id)
        measured_position, measured_velocity = bus.write_read_pdo_2(device_id, target_angle_radians, 0.0)
        _print_motion_status(
            command_angle_radians=target_angle_radians,
            measured_position=measured_position,
            measured_velocity=measured_velocity,
        )
        rate.sleep()

    print()


def run_actuator_angle_sequence(
    bus: recoil.Bus,
    device_id: int,
    *,
    target_angle_radians: float,
    return_angle_radians: float | None = None,
    cycles: int | None = None,
    position_kp: float | None = None,
    position_kd: float | None = None,
    torque_limit: float | None = None,
    max_speed_radians_per_second: float = math.radians(30.0),
    hold_seconds: float = 2.0,
    control_frequency_hz: float = 200.0,
) -> None:
    if max_speed_radians_per_second <= 0.0:
        raise ValueError("max_speed_radians_per_second must be positive")
    if hold_seconds < 0.0:
        raise ValueError("hold_seconds must be non-negative")
    if control_frequency_hz <= 0.0:
        raise ValueError("control_frequency_hz must be positive")

    sequence = build_actuator_angle_sequence(
        target_angle_radians=target_angle_radians,
        return_angle_radians=return_angle_radians,
        cycles=cycles,
    )
    rate = RateLimiter(frequency=control_frequency_hz)

    enter_actuator_position_mode(
        bus,
        device_id,
        position_kp=position_kp,
        position_kd=position_kd,
        torque_limit=torque_limit,
    )

    try:
        for stage_name, stage_target_radians in sequence:
            move_actuator_to_angle(
                bus,
                rate,
                device_id,
                target_angle_radians=stage_target_radians,
                max_speed_radians_per_second=max_speed_radians_per_second,
                hold_seconds=hold_seconds,
                stage_name=stage_name,
            )
    except KeyboardInterrupt:
        print("\nInterrupted, switching actuator back to idle.")
    finally:
        bus.set_mode(device_id, recoil.Mode.IDLE)


def run_actuator_sine_motion(
    bus: recoil.Bus,
    device_id: int,
    *,
    position_kp: float = 0.2,
    position_kd: float = 0.005,
    torque_limit: float = 0.2,
    motion_frequency_hz: float = 1.0,
    motion_amplitude_radians: float = 1.0,
    control_frequency_hz: float = 200.0,
) -> None:
    rate = RateLimiter(frequency=control_frequency_hz)

    enter_actuator_position_mode(
        bus,
        device_id,
        position_kp=position_kp,
        position_kd=position_kd,
        torque_limit=torque_limit,
    )

    try:
        while True:
            target_angle = np.sin(2.0 * np.pi * motion_frequency_hz * time.time()) * motion_amplitude_radians
            bus.feed(device_id)
            measured_position, measured_velocity = bus.write_read_pdo_2(device_id, target_angle, 0.0)
            if measured_position is not None and measured_velocity is not None:
                print(f"Measured pos: {measured_position:.3f}\tvel: {measured_velocity:.3f}")
            rate.sleep()
    finally:
        bus.set_mode(device_id, recoil.Mode.IDLE)
