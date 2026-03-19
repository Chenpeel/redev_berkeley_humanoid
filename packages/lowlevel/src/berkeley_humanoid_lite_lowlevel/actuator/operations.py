from __future__ import annotations

import time

from loop_rate_limiters import RateLimiter
import numpy as np

import berkeley_humanoid_lite_lowlevel.recoil as recoil


DEFAULT_ACTUATOR_BITRATE = 1_000_000


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

    bus.write_position_kp(device_id, position_kp)
    bus.write_position_kd(device_id, position_kd)
    bus.write_torque_limit(device_id, torque_limit)
    bus.set_mode(device_id, recoil.Mode.POSITION)
    bus.feed(device_id)

    try:
        while True:
            target_angle = np.sin(2.0 * np.pi * motion_frequency_hz * time.time()) * motion_amplitude_radians
            measured_position, measured_velocity = bus.write_read_pdo_2(device_id, target_angle, 0.0)
            if measured_position is not None and measured_velocity is not None:
                print(f"Measured pos: {measured_position:.3f}\tvel: {measured_velocity:.3f}")
            rate.sleep()
    finally:
        bus.set_mode(device_id, recoil.Mode.IDLE)
