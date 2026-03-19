from __future__ import annotations

from typing import Any

import berkeley_humanoid_lite_lowlevel.recoil as recoil


def read_actuator_configuration(bus: recoil.Bus, device_id: int) -> dict[str, Any]:
    configuration: dict[str, Any] = {
        "position_controller": {},
        "current_controller": {},
        "powerstage": {},
        "motor": {},
        "encoder": {},
    }

    configuration["device_id"] = bus._read_parameter_u32(device_id, recoil.Parameter.DEVICE_ID)
    configuration["firmware_version"] = hex(bus._read_parameter_u32(device_id, recoil.Parameter.FIRMWARE_VERSION))
    configuration["watchdog_timeout"] = bus._read_parameter_u32(device_id, recoil.Parameter.WATCHDOG_TIMEOUT)
    configuration["fast_frame_frequency"] = bus.read_fast_frame_frequency(device_id)

    configuration["position_controller"]["gear_ratio"] = bus._read_parameter_f32(
        device_id,
        recoil.Parameter.POSITION_CONTROLLER_GEAR_RATIO,
    )
    configuration["position_controller"]["position_kp"] = bus.read_position_kp(device_id)
    configuration["position_controller"]["position_ki"] = bus.read_position_ki(device_id)
    configuration["position_controller"]["velocity_kp"] = bus.read_velocity_kp(device_id)
    configuration["position_controller"]["velocity_ki"] = bus.read_velocity_ki(device_id)
    configuration["position_controller"]["torque_limit"] = bus.read_torque_limit(device_id)
    configuration["position_controller"]["velocity_limit"] = bus.read_velocity_limit(device_id)
    configuration["position_controller"]["position_limit_upper"] = bus.read_position_limit_upper(device_id)
    configuration["position_controller"]["position_limit_lower"] = bus.read_position_limit_lower(device_id)
    configuration["position_controller"]["position_offset"] = bus.read_position_offset(device_id)
    configuration["position_controller"]["torque_filter_alpha"] = bus.read_torque_filter_alpha(device_id)

    configuration["current_controller"]["i_limit"] = bus.read_current_limit(device_id)
    configuration["current_controller"]["i_kp"] = bus.read_current_kp(device_id)
    configuration["current_controller"]["i_ki"] = bus.read_current_ki(device_id)

    configuration["powerstage"]["undervoltage_threshold"] = bus._read_parameter_f32(
        device_id,
        recoil.Parameter.POWERSTAGE_UNDERVOLTAGE_THRESHOLD,
    )
    configuration["powerstage"]["overvoltage_threshold"] = bus._read_parameter_f32(
        device_id,
        recoil.Parameter.POWERSTAGE_OVERVOLTAGE_THRESHOLD,
    )
    configuration["powerstage"]["bus_voltage_filter_alpha"] = bus.read_bus_voltage_filter_alpha(device_id)

    configuration["motor"]["pole_pairs"] = bus.read_motor_pole_pairs(device_id)
    configuration["motor"]["torque_constant"] = bus.read_motor_torque_constant(device_id)
    configuration["motor"]["phase_order"] = bus.read_motor_phase_order(device_id)
    configuration["motor"]["max_calibration_current"] = bus.read_motor_calibration_current(device_id)

    configuration["encoder"]["cpr"] = bus.read_encoder_cpr(device_id)
    configuration["encoder"]["position_offset"] = bus.read_encoder_position_offset(device_id)
    configuration["encoder"]["velocity_filter_alpha"] = bus.read_encoder_velocity_filter_alpha(device_id)
    configuration["encoder"]["flux_offset"] = bus.read_encoder_flux_offset(device_id)

    return configuration


def apply_actuator_parameter_overrides(
    bus: recoil.Bus,
    device_id: int,
    *,
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
    if fast_frame_frequency is not None:
        print("Rate (before):\t", bus._read_parameter_u32(device_id, recoil.Parameter.FAST_FRAME_FREQUENCY))
        bus._write_parameter_u32(device_id, recoil.Parameter.FAST_FRAME_FREQUENCY, fast_frame_frequency)
        print("Rate (updated):\t", bus._read_parameter_u32(device_id, recoil.Parameter.FAST_FRAME_FREQUENCY))

    if gear_ratio is not None:
        print(
            "Gear Ratio (before):\t",
            bus._read_parameter_f32(device_id, recoil.Parameter.POSITION_CONTROLLER_GEAR_RATIO),
        )
        bus._write_parameter_f32(device_id, recoil.Parameter.POSITION_CONTROLLER_GEAR_RATIO, -gear_ratio)
        print(
            "Gear Ratio (updated):\t",
            bus._read_parameter_f32(device_id, recoil.Parameter.POSITION_CONTROLLER_GEAR_RATIO),
        )

    if position_kp is not None:
        print(
            "Kp (before):\t",
            bus._read_parameter_f32(device_id, recoil.Parameter.POSITION_CONTROLLER_POSITION_KP),
        )
        bus._write_parameter_f32(device_id, recoil.Parameter.POSITION_CONTROLLER_POSITION_KP, position_kp)
        print(
            "Kp (updated):\t",
            bus._read_parameter_f32(device_id, recoil.Parameter.POSITION_CONTROLLER_POSITION_KP),
        )

    if position_kd is not None:
        print(
            "Kd (before):\t",
            bus._read_parameter_f32(device_id, recoil.Parameter.POSITION_CONTROLLER_VELOCITY_KP),
        )
        bus._write_parameter_f32(device_id, recoil.Parameter.POSITION_CONTROLLER_VELOCITY_KP, position_kd)
        print(
            "Kd (updated):\t",
            bus._read_parameter_f32(device_id, recoil.Parameter.POSITION_CONTROLLER_VELOCITY_KP),
        )

    if torque_limit is not None:
        print(
            "Torque Limit (before):\t",
            bus._read_parameter_f32(device_id, recoil.Parameter.POSITION_CONTROLLER_TORQUE_LIMIT),
        )
        bus._write_parameter_f32(device_id, recoil.Parameter.POSITION_CONTROLLER_TORQUE_LIMIT, torque_limit)
        print(
            "Torque Limit (updated):\t",
            bus._read_parameter_f32(device_id, recoil.Parameter.POSITION_CONTROLLER_TORQUE_LIMIT),
        )

    if phase_order is not None:
        print("Phase order (before):\t", bus._read_parameter_i32(device_id, recoil.Parameter.MOTOR_PHASE_ORDER))
        bus._write_parameter_i32(device_id, recoil.Parameter.MOTOR_PHASE_ORDER, phase_order)
        print("Phase order (updated):\t", bus._read_parameter_i32(device_id, recoil.Parameter.MOTOR_PHASE_ORDER))

    if position_limit_lower is not None:
        print(
            "Position Lower Limit (before):\t",
            bus._read_parameter_f32(device_id, recoil.Parameter.POSITION_CONTROLLER_POSITION_LIMIT_LOWER),
        )
        bus._write_parameter_f32(
            device_id,
            recoil.Parameter.POSITION_CONTROLLER_POSITION_LIMIT_LOWER,
            position_limit_lower,
        )
        print(
            "Position Lower Limit (updated):\t",
            bus._read_parameter_f32(device_id, recoil.Parameter.POSITION_CONTROLLER_POSITION_LIMIT_LOWER),
        )

    if position_limit_upper is not None:
        print(
            "Position Upper Limit (before):\t",
            bus._read_parameter_f32(device_id, recoil.Parameter.POSITION_CONTROLLER_POSITION_LIMIT_UPPER),
        )
        bus._write_parameter_f32(
            device_id,
            recoil.Parameter.POSITION_CONTROLLER_POSITION_LIMIT_UPPER,
            position_limit_upper,
        )
        print(
            "Position Upper Limit (updated):\t",
            bus._read_parameter_f32(device_id, recoil.Parameter.POSITION_CONTROLLER_POSITION_LIMIT_UPPER),
        )

    if current_bandwidth_hz is not None:
        if phase_resistance is None or phase_inductance is None:
            raise ValueError("配置 current_bandwidth_hz 时必须同时提供 phase_resistance 和 phase_inductance。")
        print("Previous current kp:\t", bus._read_parameter_f32(device_id, recoil.Parameter.CURRENT_CONTROLLER_I_KP))
        print("Previous current ki:\t", bus._read_parameter_f32(device_id, recoil.Parameter.CURRENT_CONTROLLER_I_KI))
        bus.set_current_bandwidth(device_id, current_bandwidth_hz, phase_resistance, phase_inductance)
        print("New current kp:\t", bus._read_parameter_f32(device_id, recoil.Parameter.CURRENT_CONTROLLER_I_KP))
        print("New current ki:\t", bus._read_parameter_f32(device_id, recoil.Parameter.CURRENT_CONTROLLER_I_KI))

    if store_to_flash:
        bus.store_settings_to_flash(device_id)
        print("Settings stored to flash!")
