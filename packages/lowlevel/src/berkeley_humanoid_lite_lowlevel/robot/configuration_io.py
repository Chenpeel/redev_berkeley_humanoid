from __future__ import annotations

import time
from typing import Any

import berkeley_humanoid_lite_lowlevel.recoil as recoil

from .locomotion_runtime import LocomotionRobot


def read_robot_configuration(robot: LocomotionRobot) -> dict[str, dict[str, Any]]:
    robot_configuration: dict[str, dict[str, Any]] = {}

    for joint in robot.joint_interfaces:
        bus = joint.bus
        joint_id = joint.device_id
        joint_name = joint.joint_name

        print(f"Reading configuration for {joint_name}")

        config: dict[str, Any] = {
            "position_controller": {},
            "current_controller": {},
            "powerstage": {},
            "motor": {},
            "encoder": {},
        }

        config["device_id"] = bus._read_parameter_u32(joint_id, recoil.Parameter.DEVICE_ID)
        config["firmware_version"] = hex(bus._read_parameter_u32(joint_id, recoil.Parameter.FIRMWARE_VERSION))
        config["watchdog_timeout"] = bus._read_parameter_u32(joint_id, recoil.Parameter.WATCHDOG_TIMEOUT)
        config["fast_frame_frequency"] = bus.read_fast_frame_frequency(joint_id)

        config["position_controller"]["gear_ratio"] = bus._read_parameter_f32(joint_id, recoil.Parameter.POSITION_CONTROLLER_GEAR_RATIO)
        config["position_controller"]["position_kp"] = bus.read_position_kp(joint_id)
        config["position_controller"]["position_ki"] = bus.read_position_ki(joint_id)
        config["position_controller"]["velocity_kp"] = bus.read_velocity_kp(joint_id)
        config["position_controller"]["velocity_ki"] = bus.read_velocity_ki(joint_id)
        config["position_controller"]["torque_limit"] = bus.read_torque_limit(joint_id)
        config["position_controller"]["velocity_limit"] = bus.read_velocity_limit(joint_id)
        config["position_controller"]["position_limit_upper"] = bus.read_position_limit_upper(joint_id)
        config["position_controller"]["position_limit_lower"] = bus.read_position_limit_lower(joint_id)
        config["position_controller"]["position_offset"] = bus.read_position_offset(joint_id)
        config["position_controller"]["torque_filter_alpha"] = bus.read_torque_filter_alpha(joint_id)

        config["current_controller"]["i_limit"] = bus.read_current_limit(joint_id)
        config["current_controller"]["i_kp"] = bus.read_current_kp(joint_id)
        config["current_controller"]["i_ki"] = bus.read_current_ki(joint_id)

        config["powerstage"]["undervoltage_threshold"] = bus._read_parameter_f32(joint_id, recoil.Parameter.POWERSTAGE_UNDERVOLTAGE_THRESHOLD)
        config["powerstage"]["overvoltage_threshold"] = bus._read_parameter_f32(joint_id, recoil.Parameter.POWERSTAGE_OVERVOLTAGE_THRESHOLD)
        config["powerstage"]["bus_voltage_filter_alpha"] = bus.read_bus_voltage_filter_alpha(joint_id)

        config["motor"]["pole_pairs"] = bus.read_motor_pole_pairs(joint_id)
        config["motor"]["torque_constant"] = bus.read_motor_torque_constant(joint_id)
        config["motor"]["phase_order"] = bus.read_motor_phase_order(joint_id)
        config["motor"]["max_calibration_current"] = bus.read_motor_calibration_current(joint_id)

        config["encoder"]["cpr"] = bus.read_encoder_cpr(joint_id)
        config["encoder"]["position_offset"] = bus.read_encoder_position_offset(joint_id)
        config["encoder"]["velocity_filter_alpha"] = bus.read_encoder_velocity_filter_alpha(joint_id)
        config["encoder"]["flux_offset"] = bus.read_encoder_flux_offset(joint_id)

        robot_configuration[joint_name] = config
        time.sleep(0.1)

    return robot_configuration


def write_robot_configuration(
    robot: LocomotionRobot,
    robot_configuration: dict[str, dict[str, Any]],
    *,
    store_to_flash: bool = True,
    write_delay_seconds: float = 0.1,
) -> None:
    for joint in robot.joint_interfaces:
        bus = joint.bus
        joint_id = joint.device_id
        joint_name = joint.joint_name

        print(f"Pinging {joint_name} ... ", end="\t")
        result = bus.ping(joint_id)
        print(f"success: {result}")

        firmware_version = hex(bus._read_parameter_u32(joint_id, recoil.Parameter.FIRMWARE_VERSION))
        print(f"  firmware version: {firmware_version}")
        time.sleep(0.1)

    for joint in robot.joint_interfaces:
        bus = joint.bus
        joint_id = joint.device_id
        joint_name = joint.joint_name

        print(f"Writing configuration for {joint_name}")

        config = robot_configuration.get(joint_name)
        if config is None:
            raise ValueError(f"No configuration found for {joint_name}")

        value = config["fast_frame_frequency"]
        print(f" setting fast frame frequency to {value}")
        bus.write_fast_frame_frequency(joint_id, value)
        time.sleep(write_delay_seconds)

        value = config["position_controller"]["gear_ratio"]
        print(f" setting gear ratio to {value}")
        bus._write_parameter_f32(joint_id, recoil.Parameter.POSITION_CONTROLLER_GEAR_RATIO, value)
        time.sleep(write_delay_seconds)

        value = config["position_controller"]["position_kp"]
        print(f" setting KP to {value}")
        bus.write_position_kp(joint_id, value)
        time.sleep(write_delay_seconds)

        value = config["position_controller"]["position_ki"]
        print(f" setting KI to {value}")
        bus.write_position_ki(joint_id, value)
        time.sleep(write_delay_seconds)

        value = config["position_controller"]["velocity_kp"]
        print(f" setting KD to {value}")
        bus.write_velocity_kp(joint_id, value)
        time.sleep(write_delay_seconds)

        value = config["position_controller"]["velocity_ki"]
        print(f" setting velocity KI to {value}")
        bus.write_velocity_ki(joint_id, value)
        time.sleep(write_delay_seconds)

        value = config["position_controller"]["torque_limit"]
        print(f" setting torque limit to {value}")
        bus.write_torque_limit(joint_id, value)
        time.sleep(write_delay_seconds)

        value = config["position_controller"]["velocity_limit"]
        print(f" setting velocity limit to {value}")
        bus.write_velocity_limit(joint_id, value)
        time.sleep(write_delay_seconds)

        value = config["position_controller"]["position_limit_lower"]
        print(f" setting position limit lower to {value}")
        bus.write_position_limit_lower(joint_id, value)
        time.sleep(write_delay_seconds)

        value = config["position_controller"]["position_limit_upper"]
        print(f" setting position limit upper to {value}")
        bus.write_position_limit_upper(joint_id, value)
        time.sleep(write_delay_seconds)

        value = config["position_controller"]["position_offset"]
        print(f" setting position offset to {value}")
        bus.write_position_offset(joint_id, value)
        time.sleep(write_delay_seconds)

        value = config["position_controller"]["torque_filter_alpha"]
        print(f" setting torque filter alpha to {value}")
        bus.write_torque_filter_alpha(joint_id, value)
        time.sleep(write_delay_seconds)

        value = config["current_controller"]["i_limit"]
        print(f" setting current limit to {value}")
        bus.write_current_limit(joint_id, value)
        time.sleep(write_delay_seconds)

        value = config["current_controller"]["i_kp"]
        print(f" setting current Kp to {value}")
        bus.write_current_kp(joint_id, value)
        time.sleep(write_delay_seconds)

        value = config["current_controller"]["i_ki"]
        print(f" setting current Ki to {value}")
        bus.write_current_ki(joint_id, value)
        time.sleep(write_delay_seconds)

        value = config["motor"]["pole_pairs"]
        print(f" setting pole pairs to {value}")
        bus.write_motor_pole_pairs(joint_id, value)
        time.sleep(write_delay_seconds)

        value = config["motor"]["torque_constant"]
        print(f" setting torque constant to {value}")
        bus.write_motor_torque_constant(joint_id, value)
        time.sleep(write_delay_seconds)

        value = config["motor"]["phase_order"]
        print(f" setting phase order to {value}")
        bus.write_motor_phase_order(joint_id, value)
        time.sleep(write_delay_seconds)

        value = config["motor"]["max_calibration_current"]
        print(f" setting max calibration current to {value}")
        bus.write_motor_calibration_current(joint_id, value)
        time.sleep(write_delay_seconds)

        value = config["encoder"]["cpr"]
        print(f" setting cpr to {value}")
        bus.write_encoder_cpr(joint_id, value)
        time.sleep(write_delay_seconds)

        value = config["encoder"]["position_offset"]
        print(f" setting position offset to {value}")
        bus.write_encoder_position_offset(joint_id, value)
        time.sleep(write_delay_seconds)

        value = config["encoder"]["velocity_filter_alpha"]
        print(f" setting velocity filter alpha to {value}")
        bus.write_encoder_velocity_filter_alpha(joint_id, value)
        time.sleep(write_delay_seconds)

        value = config["encoder"]["flux_offset"]
        print(f" setting flux offset to {value}")
        bus.write_encoder_flux_offset(joint_id, value)
        time.sleep(write_delay_seconds)

        if store_to_flash:
            print(" storing to flash")
            bus.store_settings_to_flash(joint_id)
            time.sleep(0.2)

        time.sleep(0.5)
