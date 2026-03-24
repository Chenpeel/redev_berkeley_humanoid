# Copyright (c) 2025, The Berkeley Humanoid Lite Project Developers.

from __future__ import annotations

import struct
import time
from typing import TYPE_CHECKING

import numpy as np
from cc.udp import UDP
from loop_rate_limiters import RateLimiter

from berkeley_humanoid_lite_lowlevel.robot.command_source import GamepadCommandSource, LocomotionCommand
from berkeley_humanoid_lite_lowlevel.robot.control_state import LocomotionControlState
from berkeley_humanoid_lite_lowlevel.robot.locomotion_specification import build_leg_locomotion_robot_specification

if TYPE_CHECKING:
    from berkeley_humanoid_lite_lowlevel.policy.configuration import PolicyDeploymentConfiguration


DEFAULT_GAMEPAD_UDP_HOST = "127.0.0.1"
DEFAULT_GAMEPAD_UDP_PORT = 10011
DEFAULT_GAMEPAD_UDP_RATE_HZ = 20.0


def _format_array(values: np.ndarray) -> str:
    return np.array2string(np.asarray(values, dtype=np.float32), precision=3, suppress_small=True, floatmode="fixed")


def _format_control_state(value: object) -> str:
    try:
        return LocomotionControlState(int(value)).name
    except (TypeError, ValueError):
        return f"UNKNOWN({value})"


def print_locomotion_debug_snapshot(
    *,
    step_index: int,
    robot: object,
    observations: np.ndarray,
    actions: np.ndarray,
) -> None:
    joint_count = int(robot.specification.joint_count)
    command_start = 7 + joint_count * 2 + 1
    command_velocity = observations[command_start : command_start + 3]
    targets = np.asarray(robot.actuators.joint_position_target, dtype=np.float32)
    measured = np.asarray(robot.joint_position_measured, dtype=np.float32)
    offsets = np.asarray(robot.position_offsets, dtype=np.float32)
    directions = np.asarray(robot.joint_axis_directions, dtype=np.float32)
    initialization_positions = np.asarray(robot.specification.initialization_positions, dtype=np.float32)
    position_error = targets - measured
    delta_to_initialization = initialization_positions - measured
    raw_targets = (targets + offsets) * directions
    raw_measured = (measured + offsets) * directions
    raw_initialization_targets = (initialization_positions + offsets) * directions
    raw_delta_to_initialization = raw_initialization_targets - raw_measured

    print(
        "[DEBUG] "
        f"step={step_index} "
        f"state={_format_control_state(getattr(robot, 'state', None))} "
        f"requested={_format_control_state(getattr(robot, 'requested_state', None))} "
        f"cmd=({command_velocity[0]:+.3f}, {command_velocity[1]:+.3f}, {command_velocity[2]:+.3f})"
    )
    print("[DEBUG] actions  =", _format_array(actions))
    print("[DEBUG] targets  =", _format_array(targets))
    print("[DEBUG] measured =", _format_array(measured))
    print("[DEBUG] error    =", _format_array(position_error))
    print("[DEBUG] offsets  =", _format_array(offsets))
    print("[DEBUG] raw_tgt  =", _format_array(raw_targets))
    print("[DEBUG] raw_meas =", _format_array(raw_measured))
    print("[DEBUG] init_err =", _format_array(delta_to_initialization))
    print("[DEBUG] init_raw =", _format_array(raw_delta_to_initialization))


def create_observation_stream(configuration: PolicyDeploymentConfiguration) -> UDP:
    return UDP(
        ("0.0.0.0", int(configuration.ip_policy_obs_port)),
        (str(configuration.ip_host_addr), int(configuration.ip_policy_obs_port)),
    )


def create_gamepad_command_stream(*, host: str = DEFAULT_GAMEPAD_UDP_HOST, port: int = DEFAULT_GAMEPAD_UDP_PORT) -> UDP:
    return UDP(
        recv_addr=None,
        send_addr=(host, int(port)),
    )


def encode_gamepad_command_packet(command: LocomotionCommand) -> bytes:
    """编码为 native runtime 约定的 joystick UDP 包: 1 byte mode + 3 float32。"""
    return struct.pack(
        "<Bfff",
        int(command.requested_state),
        float(command.velocity_x),
        float(command.velocity_y),
        float(command.velocity_yaw),
    )


def create_locomotion_robot(
    *,
    left_leg_bus: str = "can0",
    right_leg_bus: str = "can1",
    enable_imu: bool = True,
    enable_command_source: bool = True,
):
    from berkeley_humanoid_lite_lowlevel.robot import LocomotionRobot

    specification = build_leg_locomotion_robot_specification(
        left_leg_bus=left_leg_bus,
        right_leg_bus=right_leg_bus,
    )
    return LocomotionRobot(
        specification=specification,
        enable_imu=enable_imu,
        enable_command_source=enable_command_source,
    )


def run_locomotion_loop(
    configuration: PolicyDeploymentConfiguration,
    *,
    left_leg_bus: str = "can0",
    right_leg_bus: str = "can1",
    debug: bool = False,
    debug_every: int = 25,
) -> None:
    from berkeley_humanoid_lite_lowlevel.policy.controller import PolicyController

    if debug_every <= 0:
        raise ValueError("debug_every must be positive")

    print(f"Policy frequency: {1 / configuration.policy_dt} Hz")
    if debug:
        print(f"Debug snapshots enabled every {debug_every} policy steps")

    controller = PolicyController(configuration)
    controller.load_policy()
    rate = RateLimiter(1 / configuration.policy_dt)

    robot = None
    observation_stream = None
    try:
        robot = create_locomotion_robot(
            left_leg_bus=left_leg_bus,
            right_leg_bus=right_leg_bus,
        )
        observation_stream = create_observation_stream(configuration)
        robot.enter_damping_mode()
        observations = robot.reset()
        step_index = 0
        previous_state = None
        previous_requested_state = None

        while True:
            actions = controller.compute_actions(observations)
            observations = robot.step(actions)
            if debug:
                state_changed = (
                    robot.state != previous_state
                    or robot.requested_state != previous_requested_state
                )
                if state_changed or step_index % debug_every == 0:
                    print_locomotion_debug_snapshot(
                        step_index=step_index,
                        robot=robot,
                        observations=observations,
                        actions=actions,
                    )
                previous_state = robot.state
                previous_requested_state = robot.requested_state
            observation_stream.send_numpy(observations)
            rate.sleep()
            step_index += 1
    except KeyboardInterrupt:
        print("Stopping locomotion loop.")
    finally:
        if observation_stream is not None:
            observation_stream.stop()
        if robot is not None:
            robot.stop()


def run_idle_stream(
    configuration: PolicyDeploymentConfiguration,
    *,
    left_leg_bus: str = "can0",
    right_leg_bus: str = "can1",
) -> None:
    robot = None
    observation_stream = None
    try:
        robot = create_locomotion_robot(
            left_leg_bus=left_leg_bus,
            right_leg_bus=right_leg_bus,
        )
        observation_stream = create_observation_stream(configuration)
        robot.enter_damping_mode()
        robot.reset()

        while True:
            actions = np.zeros((robot.specification.joint_count,), dtype=np.float32)
            observations = robot.step(actions)
            observation_stream.send_numpy(observations)
            print(robot.joint_position_measured)
    except KeyboardInterrupt:
        print("Stopping idle stream.")
    finally:
        if observation_stream is not None:
            observation_stream.stop()
        if robot is not None:
            robot.stop()


def check_locomotion_connection(
    *,
    left_leg_bus: str = "can0",
    right_leg_bus: str = "can1",
) -> None:
    robot = create_locomotion_robot(
        left_leg_bus=left_leg_bus,
        right_leg_bus=right_leg_bus,
        enable_imu=False,
        enable_command_source=False,
    )
    try:
        robot.check_connection()
    finally:
        robot.shutdown()


def create_policy_inference_smoke_test_observations(
    configuration: PolicyDeploymentConfiguration,
    *,
    command_velocity: tuple[float, float, float] = (0.0, 0.0, 0.0),
) -> np.ndarray:
    observations = np.zeros((7 + configuration.num_actions * 2 + 1 + 3,), dtype=np.float32)
    observations[0] = 1.0

    if configuration.num_actions == configuration.num_joints:
        default_joint_positions = np.array(configuration.default_joint_positions, dtype=np.float32)
    else:
        default_joint_positions = np.array(configuration.default_joint_positions[10:], dtype=np.float32)

    observations[7 : 7 + configuration.num_actions] = default_joint_positions
    observations[7 + configuration.num_actions * 2 + 1 : 7 + configuration.num_actions * 2 + 4] = np.asarray(
        command_velocity,
        dtype=np.float32,
    )
    return observations


def run_policy_inference_smoke_test(
    configuration: PolicyDeploymentConfiguration,
    *,
    command_velocity: tuple[float, float, float] = (0.0, 0.0, 0.0),
) -> np.ndarray:
    from berkeley_humanoid_lite_lowlevel.policy.controller import PolicyController

    controller = PolicyController(configuration)
    controller.load_policy()
    observations = create_policy_inference_smoke_test_observations(
        configuration,
        command_velocity=command_velocity,
    )
    actions = controller.compute_actions(observations)

    print(f"Observation shape: {tuple(observations.shape)}")
    print(
        "Command velocity:",
        f"vx={command_velocity[0]:.3f}",
        f"vy={command_velocity[1]:.3f}",
        f"vyaw={command_velocity[2]:.3f}",
    )
    print("Actions:", np.array2string(actions, precision=4, suppress_small=True))
    print(
        "Action stats:",
        f"min={float(actions.min()):.4f}",
        f"max={float(actions.max()):.4f}",
        f"mean={float(actions.mean()):.4f}",
    )
    return actions


def stream_gamepad_commands() -> None:
    command_source = GamepadCommandSource()

    try:
        command_source.start()

        while True:
            command = command_source.snapshot()
            print(
                command.requested_state,
                f"{command.velocity_x:.2f}",
                f"{command.velocity_y:.2f}",
                f"{command.velocity_yaw:.2f}",
            )
            time.sleep(0.05)
    except KeyboardInterrupt:
        print("Stopping gamepad stream.")
    finally:
        command_source.stop()


def broadcast_gamepad_commands(
    *,
    host: str = DEFAULT_GAMEPAD_UDP_HOST,
    port: int = DEFAULT_GAMEPAD_UDP_PORT,
    rate_hz: float = DEFAULT_GAMEPAD_UDP_RATE_HZ,
) -> None:
    if rate_hz <= 0.0:
        raise ValueError("rate_hz must be positive")

    command_source = GamepadCommandSource()
    rate = RateLimiter(rate_hz)
    command_stream = None

    try:
        command_source.start()
        command_stream = create_gamepad_command_stream(host=host, port=port)
        print(f"Broadcasting gamepad commands to {host}:{port} at {rate_hz:.2f} Hz")

        while True:
            command = command_source.snapshot()
            command_stream.send(encode_gamepad_command_packet(command))
            print(
                command.requested_state,
                f"{command.velocity_x:.2f}",
                f"{command.velocity_y:.2f}",
                f"{command.velocity_yaw:.2f}",
                end="\r",
            )
            rate.sleep()
    except KeyboardInterrupt:
        print()
        print("Stopping gamepad UDP broadcast.")
    finally:
        command_source.stop()
        if command_stream is not None:
            command_stream.stop()
