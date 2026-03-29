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
from berkeley_humanoid_lite_lowlevel.robot.locomotion_diagnostics import LocomotionDiagnosticSnapshot
from berkeley_humanoid_lite_lowlevel.robot.locomotion_specification import build_leg_locomotion_robot_specification
from berkeley_humanoid_lite_lowlevel.runtime_paths import get_pose_alignment_path
from berkeley_humanoid_lite_lowlevel.workflows.imu import (
    DEFAULT_IMU_BAUDRATE,
    DEFAULT_IMU_PROBE_DURATION,
    DEFAULT_IMU_PROTOCOL,
    DEFAULT_IMU_SERIAL_DEVICE,
    DEFAULT_IMU_TIMEOUT,
    resolve_imu_stream_configuration,
)

if TYPE_CHECKING:
    from berkeley_humanoid_lite_lowlevel.policy.configuration import PolicyDeploymentConfiguration


DEFAULT_GAMEPAD_UDP_HOST = "127.0.0.1"
DEFAULT_GAMEPAD_UDP_PORT = 10011
DEFAULT_GAMEPAD_UDP_RATE_HZ = 20.0
DEFAULT_LOCOMOTION_IMU_WAIT_TIMEOUT = 2.0
DEFAULT_LOCOMOTION_POSE_ALIGNMENT_PATH = str(get_pose_alignment_path())


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
    snapshot: LocomotionDiagnosticSnapshot,
) -> None:
    position_error = snapshot.targets - snapshot.measured

    print(
        "[DEBUG] "
        f"step={step_index} "
        f"state={_format_control_state(snapshot.state)} "
        f"requested={_format_control_state(snapshot.requested_state)} "
        "cmd=("
        f"{snapshot.command_velocity[0]:+.3f}, "
        f"{snapshot.command_velocity[1]:+.3f}, "
        f"{snapshot.command_velocity[2]:+.3f})"
    )
    print("[DEBUG] actions  =", _format_array(snapshot.actions))
    print("[DEBUG] targets  =", _format_array(snapshot.targets))
    print("[DEBUG] measured =", _format_array(snapshot.measured))
    print("[DEBUG] error    =", _format_array(position_error))
    print("[DEBUG] offsets  =", _format_array(snapshot.offsets))
    print("[DEBUG] raw_tgt  =", _format_array(snapshot.raw_targets))
    print("[DEBUG] raw_meas =", _format_array(snapshot.raw_measured))
    print("[DEBUG] stand_err=", _format_array(snapshot.delta_to_standing))
    print("[DEBUG] stand_raw=", _format_array(snapshot.raw_delta_to_standing))
    print("[DEBUG] init_err =", _format_array(snapshot.delta_to_initialization))
    print("[DEBUG] init_raw =", _format_array(snapshot.raw_delta_to_initialization))
    print(
        "[DEBUG] risk    =",
        f"init:{snapshot.risk_joint_name} raw_init_delta={snapshot.risk_raw_delta_to_initialization:+.3f} "
        f"stand:{snapshot.standing_risk_joint_name} raw_stand_delta={snapshot.risk_raw_delta_to_standing:+.3f} "
        f"dry_run={snapshot.dry_run}",
    )


def print_locomotion_imu_debug_line(
    *,
    step_index: int,
    line: str,
) -> None:
    print(f"[DEBUG][IMU] step={step_index} {line}")


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


def resolve_locomotion_imu_configuration(
    *,
    protocol: str = DEFAULT_IMU_PROTOCOL,
    device: str = DEFAULT_IMU_SERIAL_DEVICE,
    baudrate: str | int | None = DEFAULT_IMU_BAUDRATE,
    timeout: float = DEFAULT_IMU_TIMEOUT,
    probe_duration: float = DEFAULT_IMU_PROBE_DURATION,
):
    """解析 Python locomotion runtime 可直接使用的 IMU 配置。"""
    configuration = resolve_imu_stream_configuration(
        protocol=protocol,
        device=device,
        baudrate=baudrate,
        timeout=timeout,
        probe_duration=probe_duration,
    )
    if configuration.protocol != "hiwonder":
        raise ValueError(
            "Python locomotion runtime currently supports only HiWonder USB IMU streams. "
            f"Detected protocol={configuration.protocol!r} on {configuration.device}. "
            "Pass --imu-protocol hiwonder or switch to the HiWonder USB IMU."
        )
    return configuration


def resolve_policy_reference_joint_positions(configuration: PolicyDeploymentConfiguration) -> np.ndarray:
    if configuration.num_actions == configuration.num_joints:
        return np.array(configuration.default_joint_positions, dtype=np.float32)
    return np.array(configuration.default_joint_positions[10:], dtype=np.float32)


def create_locomotion_robot(
    *,
    left_leg_bus: str = "can0",
    right_leg_bus: str = "can1",
    enable_imu: bool = True,
    enable_command_source: bool = True,
    dry_run: bool = False,
    imu_device: str = "/dev/ttyUSB0",
    imu_baudrate: int | None = None,
    imu_timeout: float = DEFAULT_IMU_TIMEOUT,
    imu_wait_timeout: float = DEFAULT_LOCOMOTION_IMU_WAIT_TIMEOUT,
    require_imu_ready: bool = True,
    pose_alignment_path: str | None = None,
):
    from berkeley_humanoid_lite_lowlevel.robot import LocomotionRobot
    from berkeley_humanoid_lite_lowlevel.robot.pose_alignment import PoseAlignmentStore

    specification = build_leg_locomotion_robot_specification(
        left_leg_bus=left_leg_bus,
        right_leg_bus=right_leg_bus,
    )
    return LocomotionRobot(
        specification=specification,
        enable_imu=enable_imu,
        enable_command_source=enable_command_source,
        dry_run=dry_run,
        imu_device=imu_device,
        imu_baudrate=imu_baudrate,
        imu_read_timeout=imu_timeout,
        imu_wait_timeout=imu_wait_timeout,
        require_imu_ready=require_imu_ready,
        pose_alignment_store=PoseAlignmentStore(pose_alignment_path),
    )


def run_locomotion_loop(
    configuration: PolicyDeploymentConfiguration,
    *,
    left_leg_bus: str = "can0",
    right_leg_bus: str = "can1",
    dry_run: bool = False,
    debug: bool = False,
    debug_every: int = 25,
    debug_imu: bool = False,
    debug_imu_every: int = 25,
    imu_device: str = "/dev/ttyUSB0",
    imu_baudrate: int | None = None,
    imu_timeout: float = DEFAULT_IMU_TIMEOUT,
    imu_wait_timeout: float = DEFAULT_LOCOMOTION_IMU_WAIT_TIMEOUT,
    require_imu_ready: bool = True,
    pose_alignment_path: str | None = None,
) -> None:
    from berkeley_humanoid_lite_lowlevel.policy.controller import PolicyController

    if debug_every <= 0:
        raise ValueError("debug_every must be positive")
    if debug_imu_every <= 0:
        raise ValueError("debug_imu_every must be positive")
    if imu_wait_timeout < 0.0:
        raise ValueError("imu_wait_timeout must be non-negative")

    print(f"Policy frequency: {1 / configuration.policy_dt} Hz")
    if dry_run:
        print("Dry-run enabled: observations and targets will be computed without sending motor commands")
    if debug:
        print(f"Debug snapshots enabled every {debug_every} policy steps")
    if debug_imu:
        print(f"IMU debug enabled every {debug_imu_every} policy steps")
    if require_imu_ready:
        print(f"IMU ready check enabled with timeout {imu_wait_timeout:g}s")
    else:
        print("IMU ready check disabled")

    controller = PolicyController(configuration)
    controller.load_policy()
    rate = RateLimiter(1 / configuration.policy_dt)

    robot = None
    observation_stream = None
    startup_completed = False
    try:
        robot_kwargs: dict[str, object] = dict(
            left_leg_bus=left_leg_bus,
            right_leg_bus=right_leg_bus,
            dry_run=dry_run,
            imu_device=imu_device,
            imu_baudrate=imu_baudrate,
            imu_timeout=imu_timeout,
            imu_wait_timeout=imu_wait_timeout,
            require_imu_ready=require_imu_ready,
        )
        if pose_alignment_path is not None:
            robot_kwargs["pose_alignment_path"] = pose_alignment_path
        robot = create_locomotion_robot(**robot_kwargs)
        observation_stream = create_observation_stream(configuration)
        if not dry_run:
            robot.enter_damping_mode()
        observations = robot.reset()
        startup_completed = True
        step_index = 0
        previous_state = None
        previous_requested_state = None

        while True:
            actions = controller.compute_actions(observations)
            observations = robot.step(actions)
            state_changed = (
                robot.state != previous_state
                or robot.requested_state != previous_requested_state
            )
            if debug:
                if state_changed or step_index % debug_every == 0:
                    snapshot = robot.create_diagnostic_snapshot(
                        observations=observations,
                        actions=actions,
                    )
                    print_locomotion_debug_snapshot(
                        step_index=step_index,
                        snapshot=snapshot,
                    )
            if debug_imu:
                if state_changed or step_index % debug_imu_every == 0:
                    imu_debug_line = robot.create_imu_debug_line()
                    if imu_debug_line is not None:
                        print_locomotion_imu_debug_line(
                            step_index=step_index,
                            line=imu_debug_line,
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
            if startup_completed:
                robot.stop()
            else:
                robot.shutdown()


def run_idle_stream(
    configuration: PolicyDeploymentConfiguration,
    *,
    left_leg_bus: str = "can0",
    right_leg_bus: str = "can1",
    pose_alignment_path: str | None = None,
) -> None:
    robot = None
    observation_stream = None
    try:
        robot_kwargs: dict[str, object] = dict(
            left_leg_bus=left_leg_bus,
            right_leg_bus=right_leg_bus,
        )
        if pose_alignment_path is not None:
            robot_kwargs["pose_alignment_path"] = pose_alignment_path
        robot = create_locomotion_robot(**robot_kwargs)
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
    pose_alignment_path: str | None = None,
) -> None:
    robot_kwargs: dict[str, object] = dict(
        left_leg_bus=left_leg_bus,
        right_leg_bus=right_leg_bus,
        enable_imu=False,
        enable_command_source=False,
    )
    if pose_alignment_path is not None:
        robot_kwargs["pose_alignment_path"] = pose_alignment_path
    robot = create_locomotion_robot(**robot_kwargs)
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

    observations[7 : 7 + configuration.num_actions] = resolve_policy_reference_joint_positions(configuration)
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


def run_locomotion_pose_alignment_capture(
    configuration: PolicyDeploymentConfiguration,
    *,
    left_leg_bus: str = "can0",
    right_leg_bus: str = "can1",
    pose_alignment_path: str | None = None,
    polling_interval_seconds: float = 0.05,
    capture_window_size: int = 20,
    max_stddev_deg: float = 1.0,
    write: bool = False,
) -> None:
    from berkeley_humanoid_lite_lowlevel.robot import capture_pose_alignment_result

    reference_positions = resolve_policy_reference_joint_positions(configuration)
    robot_kwargs: dict[str, object] = dict(
        left_leg_bus=left_leg_bus,
        right_leg_bus=right_leg_bus,
        enable_imu=False,
        enable_command_source=True,
        require_imu_ready=False,
    )
    if pose_alignment_path is not None:
        robot_kwargs["pose_alignment_path"] = pose_alignment_path
    robot = create_locomotion_robot(**robot_kwargs)

    try:
        robot.enter_damping_mode()
        robot.actuators.refresh_measurements()
        if robot.command_source is None:
            raise RuntimeError("Gamepad command source is not available for locomotion pose capture.")

        capture_result = capture_pose_alignment_result(
            robot.specification,
            robot.actuators,
            robot.command_source,
            reference_positions=reference_positions,
            polling_interval_seconds=polling_interval_seconds,
            capture_window_size=capture_window_size,
            max_stddev_deg=max_stddev_deg,
        )
        if not write:
            print("Re-run with --write to persist the captured bias to the locomotion pose-alignment config.")
            return

        output_path = robot.pose_alignment_store.save_pose_alignment_bias(
            capture_result.pose_alignment_bias,
            metadata=capture_result.build_metadata(
                robot.specification,
                reference_positions=reference_positions,
            ),
        )
        print(f"saved locomotion pose alignment to {output_path}")
    finally:
        robot.shutdown()
