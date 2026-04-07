# Copyright (c) 2025, The Berkeley Humanoid Lite Project Developers.

from __future__ import annotations

import threading
import time
from collections.abc import Callable

import numpy as np
from cc.udp import UDP
from loop_rate_limiters import RateLimiter

from berkeley_humanoid_lite_lowlevel.teleoperation.input_frame import (
    FakeTeleoperationPattern,
    SweepArm,
    build_fake_bridge_data,
)
from berkeley_humanoid_lite_lowlevel.teleoperation.playback import (
    BIMANUAL_SOLVER_JOINT_NAMES,
    build_bimanual_action,
    create_teleoperation_playback_delta_bridge,
    list_ik_result_cases,
    load_ik_result_payload,
    map_ik_result_joint_targets_to_bimanual,
    resolve_ik_result_case,
)

TELEOP_BRIDGE_PORT = 11005
TELEOP_ARM_JOINT_SLICE = slice(7, 17)
TELEOP_ARM_JOINT_COUNT = TELEOP_ARM_JOINT_SLICE.stop - TELEOP_ARM_JOINT_SLICE.start


def _create_bridge_udp() -> UDP:
    return UDP(recv_addr=("0.0.0.0", TELEOP_BRIDGE_PORT), send_addr=("127.0.0.1", TELEOP_BRIDGE_PORT))


def _shutdown_bimanual_playback_robot(robot: object) -> None:
    from berkeley_humanoid_lite_lowlevel import recoil

    for bus, device_id, _ in robot.joints:
        bus.set_mode(device_id, recoil.Mode.DAMPING)
    time.sleep(0.05)
    robot.shutdown()


def build_minimal_teleoperation_observations(configuration_size: int) -> np.ndarray:
    if configuration_size < TELEOP_ARM_JOINT_SLICE.stop:
        raise ValueError("Teleoperation observations require at least 17 configuration entries.")

    observations = np.zeros(configuration_size, dtype=np.float64)
    observations[0:3] = [0.0, 0.0, 0.5]
    observations[3:7] = [1.0, 0.0, 0.0, 0.0]
    return observations


def update_offline_joint_observations(
    observations: np.ndarray,
    joint_targets: np.ndarray,
    *,
    alpha: float = 1.0,
) -> np.ndarray:
    if observations.ndim != 1 or observations.shape[0] < TELEOP_ARM_JOINT_SLICE.stop:
        raise ValueError("Unexpected teleoperation observation shape.")
    if not 0.0 < alpha <= 1.0:
        raise ValueError("Offline teleoperation observation alpha must be in (0.0, 1.0].")

    targets = np.asarray(joint_targets, dtype=np.float64)
    if targets.shape != (TELEOP_ARM_JOINT_COUNT,):
        raise ValueError("Unexpected teleoperation joint target shape.")

    current_measurements = observations[TELEOP_ARM_JOINT_SLICE].copy()
    next_measurements = current_measurements + alpha * (targets - current_measurements)
    observations[TELEOP_ARM_JOINT_SLICE] = next_measurements
    return next_measurements


def run_teleoperation_loop() -> None:
    import berkeley_humanoid_lite_lowlevel.teleoperation as teleoperation_module
    from berkeley_humanoid_lite_lowlevel.robot import Bimanual

    np.set_printoptions(precision=2)

    solver = teleoperation_module.TeleoperationIkSolver()
    robot = Bimanual()
    rate = RateLimiter(30)
    bridge_udp = _create_bridge_udp()

    observations = build_minimal_teleoperation_observations(solver.robot.model.nq)

    def update_controller() -> None:
        while True:
            bridge_data = bridge_udp.recv_dict()
            if bridge_data is not None:
                solver.update_controller(bridge_data)

    controller_thread = threading.Thread(target=update_controller, daemon=True)
    controller_thread.start()

    robot.start(kp=30, kd=2, torque_limit=2)
    robot_actions = np.zeros((12,), dtype=np.float32)

    try:
        while True:
            robot_observations = robot.step(robot_actions * robot.joint_axis_directions) * robot.joint_axis_directions
            observations[TELEOP_ARM_JOINT_SLICE] = robot_observations[0:TELEOP_ARM_JOINT_COUNT]
            joint_actions, gripper_actions = solver.update(observations)
            print(joint_actions, gripper_actions)
            robot_actions[0:TELEOP_ARM_JOINT_COUNT] = joint_actions
            robot_actions[10] = gripper_actions[0]
            robot_actions[11] = gripper_actions[1]
            rate.sleep()
    except KeyboardInterrupt:
        print("Stopping teleoperation loop.")
    finally:
        robot.stop()


def run_teleoperation_idle_loop() -> None:
    from berkeley_humanoid_lite_lowlevel.robot import Bimanual

    np.set_printoptions(precision=3, suppress=True)

    rate = RateLimiter(100)
    robot = Bimanual()
    robot.start(kp=20, kd=2, torque_limit=0.5)

    try:
        while True:
            observations = robot.step(np.zeros((12,), dtype=np.float32))
            print(observations)
            rate.sleep()
    except KeyboardInterrupt:
        print("Stopping teleoperation idle loop.")
    finally:
        robot.stop()


def check_teleoperation_connection() -> None:
    from berkeley_humanoid_lite_lowlevel.robot import Bimanual

    robot = Bimanual()
    try:
        robot.check_connection()
    finally:
        robot.shutdown()


def build_demo_bridge_data(timestamp: float) -> dict[str, dict[str, object]]:
    return build_fake_bridge_data(timestamp, pattern="dual_arm_circle")


def run_offline_teleoperation_ik(
    *,
    pattern: FakeTeleoperationPattern = "dual_arm_circle",
    frequency: float = 100.0,
    duration_seconds: float | None = 10.0,
    left_enable: bool = True,
    right_enable: bool = True,
    with_gripper: bool = False,
    sweep_arm: SweepArm = "auto",
    measurement_alpha: float = 1.0,
    print_every: int = 10,
    solver_factory: Callable[[], object] | None = None,
) -> None:
    if frequency <= 0.0:
        raise ValueError("Offline teleoperation frequency must be positive.")
    if print_every <= 0:
        raise ValueError("Offline teleoperation print frequency must be positive.")
    if duration_seconds is not None and duration_seconds <= 0.0:
        duration_seconds = None

    if solver_factory is None:
        import berkeley_humanoid_lite_lowlevel.teleoperation as teleoperation_module

        solver = teleoperation_module.TeleoperationIkSolver()
    else:
        solver = solver_factory()
    rate = RateLimiter(frequency=frequency, warn=False)
    observations = build_minimal_teleoperation_observations(solver.robot.model.nq)
    start_time = time.perf_counter()
    step_index = 0

    try:
        while True:
            elapsed_seconds = time.perf_counter() - start_time
            if duration_seconds is not None and elapsed_seconds >= duration_seconds:
                break

            bridge_data = build_fake_bridge_data(
                elapsed_seconds,
                pattern=pattern,
                left_enable=left_enable,
                right_enable=right_enable,
                with_gripper=with_gripper,
                sweep_arm=sweep_arm,
            )
            solver.update_controller(bridge_data)
            joint_targets, gripper_targets = solver.update(observations)
            joint_measurements = update_offline_joint_observations(
                observations,
                joint_targets,
                alpha=measurement_alpha,
            )

            if step_index % print_every == 0:
                print(
                    f"step={step_index:04d} "
                    f"t={elapsed_seconds:6.2f}s "
                    f"pattern={pattern} "
                    f"target={np.array2string(joint_targets, precision=3, suppress_small=True)} "
                    f"measured={np.array2string(joint_measurements, precision=3, suppress_small=True)} "
                    f"gripper={gripper_targets}"
                )

            step_index += 1
            rate.sleep()
    except KeyboardInterrupt:
        print("Stopping offline teleoperation IK loop.")


def run_teleoperation_ik_result_playback(
    *,
    input_path: str,
    pattern: str | None = None,
    sweep_arm: str = "auto",
    case_index: int | None = None,
    dry_run: bool = False,
    delta_bridge: bool = True,
    bridge_scale: float = 1.0,
    max_delta_radians: float | None = np.deg2rad(30.0),
    max_step_radians: float | None = np.deg2rad(3.0),
    position_kp: float = 20.0,
    position_kd: float = 2.0,
    torque_limit: float = 2.0,
    print_every: int = 10,
) -> None:
    if print_every <= 0:
        raise ValueError("Teleoperation IK playback print frequency must be positive.")

    payload = load_ik_result_payload(input_path)
    case = resolve_ik_result_case(
        payload,
        pattern=pattern,
        sweep_arm=sweep_arm,
        case_index=case_index,
    )

    samples = case.get("samples")
    if not isinstance(samples, list) or not samples:
        raise ValueError("Selected IK result case does not contain any samples.")

    frequency = float(case.get("frequency", payload.get("frequency", 20.0)))
    if frequency <= 0.0:
        raise ValueError("IK result frequency must be positive.")
    rate = RateLimiter(frequency=frequency, warn=False)

    listed_cases = ", ".join(
        f"{index}:{case_pattern}/{case_sweep_arm}"
        for index, case_pattern, case_sweep_arm in list_ik_result_cases(payload)
    )
    selected_case = f"{case.get('pattern')}/{case.get('sweep_arm', 'auto')}"
    print(f"Loaded IK result cases: {listed_cases}")
    print(f"Selected IK playback case: {selected_case}")
    print(f"Playback joint order: {', '.join(BIMANUAL_SOLVER_JOINT_NAMES)}")
    if delta_bridge:
        print(f"Delta bridge enabled with scale={bridge_scale:.3f}")
        if max_delta_radians is not None:
            print(f"Max delta: {np.rad2deg(max_delta_radians):.2f} deg")
        if max_step_radians is not None:
            print(f"Max step: {np.rad2deg(max_step_radians):.2f} deg")
    elif dry_run:
        print("Absolute playback dry-run: no hardware commands will be sent.")
    else:
        print("Absolute playback enabled: samples will be sent as direct joint targets.")

    robot = None
    bridge_state = None
    try:
        if not dry_run:
            from berkeley_humanoid_lite_lowlevel.robot import Bimanual

            robot = Bimanual()
            robot.start(kp=position_kp, kd=position_kd, torque_limit=torque_limit)
            initial_observations = robot.reset()
            real_start_positions = initial_observations[:10] * robot.joint_axis_directions[:10]
        else:
            robot = None
            real_start_positions = np.zeros((10,), dtype=np.float32)

        for step_index, sample in enumerate(samples):
            sample_joint_targets = map_ik_result_joint_targets_to_bimanual(
                sample["joint_targets"],
            )
            if delta_bridge:
                if bridge_state is None:
                    bridge_state = create_teleoperation_playback_delta_bridge(
                        ik_joint_positions=sample_joint_targets,
                        real_joint_positions=real_start_positions,
                        bridge_scale=bridge_scale,
                        max_delta_radians=max_delta_radians,
                        max_step_radians=max_step_radians,
                    )
                playback_joint_targets, joint_delta = bridge_state.compute_target_positions(
                    sample_joint_targets,
                )
            else:
                playback_joint_targets = sample_joint_targets
                joint_delta = np.zeros((10,), dtype=np.float32)

            action = build_bimanual_action(
                solver_joint_targets=playback_joint_targets,
                joint_axis_directions=(
                    robot.joint_axis_directions if robot is not None else np.ones((12,), dtype=np.float32)
                ),
                gripper_targets=sample.get("gripper_targets"),
            )

            if dry_run:
                measured = playback_joint_targets
            else:
                measured = robot.step(action)[:10] * robot.joint_axis_directions[:10]

            if step_index % print_every == 0:
                print(
                    f"step={step_index:04d} "
                    f"t={float(sample['timestamp']):6.2f}s "
                    f"target={np.array2string(playback_joint_targets, precision=3, suppress_small=True)} "
                    f"measured={np.array2string(measured, precision=3, suppress_small=True)} "
                    f"delta={np.array2string(joint_delta, precision=3, suppress_small=True)}"
                )

            rate.sleep()
    except KeyboardInterrupt:
        print("Stopping teleoperation IK result playback.")
    finally:
        if robot is not None:
            _shutdown_bimanual_playback_robot(robot)


def run_teleoperation_solver_demo() -> None:
    import berkeley_humanoid_lite_lowlevel.teleoperation as teleoperation_module

    solver = teleoperation_module.TeleoperationIkSolver()
    rate = RateLimiter(frequency=100.0, warn=False)

    observations = build_minimal_teleoperation_observations(solver.robot.model.nq)

    while True:
        solver.update_controller(build_demo_bridge_data(time.perf_counter()))
        joint_actions, _ = solver.update(observations)
        update_offline_joint_observations(observations, joint_actions)
        print(joint_actions)
        rate.sleep()


def stream_gripper_targets(
    *,
    port: str = "/dev/ttyUSB0",
    baudrate: int = 115200,
    left_target: float = 0.8,
    right_target: float = 0.8,
    period_seconds: float = 0.1,
) -> None:
    from berkeley_humanoid_lite_lowlevel.robot import SerialGripper

    gripper = SerialGripper(port=port, baudrate=baudrate)
    try:
        while True:
            gripper.write_targets(left_target, right_target)
            print(gripper.readline())
            time.sleep(period_seconds)
    finally:
        gripper.close()
