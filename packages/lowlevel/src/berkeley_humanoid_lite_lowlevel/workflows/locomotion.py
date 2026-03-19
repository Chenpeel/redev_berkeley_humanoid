# Copyright (c) 2025, The Berkeley Humanoid Lite Project Developers.

from __future__ import annotations

import time

from cc.udp import UDP
from loop_rate_limiters import RateLimiter
import numpy as np

from berkeley_humanoid_lite_lowlevel.policy.configuration import PolicyDeploymentConfiguration
from berkeley_humanoid_lite_lowlevel.policy.controller import PolicyController
from berkeley_humanoid_lite_lowlevel.robot import GamepadCommandSource, LocomotionRobot


def create_observation_stream(configuration: PolicyDeploymentConfiguration) -> UDP:
    return UDP(
        ("0.0.0.0", int(configuration.ip_policy_obs_port)),
        (str(configuration.ip_host_addr), int(configuration.ip_policy_obs_port)),
    )


def run_locomotion_loop(configuration: PolicyDeploymentConfiguration) -> None:
    print(f"Policy frequency: {1 / configuration.policy_dt} Hz")

    observation_stream = create_observation_stream(configuration)
    controller = PolicyController(configuration)
    controller.load_policy()
    rate = RateLimiter(1 / configuration.policy_dt)

    robot = LocomotionRobot()
    robot.enter_damping_mode()
    observations = robot.reset()

    try:
        while True:
            actions = controller.compute_actions(observations)
            observations = robot.step(actions)
            observation_stream.send_numpy(observations)
            rate.sleep()
    except KeyboardInterrupt:
        print("Stopping locomotion loop.")
    finally:
        robot.stop()


def run_idle_stream(configuration: PolicyDeploymentConfiguration) -> None:
    robot = LocomotionRobot()
    observation_stream = create_observation_stream(configuration)
    robot.enter_damping_mode()
    robot.reset()

    try:
        while True:
            actions = np.zeros((robot.specification.joint_count,), dtype=np.float32)
            observations = robot.step(actions)
            observation_stream.send_numpy(observations)
            print(robot.joint_position_measured)
    except KeyboardInterrupt:
        print("Stopping idle stream.")
    finally:
        robot.stop()


def check_locomotion_connection() -> None:
    robot = LocomotionRobot(enable_imu=False, enable_command_source=False)
    try:
        robot.check_connection()
    finally:
        robot.shutdown()


def run_policy_inference_smoke_test(configuration: PolicyDeploymentConfiguration) -> None:
    controller = PolicyController(configuration)
    controller.load_policy()
    observations = np.zeros((7 + configuration.num_actions * 2 + 1 + 3,), dtype=np.float32)
    controller.compute_actions(observations)


def stream_gamepad_commands() -> None:
    command_source = GamepadCommandSource()
    command_source.start()

    try:
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
