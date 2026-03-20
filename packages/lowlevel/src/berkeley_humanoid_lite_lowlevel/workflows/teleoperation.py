# Copyright (c) 2025, The Berkeley Humanoid Lite Project Developers.

from __future__ import annotations

import time
import threading

from cc.udp import UDP
from loop_rate_limiters import RateLimiter
import numpy as np

TELEOP_BRIDGE_PORT = 11005


def _create_bridge_udp() -> UDP:
    return UDP(recv_addr=("0.0.0.0", TELEOP_BRIDGE_PORT), send_addr=("127.0.0.1", TELEOP_BRIDGE_PORT))


def run_teleoperation_loop() -> None:
    from berkeley_humanoid_lite_lowlevel.robot import Bimanual
    from berkeley_humanoid_lite_lowlevel.teleoperation import TeleoperationIkSolver

    np.set_printoptions(precision=2)

    solver = TeleoperationIkSolver()
    robot = Bimanual()
    rate = RateLimiter(30)
    bridge_udp = _create_bridge_udp()

    observations = np.zeros(solver.robot.model.nq)
    observations[0:3] = [0.0, 0.0, 0.5]
    observations[3:7] = [1.0, 0.0, 0.0, 0.0]

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
            observations[7:17] = robot_observations[0:10]
            joint_actions, gripper_actions = solver.update(observations)
            print(joint_actions, gripper_actions)
            robot_actions[0:10] = joint_actions
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
    left_pose = np.eye(4)
    right_pose = np.eye(4)

    left_pose[0, 3] = 0.2 + 0.1 * np.cos(2.0 * timestamp)
    left_pose[1, 3] = 0.2 + 0.1 * np.sin(2.0 * timestamp)
    left_pose[2, 3] = 0.6

    right_pose[0, 3] = 0.2 + 0.1 * np.sin(2.0 * timestamp)
    right_pose[1, 3] = -0.2 - 0.1 * np.cos(2.0 * timestamp)
    right_pose[2, 3] = 0.6

    return {
        "left": {
            "pose": left_pose.tolist(),
            "button_pressed": True,
            "trigger": 0.0,
        },
        "right": {
            "pose": right_pose.tolist(),
            "button_pressed": True,
            "trigger": 0.0,
        },
    }


def run_teleoperation_solver_demo() -> None:
    from berkeley_humanoid_lite_lowlevel.teleoperation import TeleoperationIkSolver

    solver = TeleoperationIkSolver()
    rate = RateLimiter(frequency=100.0, warn=False)

    observations = np.zeros(solver.robot.model.nq)
    observations[0:3] = [0.0, 0.0, 0.5]
    observations[3:7] = [1.0, 0.0, 0.0, 0.0]

    while True:
        solver.update_controller(build_demo_bridge_data(time.perf_counter()))
        joint_actions, _ = solver.update(observations)
        observations[7:17] = joint_actions
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
