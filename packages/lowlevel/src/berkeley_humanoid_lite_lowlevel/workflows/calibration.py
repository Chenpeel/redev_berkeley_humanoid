# Copyright (c) 2025, The Berkeley Humanoid Lite Project Developers.

from __future__ import annotations

from berkeley_humanoid_lite_lowlevel.robot import (
    LocomotionRobot,
    capture_calibration_offsets,
)


def run_joint_calibration() -> None:
    robot = LocomotionRobot(enable_imu=False, enable_command_source=True)

    try:
        position_offsets = capture_calibration_offsets(
            robot.specification,
            robot.actuators,
            robot.command_source,
        )
        calibration_path = robot.calibration_store.save_position_offsets(position_offsets)
        print(f"saved calibration to {calibration_path}")
    finally:
        robot.shutdown()
