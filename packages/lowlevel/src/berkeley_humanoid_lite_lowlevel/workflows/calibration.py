# Copyright (c) 2025, The Berkeley Humanoid Lite Project Developers.

from __future__ import annotations

from berkeley_humanoid_lite_lowlevel.robot import (
    LocomotionRobot,
    build_leg_locomotion_robot_specification,
    capture_calibration_offsets,
)
from berkeley_humanoid_lite_lowlevel.robot.locomotion_specification import (
    DEFAULT_LEFT_LEG_BUS,
    DEFAULT_RIGHT_LEG_BUS,
)


def run_joint_calibration(
    *,
    left_leg_bus: str = DEFAULT_LEFT_LEG_BUS,
    right_leg_bus: str = DEFAULT_RIGHT_LEG_BUS,
) -> None:
    robot = None

    try:
        specification = build_leg_locomotion_robot_specification(
            left_leg_bus=left_leg_bus,
            right_leg_bus=right_leg_bus,
        )
        robot = LocomotionRobot(
            specification=specification,
            enable_imu=False,
            enable_command_source=True,
        )
        position_offsets = capture_calibration_offsets(
            robot.specification,
            robot.actuators,
            robot.command_source,
        )
        calibration_path = robot.calibration_store.save_position_offsets(position_offsets)
        print(f"saved calibration to {calibration_path}")
    finally:
        if robot is not None:
            robot.shutdown()
