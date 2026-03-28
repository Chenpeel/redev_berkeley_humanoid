# Copyright (c) 2025, The Berkeley Humanoid Lite Project Developers.

from __future__ import annotations

import numpy as np

from berkeley_humanoid_lite_lowlevel.robot import (
    build_leg_locomotion_robot_specification,
    capture_calibration_offsets,
)
from berkeley_humanoid_lite_lowlevel.robot.calibration import CalibrationStore
from berkeley_humanoid_lite_lowlevel.robot.command_source import GamepadCommandSource
from berkeley_humanoid_lite_lowlevel.robot.joint_transport import LocomotionActuatorArray
from berkeley_humanoid_lite_lowlevel.robot.locomotion_specification import (
    DEFAULT_LEFT_LEG_BUS,
    DEFAULT_RIGHT_LEG_BUS,
)


def run_joint_calibration(
    *,
    left_leg_bus: str = DEFAULT_LEFT_LEG_BUS,
    right_leg_bus: str = DEFAULT_RIGHT_LEG_BUS,
) -> None:
    actuator_array = None
    command_source = None

    try:
        specification = build_leg_locomotion_robot_specification(
            left_leg_bus=left_leg_bus,
            right_leg_bus=right_leg_bus,
        )
        print(
            "Joint calibration records the raw readings for the specification-defined "
            "calibration reference pose, not the mechanical limit."
        )
        # 标定必须基于原始位置读数，不能先加载已有 calibration.yaml，
        # 否则重复标定会在旧 offset 基础上继续漂移。
        #
        # 这里记录的是“spec 里定义的 calibration reference pose”对应的原始关节读数，
        # 不是让操作者把每个关节推到最远机械极限位。
        # 如果按“机械极限位”去摆机器人，保存下来的 offset 会系统性错误。
        actuator_array = LocomotionActuatorArray(
            specification=specification,
            position_offsets=np.zeros((specification.joint_count,), dtype=np.float32),
        )
        command_source = GamepadCommandSource()
        command_source.start()
        try:
            position_offsets = capture_calibration_offsets(
                specification,
                actuator_array,
                command_source,
            )
        except Exception as error:
            print(f"[ERROR] Joint calibration failed: {error}")
            print("Calibration checks:")
            print(f"  1. Confirm CAN interfaces are up: left={left_leg_bus} right={right_leg_bus}")
            print("  2. Confirm the motors are powered and the robot is connected")
            print("  3. Confirm no other process is already using the CAN buses")
            raise
        calibration_path = CalibrationStore().save_position_offsets(position_offsets)
        print(f"saved calibration to {calibration_path}")
    finally:
        if command_source is not None:
            command_source.stop()
        if actuator_array is not None:
            actuator_array.shutdown()
