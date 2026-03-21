from __future__ import annotations

import json
from pathlib import Path

from berkeley_humanoid_lite_lowlevel.robot import (
    LocomotionRobot,
    build_leg_locomotion_robot_specification,
    read_robot_configuration,
    write_robot_configuration,
)
from berkeley_humanoid_lite_lowlevel.robot.locomotion_specification import (
    DEFAULT_LEFT_LEG_BUS,
    DEFAULT_RIGHT_LEG_BUS,
)
from berkeley_humanoid_lite_lowlevel.runtime_paths import ensure_parent_directory, resolve_workspace_path


def export_robot_configuration(
    output_path: str | Path,
    *,
    left_leg_bus: str = DEFAULT_LEFT_LEG_BUS,
    right_leg_bus: str = DEFAULT_RIGHT_LEG_BUS,
) -> Path:
    specification = build_leg_locomotion_robot_specification(
        left_leg_bus=left_leg_bus,
        right_leg_bus=right_leg_bus,
    )
    robot = LocomotionRobot(
        specification=specification,
        enable_imu=False,
        enable_command_source=False,
    )
    try:
        robot.check_connection()
        configuration = read_robot_configuration(robot)
    finally:
        robot.shutdown()

    resolved_output_path = ensure_parent_directory(output_path)
    with open(resolved_output_path, "w", encoding="utf-8") as file:
        json.dump(configuration, file, indent=4)
    return resolved_output_path


def apply_robot_configuration(
    configuration_path: str | Path,
    *,
    store_to_flash: bool = True,
    left_leg_bus: str = DEFAULT_LEFT_LEG_BUS,
    right_leg_bus: str = DEFAULT_RIGHT_LEG_BUS,
) -> Path:
    resolved_configuration_path = resolve_workspace_path(configuration_path)
    with open(resolved_configuration_path, encoding="utf-8") as file:
        configuration = json.load(file)

    specification = build_leg_locomotion_robot_specification(
        left_leg_bus=left_leg_bus,
        right_leg_bus=right_leg_bus,
    )
    robot = LocomotionRobot(
        specification=specification,
        enable_imu=False,
        enable_command_source=False,
    )
    try:
        write_robot_configuration(
            robot,
            configuration,
            store_to_flash=store_to_flash,
        )
    finally:
        robot.shutdown()

    return resolved_configuration_path
