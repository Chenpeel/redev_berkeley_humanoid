from __future__ import annotations

import json
from pathlib import Path

from berkeley_humanoid_lite_lowlevel.robot import (
    LocomotionRobot,
    read_robot_configuration,
    write_robot_configuration,
)
from berkeley_humanoid_lite_lowlevel.runtime_paths import ensure_parent_directory, resolve_workspace_path


def export_robot_configuration(output_path: str | Path) -> Path:
    robot = LocomotionRobot(enable_imu=False, enable_command_source=False)
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
) -> Path:
    resolved_configuration_path = resolve_workspace_path(configuration_path)
    with open(resolved_configuration_path, "r", encoding="utf-8") as file:
        configuration = json.load(file)

    robot = LocomotionRobot(enable_imu=False, enable_command_source=False)
    try:
        write_robot_configuration(
            robot,
            configuration,
            store_to_flash=store_to_flash,
        )
    finally:
        robot.shutdown()

    return resolved_configuration_path
