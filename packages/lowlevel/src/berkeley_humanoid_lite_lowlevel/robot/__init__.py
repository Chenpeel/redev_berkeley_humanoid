from __future__ import annotations

from importlib import import_module
from typing import Any

__all__ = [
    "Bimanual",
    "CalibrationStore",
    "GamepadCommandSource",
    "JointTransportAddress",
    "LocomotionCommand",
    "LocomotionControlState",
    "LocomotionRobot",
    "LocomotionRobotSpecification",
    "PoseAlignmentStore",
    "SerialGripper",
    "XInputCode",
    "apply_pose_alignment_bias",
    "build_command_from_states",
    "build_default_locomotion_robot_specification",
    "build_leg_locomotion_robot_specification",
    "capture_calibration_offsets",
    "capture_pose_alignment_result",
    "compute_pose_alignment_bias",
    "compute_position_offsets",
    "normalized_gripper_target_to_raw_value",
    "read_robot_configuration",
    "remove_pose_alignment_bias",
    "update_limit_readings",
    "write_robot_configuration",
]


_MODULE_EXPORTS = {
    "Bimanual": ("berkeley_humanoid_lite_lowlevel.robot.bimanual", "Bimanual"),
    "CalibrationStore": ("berkeley_humanoid_lite_lowlevel.robot.calibration", "CalibrationStore"),
    "capture_calibration_offsets": (
        "berkeley_humanoid_lite_lowlevel.robot.calibration",
        "capture_calibration_offsets",
    ),
    "compute_position_offsets": (
        "berkeley_humanoid_lite_lowlevel.robot.calibration",
        "compute_position_offsets",
    ),
    "PoseAlignmentStore": (
        "berkeley_humanoid_lite_lowlevel.robot.pose_alignment",
        "PoseAlignmentStore",
    ),
    "capture_pose_alignment_result": (
        "berkeley_humanoid_lite_lowlevel.robot.pose_alignment",
        "capture_pose_alignment_result",
    ),
    "compute_pose_alignment_bias": (
        "berkeley_humanoid_lite_lowlevel.robot.pose_alignment",
        "compute_pose_alignment_bias",
    ),
    "apply_pose_alignment_bias": (
        "berkeley_humanoid_lite_lowlevel.robot.pose_alignment",
        "apply_pose_alignment_bias",
    ),
    "remove_pose_alignment_bias": (
        "berkeley_humanoid_lite_lowlevel.robot.pose_alignment",
        "remove_pose_alignment_bias",
    ),
    "GamepadCommandSource": ("berkeley_humanoid_lite_lowlevel.robot.command_source", "GamepadCommandSource"),
    "LocomotionCommand": ("berkeley_humanoid_lite_lowlevel.robot.command_source", "LocomotionCommand"),
    "XInputCode": ("berkeley_humanoid_lite_lowlevel.robot.command_source", "XInputCode"),
    "build_command_from_states": (
        "berkeley_humanoid_lite_lowlevel.robot.command_source",
        "build_command_from_states",
    ),
    "LocomotionControlState": ("berkeley_humanoid_lite_lowlevel.robot.control_state", "LocomotionControlState"),
    "normalized_gripper_target_to_raw_value": (
        "berkeley_humanoid_lite_lowlevel.robot.gripper",
        "normalized_gripper_target_to_raw_value",
    ),
    "SerialGripper": ("berkeley_humanoid_lite_lowlevel.robot.gripper", "SerialGripper"),
    "read_robot_configuration": (
        "berkeley_humanoid_lite_lowlevel.robot.configuration_io",
        "read_robot_configuration",
    ),
    "update_limit_readings": ("berkeley_humanoid_lite_lowlevel.robot.calibration", "update_limit_readings"),
    "write_robot_configuration": (
        "berkeley_humanoid_lite_lowlevel.robot.configuration_io",
        "write_robot_configuration",
    ),
    "LocomotionRobot": ("berkeley_humanoid_lite_lowlevel.robot.locomotion_runtime", "LocomotionRobot"),
    "JointTransportAddress": (
        "berkeley_humanoid_lite_lowlevel.robot.locomotion_specification",
        "JointTransportAddress",
    ),
    "LocomotionRobotSpecification": (
        "berkeley_humanoid_lite_lowlevel.robot.locomotion_specification",
        "LocomotionRobotSpecification",
    ),
    "build_default_locomotion_robot_specification": (
        "berkeley_humanoid_lite_lowlevel.robot.locomotion_specification",
        "build_default_locomotion_robot_specification",
    ),
    "build_leg_locomotion_robot_specification": (
        "berkeley_humanoid_lite_lowlevel.robot.locomotion_specification",
        "build_leg_locomotion_robot_specification",
    ),
}


def __getattr__(name: str) -> Any:
    if name not in _MODULE_EXPORTS:
        raise AttributeError(f"module {__name__!r} has no attribute {name!r}")

    module_name, attribute_name = _MODULE_EXPORTS[name]
    module = import_module(module_name)
    value = getattr(module, attribute_name)
    globals()[name] = value
    return value
