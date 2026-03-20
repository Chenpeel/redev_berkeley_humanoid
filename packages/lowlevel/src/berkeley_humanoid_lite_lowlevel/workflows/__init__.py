from __future__ import annotations

from importlib import import_module
from typing import Any


__all__ = [
    "check_actuator_connection",
    "configure_actuator",
    "export_actuator_configuration",
    "run_actuator_calibration",
    "run_actuator_motion_demo",
    "run_joint_calibration",
    "apply_robot_configuration",
    "export_robot_configuration",
    "stream_orientation",
    "check_locomotion_connection",
    "run_idle_stream",
    "run_locomotion_loop",
    "run_policy_inference_smoke_test",
    "stream_gamepad_commands",
    "broadcast_gamepad_commands",
    "check_teleoperation_connection",
    "run_teleoperation_idle_loop",
    "run_teleoperation_loop",
    "run_teleoperation_solver_demo",
    "stream_gripper_targets",
]


_MODULE_EXPORTS = {
    "check_actuator_connection": (
        "berkeley_humanoid_lite_lowlevel.workflows.actuator",
        "check_actuator_connection",
    ),
    "configure_actuator": (
        "berkeley_humanoid_lite_lowlevel.workflows.actuator",
        "configure_actuator",
    ),
    "export_actuator_configuration": (
        "berkeley_humanoid_lite_lowlevel.workflows.actuator",
        "export_actuator_configuration",
    ),
    "run_actuator_calibration": (
        "berkeley_humanoid_lite_lowlevel.workflows.actuator",
        "run_actuator_calibration",
    ),
    "run_actuator_motion_demo": (
        "berkeley_humanoid_lite_lowlevel.workflows.actuator",
        "run_actuator_motion_demo",
    ),
    "run_joint_calibration": (
        "berkeley_humanoid_lite_lowlevel.workflows.calibration",
        "run_joint_calibration",
    ),
    "apply_robot_configuration": (
        "berkeley_humanoid_lite_lowlevel.workflows.configuration",
        "apply_robot_configuration",
    ),
    "export_robot_configuration": (
        "berkeley_humanoid_lite_lowlevel.workflows.configuration",
        "export_robot_configuration",
    ),
    "stream_orientation": (
        "berkeley_humanoid_lite_lowlevel.workflows.imu",
        "stream_orientation",
    ),
    "check_locomotion_connection": (
        "berkeley_humanoid_lite_lowlevel.workflows.locomotion",
        "check_locomotion_connection",
    ),
    "run_idle_stream": (
        "berkeley_humanoid_lite_lowlevel.workflows.locomotion",
        "run_idle_stream",
    ),
    "run_locomotion_loop": (
        "berkeley_humanoid_lite_lowlevel.workflows.locomotion",
        "run_locomotion_loop",
    ),
    "run_policy_inference_smoke_test": (
        "berkeley_humanoid_lite_lowlevel.workflows.locomotion",
        "run_policy_inference_smoke_test",
    ),
    "stream_gamepad_commands": (
        "berkeley_humanoid_lite_lowlevel.workflows.locomotion",
        "stream_gamepad_commands",
    ),
    "broadcast_gamepad_commands": (
        "berkeley_humanoid_lite_lowlevel.workflows.locomotion",
        "broadcast_gamepad_commands",
    ),
    "check_teleoperation_connection": (
        "berkeley_humanoid_lite_lowlevel.workflows.teleoperation",
        "check_teleoperation_connection",
    ),
    "run_teleoperation_idle_loop": (
        "berkeley_humanoid_lite_lowlevel.workflows.teleoperation",
        "run_teleoperation_idle_loop",
    ),
    "run_teleoperation_loop": (
        "berkeley_humanoid_lite_lowlevel.workflows.teleoperation",
        "run_teleoperation_loop",
    ),
    "run_teleoperation_solver_demo": (
        "berkeley_humanoid_lite_lowlevel.workflows.teleoperation",
        "run_teleoperation_solver_demo",
    ),
    "stream_gripper_targets": (
        "berkeley_humanoid_lite_lowlevel.workflows.teleoperation",
        "stream_gripper_targets",
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
