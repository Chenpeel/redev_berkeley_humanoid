from __future__ import annotations

from importlib import import_module
from typing import Any

__all__ = [
    "SUPPORTED_FAKE_TELEOP_PATTERNS",
    "TeleopHandInput",
    "TeleopInputFrame",
    "build_fake_bridge_data",
    "build_fake_input_frame",
    "TeleoperationIkSolver",
]


_MODULE_EXPORTS = {
    "SUPPORTED_FAKE_TELEOP_PATTERNS": (
        "berkeley_humanoid_lite_lowlevel.teleoperation.input_frame",
        "SUPPORTED_FAKE_TELEOP_PATTERNS",
    ),
    "TeleopHandInput": (
        "berkeley_humanoid_lite_lowlevel.teleoperation.input_frame",
        "TeleopHandInput",
    ),
    "TeleopInputFrame": (
        "berkeley_humanoid_lite_lowlevel.teleoperation.input_frame",
        "TeleopInputFrame",
    ),
    "build_fake_bridge_data": (
        "berkeley_humanoid_lite_lowlevel.teleoperation.input_frame",
        "build_fake_bridge_data",
    ),
    "build_fake_input_frame": (
        "berkeley_humanoid_lite_lowlevel.teleoperation.input_frame",
        "build_fake_input_frame",
    ),
    "TeleoperationIkSolver": (
        "berkeley_humanoid_lite_lowlevel.teleoperation.solver",
        "TeleoperationIkSolver",
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
