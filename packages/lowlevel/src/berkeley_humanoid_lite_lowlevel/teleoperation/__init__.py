from __future__ import annotations

from importlib import import_module
from typing import Any


__all__ = ["TeleoperationIkSolver"]


_MODULE_EXPORTS = {
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
