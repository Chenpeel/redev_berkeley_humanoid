"""Berkeley Humanoid Lite 仿真与任务包。"""

from importlib import import_module
from typing import Any

__all__ = ["TASK_REGISTRATIONS", "register_tasks"]


_MODULE_EXPORTS = {
    "TASK_REGISTRATIONS": ("berkeley_humanoid_lite.tasks", "TASK_REGISTRATIONS"),
    "register_tasks": ("berkeley_humanoid_lite.tasks", "register_tasks"),
}


def __getattr__(name: str) -> Any:
    if name not in _MODULE_EXPORTS:
        raise AttributeError(f"module {__name__!r} has no attribute {name!r}")

    module_name, attribute_name = _MODULE_EXPORTS[name]
    module = import_module(module_name)
    value = getattr(module, attribute_name)
    globals()[name] = value
    return value
