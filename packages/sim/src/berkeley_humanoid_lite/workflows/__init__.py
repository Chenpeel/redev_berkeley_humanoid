from importlib import import_module
from typing import Any

__all__ = [
    "compute_policy_observation_size",
    "run_mujoco_policy_loop",
    "run_observation_visualizer",
]


_MODULE_EXPORTS = {
    "compute_policy_observation_size": (
        "berkeley_humanoid_lite.workflows.sim2real",
        "compute_policy_observation_size",
    ),
    "run_mujoco_policy_loop": ("berkeley_humanoid_lite.workflows.sim2sim", "run_mujoco_policy_loop"),
    "run_observation_visualizer": (
        "berkeley_humanoid_lite.workflows.sim2real",
        "run_observation_visualizer",
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
