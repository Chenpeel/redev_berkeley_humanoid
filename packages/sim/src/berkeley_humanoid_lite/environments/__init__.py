from __future__ import annotations

from .observations import quat_rotate_inverse

__all__ = ["MujocoEnv", "MujocoSimulator", "MujocoVisualizer", "quat_rotate_inverse"]


def __getattr__(name: str) -> object:
    if name in {"MujocoEnv", "MujocoSimulator", "MujocoVisualizer"}:
        from .mujoco import MujocoEnv, MujocoSimulator, MujocoVisualizer

        exports = {
            "MujocoEnv": MujocoEnv,
            "MujocoSimulator": MujocoSimulator,
            "MujocoVisualizer": MujocoVisualizer,
        }
        return exports[name]
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
