from __future__ import annotations

from .mujoco import run_mujoco_policy_loop as run_shared_mujoco_policy_loop


def run_mujoco_policy_loop(configuration: object) -> None:
    run_shared_mujoco_policy_loop(configuration)
