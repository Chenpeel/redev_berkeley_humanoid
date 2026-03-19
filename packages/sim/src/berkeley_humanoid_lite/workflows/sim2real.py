from __future__ import annotations

def compute_policy_observation_size(num_joints: int) -> int:
    return 7 + num_joints * 2 + 1 + 3


def run_observation_visualizer(configuration: object) -> None:
    from .mujoco import run_mujoco_visualization

    run_mujoco_visualization(configuration)
