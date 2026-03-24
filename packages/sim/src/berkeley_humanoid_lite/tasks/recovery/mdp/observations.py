from __future__ import annotations

from typing import TYPE_CHECKING

import torch
from isaaclab.managers import SceneEntityCfg
from isaaclab.sensors import ContactSensor

import berkeley_humanoid_lite.tasks.locomotion.velocity.mdp as locomotion_mdp

from ._state import get_recovery_state

if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedRLEnv


def upright_score(env: ManagerBasedRLEnv) -> torch.Tensor:
    """Return a normalized uprightness score in [0, 1]."""

    projected_gravity = locomotion_mdp.projected_gravity(env)
    lateral_tilt = torch.norm(projected_gravity[:, :2], dim=1)
    return (1.0 - lateral_tilt).clamp(0.0, 1.0)


def recovery_bucket_one_hot(env: ManagerBasedRLEnv, bucket_names: tuple[str, ...]) -> torch.Tensor:
    """Encode the sampled recovery bucket as a one-hot vector."""

    state = get_recovery_state(env, bucket_names)
    one_hot = torch.zeros(env.num_envs, len(bucket_names), device=env.device, dtype=torch.float32)
    if bucket_names:
        one_hot.scatter_(1, state.bucket_ids.unsqueeze(-1), 1.0)
    return one_hot


def recovery_phase_progress(
    env: ManagerBasedRLEnv,
    *,
    target_height: float,
    minimum_height: float,
) -> torch.Tensor:
    """Estimate coarse recovery progress from height and trunk alignment."""

    asset = env.scene["robot"]
    base_height = asset.data.root_pos_w[:, 2]
    height_ratio = (base_height - minimum_height) / max(target_height - minimum_height, 1.0e-6)
    progress = 0.5 * upright_score(env) + 0.5 * height_ratio.clamp(0.0, 1.0)
    return progress.clamp(0.0, 1.0).unsqueeze(-1)


def success_hold_progress(
    env: ManagerBasedRLEnv,
    *,
    bucket_names: tuple[str, ...],
    hold_duration_s: float,
) -> torch.Tensor:
    """Return the normalized success-hold counter for each environment."""

    state = get_recovery_state(env, bucket_names)
    required_steps = max(int(round(hold_duration_s / env.step_dt)), 1)
    progress = state.success_hold_steps.to(dtype=torch.float32) / float(required_steps)
    return progress.clamp(0.0, 1.0).unsqueeze(-1)


def contact_state_map(
    env: ManagerBasedRLEnv,
    *,
    sensor_cfg: SceneEntityCfg,
    threshold: float = 1.0,
) -> torch.Tensor:
    """Expose a binary contact map for selected bodies."""

    contact_sensor: ContactSensor = env.scene.sensors[sensor_cfg.name]
    contact_forces = contact_sensor.data.net_forces_w_history[:, :, sensor_cfg.body_ids, :].norm(dim=-1).max(dim=1)[0]
    return (contact_forces > threshold).to(dtype=torch.float32)
