from __future__ import annotations

import re
from typing import TYPE_CHECKING

import torch
from isaaclab.managers import SceneEntityCfg
from isaaclab.sensors import ContactSensor

from ._state import get_recovery_state

if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedRLEnv


def base_height_below_threshold(env: ManagerBasedRLEnv, *, minimum_height: float) -> torch.Tensor:
    """Terminate when the base collapses below the allowed recovery floor."""

    asset = env.scene["robot"]
    return asset.data.root_pos_w[:, 2] < minimum_height


def dangerous_body_contact(
    env: ManagerBasedRLEnv,
    *,
    sensor_cfg: SceneEntityCfg,
    threshold: float,
) -> torch.Tensor:
    """Terminate when explicitly dangerous body parts collide with large force."""

    contact_sensor: ContactSensor = env.scene.sensors[sensor_cfg.name]
    contact_forces = contact_sensor.data.net_forces_w_history[:, :, sensor_cfg.body_ids, :].norm(dim=-1).max(dim=1)[0]
    return torch.any(contact_forces > threshold, dim=1)


def disallowed_body_contact(
    env: ManagerBasedRLEnv,
    *,
    sensor_cfg: SceneEntityCfg,
    threshold: float,
    bucket_names: tuple[str, ...],
    bucket_allowed_contact_patterns: tuple[tuple[str, ...], ...],
) -> torch.Tensor:
    """Terminate on contacts not permitted by the currently active recovery bucket."""

    contact_sensor: ContactSensor = env.scene.sensors[sensor_cfg.name]
    contact_forces = contact_sensor.data.net_forces_w_history[:, :, sensor_cfg.body_ids, :].norm(dim=-1).max(dim=1)[0]

    sensor_body_names = list(getattr(contact_sensor, "body_names", []))
    if sensor_cfg.body_ids == slice(None):
        selected_body_names = sensor_body_names
    else:
        selected_body_names = [sensor_body_names[index] for index in sensor_cfg.body_ids]

    state = get_recovery_state(env, bucket_names)
    terminated = torch.zeros(env.num_envs, dtype=torch.bool, device=env.device)

    for bucket_index, allowed_patterns in enumerate(bucket_allowed_contact_patterns):
        env_mask = state.bucket_ids == bucket_index
        if not torch.any(env_mask):
            continue

        if not allowed_patterns:
            allowed_mask = torch.zeros(len(selected_body_names), dtype=torch.bool, device=env.device)
        else:
            compiled = [re.compile(pattern) for pattern in allowed_patterns]
            allowed_mask = torch.tensor(
                [any(pattern.fullmatch(name) for pattern in compiled) for name in selected_body_names],
                dtype=torch.bool,
                device=env.device,
            )

        disallowed_contacts = contact_forces[env_mask][:, ~allowed_mask]
        if disallowed_contacts.numel() == 0:
            continue
        terminated[env_mask] = torch.any(disallowed_contacts > threshold, dim=1)

    return terminated
