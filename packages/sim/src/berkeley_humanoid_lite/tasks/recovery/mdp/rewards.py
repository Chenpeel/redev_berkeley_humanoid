from __future__ import annotations

import re
from typing import TYPE_CHECKING

import torch
from isaaclab.managers import SceneEntityCfg
from isaaclab.sensors import ContactSensor

from ._state import get_recovery_state
from .observations import upright_score

if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedRLEnv


def base_height_error_l1(env: ManagerBasedRLEnv, *, target_height: float) -> torch.Tensor:
    """Reward the base staying close to the desired recovery height."""

    asset = env.scene["robot"]
    return torch.abs(asset.data.root_pos_w[:, 2] - target_height)


def base_height_progress(env: ManagerBasedRLEnv, *, bucket_names: tuple[str, ...]) -> torch.Tensor:
    """Reward positive base-height changes across recovery steps."""

    asset = env.scene["robot"]
    state = get_recovery_state(env, bucket_names)
    current_height = asset.data.root_pos_w[:, 2]
    progress = (current_height - state.previous_base_height).clamp_min(0.0)
    state.previous_base_height.copy_(current_height)
    return progress


def upright_progress(env: ManagerBasedRLEnv, *, bucket_names: tuple[str, ...]) -> torch.Tensor:
    """Reward positive improvement in uprightness."""

    state = get_recovery_state(env, bucket_names)
    current_upright = upright_score(env)
    progress = (current_upright - state.previous_upright_score).clamp_min(0.0)
    state.previous_upright_score.copy_(current_upright)
    return progress


def _resolve_body_names(contact_sensor: ContactSensor, body_ids: object) -> list[str]:
    sensor_body_names = list(getattr(contact_sensor, "body_names", []))
    if body_ids == slice(None):
        return sensor_body_names
    return [sensor_body_names[index] for index in body_ids]


def _allowed_contact_mask(
    contact_sensor: ContactSensor,
    body_ids: object,
    allowed_contact_patterns: tuple[str, ...],
) -> torch.Tensor:
    body_names = _resolve_body_names(contact_sensor, body_ids)
    if not body_names:
        return torch.zeros(0, dtype=torch.bool)
    if not allowed_contact_patterns:
        return torch.zeros(len(body_names), dtype=torch.bool)

    compiled = [re.compile(pattern) for pattern in allowed_contact_patterns]
    return torch.tensor(
        [any(pattern.fullmatch(name) for pattern in compiled) for name in body_names],
        dtype=torch.bool,
    )


def support_contact_transition(
    env: ManagerBasedRLEnv,
    *,
    sensor_cfg: SceneEntityCfg,
    allowed_contact_patterns: tuple[str, ...],
    threshold: float = 1.0,
) -> torch.Tensor:
    """Reward episodes that concentrate support on allowed transition contacts."""

    contact_sensor: ContactSensor = env.scene.sensors[sensor_cfg.name]
    contact_forces = contact_sensor.data.net_forces_w_history[:, :, sensor_cfg.body_ids, :].norm(dim=-1).max(dim=1)[0]
    allowed_mask = _allowed_contact_mask(contact_sensor, sensor_cfg.body_ids, allowed_contact_patterns).to(env.device)
    if allowed_mask.numel() == 0:
        return torch.zeros(env.num_envs, device=env.device)

    active_contacts = contact_forces > threshold
    allowed_contacts = active_contacts[:, allowed_mask]
    disallowed_contacts = active_contacts[:, ~allowed_mask]
    allowed_score = allowed_contacts.float().sum(dim=1)
    disallowed_penalty = disallowed_contacts.float().sum(dim=1)
    return (allowed_score - disallowed_penalty).clamp_min(0.0)


def success_hold_bonus(
    env: ManagerBasedRLEnv,
    *,
    bucket_names: tuple[str, ...],
    hold_duration_s: float,
    minimum_height: float,
    maximum_tilt_l2: float,
) -> torch.Tensor:
    """Reward holding a stable upright posture for a configured duration."""

    asset = env.scene["robot"]
    state = get_recovery_state(env, bucket_names)
    current_height = asset.data.root_pos_w[:, 2]
    current_upright = upright_score(env)
    current_tilt = 1.0 - current_upright

    hold_condition = (current_height >= minimum_height) & (current_tilt <= maximum_tilt_l2)
    state.success_hold_steps = torch.where(
        hold_condition,
        state.success_hold_steps + 1,
        torch.zeros_like(state.success_hold_steps),
    )

    required_steps = max(int(round(hold_duration_s / env.step_dt)), 1)
    return (state.success_hold_steps >= required_steps).to(dtype=torch.float32)
