from __future__ import annotations

from dataclasses import asdict, is_dataclass
from typing import TYPE_CHECKING

import torch

import berkeley_humanoid_lite.tasks.locomotion.velocity.mdp as locomotion_mdp

from ._state import get_recovery_state, reset_recovery_metrics
from .observations import upright_score

if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedEnv


def _ensure_env_ids(env: ManagerBasedEnv, env_ids: torch.Tensor | None) -> torch.Tensor:
    if env_ids is not None:
        return env_ids
    return torch.arange(env.scene.num_envs, device=env.device)


def _profile_dict(profile: object) -> dict[str, object]:
    if is_dataclass(profile):
        return asdict(profile)
    return dict(profile)


def select_pose_bucket(
    env: ManagerBasedEnv,
    env_ids: torch.Tensor | None,
    *,
    bucket_names: tuple[str, ...],
    sampling_weights: tuple[float, ...] | None = None,
) -> None:
    """Sample and store the active reset bucket for each environment."""

    env_ids = _ensure_env_ids(env, env_ids)
    state = get_recovery_state(env, bucket_names)
    if not bucket_names:
        return

    weights = sampling_weights if sampling_weights is not None else tuple(1.0 for _ in bucket_names)
    probs = torch.tensor(weights, dtype=torch.float32, device=env.device)
    probs = probs / probs.sum().clamp_min(1.0e-6)
    sampled = torch.multinomial(probs, num_samples=env_ids.numel(), replacement=True)
    state.bucket_ids[env_ids] = sampled
    reset_recovery_metrics(state, env_ids)


def reset_to_pose_buckets(
    env: ManagerBasedEnv,
    env_ids: torch.Tensor | None,
    *,
    buckets: tuple[object, ...],
) -> None:
    """Reset root state and joints using per-bucket distributions."""

    env_ids = _ensure_env_ids(env, env_ids)
    bucket_names = tuple(str(_profile_dict(bucket)["name"]) for bucket in buckets)
    sampling_weights = tuple(float(_profile_dict(bucket).get("sampling_weight", 1.0)) for bucket in buckets)
    state = get_recovery_state(env, bucket_names)

    # Always re-sample on reset so each episode can change recovery posture.
    select_pose_bucket(env, env_ids, bucket_names=bucket_names, sampling_weights=sampling_weights)

    for bucket_index, bucket in enumerate(buckets):
        bucket_profile = _profile_dict(bucket)
        bucket_mask = state.bucket_ids[env_ids] == bucket_index
        bucket_env_ids = env_ids[bucket_mask]
        if bucket_env_ids.numel() == 0:
            continue

        locomotion_mdp.reset_root_state_uniform(
            env,
            bucket_env_ids,
            pose_range=bucket_profile["pose_range"],
            velocity_range=bucket_profile["velocity_range"],
        )
        locomotion_mdp.reset_joints_by_scale(
            env,
            bucket_env_ids,
            position_range=bucket_profile["joint_position_range"],
            velocity_range=bucket_profile["joint_velocity_range"],
        )

    asset = env.scene["robot"]
    base_height = asset.data.root_pos_w[:, 2]
    current_upright = upright_score(env)
    reset_recovery_metrics(state, env_ids, base_height=base_height[env_ids], upright_score=current_upright[env_ids])
