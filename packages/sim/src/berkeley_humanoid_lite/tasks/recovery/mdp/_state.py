from __future__ import annotations

from dataclasses import dataclass

import torch

_RECOVERY_STATE_KEY = "_berkeley_recovery_state"


@dataclass
class RecoveryRuntimeState:
    """Per-environment runtime buffers shared by recovery rewards and observations."""

    bucket_names: tuple[str, ...]
    bucket_ids: torch.Tensor
    success_hold_steps: torch.Tensor
    previous_base_height: torch.Tensor
    previous_upright_score: torch.Tensor


def get_recovery_state(env: object, bucket_names: tuple[str, ...]) -> RecoveryRuntimeState:
    """Create or fetch the shared recovery runtime state."""

    device = getattr(env, "device", None)
    if device is None and hasattr(env, "scene"):
        device = getattr(env.scene, "device", None)
    if device is None:
        device = torch.device("cpu")

    num_envs = getattr(env, "num_envs", None)
    if num_envs is None and hasattr(env, "scene"):
        num_envs = env.scene.num_envs
    if num_envs is None:
        raise AttributeError("Recovery state initialization requires env.num_envs or env.scene.num_envs.")

    state = getattr(env, _RECOVERY_STATE_KEY, None)
    if state is None or state.bucket_names != bucket_names or state.bucket_ids.shape[0] != num_envs:
        state = RecoveryRuntimeState(
            bucket_names=bucket_names,
            bucket_ids=torch.zeros(num_envs, dtype=torch.long, device=device),
            success_hold_steps=torch.zeros(num_envs, dtype=torch.long, device=device),
            previous_base_height=torch.zeros(num_envs, dtype=torch.float32, device=device),
            previous_upright_score=torch.zeros(num_envs, dtype=torch.float32, device=device),
        )
        setattr(env, _RECOVERY_STATE_KEY, state)
    return state


def reset_recovery_metrics(
    state: RecoveryRuntimeState,
    env_ids: torch.Tensor,
    *,
    base_height: torch.Tensor | None = None,
    upright_score: torch.Tensor | None = None,
) -> None:
    """Reset per-episode buffers for the selected environments."""

    state.success_hold_steps[env_ids] = 0
    if base_height is None:
        state.previous_base_height[env_ids] = 0.0
    else:
        state.previous_base_height[env_ids] = base_height
    if upright_score is None:
        state.previous_upright_score[env_ids] = 0.0
    else:
        state.previous_upright_score[env_ids] = upright_score
