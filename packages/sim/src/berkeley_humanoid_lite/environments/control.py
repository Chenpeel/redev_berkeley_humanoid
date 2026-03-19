from __future__ import annotations

from collections.abc import Sequence

import torch


def compute_pd_torques(
    actions: torch.Tensor,
    *,
    num_joints: int,
    action_indices: Sequence[int],
    joint_positions: torch.Tensor,
    joint_velocities: torch.Tensor,
    joint_kp: torch.Tensor,
    joint_kd: torch.Tensor,
    effort_limits: torch.Tensor,
) -> torch.Tensor:
    """根据动作目标和关节状态计算裁剪后的 PD 力矩。"""
    target_positions = torch.zeros(
        (num_joints,),
        dtype=joint_positions.dtype,
        device=joint_positions.device,
    )
    target_positions[list(action_indices)] = torch.as_tensor(
        actions,
        dtype=joint_positions.dtype,
        device=joint_positions.device,
    )
    output_torques = joint_kp * (target_positions - joint_positions) - joint_kd * joint_velocities
    return torch.clip(output_torques, -effort_limits, effort_limits)
