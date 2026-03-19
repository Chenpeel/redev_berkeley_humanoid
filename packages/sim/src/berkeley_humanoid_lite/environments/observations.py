from __future__ import annotations

from collections.abc import Sequence
from dataclasses import dataclass

import numpy as np
import torch
from berkeley_humanoid_lite_lowlevel.robot.command_source import LocomotionCommand
from berkeley_humanoid_lite_lowlevel.robot.control_state import LocomotionControlState
from berkeley_humanoid_lite_lowlevel.robot.orientation_math import quat_rotate_inverse

__all__ = [
    "CommandObservationState",
    "SensorLayout",
    "build_policy_observation",
    "quat_rotate_inverse",
    "update_command_state",
]


@dataclass(frozen=True)
class SensorLayout:
    """描述 MuJoCo 传感器数据在观测向量中的切片布局。"""

    num_joints: int
    base_sensor_offset: int

    def joint_positions(self, sensordata: Sequence[float] | np.ndarray | torch.Tensor) -> torch.Tensor:
        return _slice_as_tensor(sensordata, 0, self.num_joints)

    def joint_velocities(self, sensordata: Sequence[float] | np.ndarray | torch.Tensor) -> torch.Tensor:
        return _slice_as_tensor(sensordata, self.num_joints, self.num_joints * 2)

    def base_quaternion(self, sensordata: Sequence[float] | np.ndarray | torch.Tensor) -> torch.Tensor:
        return _slice_as_tensor(sensordata, self.base_sensor_offset, self.base_sensor_offset + 4)

    def base_angular_velocity(self, sensordata: Sequence[float] | np.ndarray | torch.Tensor) -> torch.Tensor:
        return _slice_as_tensor(sensordata, self.base_sensor_offset + 4, self.base_sensor_offset + 7)


@dataclass(frozen=True)
class CommandObservationState:
    mode: float
    velocity_x: float
    velocity_y: float
    velocity_yaw: float

    def as_tensor(self) -> torch.Tensor:
        return torch.tensor(
            [self.mode, self.velocity_x, self.velocity_y, self.velocity_yaw],
            dtype=torch.float32,
        )


def _slice_as_tensor(
    sensordata: Sequence[float] | np.ndarray | torch.Tensor,
    start: int,
    end: int,
) -> torch.Tensor:
    return torch.as_tensor(sensordata[start:end], dtype=torch.float32)


def update_command_state(
    command: LocomotionCommand,
    *,
    current_mode: float,
    lateral_velocity_scale: float = 0.5,
) -> CommandObservationState:
    """将实时控制命令转换成策略观测里稳定的命令状态。"""
    mode = current_mode
    if command.requested_state != LocomotionControlState.INVALID:
        mode = float(command.requested_state)

    return CommandObservationState(
        mode=mode,
        velocity_x=command.velocity_x,
        velocity_y=command.velocity_y * lateral_velocity_scale,
        velocity_yaw=command.velocity_yaw,
    )


def build_policy_observation(
    *,
    base_quat: torch.Tensor,
    base_ang_vel: torch.Tensor,
    joint_positions: torch.Tensor,
    joint_velocities: torch.Tensor,
    action_indices: Sequence[int],
    command_state: CommandObservationState,
) -> torch.Tensor:
    """按策略约定顺序拼装观测向量。"""
    indices = list(action_indices)
    return torch.cat(
        [
            base_quat,
            base_ang_vel,
            joint_positions[indices],
            joint_velocities[indices],
            command_state.as_tensor(),
        ],
        dim=-1,
    )
