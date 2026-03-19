from __future__ import annotations

import time
from collections.abc import Callable, Sequence
from typing import Protocol

import numpy as np

_DEFAULT_BASE_QUATERNION = np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float32)
_ZERO_BASE_LINEAR_VELOCITY = np.zeros(3, dtype=np.float32)


class MujocoStateBuffers(Protocol):
    qpos: np.ndarray
    qvel: np.ndarray


def compute_physics_substeps(policy_dt: float, physics_dt: float) -> int:
    """根据策略频率和物理频率计算每个策略周期的物理子步数。"""
    if policy_dt <= 0.0:
        raise ValueError("policy_dt 必须为正数。")
    if physics_dt <= 0.0:
        raise ValueError("physics_dt 必须为正数。")
    return int(np.round(policy_dt / physics_dt))


def reset_visualizer_state(
    state: MujocoStateBuffers,
    *,
    num_dofs: int,
) -> None:
    """将 visualizer 的运行时状态重置为原点静止姿态。"""
    state.qpos[0:3] = 0.0
    state.qpos[3:7] = _DEFAULT_BASE_QUATERNION
    state.qpos[7 : 7 + num_dofs] = 0.0
    state.qvel[:] = 0.0


def apply_visualizer_observation(
    state: MujocoStateBuffers,
    robot_observations: Sequence[float] | np.ndarray,
    *,
    num_dofs: int,
) -> None:
    """把 real-to-sim 观测写回 MuJoCo runtime 缓冲区。"""
    observation_array = np.asarray(robot_observations, dtype=np.float32)
    joint_position_end = 7 + num_dofs
    joint_velocity_end = joint_position_end + num_dofs

    state.qpos[0:3] = 0.0
    state.qpos[3:7] = observation_array[0:4]
    state.qvel[0:3] = _ZERO_BASE_LINEAR_VELOCITY
    state.qvel[3:6] = observation_array[4:7]
    state.qpos[7:] = observation_array[7:joint_position_end]
    state.qvel[6:] = observation_array[joint_position_end:joint_velocity_end]


def reset_simulator_state(
    state: MujocoStateBuffers,
    *,
    default_base_position: Sequence[float] | np.ndarray,
    default_joint_positions: Sequence[float] | np.ndarray,
) -> None:
    """把 simulator 的运行时状态重置为部署配置里的默认姿态。"""
    state.qpos[0:3] = np.asarray(default_base_position, dtype=np.float32)
    state.qpos[3:7] = _DEFAULT_BASE_QUATERNION
    state.qpos[7:] = np.asarray(default_joint_positions, dtype=np.float32)
    state.qvel[:] = 0.0


def compute_remaining_step_time(
    *,
    policy_dt: float,
    step_duration: float,
) -> float:
    """根据目标策略周期和本轮耗时计算剩余等待时间。"""
    if policy_dt <= 0.0:
        raise ValueError("policy_dt 必须为正数。")
    return max(policy_dt - step_duration, 0.0)


def pace_policy_step(
    policy_dt: float,
    step_start_time: float,
    *,
    time_fn: Callable[[], float] = time.perf_counter,
    sleep_fn: Callable[[float], None] = time.sleep,
) -> float:
    """根据策略周期节流当前 step，返回本轮实际休眠时间。"""
    elapsed_time = time_fn() - step_start_time
    remaining_time = compute_remaining_step_time(
        policy_dt=policy_dt,
        step_duration=elapsed_time,
    )
    if remaining_time > 0.0:
        sleep_fn(remaining_time)
    return remaining_time
