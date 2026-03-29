from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import mujoco
import mujoco.viewer
from berkeley_humanoid_lite_assets.paths import get_mjcf_path
from berkeley_humanoid_lite_assets.robots import get_variant_for_joint_count


@dataclass(frozen=True)
class MujocoSession:
    model: Any
    data: Any
    viewer: Any


def resolve_mjcf_scene_path(num_joints: int) -> Path:
    """根据关节数量解析 MuJoCo 场景文件路径。"""
    robot_variant = get_variant_for_joint_count(num_joints)
    return get_mjcf_path(robot_variant.mjcf_scene_file_name)


def create_mujoco_session(
    *,
    num_joints: int,
    physics_dt: float,
    model_loader: Callable[[str], Any] = mujoco.MjModel.from_xml_path,
    data_factory: Callable[[Any], Any] = mujoco.MjData,
    viewer_factory: Callable[[Any, Any], Any] = mujoco.viewer.launch_passive,
) -> MujocoSession:
    """创建 MuJoCo model/data/viewer 会话。"""
    model = model_loader(str(resolve_mjcf_scene_path(num_joints)))
    data = data_factory(model)
    model.opt.timestep = physics_dt
    viewer = viewer_factory(model, data)
    return MujocoSession(model=model, data=data, viewer=viewer)


def step_mujoco_session(
    session: MujocoSession,
    *,
    step_fn: Callable[[Any, Any], None] = mujoco.mj_step,
) -> None:
    """执行一次 MuJoCo 物理步进。"""
    step_fn(session.model, session.data)


def forward_mujoco_session(
    session: MujocoSession,
    *,
    forward_fn: Callable[[Any, Any], None] = mujoco.mj_forward,
) -> None:
    """在不推进时间的情况下刷新 MuJoCo 派生状态和传感器缓存。"""
    forward_fn(session.model, session.data)


def sync_mujoco_viewer(session: MujocoSession) -> None:
    """刷新 MuJoCo passive viewer。"""
    session.viewer.sync()


def close_mujoco_session(session: MujocoSession) -> None:
    """防御式关闭 MuJoCo viewer。"""
    close_method = getattr(session.viewer, "close", None)
    if callable(close_method):
        close_method()
