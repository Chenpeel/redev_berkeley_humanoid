from __future__ import annotations

from dataclasses import dataclass
from typing import Any

import gymnasium as gym

from berkeley_humanoid_lite.tasks.balance.recovery.config.biped import agents as biped_recovery_agents
from berkeley_humanoid_lite.tasks.balance.recovery.config.biped import env_cfg as biped_recovery_env_cfg
from berkeley_humanoid_lite.tasks.balance.recovery.config.humanoid import agents as humanoid_recovery_agents
from berkeley_humanoid_lite.tasks.balance.recovery.config.humanoid import env_cfg as humanoid_recovery_env_cfg
from berkeley_humanoid_lite.tasks.balance.stand.config.biped import agents as biped_stand_agents
from berkeley_humanoid_lite.tasks.balance.stand.config.biped import env_cfg as biped_stand_env_cfg
from berkeley_humanoid_lite.tasks.balance.stand.config.humanoid import agents as humanoid_stand_agents
from berkeley_humanoid_lite.tasks.balance.stand.config.humanoid import env_cfg as humanoid_stand_env_cfg
from berkeley_humanoid_lite.tasks.locomotion.velocity.config.biped import agents as biped_agents
from berkeley_humanoid_lite.tasks.locomotion.velocity.config.biped import env_cfg as biped_env_cfg
from berkeley_humanoid_lite.tasks.locomotion.velocity.config.humanoid import agents as humanoid_agents
from berkeley_humanoid_lite.tasks.locomotion.velocity.config.humanoid import env_cfg as humanoid_env_cfg


@dataclass(frozen=True)
class TaskRegistration:
    """描述一个需要显式注册的 Gym 任务。"""

    task_id: str
    entry_point: str
    env_cfg_entry_point: Any
    runner_cfg_entry_point: Any
    disable_env_checker: bool = True

    def as_kwargs(self) -> dict[str, Any]:
        """返回传给 `gym.register` 的关键字参数。"""
        return {
            "entry_point": self.entry_point,
            "disable_env_checker": self.disable_env_checker,
            "kwargs": {
                "env_cfg_entry_point": self.env_cfg_entry_point,
                "rsl_rl_cfg_entry_point": self.runner_cfg_entry_point,
            },
        }


TASK_REGISTRATIONS: tuple[TaskRegistration, ...] = (
    TaskRegistration(
        task_id="Velocity-Berkeley-Humanoid-Lite-v0",
        entry_point="isaaclab.envs:ManagerBasedRLEnv",
        env_cfg_entry_point=humanoid_env_cfg.BerkeleyHumanoidLiteEnvCfg,
        runner_cfg_entry_point=humanoid_agents.rsl_rl_ppo_cfg.BerkeleyHumanoidLitePPORunnerCfg,
    ),
    TaskRegistration(
        task_id="Velocity-Berkeley-Humanoid-Lite-Biped-v0",
        entry_point="isaaclab.envs:ManagerBasedRLEnv",
        env_cfg_entry_point=biped_env_cfg.BerkeleyHumanoidLiteBipedEnvCfg,
        runner_cfg_entry_point=biped_agents.rsl_rl_ppo_cfg.BerkeleyHumanoidLiteBipedPPORunnerCfg,
    ),
    TaskRegistration(
        task_id="Stand-Berkeley-Humanoid-Lite-v0",
        entry_point="isaaclab.envs:ManagerBasedRLEnv",
        env_cfg_entry_point=humanoid_stand_env_cfg.BerkeleyHumanoidLiteStandEnvCfg,
        runner_cfg_entry_point=humanoid_stand_agents.rsl_rl_ppo_cfg.BerkeleyHumanoidLiteStandPPORunnerCfg,
    ),
    TaskRegistration(
        task_id="Stand-Berkeley-Humanoid-Lite-Biped-v0",
        entry_point="isaaclab.envs:ManagerBasedRLEnv",
        env_cfg_entry_point=biped_stand_env_cfg.BerkeleyHumanoidLiteBipedStandEnvCfg,
        runner_cfg_entry_point=biped_stand_agents.rsl_rl_ppo_cfg.BerkeleyHumanoidLiteBipedStandPPORunnerCfg,
    ),
    TaskRegistration(
        task_id="Recovery-Berkeley-Humanoid-Lite-v0",
        entry_point="isaaclab.envs:ManagerBasedRLEnv",
        env_cfg_entry_point=humanoid_recovery_env_cfg.BerkeleyHumanoidLiteRecoveryEnvCfg,
        runner_cfg_entry_point=humanoid_recovery_agents.rsl_rl_ppo_cfg.BerkeleyHumanoidLiteRecoveryPPORunnerCfg,
    ),
    TaskRegistration(
        task_id="Recovery-Berkeley-Humanoid-Lite-Biped-v0",
        entry_point="isaaclab.envs:ManagerBasedRLEnv",
        env_cfg_entry_point=biped_recovery_env_cfg.BerkeleyHumanoidLiteBipedRecoveryEnvCfg,
        runner_cfg_entry_point=biped_recovery_agents.rsl_rl_ppo_cfg.BerkeleyHumanoidLiteBipedRecoveryPPORunnerCfg,
    ),
)


def register_tasks(force: bool = False) -> tuple[str, ...]:
    """显式注册所有任务，并返回已注册任务 ID。"""
    registered_task_ids: list[str] = []
    for registration in TASK_REGISTRATIONS:
        _register_task(registration, force=force)
        registered_task_ids.append(registration.task_id)
    return tuple(registered_task_ids)


def _register_task(registration: TaskRegistration, force: bool) -> None:
    if registration.task_id in gym.registry:
        if not force:
            return
        gym.registry.pop(registration.task_id, None)

    gym.register(id=registration.task_id, **registration.as_kwargs())
