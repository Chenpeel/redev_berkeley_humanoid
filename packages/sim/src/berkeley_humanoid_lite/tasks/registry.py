from __future__ import annotations

import importlib
import importlib.util
from dataclasses import dataclass
from typing import Any

import gymnasium as gym

from berkeley_humanoid_lite.tasks.balance.recovery.config.biped import agents as biped_push_recovery_agents
from berkeley_humanoid_lite.tasks.balance.recovery.config.biped import env_cfg as biped_push_recovery_env_cfg
from berkeley_humanoid_lite.tasks.balance.recovery.config.humanoid import agents as humanoid_push_recovery_agents
from berkeley_humanoid_lite.tasks.balance.recovery.config.humanoid import env_cfg as humanoid_push_recovery_env_cfg
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


def _module_exists(module_name: str) -> bool:
    try:
        return importlib.util.find_spec(module_name) is not None
    except ModuleNotFoundError:
        return False


def _build_optional_task_registration(
    *,
    task_id: str,
    env_module_name: str,
    env_cfg_class_name: str,
    runner_module_name: str,
    runner_cfg_class_name: str,
) -> TaskRegistration | None:
    if not _module_exists(env_module_name) or not _module_exists(runner_module_name):
        return None

    env_module = importlib.import_module(env_module_name)
    runner_module = importlib.import_module(runner_module_name)
    return TaskRegistration(
        task_id=task_id,
        entry_point="isaaclab.envs:ManagerBasedRLEnv",
        env_cfg_entry_point=getattr(env_module, env_cfg_class_name),
        runner_cfg_entry_point=getattr(runner_module, runner_cfg_class_name),
    )


_BASE_TASK_REGISTRATIONS: tuple[TaskRegistration, ...] = (
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
        task_id="Push-Recovery-Berkeley-Humanoid-Lite-v0",
        entry_point="isaaclab.envs:ManagerBasedRLEnv",
        env_cfg_entry_point=humanoid_push_recovery_env_cfg.BerkeleyHumanoidLitePushRecoveryEnvCfg,
        runner_cfg_entry_point=humanoid_push_recovery_agents.rsl_rl_ppo_cfg.BerkeleyHumanoidLitePushRecoveryPPORunnerCfg,
    ),
    TaskRegistration(
        task_id="Push-Recovery-Berkeley-Humanoid-Lite-Biped-v0",
        entry_point="isaaclab.envs:ManagerBasedRLEnv",
        env_cfg_entry_point=biped_push_recovery_env_cfg.BerkeleyHumanoidLiteBipedPushRecoveryEnvCfg,
        runner_cfg_entry_point=biped_push_recovery_agents.rsl_rl_ppo_cfg.BerkeleyHumanoidLiteBipedPushRecoveryPPORunnerCfg,
    ),
)


def _build_optional_recovery_task_registrations() -> tuple[TaskRegistration, ...]:
    optional_registrations = (
        _build_optional_task_registration(
            task_id="Posture-Recovery-Berkeley-Humanoid-Lite-v0",
            env_module_name="berkeley_humanoid_lite.tasks.recovery.posture_recovery.config.humanoid.env_cfg",
            env_cfg_class_name="BerkeleyHumanoidLitePostureRecoveryEnvCfg",
            runner_module_name="berkeley_humanoid_lite.tasks.recovery.posture_recovery.config.humanoid.agents.rsl_rl_ppo_cfg",
            runner_cfg_class_name="BerkeleyHumanoidLitePostureRecoveryPPORunnerCfg",
        ),
        _build_optional_task_registration(
            task_id="Posture-Recovery-Berkeley-Humanoid-Lite-Biped-v0",
            env_module_name="berkeley_humanoid_lite.tasks.recovery.posture_recovery.config.biped.env_cfg",
            env_cfg_class_name="BerkeleyHumanoidLiteBipedPostureRecoveryEnvCfg",
            runner_module_name="berkeley_humanoid_lite.tasks.recovery.posture_recovery.config.biped.agents.rsl_rl_ppo_cfg",
            runner_cfg_class_name="BerkeleyHumanoidLiteBipedPostureRecoveryPPORunnerCfg",
        ),
        _build_optional_task_registration(
            task_id="Getup-Berkeley-Humanoid-Lite-v0",
            env_module_name="berkeley_humanoid_lite.tasks.recovery.getup.config.humanoid.env_cfg",
            env_cfg_class_name="BerkeleyHumanoidLiteGetupEnvCfg",
            runner_module_name="berkeley_humanoid_lite.tasks.recovery.getup.config.humanoid.agents.rsl_rl_ppo_cfg",
            runner_cfg_class_name="BerkeleyHumanoidLiteGetupPPORunnerCfg",
        ),
        _build_optional_task_registration(
            task_id="Getup-Berkeley-Humanoid-Lite-Biped-v0",
            env_module_name="berkeley_humanoid_lite.tasks.recovery.getup.config.biped.env_cfg",
            env_cfg_class_name="BerkeleyHumanoidLiteBipedGetupEnvCfg",
            runner_module_name="berkeley_humanoid_lite.tasks.recovery.getup.config.biped.agents.rsl_rl_ppo_cfg",
            runner_cfg_class_name="BerkeleyHumanoidLiteBipedGetupPPORunnerCfg",
        ),
    )
    return tuple(registration for registration in optional_registrations if registration is not None)


TASK_REGISTRATIONS: tuple[TaskRegistration, ...] = (
    *_BASE_TASK_REGISTRATIONS,
    *_build_optional_recovery_task_registrations(),
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
