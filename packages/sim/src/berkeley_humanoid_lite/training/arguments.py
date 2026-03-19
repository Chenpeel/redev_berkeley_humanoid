from __future__ import annotations

import argparse
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from isaaclab_rl.rsl_rl import RslRlOnPolicyRunnerCfg


def add_rsl_rl_args(parser: argparse.ArgumentParser) -> None:
    """向解析器追加 RSL-RL 参数。"""
    argument_group = parser.add_argument_group("rsl_rl", description="Arguments for the RSL-RL agent.")
    argument_group.add_argument(
        "--experiment_name",
        type=str,
        default=None,
        help="Name of the experiment folder where logs are stored.",
    )
    argument_group.add_argument("--run_name", type=str, default=None, help="Run name suffix.")
    argument_group.add_argument("--resume", type=bool, default=None, help="Whether to resume from a checkpoint.")
    argument_group.add_argument("--load_run", type=str, default=None, help="Run directory pattern to resume from.")
    argument_group.add_argument("--checkpoint", type=str, default=None, help="Checkpoint filename pattern.")
    argument_group.add_argument(
        "--logger",
        type=str,
        default=None,
        choices={"wandb", "tensorboard", "neptune"},
        help="Logger module to use.",
    )
    argument_group.add_argument(
        "--log_project_name",
        type=str,
        default=None,
        help="Project name when using wandb or neptune.",
    )


def parse_rsl_rl_cfg(task_name: str, arguments: argparse.Namespace) -> RslRlOnPolicyRunnerCfg:
    """按任务名读取并覆写 RSL-RL 配置。"""
    from isaaclab_tasks.utils.parse_cfg import load_cfg_from_registry

    runner_configuration: RslRlOnPolicyRunnerCfg = load_cfg_from_registry(task_name, "rsl_rl_cfg_entry_point")
    return update_rsl_rl_cfg(runner_configuration, arguments)


def to_rsl_rl_runner_cfg(agent_cfg: RslRlOnPolicyRunnerCfg) -> dict[str, object]:
    """转成 rsl_rl runner 可直接消费的 dict。"""
    agent_cfg_dict = agent_cfg.to_dict()
    policy_cfg = agent_cfg_dict.get("policy", {})
    if policy_cfg.get("state_dependent_std") is False:
        policy_cfg.pop("state_dependent_std", None)
    return agent_cfg_dict


def update_rsl_rl_cfg(agent_cfg: RslRlOnPolicyRunnerCfg, arguments: argparse.Namespace) -> RslRlOnPolicyRunnerCfg:
    """用 CLI 参数覆写 RSL-RL 配置。"""
    if hasattr(arguments, "seed") and arguments.seed is not None:
        agent_cfg.seed = arguments.seed
    if arguments.experiment_name is not None:
        agent_cfg.experiment_name = arguments.experiment_name
    if arguments.resume is not None:
        agent_cfg.resume = arguments.resume
    if arguments.load_run is not None:
        agent_cfg.load_run = arguments.load_run
    if arguments.checkpoint is not None:
        agent_cfg.load_checkpoint = arguments.checkpoint
    if arguments.run_name is not None:
        agent_cfg.run_name = arguments.run_name
    if arguments.logger is not None:
        agent_cfg.logger = arguments.logger
    if agent_cfg.logger in {"wandb", "neptune"} and arguments.log_project_name:
        agent_cfg.wandb_project = arguments.log_project_name
        agent_cfg.neptune_project = arguments.log_project_name
    return agent_cfg
