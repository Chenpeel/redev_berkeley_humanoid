from __future__ import annotations

import argparse
from pathlib import Path
from typing import Any, Mapping

from omegaconf import DictConfig, OmegaConf

from berkeley_humanoid_lite_lowlevel.runtime_paths import get_policy_config_path, resolve_workspace_path


PolicyDeploymentConfiguration = DictConfig


def create_policy_deployment_configuration(values: Mapping[str, Any]) -> PolicyDeploymentConfiguration:
    configuration = OmegaConf.create(dict(values))
    if not isinstance(configuration, DictConfig):
        raise TypeError("策略配置必须是映射类型。")
    return configuration


def load_policy_deployment_configuration(config_path: str | Path) -> PolicyDeploymentConfiguration:
    resolved_config_path = resolve_workspace_path(config_path)
    print("Loading config file from", resolved_config_path)

    with open(resolved_config_path, "r", encoding="utf-8") as file:
        configuration = OmegaConf.load(file)

    if not isinstance(configuration, DictConfig):
        raise TypeError(f"策略配置必须是映射类型: {resolved_config_path}")
    return configuration


def load_policy_deployment_configuration_from_cli(
    default_file_name: str = "policy_biped_50hz.yaml",
) -> PolicyDeploymentConfiguration:
    parser = argparse.ArgumentParser(description="Policy deployment configuration loader")
    parser.add_argument(
        "--config",
        type=str,
        default=str(get_policy_config_path(default_file_name)),
        help="Path to the deployment configuration file",
    )
    arguments = parser.parse_args()
    return load_policy_deployment_configuration(arguments.config)
