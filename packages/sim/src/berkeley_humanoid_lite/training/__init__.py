from importlib import import_module
from typing import Any

__all__ = [
    "add_rsl_rl_args",
    "build_hydra_runtime_args",
    "build_policy_deployment_configuration",
    "dump_pickle",
    "dump_training_configuration",
    "export_policy_models",
    "get_policy_export_config_path",
    "get_rsl_rl_logs_dir",
    "parse_rsl_rl_cfg",
    "resolve_checkpoint_path",
    "run_policy_playback",
    "run_training",
    "save_policy_deployment_configuration",
    "to_rsl_rl_runner_cfg",
    "update_rsl_rl_cfg",
]


_MODULE_EXPORTS = {
    "add_rsl_rl_args": ("berkeley_humanoid_lite.training.arguments", "add_rsl_rl_args"),
    "build_hydra_runtime_args": ("berkeley_humanoid_lite.training.arguments", "build_hydra_runtime_args"),
    "parse_rsl_rl_cfg": ("berkeley_humanoid_lite.training.arguments", "parse_rsl_rl_cfg"),
    "to_rsl_rl_runner_cfg": ("berkeley_humanoid_lite.training.arguments", "to_rsl_rl_runner_cfg"),
    "update_rsl_rl_cfg": ("berkeley_humanoid_lite.training.arguments", "update_rsl_rl_cfg"),
    "dump_pickle": ("berkeley_humanoid_lite.training.artifacts", "dump_pickle"),
    "dump_training_configuration": (
        "berkeley_humanoid_lite.training.artifacts",
        "dump_training_configuration",
    ),
    "save_policy_deployment_configuration": (
        "berkeley_humanoid_lite.training.artifacts",
        "save_policy_deployment_configuration",
    ),
    "resolve_checkpoint_path": ("berkeley_humanoid_lite.training.checkpoints", "resolve_checkpoint_path"),
    "build_policy_deployment_configuration": (
        "berkeley_humanoid_lite.training.export",
        "build_policy_deployment_configuration",
    ),
    "export_policy_models": ("berkeley_humanoid_lite.training.export", "export_policy_models"),
    "get_policy_export_config_path": ("berkeley_humanoid_lite.training.paths", "get_policy_export_config_path"),
    "get_rsl_rl_logs_dir": ("berkeley_humanoid_lite.training.paths", "get_rsl_rl_logs_dir"),
    "run_policy_playback": ("berkeley_humanoid_lite.training.workflows", "run_policy_playback"),
    "run_training": ("berkeley_humanoid_lite.training.workflows", "run_training"),
}


def __getattr__(name: str) -> Any:
    if name not in _MODULE_EXPORTS:
        raise AttributeError(f"module {__name__!r} has no attribute {name!r}")

    module_name, attribute_name = _MODULE_EXPORTS[name]
    module = import_module(module_name)
    value = getattr(module, attribute_name)
    globals()[name] = value
    return value
