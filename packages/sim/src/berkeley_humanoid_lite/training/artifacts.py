from __future__ import annotations

from pathlib import Path
import pickle


def dump_pickle(path: str | Path, data: object) -> Path:
    """将对象安全写入 pkl 文件。"""
    resolved_path = Path(path)
    if resolved_path.suffix != ".pkl":
        resolved_path = resolved_path.with_suffix(".pkl")
    resolved_path.parent.mkdir(parents=True, exist_ok=True)
    with resolved_path.open("wb") as file:
        pickle.dump(data, file)
    return resolved_path


def dump_training_configuration(log_dir: str | Path, env_cfg: object, agent_cfg: object) -> None:
    """导出训练用 env/agent 配置。"""
    from isaaclab.utils.io import dump_yaml

    resolved_log_dir = Path(log_dir)
    parameters_directory = resolved_log_dir / "params"
    parameters_directory.mkdir(parents=True, exist_ok=True)

    dump_yaml(str(parameters_directory / "env.yaml"), env_cfg)
    dump_yaml(str(parameters_directory / "agent.yaml"), agent_cfg)
    dump_pickle(parameters_directory / "env.pkl", env_cfg)
    dump_pickle(parameters_directory / "agent.pkl", agent_cfg)


def save_policy_deployment_configuration(configuration: dict[str, object], output_path: str | Path) -> Path:
    """保存部署配置 YAML。"""
    from omegaconf import OmegaConf

    resolved_output_path = Path(output_path)
    resolved_output_path.parent.mkdir(parents=True, exist_ok=True)
    OmegaConf.save(configuration, str(resolved_output_path))
    return resolved_output_path
