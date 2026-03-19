from __future__ import annotations

from pathlib import Path
import pickle
from typing import Final


_PICKLE_FALLBACK_EXCEPTIONS: Final = (
    AttributeError,
    pickle.PickleError,
    TypeError,
)


def dump_pickle(path: str | Path, data: object) -> Path | None:
    """尽力写入 pkl 文件；对象不可序列化时返回 `None`。"""
    resolved_path = Path(path)
    if resolved_path.suffix != ".pkl":
        resolved_path = resolved_path.with_suffix(".pkl")
    resolved_path.parent.mkdir(parents=True, exist_ok=True)
    try:
        with resolved_path.open("wb") as file:
            pickle.dump(data, file)
    except _PICKLE_FALLBACK_EXCEPTIONS as error:
        print(f"[WARN] Skip pickle export for '{resolved_path.name}': {error}")
        if resolved_path.exists():
            resolved_path.unlink()
        return None
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
