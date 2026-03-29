from __future__ import annotations

from pathlib import Path


def get_workspace_root() -> Path:
    """返回主仓库根目录。"""
    current = Path(__file__).resolve()
    for candidate in current.parents:
        if (candidate / "pyproject.toml").exists() and (candidate / "packages").exists():
            return candidate
    raise FileNotFoundError("无法定位主仓库根目录。")


def resolve_workspace_path(path: str | Path) -> Path:
    """将相对路径解析到主仓库根目录。"""
    candidate = Path(path)
    if candidate.is_absolute():
        return candidate

    current_working_directory_path = (Path.cwd() / candidate).resolve()
    if current_working_directory_path.exists():
        return current_working_directory_path

    return (get_workspace_root() / candidate).resolve()


def get_configs_dir() -> Path:
    return get_workspace_root() / "configs"


def get_artifacts_dir() -> Path:
    return get_workspace_root() / "artifacts"


def get_hardware_configs_dir() -> Path:
    return get_configs_dir() / "hardware"


def get_policy_config_path(file_name: str = "policy_biped_50hz.yaml") -> Path:
    return get_configs_dir() / "policies" / file_name


def get_calibration_path(file_name: str = "calibration.yaml") -> Path:
    return get_artifacts_dir() / "calibration" / file_name


def get_pose_alignment_path(file_name: str = "locomotion_pose_alignment.yaml") -> Path:
    return get_artifacts_dir() / "calibration" / file_name


def get_hardware_config_path(file_name: str) -> Path:
    return get_hardware_configs_dir() / file_name


def ensure_parent_directory(path: str | Path) -> Path:
    resolved_path = resolve_workspace_path(path)
    resolved_path.parent.mkdir(parents=True, exist_ok=True)
    return resolved_path
