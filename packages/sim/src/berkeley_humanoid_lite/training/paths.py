from __future__ import annotations

from pathlib import Path


_WORKSPACE_ROOT = Path(__file__).resolve().parents[5]


def get_workspace_root() -> Path:
    return _WORKSPACE_ROOT


def get_rsl_rl_logs_dir() -> Path:
    return _WORKSPACE_ROOT / "artifacts" / "logs" / "rsl_rl"


def get_policy_export_config_path() -> Path:
    return _WORKSPACE_ROOT / "configs" / "policies" / "policy_latest.yaml"
