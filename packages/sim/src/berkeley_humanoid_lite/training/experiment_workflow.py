from __future__ import annotations

import json
import re
import shlex
import shutil
from collections.abc import Sequence
from dataclasses import dataclass
from datetime import date
from pathlib import Path

from omegaconf import OmegaConf

from .checkpoints import resolve_checkpoint_path
from .paths import get_policy_export_config_path, get_rsl_rl_logs_dir, get_workspace_root

DEFAULT_CHECKPOINT_PATTERN = r"model_.*\.pt"
DEFAULT_BOOTSTRAP_RUN_NAME = "bootstrap_from_stand"


@dataclass(frozen=True)
class ExperimentPreset:
    """描述一条服务器训练/导出实验线。"""

    name: str
    task_name: str
    experiment_name: str
    run_label: str
    description: str
    default_max_iterations: int | None = None
    default_seed: int = 1
    resume_load_run: str | None = None
    resume_checkpoint_pattern: str = DEFAULT_CHECKPOINT_PATTERN


@dataclass(frozen=True)
class StageResult:
    """描述一次 rescue checkpoint staging 的结果。"""

    source_checkpoint_path: Path
    staged_checkpoint_path: Path
    staged_run_directory: Path
    manifest_path: Path


@dataclass(frozen=True)
class ArchiveResult:
    """描述一次导出策略归档的结果。"""

    source_run_directory: Path
    source_policy_path: Path
    source_config_path: Path
    archived_policy_path: Path
    archived_config_path: Path


EXPERIMENT_PRESETS: dict[str, ExperimentPreset] = {
    "velocity-smoke": ExperimentPreset(
        name="velocity-smoke",
        task_name="Velocity-Berkeley-Humanoid-Lite-Biped-v0",
        experiment_name="ft_velocity_biped_gnorm",
        run_label="velocity_smoke",
        description="训练、checkpoint、回放、导出链路冒烟验证。",
        default_max_iterations=200,
    ),
    "velocity-full": ExperimentPreset(
        name="velocity-full",
        task_name="Velocity-Berkeley-Humanoid-Lite-Biped-v0",
        experiment_name="ft_velocity_biped_gnorm",
        run_label="velocity_full",
        description="direct velocity 主线完整训练。",
    ),
    "stand-rescue": ExperimentPreset(
        name="stand-rescue",
        task_name="Stand-Berkeley-Humanoid-Lite-Biped-v0",
        experiment_name="ft_stand_biped_gnorm",
        run_label="stand_rescue",
        description="direct velocity 无法压住零命令段时的救援 stand 训练。",
    ),
    "velocity-from-stand-rescue": ExperimentPreset(
        name="velocity-from-stand-rescue",
        task_name="Velocity-Berkeley-Humanoid-Lite-Biped-v0",
        experiment_name="ft_velocity_biped_gnorm",
        run_label="velocity_from_stand_rescue",
        description="stand rescue 收敛后回到 velocity 恢复步态。",
        resume_load_run=DEFAULT_BOOTSTRAP_RUN_NAME,
    ),
}


def get_experiment_preset(name: str) -> ExperimentPreset:
    """返回命名实验预设。"""
    if name not in EXPERIMENT_PRESETS:
        available_names = ", ".join(sorted(EXPERIMENT_PRESETS))
        raise KeyError(f"Unknown experiment preset {name!r}. Available presets: {available_names}.")
    return EXPERIMENT_PRESETS[name]


def build_run_name(
    preset: ExperimentPreset,
    *,
    seed: int | None = None,
    date_tag: str | None = None,
    run_name: str | None = None,
) -> str:
    """按统一命名规则生成 run_name。"""
    if run_name:
        return run_name

    resolved_seed = preset.default_seed if seed is None else seed
    resolved_date_tag = date.today().isoformat() if date_tag is None else date_tag
    return f"s{resolved_seed}_{preset.run_label}_{resolved_date_tag}"


def build_run_pattern(run_name: str) -> str:
    """返回匹配指定 run_name 的正则模式。"""
    return rf".*{re.escape(run_name)}"


def get_experiment_root(
    experiment_name: str,
    *,
    logs_dir: Path | None = None,
) -> Path:
    """返回实验根目录。"""
    resolved_logs_dir = get_rsl_rl_logs_dir() if logs_dir is None else Path(logs_dir)
    return resolved_logs_dir / experiment_name


def build_train_command(
    preset: ExperimentPreset,
    *,
    python_executable: str,
    workspace_root: Path | None = None,
    seed: int | None = None,
    date_tag: str | None = None,
    run_name: str | None = None,
    max_iterations: int | None = None,
    headless: bool = True,
) -> list[str]:
    """构造训练命令。"""
    resolved_workspace_root = get_workspace_root() if workspace_root is None else Path(workspace_root)
    resolved_seed = preset.default_seed if seed is None else seed
    resolved_run_name = build_run_name(
        preset,
        seed=resolved_seed,
        date_tag=date_tag,
        run_name=run_name,
    )

    command = [
        python_executable,
        str(resolved_workspace_root / "apps" / "rsl_rl" / "train.py"),
        "--task",
        preset.task_name,
        "--experiment_name",
        preset.experiment_name,
        "--run_name",
        resolved_run_name,
        "--seed",
        str(resolved_seed),
    ]

    resolved_max_iterations = preset.default_max_iterations if max_iterations is None else max_iterations
    if resolved_max_iterations is not None:
        command.extend(["--max_iterations", str(resolved_max_iterations)])

    if preset.resume_load_run is not None:
        command.extend(
            [
                "--resume",
                "True",
                "--load_run",
                preset.resume_load_run,
                "--checkpoint",
                preset.resume_checkpoint_pattern,
            ]
        )

    if headless:
        command.append("--headless")
    return command


def build_play_command(
    preset: ExperimentPreset,
    *,
    python_executable: str,
    workspace_root: Path | None = None,
    seed: int | None = None,
    date_tag: str | None = None,
    run_name: str | None = None,
    run_pattern: str | None = None,
    checkpoint_pattern: str = DEFAULT_CHECKPOINT_PATTERN,
    headless: bool = True,
) -> list[str]:
    """构造回放与导出命令。"""
    resolved_workspace_root = get_workspace_root() if workspace_root is None else Path(workspace_root)
    resolved_run_name = build_run_name(
        preset,
        seed=seed,
        date_tag=date_tag,
        run_name=run_name,
    )
    resolved_run_pattern = build_run_pattern(resolved_run_name) if run_pattern is None else run_pattern

    command = [
        python_executable,
        str(resolved_workspace_root / "apps" / "rsl_rl" / "play.py"),
        "--task",
        preset.task_name,
        "--experiment_name",
        preset.experiment_name,
        "--load_run",
        resolved_run_pattern,
        "--checkpoint",
        checkpoint_pattern,
    ]
    if headless:
        command.append("--headless")
    return command


def render_command(command: Sequence[str]) -> str:
    """将命令渲染为适合终端复制的字符串。"""
    return " ".join(shlex.quote(part) for part in command)


def resolve_run_directory(
    experiment_name: str,
    *,
    run_pattern: str,
    checkpoint_pattern: str = DEFAULT_CHECKPOINT_PATTERN,
    logs_dir: Path | None = None,
) -> Path:
    """解析匹配到的 run 目录。"""
    checkpoint_path = resolve_checkpoint_path(
        get_experiment_root(experiment_name, logs_dir=logs_dir),
        run_pattern,
        checkpoint_pattern,
    )
    return checkpoint_path.parent


def stage_checkpoint_for_resume(
    *,
    source_experiment_name: str,
    source_run_pattern: str,
    target_experiment_name: str,
    bootstrap_run_name: str = DEFAULT_BOOTSTRAP_RUN_NAME,
    checkpoint_pattern: str = DEFAULT_CHECKPOINT_PATTERN,
    logs_dir: Path | None = None,
    overwrite: bool = False,
) -> StageResult:
    """把 stand rescue checkpoint staged 到 velocity experiment 根目录。"""
    source_checkpoint_path = resolve_checkpoint_path(
        get_experiment_root(source_experiment_name, logs_dir=logs_dir),
        source_run_pattern,
        checkpoint_pattern,
    )
    staged_run_directory = get_experiment_root(target_experiment_name, logs_dir=logs_dir) / bootstrap_run_name
    staged_run_directory.mkdir(parents=True, exist_ok=True)

    staged_checkpoint_path = staged_run_directory / source_checkpoint_path.name
    if staged_checkpoint_path.exists() and not overwrite:
        raise FileExistsError(
            f"Staged checkpoint already exists: '{staged_checkpoint_path}'. Pass overwrite=True to replace it."
        )
    shutil.copy2(source_checkpoint_path, staged_checkpoint_path)

    for relative_path in (
        Path("params") / "env.yaml",
        Path("params") / "agent.yaml",
        Path("params") / "env.pkl",
        Path("params") / "agent.pkl",
    ):
        source_path = source_checkpoint_path.parent / relative_path
        if source_path.exists():
            target_path = staged_run_directory / relative_path
            target_path.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(source_path, target_path)

    manifest_path = staged_run_directory / "stage_manifest.json"
    manifest_path.write_text(
        json.dumps(
            {
                "source_experiment_name": source_experiment_name,
                "source_run_directory": source_checkpoint_path.parent.as_posix(),
                "source_checkpoint_path": source_checkpoint_path.as_posix(),
                "staged_checkpoint_path": staged_checkpoint_path.as_posix(),
                "bootstrap_run_name": bootstrap_run_name,
            },
            indent=2,
            sort_keys=True,
        )
        + "\n",
        encoding="utf-8",
    )
    return StageResult(
        source_checkpoint_path=source_checkpoint_path,
        staged_checkpoint_path=staged_checkpoint_path,
        staged_run_directory=staged_run_directory,
        manifest_path=manifest_path,
    )


def archive_exported_policy(
    *,
    source_experiment_name: str,
    source_run_pattern: str,
    archive_tag: str,
    checkpoint_pattern: str = DEFAULT_CHECKPOINT_PATTERN,
    workspace_root: Path | None = None,
    logs_dir: Path | None = None,
    latest_config_path: Path | None = None,
) -> ArchiveResult:
    """把 play.py 导出的 policy.onnx 和 policy_latest.yaml 归档成固定文件名。"""
    resolved_workspace_root = get_workspace_root() if workspace_root is None else Path(workspace_root)
    source_run_directory = resolve_run_directory(
        source_experiment_name,
        run_pattern=source_run_pattern,
        checkpoint_pattern=checkpoint_pattern,
        logs_dir=logs_dir,
    )
    source_policy_path = source_run_directory / "exported" / "policy.onnx"
    if not source_policy_path.exists():
        raise FileNotFoundError(f"Exported ONNX policy does not exist: '{source_policy_path}'.")

    resolved_latest_config_path = (
        get_policy_export_config_path() if latest_config_path is None else Path(latest_config_path)
    )
    if not resolved_latest_config_path.exists():
        raise FileNotFoundError(
            f"Deployment config does not exist: '{resolved_latest_config_path}'. Run play.py export first."
        )

    archived_policy_path = resolved_workspace_root / "artifacts" / "checkpoints" / f"policy_biped_{archive_tag}.onnx"
    archived_policy_path.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(source_policy_path, archived_policy_path)

    archived_config_path = resolved_workspace_root / "configs" / "policies" / f"policy_biped_{archive_tag}.yaml"
    archived_config_path.parent.mkdir(parents=True, exist_ok=True)
    configuration = OmegaConf.to_container(OmegaConf.load(resolved_latest_config_path), resolve=True)
    if not isinstance(configuration, dict):
        raise ValueError(f"Expected mapping deployment config, got {type(configuration)!r}.")

    configuration["policy_checkpoint_path"] = archived_policy_path.relative_to(resolved_workspace_root).as_posix()
    OmegaConf.save(configuration, str(archived_config_path))

    return ArchiveResult(
        source_run_directory=source_run_directory,
        source_policy_path=source_policy_path,
        source_config_path=resolved_latest_config_path,
        archived_policy_path=archived_policy_path,
        archived_config_path=archived_config_path,
    )
