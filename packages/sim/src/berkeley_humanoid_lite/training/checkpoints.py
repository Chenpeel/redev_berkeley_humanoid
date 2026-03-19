from __future__ import annotations

from pathlib import Path
import re


def _list_directories(path: Path) -> list[str]:
    if not path.is_dir():
        return []
    return sorted(entry.name for entry in path.iterdir() if entry.is_dir())


def _list_files(path: Path) -> list[str]:
    if not path.is_dir():
        return []
    return sorted(entry.name for entry in path.iterdir() if entry.is_file())


def resolve_checkpoint_path(log_root_path: str | Path, load_run: str, load_checkpoint: str) -> Path:
    """解析要恢复的 checkpoint 路径。"""
    resolved_log_root_path = Path(log_root_path)
    if not resolved_log_root_path.is_dir():
        experiments_root = resolved_log_root_path.parent
        available_experiments = _list_directories(experiments_root)
        message = [
            f"Experiment log directory does not exist: '{resolved_log_root_path}'.",
            "Run `python apps/rsl_rl/train.py --task <task>` first, or pass `--experiment_name` to an existing directory.",
        ]
        if available_experiments:
            message.append(
                f"Available experiment directories under '{experiments_root}': {', '.join(available_experiments)}."
            )
        else:
            message.append(f"No experiment directories were found under '{experiments_root}'.")
        raise FileNotFoundError(" ".join(message))

    matching_runs = [
        entry
        for entry in resolved_log_root_path.iterdir()
        if entry.is_dir() and re.match(load_run, entry.name)
    ]
    matching_runs.sort()
    if not matching_runs:
        available_runs = _list_directories(resolved_log_root_path)
        message = [f"Unable to resolve a run from '{resolved_log_root_path}' with `--load_run {load_run}`."]
        if available_runs:
            message.append(f"Available run directories: {', '.join(available_runs)}.")
        else:
            message.append("No run directories were found. Train a policy first.")
        raise FileNotFoundError(" ".join(message))

    candidate_runs: list[tuple[Path, list[str]]] = []
    for run_path in matching_runs:
        model_checkpoints = [file_name for file_name in _list_files(run_path) if re.match(load_checkpoint, file_name)]
        if model_checkpoints:
            model_checkpoints.sort(key=lambda file_name: f"{file_name:0>15}")
            candidate_runs.append((run_path, model_checkpoints))

    if not candidate_runs:
        selected_run = matching_runs[-1]
        message = [
            f"Unable to resolve a checkpoint from '{selected_run}' with "
            f"`--load_run {load_run}` and `--checkpoint {load_checkpoint}`."
        ]
        available_files = _list_files(selected_run)
        if available_files:
            message.append(f"Available files in the selected run: {', '.join(available_files)}.")
        else:
            message.append("The selected run directory is empty.")
        raise FileNotFoundError(" ".join(message))

    candidate_runs.sort(key=lambda item: item[0].as_posix())
    run_path, model_checkpoints = candidate_runs[-1]
    return run_path / model_checkpoints[-1]
