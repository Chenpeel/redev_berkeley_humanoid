from __future__ import annotations

import argparse
import subprocess
import sys

from berkeley_humanoid_lite.training.experiment_workflow import (
    DEFAULT_BOOTSTRAP_RUN_NAME,
    DEFAULT_CHECKPOINT_PATTERN,
    archive_exported_policy,
    build_play_command,
    build_run_name,
    build_run_pattern,
    build_train_command,
    get_experiment_preset,
    render_command,
    stage_checkpoint_for_resume,
)


def _add_common_run_arguments(parser: argparse.ArgumentParser) -> None:
    parser.add_argument("--preset", required=True, help="预设名，如 velocity-smoke / velocity-full / stand-rescue。")
    parser.add_argument("--seed", type=int, default=None, help="覆盖预设 seed。")
    parser.add_argument("--date-tag", type=str, default=None, help="覆盖默认日期标签。")
    parser.add_argument("--run-name", type=str, default=None, help="直接指定 run_name，优先级高于 seed/date-tag。")
    parser.add_argument("--checkpoint-pattern", type=str, default=DEFAULT_CHECKPOINT_PATTERN, help="checkpoint 正则。")
    parser.add_argument("--no-headless", action="store_true", help="禁用 --headless。")
    parser.add_argument("--execute", action="store_true", help="实际执行命令；默认只打印命令。")


def _run_or_print(command: list[str], *, execute: bool) -> int:
    print(render_command(command))
    if not execute:
        return 0
    completed_process = subprocess.run(command, check=False)
    return completed_process.returncode


def main() -> int:
    parser = argparse.ArgumentParser(description="Manage server-ready RSL-RL training workflows.")
    subparsers = parser.add_subparsers(dest="command", required=True)

    show_parser = subparsers.add_parser("show", help="打印指定预设的训练和回放命令。")
    _add_common_run_arguments(show_parser)
    show_parser.add_argument("--max-iterations", type=int, default=None, help="覆盖训练迭代数。")
    show_parser.add_argument("--run-pattern", type=str, default=None, help="覆盖回放阶段的 run 匹配正则。")

    train_parser = subparsers.add_parser("train", help="打印或执行预设训练命令。")
    _add_common_run_arguments(train_parser)
    train_parser.add_argument("--max-iterations", type=int, default=None, help="覆盖训练迭代数。")

    play_parser = subparsers.add_parser("play", help="打印或执行预设回放导出命令。")
    _add_common_run_arguments(play_parser)
    play_parser.add_argument("--run-pattern", type=str, default=None, help="覆盖默认 run 匹配正则。")

    stage_parser = subparsers.add_parser("stage", help="把 rescue checkpoint staged 到目标 experiment。")
    stage_parser.add_argument("--source-preset", required=True, help="来源预设，通常为 stand-rescue。")
    stage_parser.add_argument("--source-seed", type=int, default=None, help="来源 run 的 seed。")
    stage_parser.add_argument("--source-date-tag", type=str, default=None, help="来源 run 的日期标签。")
    stage_parser.add_argument("--source-run-name", type=str, default=None, help="来源 run_name。")
    stage_parser.add_argument("--source-run-pattern", type=str, default=None, help="显式覆盖来源 run 匹配正则。")
    stage_parser.add_argument("--target-experiment", required=True, help="目标 experiment_name。")
    stage_parser.add_argument(
        "--bootstrap-run-name",
        type=str,
        default=DEFAULT_BOOTSTRAP_RUN_NAME,
        help="目标 bootstrap run 目录名。",
    )
    stage_parser.add_argument(
        "--checkpoint-pattern",
        type=str,
        default=DEFAULT_CHECKPOINT_PATTERN,
        help="checkpoint 正则。",
    )
    stage_parser.add_argument("--overwrite", action="store_true", help="允许覆盖已有 staged checkpoint。")

    archive_parser = subparsers.add_parser("archive", help="归档最新导出的 ONNX 与 YAML。")
    archive_parser.add_argument("--preset", required=True, help="来源预设。")
    archive_parser.add_argument("--seed", type=int, default=None, help="来源 run 的 seed。")
    archive_parser.add_argument("--date-tag", type=str, default=None, help="来源 run 的日期标签。")
    archive_parser.add_argument("--run-name", type=str, default=None, help="来源 run_name。")
    archive_parser.add_argument("--run-pattern", type=str, default=None, help="显式覆盖来源 run 匹配正则。")
    archive_parser.add_argument(
        "--checkpoint-pattern",
        type=str,
        default=DEFAULT_CHECKPOINT_PATTERN,
        help="checkpoint 正则。",
    )
    archive_parser.add_argument("--archive-tag", required=True, help="归档标签，如 velocity_full_2026-03-28。")

    arguments = parser.parse_args()
    python_executable = sys.executable

    if arguments.command in {"show", "train", "play"}:
        preset = get_experiment_preset(arguments.preset)
        headless = not arguments.no_headless

        if arguments.command in {"show", "train"}:
            train_command = build_train_command(
                preset,
                python_executable=python_executable,
                seed=arguments.seed,
                date_tag=arguments.date_tag,
                run_name=arguments.run_name,
                max_iterations=arguments.max_iterations,
                headless=headless,
            )
            if arguments.command == "show":
                print("train:")
                print(render_command(train_command))
            else:
                return _run_or_print(train_command, execute=arguments.execute)

        if arguments.command in {"show", "play"}:
            play_command = build_play_command(
                preset,
                python_executable=python_executable,
                seed=arguments.seed,
                date_tag=arguments.date_tag,
                run_name=arguments.run_name,
                run_pattern=arguments.run_pattern,
                checkpoint_pattern=arguments.checkpoint_pattern,
                headless=headless,
            )
            if arguments.command == "show":
                print("play:")
                print(render_command(play_command))
                return 0
            return _run_or_print(play_command, execute=arguments.execute)

    if arguments.command == "stage":
        source_preset = get_experiment_preset(arguments.source_preset)
        source_run_name = build_run_name(
            source_preset,
            seed=arguments.source_seed,
            date_tag=arguments.source_date_tag,
            run_name=arguments.source_run_name,
        )
        source_run_pattern = (
            build_run_pattern(source_run_name) if arguments.source_run_pattern is None else arguments.source_run_pattern
        )
        result = stage_checkpoint_for_resume(
            source_experiment_name=source_preset.experiment_name,
            source_run_pattern=source_run_pattern,
            target_experiment_name=arguments.target_experiment,
            bootstrap_run_name=arguments.bootstrap_run_name,
            checkpoint_pattern=arguments.checkpoint_pattern,
            overwrite=arguments.overwrite,
        )
        print(f"source_checkpoint: {result.source_checkpoint_path}")
        print(f"staged_checkpoint: {result.staged_checkpoint_path}")
        print(f"manifest: {result.manifest_path}")
        return 0

    if arguments.command == "archive":
        preset = get_experiment_preset(arguments.preset)
        resolved_run_name = build_run_name(
            preset,
            seed=arguments.seed,
            date_tag=arguments.date_tag,
            run_name=arguments.run_name,
        )
        resolved_run_pattern = (
            build_run_pattern(resolved_run_name) if arguments.run_pattern is None else arguments.run_pattern
        )
        result = archive_exported_policy(
            source_experiment_name=preset.experiment_name,
            source_run_pattern=resolved_run_pattern,
            archive_tag=arguments.archive_tag,
            checkpoint_pattern=arguments.checkpoint_pattern,
        )
        print(f"source_run_directory: {result.source_run_directory}")
        print(f"archived_policy_path: {result.archived_policy_path}")
        print(f"archived_config_path: {result.archived_config_path}")
        return 0

    raise AssertionError(f"Unhandled command: {arguments.command}")


if __name__ == "__main__":
    raise SystemExit(main())
