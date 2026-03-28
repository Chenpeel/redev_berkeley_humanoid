from __future__ import annotations

import tempfile
import unittest
from pathlib import Path

from berkeley_humanoid_lite.training.experiment_workflow import (
    archive_exported_policy,
    build_play_command,
    build_run_name,
    build_train_command,
    get_experiment_preset,
    stage_checkpoint_for_resume,
)
from omegaconf import OmegaConf


class ExperimentWorkflowTestCase(unittest.TestCase):
    def test_build_train_command_uses_velocity_smoke_defaults(self) -> None:
        preset = get_experiment_preset("velocity-smoke")

        command = build_train_command(
            preset,
            python_executable="/usr/bin/python3",
            workspace_root=Path("/workspace"),
            date_tag="2026-03-28",
        )

        self.assertEqual(command[0], "/usr/bin/python3")
        self.assertIn("/workspace/apps/rsl_rl/train.py", command)
        self.assertIn("Velocity-Berkeley-Humanoid-Lite-Biped-v0", command)
        self.assertIn("ft_velocity_biped_gnorm", command)
        self.assertIn("s1_velocity_smoke_2026-03-28", command)
        self.assertIn("200", command)
        self.assertIn("--headless", command)

    def test_build_play_command_uses_run_name_pattern_for_velocity_full(self) -> None:
        preset = get_experiment_preset("velocity-full")

        command = build_play_command(
            preset,
            python_executable="/usr/bin/python3",
            workspace_root=Path("/workspace"),
            date_tag="2026-03-28",
        )

        self.assertEqual(command[0], "/usr/bin/python3")
        self.assertIn("/workspace/apps/rsl_rl/play.py", command)
        self.assertIn("ft_velocity_biped_gnorm", command)
        self.assertIn(r".*s1_velocity_full_2026\-03\-28", command)
        self.assertIn(r"model_.*\.pt", command)

    def test_stage_checkpoint_for_resume_copies_checkpoint_params_and_manifest(self) -> None:
        with tempfile.TemporaryDirectory() as temporary_directory:
            logs_dir = Path(temporary_directory) / "artifacts" / "untested_ckpts" / "rsl_rl"
            source_run_directory = logs_dir / "ft_stand_biped_gnorm" / "2026-03-29_10-00-00_s1_stand_rescue_2026-03-29"
            source_run_directory.mkdir(parents=True)
            checkpoint_path = source_run_directory / "model_0200.pt"
            checkpoint_path.write_text("checkpoint", encoding="utf-8")

            params_directory = source_run_directory / "params"
            params_directory.mkdir()
            (params_directory / "env.yaml").write_text("env: 1\n", encoding="utf-8")
            (params_directory / "agent.yaml").write_text("agent: 1\n", encoding="utf-8")

            result = stage_checkpoint_for_resume(
                source_experiment_name="ft_stand_biped_gnorm",
                source_run_pattern=r".*s1_stand_rescue_2026-03-29",
                target_experiment_name="ft_velocity_biped_gnorm",
                logs_dir=logs_dir,
            )

            self.assertTrue(result.staged_checkpoint_path.exists())
            self.assertEqual(result.staged_checkpoint_path.read_text(encoding="utf-8"), "checkpoint")
            self.assertTrue((result.staged_run_directory / "params" / "env.yaml").exists())
            self.assertTrue((result.staged_run_directory / "params" / "agent.yaml").exists())
            manifest = result.manifest_path.read_text(encoding="utf-8")
            self.assertIn("ft_stand_biped_gnorm", manifest)
            self.assertIn("bootstrap_from_stand", manifest)

    def test_archive_exported_policy_copies_onnx_and_rewrites_checkpoint_path(self) -> None:
        with tempfile.TemporaryDirectory() as temporary_directory:
            workspace_root = Path(temporary_directory)
            logs_dir = workspace_root / "artifacts" / "untested_ckpts" / "rsl_rl"
            source_run_directory = (
                logs_dir / "ft_velocity_biped_gnorm" / "2026-03-28_10-00-00_s1_velocity_full_2026-03-28"
            )
            exported_directory = source_run_directory / "exported"
            exported_directory.mkdir(parents=True)
            (exported_directory / "policy.onnx").write_text("onnx", encoding="utf-8")
            (source_run_directory / "model_0600.pt").write_text("checkpoint", encoding="utf-8")

            latest_config_path = workspace_root / "configs" / "policies" / "policy_latest.yaml"
            latest_config_path.parent.mkdir(parents=True)
            OmegaConf.save(
                {
                    "policy_checkpoint_path": "artifacts/untested_ckpts/rsl_rl/tmp/exported/policy.onnx",
                    "num_actions": 12,
                },
                str(latest_config_path),
            )

            result = archive_exported_policy(
                source_experiment_name="ft_velocity_biped_gnorm",
                source_run_pattern=r".*s1_velocity_full_2026-03-28",
                archive_tag="velocity_full_2026-03-28",
                workspace_root=workspace_root,
                logs_dir=logs_dir,
                latest_config_path=latest_config_path,
            )

            self.assertTrue(result.archived_policy_path.exists())
            self.assertEqual(result.archived_policy_path.read_text(encoding="utf-8"), "onnx")
            archived_configuration = OmegaConf.to_container(OmegaConf.load(result.archived_config_path), resolve=True)
            self.assertEqual(
                archived_configuration["policy_checkpoint_path"],
                "artifacts/checkpoints/policy_biped_velocity_full_2026-03-28.onnx",
            )

    def test_build_run_name_can_override_with_explicit_run_name(self) -> None:
        preset = get_experiment_preset("stand-rescue")

        self.assertEqual(
            build_run_name(preset, run_name="manual_name"),
            "manual_name",
        )


if __name__ == "__main__":
    unittest.main()
