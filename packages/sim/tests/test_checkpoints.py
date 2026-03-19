from __future__ import annotations

from pathlib import Path
import tempfile
import unittest

from berkeley_humanoid_lite.training.checkpoints import resolve_checkpoint_path


class CheckpointResolutionTestCase(unittest.TestCase):
    def test_resolve_checkpoint_path_returns_latest_matching_file(self) -> None:
        with tempfile.TemporaryDirectory() as temporary_directory:
            log_root_path = Path(temporary_directory) / "experiment"
            run_a = log_root_path / "2026-01-01_00-00-00"
            run_b = log_root_path / "2026-01-02_00-00-00"
            run_a.mkdir(parents=True)
            run_b.mkdir(parents=True)
            (run_a / "model_0001.pt").write_text("a", encoding="utf-8")
            (run_b / "model_0001.pt").write_text("b", encoding="utf-8")
            (run_b / "model_0010.pt").write_text("c", encoding="utf-8")

            checkpoint_path = resolve_checkpoint_path(log_root_path, ".*", "model_.*\\.pt")

            self.assertEqual(checkpoint_path, run_b / "model_0010.pt")

    def test_resolve_checkpoint_path_raises_when_directory_is_missing(self) -> None:
        with tempfile.TemporaryDirectory() as temporary_directory:
            missing_path = Path(temporary_directory) / "missing"

            with self.assertRaises(FileNotFoundError):
                resolve_checkpoint_path(missing_path, ".*", ".*")


if __name__ == "__main__":
    unittest.main()
