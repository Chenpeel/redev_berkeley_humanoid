from __future__ import annotations

from pathlib import Path
import tempfile
import unittest

from berkeley_humanoid_lite.training.artifacts import dump_pickle


class ArtifactExportTestCase(unittest.TestCase):
    def test_dump_pickle_returns_none_for_unpicklable_local_object(self) -> None:
        class LocalConfig:
            pass

        with tempfile.TemporaryDirectory() as temporary_directory:
            output_path = Path(temporary_directory) / "local.pkl"
            result = dump_pickle(output_path, LocalConfig())

            self.assertIsNone(result)
            self.assertFalse(output_path.exists())


if __name__ == "__main__":
    unittest.main()
