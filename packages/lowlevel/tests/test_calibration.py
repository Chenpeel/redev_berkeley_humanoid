from __future__ import annotations

import tempfile
import unittest
from pathlib import Path

import numpy as np

from berkeley_humanoid_lite_lowlevel.robot.calibration import (
    CalibrationStore,
    compute_position_offsets,
    update_limit_readings,
)


class CalibrationTestCase(unittest.TestCase):
    def test_update_limit_readings_uses_selectors(self) -> None:
        limit_readings = np.array([1.0, 1.0, 1.0], dtype=np.float32)
        joint_readings = np.array([0.5, 2.0, 0.8], dtype=np.float32)
        selectors = ("min", "max", "min")

        updated_readings = update_limit_readings(limit_readings, joint_readings, selectors)

        np.testing.assert_allclose(updated_readings, np.array([0.5, 2.0, 0.8], dtype=np.float32))

    def test_compute_position_offsets_returns_difference(self) -> None:
        limit_readings = np.array([1.2, -0.4], dtype=np.float32)
        reference_positions = np.array([1.0, -0.5], dtype=np.float32)

        offsets = compute_position_offsets(limit_readings, reference_positions)

        np.testing.assert_allclose(
            offsets,
            np.array([0.2, 0.1], dtype=np.float32),
            rtol=1e-6,
            atol=1e-8,
        )

    def test_calibration_store_round_trip(self) -> None:
        offsets = np.array([0.25, -0.5], dtype=np.float32)

        with tempfile.TemporaryDirectory() as temporary_directory:
            store = CalibrationStore(Path(temporary_directory) / "calibration.yaml")
            saved_path = store.save_position_offsets(offsets)
            reloaded_offsets = store.load_position_offsets(joint_count=2)

        self.assertTrue(saved_path.name.endswith("calibration.yaml"))
        np.testing.assert_allclose(reloaded_offsets, offsets)


if __name__ == "__main__":
    unittest.main()
