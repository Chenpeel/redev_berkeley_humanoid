from __future__ import annotations

import tempfile
import unittest
from pathlib import Path

import numpy as np
from berkeley_humanoid_lite_lowlevel.robot.calibration import (
    CalibrationStore,
    capture_calibration_offsets,
    compute_position_offsets,
    update_limit_readings,
)
from berkeley_humanoid_lite_lowlevel.robot.control_state import LocomotionControlState
from berkeley_humanoid_lite_lowlevel.robot.locomotion_specification import (
    JointTransportAddress,
    LocomotionRobotSpecification,
)


class CalibrationTestCase(unittest.TestCase):
    def _build_specification(self) -> LocomotionRobotSpecification:
        return LocomotionRobotSpecification(
            joint_addresses=(
                JointTransportAddress("can0", 1, "joint_1"),
                JointTransportAddress("can0", 2, "joint_2"),
            ),
            mirrored_joint_pairs=((0, 1),),
            joint_axis_directions=np.array([1.0, 1.0], dtype=np.float32),
            initialization_positions=np.zeros((2,), dtype=np.float32),
            calibration_reference_positions=np.zeros((2,), dtype=np.float32),
            calibration_limit_selectors=("min", "min"),
        )

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

    def test_capture_calibration_offsets_fails_when_initial_measurements_are_missing(self) -> None:
        class FakeActuatorArray:
            position_measurements_complete = False
            missing_position_measurement_names = ("joint_1", "joint_2")

            def read_positions(self) -> np.ndarray:
                return np.zeros((2,), dtype=np.float32)

        class FakeCommandSource:
            def snapshot(self) -> object:
                return type("Snapshot", (), {"requested_state": LocomotionControlState.IDLE})()

        with self.assertRaisesRegex(
            RuntimeError,
            "Missing joints: joint_1, joint_2",
        ):
            capture_calibration_offsets(
                self._build_specification(),
                FakeActuatorArray(),
                FakeCommandSource(),
            )


if __name__ == "__main__":
    unittest.main()
