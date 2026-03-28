from __future__ import annotations

import tempfile
import unittest
from pathlib import Path

import numpy as np
from berkeley_humanoid_lite_lowlevel.robot.calibration import (
    CalibrationStore,
    capture_calibration_offsets,
    capture_calibration_result,
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
            standing_positions=np.zeros((2,), dtype=np.float32),
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

    def test_calibration_store_saves_metadata_without_breaking_offset_loading(self) -> None:
        offsets = np.array([0.1, -0.2], dtype=np.float32)

        with tempfile.TemporaryDirectory() as temporary_directory:
            calibration_path = Path(temporary_directory) / "calibration.yaml"
            store = CalibrationStore(calibration_path)
            store.save_position_offsets(
                offsets,
                metadata={
                    "schema_version": 2,
                    "capture": {"sample_count": 20},
                },
            )
            reloaded_offsets = store.load_position_offsets(joint_count=2)

            saved_text = calibration_path.read_text(encoding="utf-8")

        np.testing.assert_allclose(reloaded_offsets, offsets)
        self.assertIn("schema_version: 2", saved_text)
        self.assertIn("sample_count: 20", saved_text)

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

    def test_capture_calibration_result_uses_recent_window_average(self) -> None:
        reference = np.array([0.0, 0.0], dtype=np.float32)
        stable_samples = [
            np.array([0.010, -0.015], dtype=np.float32),
            np.array([0.011, -0.014], dtype=np.float32),
            np.array([0.009, -0.016], dtype=np.float32),
            np.array([0.010, -0.015], dtype=np.float32),
        ]

        class FakeActuatorArray:
            position_measurements_complete = True

            def __init__(self) -> None:
                self._samples = [np.array([0.45, -0.35], dtype=np.float32), *stable_samples]

            def read_positions(self) -> np.ndarray:
                if len(self._samples) > 1:
                    return self._samples.pop(0)
                return self._samples[0]

        class FakeCommandSource:
            def __init__(self) -> None:
                self._states = [
                    LocomotionControlState.INVALID,
                    LocomotionControlState.INVALID,
                    LocomotionControlState.INVALID,
                    LocomotionControlState.INVALID,
                    LocomotionControlState.IDLE,
                ]

            def snapshot(self) -> object:
                requested_state = self._states.pop(0)
                return type("Snapshot", (), {"requested_state": requested_state})()

        specification = self._build_specification()
        specification = LocomotionRobotSpecification(
            joint_addresses=specification.joint_addresses,
            mirrored_joint_pairs=specification.mirrored_joint_pairs,
            joint_axis_directions=specification.joint_axis_directions,
            initialization_positions=specification.initialization_positions,
            standing_positions=reference,
            calibration_reference_positions=reference,
            calibration_limit_selectors=specification.calibration_limit_selectors,
        )

        result = capture_calibration_result(
            specification,
            FakeActuatorArray(),
            FakeCommandSource(),
            polling_interval_seconds=0.0,
            capture_window_size=4,
            max_stddev_deg=1.0,
        )

        expected_average = np.mean(np.stack(stable_samples, axis=0), axis=0)
        np.testing.assert_allclose(result.averaged_readings, expected_average)
        np.testing.assert_allclose(result.position_offsets, expected_average)
        self.assertEqual(result.sample_count, 4)
        self.assertLess(result.max_stddev_deg, 1.0)

    def test_capture_calibration_result_rejects_pose_that_is_not_stable_enough(self) -> None:
        class FakeActuatorArray:
            position_measurements_complete = True

            def __init__(self) -> None:
                self._samples = [
                    np.array([0.0, 0.0], dtype=np.float32),
                    np.array([0.0, 0.0], dtype=np.float32),
                    np.array([0.08, -0.08], dtype=np.float32),
                    np.array([-0.08, 0.08], dtype=np.float32),
                ]

            def read_positions(self) -> np.ndarray:
                if len(self._samples) > 1:
                    return self._samples.pop(0)
                return self._samples[0]

        class FakeCommandSource:
            def __init__(self) -> None:
                self._states = [
                    LocomotionControlState.INVALID,
                    LocomotionControlState.INVALID,
                    LocomotionControlState.INVALID,
                    LocomotionControlState.IDLE,
                ]

            def snapshot(self) -> object:
                requested_state = self._states.pop(0)
                return type("Snapshot", (), {"requested_state": requested_state})()

        with self.assertRaisesRegex(RuntimeError, "not stable enough"):
            capture_calibration_result(
                self._build_specification(),
                FakeActuatorArray(),
                FakeCommandSource(),
                polling_interval_seconds=0.0,
                capture_window_size=3,
                max_stddev_deg=1.0,
            )


if __name__ == "__main__":
    unittest.main()
