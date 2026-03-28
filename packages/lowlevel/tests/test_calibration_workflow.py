from __future__ import annotations

import contextlib
import io
import unittest
from unittest import mock

import numpy as np
from berkeley_humanoid_lite_lowlevel.workflows import calibration as calibration_workflow


class CalibrationWorkflowTests(unittest.TestCase):
    def test_run_joint_calibration_supports_custom_leg_buses(self) -> None:
        captured: dict[str, object] = {}

        class FakeActuatorArray:
            def __init__(self, specification=None, position_offsets=None, **_: object) -> None:
                captured["specification"] = specification
                captured["position_offsets"] = np.asarray(position_offsets, dtype=np.float32)

            def shutdown(self) -> None:
                captured["shutdown"] = True

        class FakeCommandSource:
            def start(self) -> None:
                captured["command_started"] = True

            def stop(self) -> None:
                captured["command_stopped"] = True

        class FakeCaptureResult:
            def __init__(self, offsets: np.ndarray) -> None:
                self.position_offsets = np.asarray(offsets, dtype=np.float32)

            def build_metadata(self, specification: object) -> dict[str, object]:
                captured["metadata_specification"] = specification
                return {"schema_version": 2}

        class FakeStore:
            def save_position_offsets(self, offsets: np.ndarray, *, metadata: dict[str, object] | None = None) -> str:
                captured["saved_offsets"] = np.asarray(offsets, dtype=np.float32)
                captured["saved_metadata"] = metadata
                return "artifacts/calibration.yaml"

        with (
            mock.patch.object(calibration_workflow, "LocomotionActuatorArray", FakeActuatorArray),
            mock.patch.object(calibration_workflow, "GamepadCommandSource", FakeCommandSource),
            mock.patch.object(calibration_workflow, "CalibrationStore", FakeStore),
            mock.patch.object(
                calibration_workflow,
                "capture_calibration_result",
                side_effect=lambda specification, *_args: FakeCaptureResult(specification.initialization_positions),
            ),
        ):
            calibration_workflow.run_joint_calibration(left_leg_bus="can2", right_leg_bus="can3")

        specification = captured["specification"]
        self.assertIsNotNone(specification)
        self.assertTrue(captured.get("shutdown"))
        self.assertTrue(captured.get("command_started"))
        self.assertTrue(captured.get("command_stopped"))
        self.assertTrue(all(address.bus_name == "can2" for address in specification.joint_addresses[:6]))
        self.assertTrue(all(address.bus_name == "can3" for address in specification.joint_addresses[6:]))
        np.testing.assert_allclose(
            captured["position_offsets"],
            np.zeros((specification.joint_count,), dtype=np.float32),
        )
        np.testing.assert_allclose(
            captured["saved_offsets"],
            specification.initialization_positions,
        )
        self.assertEqual(captured["saved_metadata"], {"schema_version": 2})

    def test_run_joint_calibration_prints_reference_pose_note(self) -> None:
        class FakeActuatorArray:
            def __init__(self, specification=None, position_offsets=None, **_: object) -> None:
                self.specification = specification
                self.position_offsets = np.asarray(position_offsets, dtype=np.float32)

            def shutdown(self) -> None:
                return None

        class FakeCommandSource:
            def start(self) -> None:
                return None

            def stop(self) -> None:
                return None

        class FakeCaptureResult:
            def __init__(self, offsets: np.ndarray) -> None:
                self.position_offsets = np.asarray(offsets, dtype=np.float32)

            def build_metadata(self, specification: object) -> dict[str, object]:
                return {"schema_version": 2}

        class FakeStore:
            def save_position_offsets(self, offsets: np.ndarray, *, metadata: dict[str, object] | None = None) -> str:
                return "artifacts/calibration.yaml"

        stdout = io.StringIO()
        with contextlib.redirect_stdout(stdout):
            with (
                mock.patch.object(calibration_workflow, "LocomotionActuatorArray", FakeActuatorArray),
                mock.patch.object(calibration_workflow, "GamepadCommandSource", FakeCommandSource),
                mock.patch.object(calibration_workflow, "CalibrationStore", FakeStore),
                mock.patch.object(
                    calibration_workflow,
                    "capture_calibration_result",
                    return_value=FakeCaptureResult(np.zeros((12,), dtype=np.float32)),
                ),
            ):
                calibration_workflow.run_joint_calibration()

        output = stdout.getvalue()
        self.assertIn("calibration reference pose", output)
        self.assertIn("not the mechanical limit", output)

    def test_run_joint_calibration_prints_hardware_checks_on_failure(self) -> None:
        class FakeActuatorArray:
            def __init__(self, specification=None, position_offsets=None, **_: object) -> None:
                self.specification = specification
                self.position_offsets = np.asarray(position_offsets, dtype=np.float32)

            def shutdown(self) -> None:
                return None

        class FakeCommandSource:
            def start(self) -> None:
                return None

            def stop(self) -> None:
                return None

        stdout = io.StringIO()
        with self.assertRaisesRegex(RuntimeError, "hardware read failed"):
            with contextlib.redirect_stdout(stdout):
                with (
                    mock.patch.object(calibration_workflow, "LocomotionActuatorArray", FakeActuatorArray),
                    mock.patch.object(calibration_workflow, "GamepadCommandSource", FakeCommandSource),
                    mock.patch.object(
                        calibration_workflow,
                        "capture_calibration_result",
                        side_effect=RuntimeError("hardware read failed"),
                    ),
                ):
                    calibration_workflow.run_joint_calibration(left_leg_bus="can2", right_leg_bus="can3")

        output = stdout.getvalue()
        self.assertIn("Joint calibration failed", output)
        self.assertIn("left=can2 right=can3", output)
        self.assertIn("motors are powered", output)


if __name__ == "__main__":
    unittest.main()
