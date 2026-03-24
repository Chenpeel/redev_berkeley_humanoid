from __future__ import annotations

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

        class FakeStore:
            def save_position_offsets(self, offsets: np.ndarray) -> str:
                captured["saved_offsets"] = np.asarray(offsets, dtype=np.float32)
                return "artifacts/calibration.yaml"

        with (
            mock.patch.object(calibration_workflow, "LocomotionActuatorArray", FakeActuatorArray),
            mock.patch.object(calibration_workflow, "GamepadCommandSource", FakeCommandSource),
            mock.patch.object(calibration_workflow, "CalibrationStore", FakeStore),
            mock.patch.object(
                calibration_workflow,
                "capture_calibration_offsets",
                side_effect=lambda specification, *_args: specification.initialization_positions,
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


if __name__ == "__main__":
    unittest.main()
