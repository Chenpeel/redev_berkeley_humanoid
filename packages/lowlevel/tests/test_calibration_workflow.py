from __future__ import annotations

import unittest
from unittest import mock

from berkeley_humanoid_lite_lowlevel.workflows import calibration as calibration_workflow


class CalibrationWorkflowTests(unittest.TestCase):
    def test_run_joint_calibration_supports_custom_leg_buses(self) -> None:
        captured: dict[str, object] = {}

        class FakeRobot:
            def __init__(self, specification=None, **_: object) -> None:
                captured["specification"] = specification
                self.specification = specification
                self.actuators = object()
                self.command_source = object()
                self.calibration_store = type(
                    "Store",
                    (),
                    {"save_position_offsets": staticmethod(lambda offsets: "artifacts/calibration.yaml")},
                )()

            def shutdown(self) -> None:
                captured["shutdown"] = True

        with (
            mock.patch.object(calibration_workflow, "LocomotionRobot", FakeRobot),
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
        self.assertTrue(all(address.bus_name == "can2" for address in specification.joint_addresses[:6]))
        self.assertTrue(all(address.bus_name == "can3" for address in specification.joint_addresses[6:]))


if __name__ == "__main__":
    unittest.main()
