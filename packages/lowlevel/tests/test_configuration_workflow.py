from __future__ import annotations

import json
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

from berkeley_humanoid_lite_lowlevel.workflows import configuration as configuration_workflow


class ConfigurationWorkflowTests(unittest.TestCase):
    def test_export_robot_configuration_supports_custom_leg_buses(self) -> None:
        captured: dict[str, object] = {}

        class FakeRobot:
            def __init__(self, specification=None, **_: object) -> None:
                captured["specification"] = specification

            def check_connection(self) -> None:
                captured["checked"] = True

            def shutdown(self) -> None:
                captured["shutdown"] = True

        with tempfile.TemporaryDirectory() as temporary_directory:
            output_path = Path(temporary_directory) / "robot_configuration.json"

            with (
                patch.object(configuration_workflow, "LocomotionRobot", FakeRobot),
                patch.object(configuration_workflow, "read_robot_configuration", lambda _robot: {"id": 1}),
            ):
                saved_path = configuration_workflow.export_robot_configuration(
                    output_path,
                    left_leg_bus="can2",
                    right_leg_bus="can3",
                )

            self.assertEqual(saved_path, output_path)
            self.assertTrue(captured.get("checked"))
            self.assertTrue(captured.get("shutdown"))
            specification = captured["specification"]
            self.assertIsNotNone(specification)
            self.assertTrue(all(address.bus_name == "can2" for address in specification.joint_addresses[:6]))
            self.assertTrue(all(address.bus_name == "can3" for address in specification.joint_addresses[6:]))
            self.assertEqual(json.loads(output_path.read_text(encoding="utf-8")), {"id": 1})

    def test_apply_robot_configuration_supports_custom_leg_buses(self) -> None:
        captured: dict[str, object] = {}

        class FakeRobot:
            def __init__(self, specification=None, **_: object) -> None:
                captured["specification"] = specification

            def shutdown(self) -> None:
                captured["shutdown"] = True

        with tempfile.TemporaryDirectory() as temporary_directory:
            input_path = Path(temporary_directory) / "robot_configuration.json"
            input_path.write_text(json.dumps({"id": 1}), encoding="utf-8")

            with (
                patch.object(configuration_workflow, "LocomotionRobot", FakeRobot),
                patch.object(configuration_workflow, "write_robot_configuration") as write_robot_configuration,
            ):
                resolved_path = configuration_workflow.apply_robot_configuration(
                    input_path,
                    left_leg_bus="can2",
                    right_leg_bus="can3",
                    store_to_flash=False,
                )

        self.assertEqual(resolved_path, input_path)
        self.assertTrue(captured.get("shutdown"))
        specification = captured["specification"]
        self.assertIsNotNone(specification)
        self.assertTrue(all(address.bus_name == "can2" for address in specification.joint_addresses[:6]))
        self.assertTrue(all(address.bus_name == "can3" for address in specification.joint_addresses[6:]))
        write_robot_configuration.assert_called_once()
        _, configuration = write_robot_configuration.call_args.args
        self.assertEqual(configuration, {"id": 1})
        self.assertFalse(write_robot_configuration.call_args.kwargs["store_to_flash"])


if __name__ == "__main__":
    unittest.main()
