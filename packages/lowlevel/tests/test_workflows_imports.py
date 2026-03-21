from __future__ import annotations

import importlib
import sys
import unittest


class WorkflowImportTests(unittest.TestCase):
    @staticmethod
    def _clear_modules(module_prefix: str) -> None:
        for module_name in list(sys.modules):
            if module_name == module_prefix or module_name.startswith(f"{module_prefix}."):
                sys.modules.pop(module_name)

    def test_stream_orientation_does_not_import_actuator_workflow(self) -> None:
        module_prefix = "berkeley_humanoid_lite_lowlevel.workflows"
        self._clear_modules(module_prefix)

        workflows = importlib.import_module(module_prefix)

        self.assertNotIn(f"{module_prefix}.actuator", sys.modules)

        stream_orientation = workflows.stream_orientation

        self.assertTrue(callable(stream_orientation))
        self.assertIn(f"{module_prefix}.imu", sys.modules)
        self.assertNotIn(f"{module_prefix}.actuator", sys.modules)

    def test_actuator_angle_test_is_exposed_via_lazy_exports(self) -> None:
        module_prefix = "berkeley_humanoid_lite_lowlevel.workflows"
        self._clear_modules(module_prefix)

        workflows = importlib.import_module(module_prefix)

        self.assertNotIn(f"{module_prefix}.actuator", sys.modules)

        run_actuator_angle_test = workflows.run_actuator_angle_test

        self.assertTrue(callable(run_actuator_angle_test))
        self.assertIn(f"{module_prefix}.actuator", sys.modules)

    def test_gamepad_workflows_do_not_import_locomotion_runtime(self) -> None:
        workflow_prefix = "berkeley_humanoid_lite_lowlevel.workflows"
        robot_prefix = "berkeley_humanoid_lite_lowlevel.robot"
        self._clear_modules(workflow_prefix)
        self._clear_modules(robot_prefix)

        workflows = importlib.import_module(workflow_prefix)

        self.assertNotIn(f"{robot_prefix}.locomotion_runtime", sys.modules)

        stream_gamepad_commands = workflows.stream_gamepad_commands
        broadcast_gamepad_commands = workflows.broadcast_gamepad_commands

        self.assertTrue(callable(stream_gamepad_commands))
        self.assertTrue(callable(broadcast_gamepad_commands))
        self.assertIn(f"{workflow_prefix}.locomotion", sys.modules)
        self.assertNotIn(f"{robot_prefix}.locomotion_runtime", sys.modules)


if __name__ == "__main__":
    unittest.main()
