from __future__ import annotations

import importlib
import sys
import unittest


class TeleoperationImportTests(unittest.TestCase):
    @staticmethod
    def _clear_modules(module_prefix: str) -> None:
        for module_name in list(sys.modules):
            if module_name == module_prefix or module_name.startswith(f"{module_prefix}."):
                sys.modules.pop(module_name)

    def test_teleoperation_workflows_do_not_import_unrelated_heavy_modules(self) -> None:
        workflow_prefix = "berkeley_humanoid_lite_lowlevel.workflows"
        robot_prefix = "berkeley_humanoid_lite_lowlevel.robot"
        teleoperation_prefix = "berkeley_humanoid_lite_lowlevel.teleoperation"
        self._clear_modules(workflow_prefix)
        self._clear_modules(robot_prefix)
        self._clear_modules(teleoperation_prefix)

        workflows = importlib.import_module(workflow_prefix)

        self.assertNotIn(f"{robot_prefix}.bimanual", sys.modules)
        self.assertNotIn(f"{robot_prefix}.gripper", sys.modules)
        self.assertNotIn(f"{teleoperation_prefix}.solver", sys.modules)

        check_teleoperation_connection = workflows.check_teleoperation_connection
        run_teleoperation_idle_loop = workflows.run_teleoperation_idle_loop
        run_teleoperation_solver_demo = workflows.run_teleoperation_solver_demo
        stream_gripper_targets = workflows.stream_gripper_targets

        self.assertTrue(callable(check_teleoperation_connection))
        self.assertTrue(callable(run_teleoperation_idle_loop))
        self.assertTrue(callable(run_teleoperation_solver_demo))
        self.assertTrue(callable(stream_gripper_targets))
        self.assertIn(f"{workflow_prefix}.teleoperation", sys.modules)
        self.assertNotIn(f"{robot_prefix}.bimanual", sys.modules)
        self.assertNotIn(f"{robot_prefix}.gripper", sys.modules)
        self.assertNotIn(f"{teleoperation_prefix}.solver", sys.modules)

    def test_importing_teleoperation_package_does_not_import_solver(self) -> None:
        teleoperation_prefix = "berkeley_humanoid_lite_lowlevel.teleoperation"
        self._clear_modules(teleoperation_prefix)

        teleoperation = importlib.import_module(teleoperation_prefix)

        self.assertIsNotNone(teleoperation)
        self.assertNotIn(f"{teleoperation_prefix}.solver", sys.modules)


if __name__ == "__main__":
    unittest.main()
