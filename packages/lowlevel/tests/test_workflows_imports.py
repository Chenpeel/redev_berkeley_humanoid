from __future__ import annotations

import importlib
import sys
import unittest


class WorkflowImportTests(unittest.TestCase):
    def test_stream_orientation_does_not_import_actuator_workflow(self) -> None:
        module_prefix = "berkeley_humanoid_lite_lowlevel.workflows"
        for module_name in list(sys.modules):
            if module_name == module_prefix or module_name.startswith(f"{module_prefix}."):
                sys.modules.pop(module_name)

        workflows = importlib.import_module(module_prefix)

        self.assertNotIn(f"{module_prefix}.actuator", sys.modules)

        stream_orientation = workflows.stream_orientation

        self.assertTrue(callable(stream_orientation))
        self.assertIn(f"{module_prefix}.imu", sys.modules)
        self.assertNotIn(f"{module_prefix}.actuator", sys.modules)


if __name__ == "__main__":
    unittest.main()
