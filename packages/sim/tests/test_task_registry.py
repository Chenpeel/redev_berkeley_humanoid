from __future__ import annotations

import unittest

import gymnasium as gym

try:
    from berkeley_humanoid_lite.tasks import TASK_REGISTRATIONS, register_tasks
except ModuleNotFoundError as error:
    TASK_REGISTRATIONS = ()
    register_tasks = None
    TASK_IMPORT_ERROR = error
else:
    TASK_IMPORT_ERROR = None


@unittest.skipIf(register_tasks is None, f"isaaclab-dependent task registry import unavailable: {TASK_IMPORT_ERROR}")
class TaskRegistryTestCase(unittest.TestCase):
    def test_task_registration_ids_cover_balance_and_recovery_tasks(self) -> None:
        task_ids = {registration.task_id for registration in TASK_REGISTRATIONS}

        self.assertIn("Velocity-Berkeley-Humanoid-Lite-v0", task_ids)
        self.assertIn("Velocity-Berkeley-Humanoid-Lite-Biped-v0", task_ids)
        self.assertIn("Stand-Berkeley-Humanoid-Lite-v0", task_ids)
        self.assertIn("Stand-Berkeley-Humanoid-Lite-Biped-v0", task_ids)
        self.assertIn("Recovery-Berkeley-Humanoid-Lite-v0", task_ids)
        self.assertIn("Recovery-Berkeley-Humanoid-Lite-Biped-v0", task_ids)

    def test_register_tasks_adds_new_balance_tasks_to_gym_registry(self) -> None:
        registered_task_ids = set(register_tasks(force=True))

        self.assertIn("Stand-Berkeley-Humanoid-Lite-v0", registered_task_ids)
        self.assertIn("Recovery-Berkeley-Humanoid-Lite-v0", registered_task_ids)
        self.assertIn("Stand-Berkeley-Humanoid-Lite-Biped-v0", registered_task_ids)
        self.assertIn("Recovery-Berkeley-Humanoid-Lite-Biped-v0", registered_task_ids)

        for task_id in registered_task_ids:
            self.assertIn(task_id, gym.registry)


if __name__ == "__main__":
    unittest.main()
