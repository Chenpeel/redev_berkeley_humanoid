from __future__ import annotations

import importlib.util
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


def _module_exists(module_name: str) -> bool:
    try:
        return importlib.util.find_spec(module_name) is not None
    except ModuleNotFoundError:
        return False


OPTIONAL_RECOVERY_TASK_MODULES: tuple[tuple[str, str, str], ...] = (
    (
        "Posture-Recovery-Berkeley-Humanoid-Lite-v0",
        "berkeley_humanoid_lite.tasks.recovery.posture_recovery.config.humanoid.env_cfg",
        "berkeley_humanoid_lite.tasks.recovery.posture_recovery.config.humanoid.agents.rsl_rl_ppo_cfg",
    ),
    (
        "Posture-Recovery-Berkeley-Humanoid-Lite-Biped-v0",
        "berkeley_humanoid_lite.tasks.recovery.posture_recovery.config.biped.env_cfg",
        "berkeley_humanoid_lite.tasks.recovery.posture_recovery.config.biped.agents.rsl_rl_ppo_cfg",
    ),
    (
        "Getup-Berkeley-Humanoid-Lite-v0",
        "berkeley_humanoid_lite.tasks.recovery.getup.config.humanoid.env_cfg",
        "berkeley_humanoid_lite.tasks.recovery.getup.config.humanoid.agents.rsl_rl_ppo_cfg",
    ),
    (
        "Getup-Berkeley-Humanoid-Lite-Biped-v0",
        "berkeley_humanoid_lite.tasks.recovery.getup.config.biped.env_cfg",
        "berkeley_humanoid_lite.tasks.recovery.getup.config.biped.agents.rsl_rl_ppo_cfg",
    ),
)


@unittest.skipIf(register_tasks is None, f"isaaclab-dependent task registry import unavailable: {TASK_IMPORT_ERROR}")
class TaskRegistryTestCase(unittest.TestCase):
    def test_task_registration_ids_cover_balance_and_push_recovery_tasks(self) -> None:
        task_ids = {registration.task_id for registration in TASK_REGISTRATIONS}

        self.assertIn("Velocity-Berkeley-Humanoid-Lite-v0", task_ids)
        self.assertIn("Velocity-Berkeley-Humanoid-Lite-Biped-v0", task_ids)
        self.assertIn("Stand-Berkeley-Humanoid-Lite-v0", task_ids)
        self.assertIn("Stand-Berkeley-Humanoid-Lite-Biped-v0", task_ids)
        self.assertIn("Push-Recovery-Berkeley-Humanoid-Lite-v0", task_ids)
        self.assertIn("Push-Recovery-Berkeley-Humanoid-Lite-Biped-v0", task_ids)

    def test_task_registration_ids_include_optional_recovery_tasks_when_modules_exist(self) -> None:
        task_ids = {registration.task_id for registration in TASK_REGISTRATIONS}

        for task_id, env_module_name, runner_module_name in OPTIONAL_RECOVERY_TASK_MODULES:
            modules_exist = _module_exists(env_module_name) and _module_exists(runner_module_name)
            if modules_exist:
                self.assertIn(task_id, task_ids)
            else:
                self.assertNotIn(task_id, task_ids)

    def test_register_tasks_adds_new_balance_tasks_to_gym_registry(self) -> None:
        registered_task_ids = set(register_tasks(force=True))

        self.assertIn("Stand-Berkeley-Humanoid-Lite-v0", registered_task_ids)
        self.assertIn("Push-Recovery-Berkeley-Humanoid-Lite-v0", registered_task_ids)
        self.assertIn("Stand-Berkeley-Humanoid-Lite-Biped-v0", registered_task_ids)
        self.assertIn("Push-Recovery-Berkeley-Humanoid-Lite-Biped-v0", registered_task_ids)

        for task_id, env_module_name, runner_module_name in OPTIONAL_RECOVERY_TASK_MODULES:
            modules_exist = _module_exists(env_module_name) and _module_exists(runner_module_name)
            if modules_exist:
                self.assertIn(task_id, registered_task_ids)
            else:
                self.assertNotIn(task_id, registered_task_ids)

        for task_id in registered_task_ids:
            self.assertIn(task_id, gym.registry)


if __name__ == "__main__":
    unittest.main()
