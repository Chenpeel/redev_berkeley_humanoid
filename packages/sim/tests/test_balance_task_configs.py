from __future__ import annotations

import unittest

try:
    from berkeley_humanoid_lite.tasks.balance.recovery.config.biped import env_cfg as biped_push_recovery_env_cfg
    from berkeley_humanoid_lite.tasks.balance.recovery.config.humanoid import env_cfg as humanoid_push_recovery_env_cfg
    from berkeley_humanoid_lite.tasks.balance.stand.config.humanoid import env_cfg as humanoid_stand_env_cfg
except ModuleNotFoundError as error:
    BALANCE_IMPORT_ERROR = error
else:
    BALANCE_IMPORT_ERROR = None


@unittest.skipIf(
    BALANCE_IMPORT_ERROR is not None,
    f"isaaclab-dependent balance config import unavailable: {BALANCE_IMPORT_ERROR}",
)
class BalanceTaskConfigTestCase(unittest.TestCase):
    def test_humanoid_push_recovery_enables_interval_push_event(self) -> None:
        events_cfg = humanoid_push_recovery_env_cfg.EventsCfg()

        self.assertTrue(hasattr(events_cfg, "push_robot"))
        self.assertEqual(events_cfg.push_robot.mode, "interval")
        self.assertEqual(events_cfg.push_robot.interval_range_s, (4.0, 6.0))

    def test_push_recovery_rewards_add_in_place_penalties(self) -> None:
        humanoid_rewards = humanoid_push_recovery_env_cfg.RewardsCfg()
        biped_rewards = biped_push_recovery_env_cfg.RewardsCfg()
        stand_rewards = humanoid_stand_env_cfg.RewardsCfg()

        self.assertTrue(hasattr(humanoid_rewards, "feet_slide"))
        self.assertTrue(hasattr(humanoid_rewards, "feet_air_time_penalty"))
        self.assertTrue(hasattr(biped_rewards, "feet_slide"))
        self.assertTrue(hasattr(biped_rewards, "feet_air_time_penalty"))
        self.assertFalse(hasattr(stand_rewards, "feet_air_time_penalty"))


if __name__ == "__main__":
    unittest.main()
