from __future__ import annotations

import unittest

from berkeley_humanoid_lite.training.paths import get_rsl_rl_logs_dir
from berkeley_humanoid_lite.workflows import compute_policy_observation_size


class WorkflowHelpersTestCase(unittest.TestCase):
    def test_compute_policy_observation_size_matches_layout(self) -> None:
        self.assertEqual(compute_policy_observation_size(12), 35)
        self.assertEqual(compute_policy_observation_size(22), 55)

    def test_rsl_rl_logs_dir_points_to_untested_checkpoints(self) -> None:
        self.assertEqual(get_rsl_rl_logs_dir().as_posix().split("/")[-2:], ["untested_ckpts", "rsl_rl"])


if __name__ == "__main__":
    unittest.main()
