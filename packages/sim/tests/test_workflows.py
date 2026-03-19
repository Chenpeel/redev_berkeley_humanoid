from __future__ import annotations

import unittest

from berkeley_humanoid_lite.workflows import compute_policy_observation_size


class WorkflowHelpersTestCase(unittest.TestCase):
    def test_compute_policy_observation_size_matches_layout(self) -> None:
        self.assertEqual(compute_policy_observation_size(12), 35)
        self.assertEqual(compute_policy_observation_size(22), 55)


if __name__ == "__main__":
    unittest.main()
