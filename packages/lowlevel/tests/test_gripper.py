from __future__ import annotations

import unittest

from berkeley_humanoid_lite_lowlevel.robot.gripper import (
    normalized_gripper_target_to_raw_value,
    normalize_gripper_target,
)


class GripperTestCase(unittest.TestCase):
    def test_normalize_gripper_target_clamps_to_unit_interval(self) -> None:
        self.assertEqual(normalize_gripper_target(-1.0), 0.0)
        self.assertEqual(normalize_gripper_target(0.25), 0.25)
        self.assertEqual(normalize_gripper_target(2.0), 1.0)

    def test_normalized_gripper_target_to_raw_value_matches_protocol_range(self) -> None:
        self.assertAlmostEqual(normalized_gripper_target_to_raw_value(0.0), 0.2)
        self.assertAlmostEqual(normalized_gripper_target_to_raw_value(0.5), 0.5)
        self.assertAlmostEqual(normalized_gripper_target_to_raw_value(1.0), 0.8)


if __name__ == "__main__":
    unittest.main()
