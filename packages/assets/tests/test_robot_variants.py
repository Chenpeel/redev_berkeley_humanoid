import unittest

from berkeley_humanoid_lite_assets.robots.joints import (
    FULL_BODY_JOINT_NAMES,
    LEG_JOINT_NAMES,
)
from berkeley_humanoid_lite_assets.robots.variants import (
    BIPED_VARIANT,
    FULL_BODY_VARIANT,
    ROBOT_VARIANTS,
)


class RobotVariantSpecificationTests(unittest.TestCase):
    def test_variant_dictionary_contains_expected_entries(self) -> None:
        self.assertIn(BIPED_VARIANT.name, ROBOT_VARIANTS)
        self.assertIn(FULL_BODY_VARIANT.name, ROBOT_VARIANTS)
        self.assertEqual({"biped", "full_body"}, set(ROBOT_VARIANTS))

    def test_biped_variant_joint_set_is_leg_only(self) -> None:
        self.assertEqual(BIPED_VARIANT.joint_names, tuple(LEG_JOINT_NAMES))
        self.assertEqual(set(BIPED_VARIANT.initial_joint_positions), set(LEG_JOINT_NAMES))

    def test_full_body_variant_joint_set_is_complete(self) -> None:
        self.assertEqual(FULL_BODY_VARIANT.joint_names, tuple(FULL_BODY_JOINT_NAMES))
        self.assertEqual(set(FULL_BODY_VARIANT.initial_joint_positions), set(FULL_BODY_JOINT_NAMES))


if __name__ == "__main__":
    unittest.main()
