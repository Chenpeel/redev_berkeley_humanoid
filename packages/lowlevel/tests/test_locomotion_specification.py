from __future__ import annotations

import unittest

import numpy as np
from berkeley_humanoid_lite_lowlevel.policy.configuration import load_policy_deployment_configuration
from berkeley_humanoid_lite_lowlevel.robot.command_source import build_command_from_states
from berkeley_humanoid_lite_lowlevel.robot.control_state import LocomotionControlState
from berkeley_humanoid_lite_lowlevel.robot.locomotion_specification import (
    build_default_locomotion_robot_specification,
    build_leg_locomotion_robot_specification,
)
from berkeley_humanoid_lite_lowlevel.runtime_paths import get_policy_config_path


class LocomotionSpecificationTestCase(unittest.TestCase):
    def test_default_specification_is_consistent(self) -> None:
        specification = build_default_locomotion_robot_specification()

        self.assertEqual(specification.joint_count, 12)
        self.assertEqual(len(specification.mirrored_joint_pairs), 6)
        self.assertEqual(specification.joint_axis_directions.shape, (12,))
        self.assertEqual(specification.initialization_positions.shape, (12,))
        self.assertEqual(specification.calibration_reference_positions.shape, (12,))
        self.assertEqual(len(specification.calibration_limit_selectors), 12)
        self.assertEqual(specification.joint_names[0], "left_hip_roll_joint")

    def test_leg_specification_supports_custom_bus_names(self) -> None:
        specification = build_leg_locomotion_robot_specification(
            left_leg_bus="can2",
            right_leg_bus="can3",
        )

        self.assertTrue(all(address.bus_name == "can2" for address in specification.joint_addresses[:6]))
        self.assertTrue(all(address.bus_name == "can3" for address in specification.joint_addresses[6:]))
        self.assertEqual(specification.joint_addresses[0].device_id, 1)
        self.assertEqual(specification.joint_addresses[2].device_id, 5)
        self.assertEqual(specification.joint_addresses[6].device_id, 2)
        self.assertEqual(specification.joint_addresses[8].device_id, 6)

    def test_leg_specification_initialization_pose_matches_source_rl_init_pose(self) -> None:
        specification = build_leg_locomotion_robot_specification()
        expected_initialization_positions = np.array(
            [0.0, 0.0, -0.2, 0.4, -0.3, 0.0, 0.0, 0.0, -0.2, 0.4, -0.3, 0.0],
            dtype=np.float32,
        )

        np.testing.assert_allclose(
            specification.initialization_positions,
            expected_initialization_positions,
        )
        self.assertFalse(
            np.allclose(
                specification.initialization_positions,
                specification.calibration_reference_positions,
            )
        )

    def test_policy_default_joint_positions_match_source_rl_init_pose(self) -> None:
        specification = build_leg_locomotion_robot_specification()
        expected_leg_positions = specification.initialization_positions

        for file_name in (
            "policy_biped_50hz.yaml",
            "policy_biped_25hz_a.yaml",
            "policy_biped_25hz_b.yaml",
            "policy_humanoid.yaml",
            "policy_humanoid_legs.yaml",
            "policy_video.yaml",
        ):
            configuration = load_policy_deployment_configuration(get_policy_config_path(file_name))
            default_joint_positions = np.asarray(configuration.default_joint_positions, dtype=np.float32)
            leg_positions = (
                default_joint_positions
                if default_joint_positions.shape == expected_leg_positions.shape
                else default_joint_positions[-expected_leg_positions.shape[0]:]
            )
            np.testing.assert_allclose(leg_positions, expected_leg_positions)
            self.assertFalse(
                np.allclose(
                    leg_positions,
                    specification.calibration_reference_positions,
                )
            )

    def test_build_command_from_states_maps_mode_and_axes(self) -> None:
        states = {
            "ABS_X": -16384,
            "ABS_Y": 8192,
            "ABS_RX": -32768,
            "BTN_SOUTH": 1,
            "BTN_TR": 1,
        }

        command = build_command_from_states(states, stick_sensitivity=1.0, dead_zone=0.0)

        self.assertEqual(command.requested_state, LocomotionControlState.POLICY_CONTROL)
        np.testing.assert_allclose(
            np.array([command.velocity_x, command.velocity_y, command.velocity_yaw], dtype=np.float32),
            np.array([-0.25, 1.0, 0.5], dtype=np.float32),
        )

    def test_build_command_from_states_supports_unsigned_linux_axis_ranges(self) -> None:
        states = {
            "ABS_X": 32768,
            "ABS_Y": 32768,
            "ABS_RX": 49152,
        }

        command = build_command_from_states(
            states,
            stick_sensitivity=1.0,
            dead_zone=0.0,
            axis_modes={
                "ABS_X": "unsigned",
                "ABS_Y": "unsigned",
                "ABS_RX": "unsigned",
            },
        )

        np.testing.assert_allclose(
            np.array([command.velocity_x, command.velocity_y, command.velocity_yaw], dtype=np.float32),
            np.array([0.0, -0.5, 0.0], dtype=np.float32),
            atol=1e-4,
        )


if __name__ == "__main__":
    unittest.main()
