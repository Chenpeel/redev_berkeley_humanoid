from __future__ import annotations

import unittest

import numpy as np
from berkeley_humanoid_lite_lowlevel.robot.command_source import build_command_from_states
from berkeley_humanoid_lite_lowlevel.robot.control_state import LocomotionControlState
from berkeley_humanoid_lite_lowlevel.robot.locomotion_specification import (
    build_default_locomotion_robot_specification,
)


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
