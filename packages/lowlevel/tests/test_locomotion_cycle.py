from __future__ import annotations

import unittest

import numpy as np

from berkeley_humanoid_lite_lowlevel.robot.control_state import LocomotionControlState
from berkeley_humanoid_lite_lowlevel.robot.locomotion_cycle import (
    LocomotionCycleContext,
    advance_locomotion_cycle,
)


class LocomotionCycleTests(unittest.TestCase):
    def test_idle_to_initializing_captures_current_measurement_as_start(self) -> None:
        measured_positions = np.array([0.4, -0.3], dtype=np.float32)

        result = advance_locomotion_cycle(
            LocomotionCycleContext(
                state=LocomotionControlState.IDLE,
                requested_state=LocomotionControlState.INITIALIZING,
                initialization_progress=0.5,
                initialization_step=0.01,
                starting_positions=np.zeros((2,), dtype=np.float32),
                measured_positions=measured_positions,
                policy_actions=np.array([0.1, 0.2], dtype=np.float32),
                initialization_positions=np.array([0.0, 0.0], dtype=np.float32),
            )
        )

        self.assertEqual(result.state, LocomotionControlState.INITIALIZING)
        self.assertEqual(result.initialization_progress, 0.0)
        np.testing.assert_allclose(result.starting_positions, measured_positions)
        np.testing.assert_allclose(result.joint_position_target, measured_positions)
        self.assertTrue(result.enter_position_mode)

    def test_initializing_interpolates_towards_initialization_positions(self) -> None:
        result = advance_locomotion_cycle(
            LocomotionCycleContext(
                state=LocomotionControlState.INITIALIZING,
                requested_state=LocomotionControlState.INVALID,
                initialization_progress=0.25,
                initialization_step=0.25,
                starting_positions=np.array([1.0, -1.0], dtype=np.float32),
                measured_positions=np.array([1.0, -1.0], dtype=np.float32),
                policy_actions=np.array([0.5, -0.5], dtype=np.float32),
                initialization_positions=np.array([0.0, 0.0], dtype=np.float32),
            )
        )

        self.assertEqual(result.state, LocomotionControlState.INITIALIZING)
        self.assertAlmostEqual(result.initialization_progress, 0.5)
        np.testing.assert_allclose(result.joint_position_target, np.array([0.5, -0.5], dtype=np.float32))

    def test_initializing_to_policy_control_holds_initialization_pose_until_next_cycle(self) -> None:
        initialization_positions = np.array([0.2, -0.2], dtype=np.float32)

        result = advance_locomotion_cycle(
            LocomotionCycleContext(
                state=LocomotionControlState.INITIALIZING,
                requested_state=LocomotionControlState.POLICY_CONTROL,
                initialization_progress=1.0,
                initialization_step=0.01,
                starting_positions=np.array([0.4, -0.4], dtype=np.float32),
                measured_positions=np.array([0.2, -0.2], dtype=np.float32),
                policy_actions=np.array([0.8, -0.8], dtype=np.float32),
                initialization_positions=initialization_positions,
            )
        )

        self.assertEqual(result.state, LocomotionControlState.POLICY_CONTROL)
        np.testing.assert_allclose(result.joint_position_target, initialization_positions)
        self.assertFalse(result.enter_damping_mode)

    def test_policy_control_to_idle_requests_damping_without_rewriting_actions(self) -> None:
        actions = np.array([0.7, -0.1], dtype=np.float32)

        result = advance_locomotion_cycle(
            LocomotionCycleContext(
                state=LocomotionControlState.POLICY_CONTROL,
                requested_state=LocomotionControlState.IDLE,
                initialization_progress=1.0,
                initialization_step=0.01,
                starting_positions=np.array([0.0, 0.0], dtype=np.float32),
                measured_positions=np.array([0.3, -0.3], dtype=np.float32),
                policy_actions=actions,
                initialization_positions=np.array([0.0, 0.0], dtype=np.float32),
            )
        )

        self.assertEqual(result.state, LocomotionControlState.IDLE)
        np.testing.assert_allclose(result.joint_position_target, actions)
        self.assertTrue(result.enter_damping_mode)


if __name__ == "__main__":
    unittest.main()
