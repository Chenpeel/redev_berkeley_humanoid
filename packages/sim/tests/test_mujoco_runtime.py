from __future__ import annotations

import unittest
from dataclasses import dataclass

import numpy as np
from berkeley_humanoid_lite.environments.runtime import (
    apply_visualizer_observation,
    compute_physics_substeps,
    compute_remaining_step_time,
    pace_policy_step,
    reset_simulator_state,
    reset_visualizer_state,
)


@dataclass
class FakeMujocoState:
    qpos: np.ndarray
    qvel: np.ndarray


class MujocoRuntimeTestCase(unittest.TestCase):
    def test_compute_physics_substeps_uses_rounded_ratio(self) -> None:
        self.assertEqual(compute_physics_substeps(0.02, 0.005), 4)
        self.assertEqual(compute_physics_substeps(0.021, 0.005), 4)

    def test_reset_visualizer_state_restores_origin_pose(self) -> None:
        state = FakeMujocoState(
            qpos=np.full(11, 7.0, dtype=np.float32),
            qvel=np.full(10, 9.0, dtype=np.float32),
        )

        reset_visualizer_state(state, num_dofs=4)

        np.testing.assert_allclose(state.qpos, np.array([0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]))
        np.testing.assert_allclose(state.qvel, np.zeros(10, dtype=np.float32))

    def test_apply_visualizer_observation_writes_runtime_buffers(self) -> None:
        state = FakeMujocoState(
            qpos=np.zeros(11, dtype=np.float32),
            qvel=np.zeros(10, dtype=np.float32),
        )
        observation = np.array(
            [
                1.0,
                0.0,
                0.0,
                0.0,
                0.1,
                -0.2,
                0.3,
                10.0,
                11.0,
                12.0,
                13.0,
                20.0,
                21.0,
                22.0,
                23.0,
                3.0,
                0.4,
                0.5,
                0.6,
            ],
            dtype=np.float32,
        )

        apply_visualizer_observation(state, observation, num_dofs=4)

        np.testing.assert_allclose(state.qpos, np.array([0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 10.0, 11.0, 12.0, 13.0]))
        np.testing.assert_allclose(state.qvel, np.array([0.0, 0.0, 0.0, 0.1, -0.2, 0.3, 20.0, 21.0, 22.0, 23.0]))

    def test_reset_simulator_state_uses_default_pose(self) -> None:
        state = FakeMujocoState(
            qpos=np.full(11, -1.0, dtype=np.float32),
            qvel=np.full(10, -2.0, dtype=np.float32),
        )

        reset_simulator_state(
            state,
            default_base_position=[0.5, -0.1, 0.8],
            default_joint_positions=[1.0, 2.0, 3.0, 4.0],
        )

        np.testing.assert_allclose(state.qpos, np.array([0.5, -0.1, 0.8, 1.0, 0.0, 0.0, 0.0, 1.0, 2.0, 3.0, 4.0]))
        np.testing.assert_allclose(state.qvel, np.zeros(10, dtype=np.float32))

    def test_compute_remaining_step_time_clamps_to_zero(self) -> None:
        self.assertAlmostEqual(compute_remaining_step_time(policy_dt=0.02, step_duration=0.013), 0.007)
        self.assertEqual(compute_remaining_step_time(policy_dt=0.02, step_duration=0.03), 0.0)

    def test_pace_policy_step_only_sleeps_for_remaining_time(self) -> None:
        sleep_calls: list[float] = []

        remaining_time = pace_policy_step(
            0.02,
            10.0,
            time_fn=lambda: 10.012,
            sleep_fn=sleep_calls.append,
        )

        self.assertAlmostEqual(remaining_time, 0.008)
        self.assertEqual(len(sleep_calls), 1)
        self.assertAlmostEqual(sleep_calls[0], 0.008)

        no_wait_time = pace_policy_step(
            0.02,
            10.0,
            time_fn=lambda: 10.025,
            sleep_fn=sleep_calls.append,
        )

        self.assertEqual(no_wait_time, 0.0)
        self.assertEqual(len(sleep_calls), 1)


if __name__ == "__main__":
    unittest.main()
