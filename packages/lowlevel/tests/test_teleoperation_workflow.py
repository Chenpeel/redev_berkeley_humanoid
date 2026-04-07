from __future__ import annotations

import unittest
from types import SimpleNamespace
from unittest import mock

import berkeley_humanoid_lite_lowlevel.teleoperation as teleoperation_module
import numpy as np
from berkeley_humanoid_lite_lowlevel.workflows import teleoperation as teleoperation_workflow


class FakeRateLimiter:
    instances: list[FakeRateLimiter] = []

    def __init__(self, *args: object, **kwargs: object) -> None:
        self.args = args
        self.kwargs = kwargs
        self.sleep_count = 0
        FakeRateLimiter.instances.append(self)

    def sleep(self) -> None:
        self.sleep_count += 1


class FakeSolver:
    instances: list[FakeSolver] = []

    def __init__(self) -> None:
        self.robot = SimpleNamespace(model=SimpleNamespace(nq=17))
        self.bridge_updates: list[dict[str, dict[str, object]]] = []
        self.observation_updates: list[np.ndarray] = []
        FakeSolver.instances.append(self)

    def update_controller(self, bridge_data: dict[str, dict[str, object]]) -> None:
        self.bridge_updates.append(bridge_data)

    def update(self, observations: np.ndarray) -> tuple[np.ndarray, tuple[float, float]]:
        self.observation_updates.append(observations.copy())
        return np.full(10, 4.0, dtype=np.float64), (0.25, 0.75)


class TeleoperationWorkflowTests(unittest.TestCase):
    def setUp(self) -> None:
        FakeRateLimiter.instances.clear()
        FakeSolver.instances.clear()

    def test_build_fake_bridge_data_static_pose_keeps_contract(self) -> None:
        bridge_data = teleoperation_module.build_fake_bridge_data(
            1.25,
            pattern="static_pose",
            left_enable=False,
            right_enable=True,
            with_gripper=True,
        )

        self.assertEqual(set(bridge_data), {"left", "right"})
        self.assertFalse(bridge_data["left"]["button_pressed"])
        self.assertTrue(bridge_data["right"]["button_pressed"])
        self.assertEqual(np.asarray(bridge_data["left"]["pose"]).shape, (4, 4))
        self.assertEqual(np.asarray(bridge_data["right"]["pose"]).shape, (4, 4))
        self.assertGreaterEqual(float(bridge_data["left"]["trigger"]), 0.0)
        self.assertLessEqual(float(bridge_data["right"]["trigger"]), 1.0)

    def test_single_arm_sweep_auto_selects_enabled_arm(self) -> None:
        frame_at_start = teleoperation_module.build_fake_input_frame(
            0.0,
            pattern="single_arm_sweep",
            left_enable=False,
            right_enable=True,
            sweep_arm="auto",
        )
        frame_later = teleoperation_module.build_fake_input_frame(
            0.4,
            pattern="single_arm_sweep",
            left_enable=False,
            right_enable=True,
            sweep_arm="auto",
        )

        np.testing.assert_allclose(frame_at_start.left.pose, frame_later.left.pose)
        self.assertFalse(np.allclose(frame_at_start.right.pose, frame_later.right.pose))

    def test_build_minimal_teleoperation_observations_initializes_base_pose(self) -> None:
        observations = teleoperation_workflow.build_minimal_teleoperation_observations(17)

        np.testing.assert_allclose(observations[0:3], np.array([0.0, 0.0, 0.5]))
        np.testing.assert_allclose(observations[3:7], np.array([1.0, 0.0, 0.0, 0.0]))
        np.testing.assert_allclose(observations[7:17], np.zeros(10))

    def test_update_offline_joint_observations_applies_first_order_lag(self) -> None:
        observations = teleoperation_workflow.build_minimal_teleoperation_observations(17)

        next_measurements = teleoperation_workflow.update_offline_joint_observations(
            observations,
            np.full(10, 4.0, dtype=np.float64),
            alpha=0.25,
        )

        np.testing.assert_allclose(next_measurements, np.ones(10))
        np.testing.assert_allclose(observations[7:17], np.ones(10))

    def test_run_offline_teleoperation_ik_updates_measurements_between_steps(self) -> None:
        with (
            mock.patch.object(teleoperation_workflow, "RateLimiter", FakeRateLimiter),
            mock.patch.object(teleoperation_workflow.time, "perf_counter", side_effect=[0.0, 0.0, 0.01, 0.03]),
            mock.patch.object(teleoperation_workflow, "print"),
        ):
            teleoperation_workflow.run_offline_teleoperation_ik(
                pattern="dual_arm_circle",
                frequency=50.0,
                duration_seconds=0.02,
                measurement_alpha=0.25,
                print_every=1,
                solver_factory=FakeSolver,
            )

        solver = FakeSolver.instances[0]
        self.assertEqual(len(solver.bridge_updates), 2)
        self.assertEqual(len(solver.observation_updates), 2)
        np.testing.assert_allclose(solver.observation_updates[0][7:17], np.zeros(10))
        np.testing.assert_allclose(solver.observation_updates[1][7:17], np.ones(10))
        self.assertEqual(FakeRateLimiter.instances[0].sleep_count, 2)


if __name__ == "__main__":
    unittest.main()
