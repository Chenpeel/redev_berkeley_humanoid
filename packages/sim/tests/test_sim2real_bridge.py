from __future__ import annotations

import unittest
from types import ModuleType, SimpleNamespace
from unittest.mock import patch

import numpy as np

from berkeley_humanoid_lite.workflows.sim2real import (
    create_mujoco_joint_delta_bridge,
    resolve_mujoco_bridge_joint_indices,
    run_mujoco_joint_position_bridge,
)
from berkeley_humanoid_lite_lowlevel.robot.control_state import LocomotionControlState


class Sim2RealBridgeHelpersTestCase(unittest.TestCase):
    def test_resolve_mujoco_bridge_joint_indices_supports_leg_prefix_names(self) -> None:
        indices = resolve_mujoco_bridge_joint_indices(
            sim_joint_names=(
                "leg_left_hip_roll_joint",
                "leg_left_knee_pitch_joint",
                "leg_right_hip_roll_joint",
            ),
            robot_joint_names=(
                "left_knee_pitch_joint",
                "right_hip_roll_joint",
            ),
        )

        np.testing.assert_array_equal(indices, np.array([1, 2], dtype=np.int64))

    def test_mujoco_joint_delta_bridge_uses_relative_deltas_and_step_clamp(self) -> None:
        bridge = create_mujoco_joint_delta_bridge(
            sim_joint_positions=np.array([0.1, -0.2], dtype=np.float32),
            real_joint_positions=np.array([1.0, 2.0], dtype=np.float32),
            sim_joint_indices=np.array([0, 1], dtype=np.int64),
            max_delta_radians=0.5,
            max_step_radians=0.1,
        )

        targets, delta = bridge.compute_target_positions(np.array([0.25, -0.6], dtype=np.float32))

        np.testing.assert_allclose(delta, np.array([0.15, -0.4], dtype=np.float32))
        np.testing.assert_allclose(targets, np.array([1.1, 1.9], dtype=np.float32))

    def test_mujoco_joint_delta_bridge_applies_uniform_scale(self) -> None:
        bridge = create_mujoco_joint_delta_bridge(
            sim_joint_positions=np.array([0.1, -0.2], dtype=np.float32),
            real_joint_positions=np.array([1.0, 2.0], dtype=np.float32),
            sim_joint_indices=np.array([0, 1], dtype=np.int64),
            bridge_scale=1.5,
            max_step_radians=1.0,
        )

        targets, delta = bridge.compute_target_positions(np.array([0.3, -0.4], dtype=np.float32))

        np.testing.assert_allclose(delta, np.array([0.3, -0.3], dtype=np.float32))
        np.testing.assert_allclose(targets, np.array([1.3, 1.7], dtype=np.float32))


class FakePolicyController:
    def __init__(self, configuration: object) -> None:
        self.configuration = configuration

    def load_policy(self) -> None:
        return None

    def compute_actions(self, observations: np.ndarray) -> np.ndarray:
        return np.array([0.0, 0.0], dtype=np.float32)


class FakeRobot:
    def __init__(self) -> None:
        self.specification = SimpleNamespace(
            joint_names=("left_hip_roll_joint", "right_hip_roll_joint"),
            joint_count=2,
        )
        self._POLICY_ENTRY_GATE_MAX_ABS_DELTA_DEG = 10.0
        self.actuators = SimpleNamespace(
            configure_damping_mode=lambda **kwargs: None,
        )
        self.state = LocomotionControlState.IDLE
        self.requested_state = LocomotionControlState.INVALID
        self.initialization_progress = 0.0
        self.joint_position_measured = np.array([1.0, -1.0], dtype=np.float32)
        self.enter_damping_mode_called = False
        self.stop_called = False
        self.shutdown_called = False
        self.last_targets: np.ndarray | None = None
        self._policy_control_updates = 0

    def enter_damping_mode(self) -> None:
        self.enter_damping_mode_called = True

    def reset(self) -> np.ndarray:
        return np.zeros((15,), dtype=np.float32)

    def step(self, actions: np.ndarray) -> np.ndarray:
        if self.requested_state == LocomotionControlState.INITIALIZING:
            self.state = LocomotionControlState.INITIALIZING
            self.initialization_progress = 1.0
            return np.zeros((15,), dtype=np.float32)

        if self.requested_state == LocomotionControlState.POLICY_CONTROL:
            self.last_targets = np.asarray(actions, dtype=np.float32).copy()
            if self.state != LocomotionControlState.POLICY_CONTROL:
                self.state = LocomotionControlState.POLICY_CONTROL
                self.initialization_progress = 1.0
                return np.zeros((15,), dtype=np.float32)
            self._policy_control_updates += 1
            raise KeyboardInterrupt()

        return np.zeros((15,), dtype=np.float32)

    def stop(self) -> None:
        self.stop_called = True

    def shutdown(self) -> None:
        self.shutdown_called = True


class FakeSimulator:
    instances: list[FakeSimulator] = []

    def __init__(self, configuration: object) -> None:
        self.cfg = configuration
        self.closed = False
        self._joint_positions = np.array([0.0, 0.5], dtype=np.float32)
        self.__class__.instances.append(self)

    def reset(self) -> np.ndarray:
        return np.zeros((15,), dtype=np.float32)

    def get_joint_positions(self) -> np.ndarray:
        return self._joint_positions.copy()

    def step(self, actions: object) -> np.ndarray:
        self._joint_positions = np.array([0.2, 0.1], dtype=np.float32)
        return np.zeros((15,), dtype=np.float32)

    def close(self) -> None:
        self.closed = True


class Sim2RealBridgeLifecycleTestCase(unittest.TestCase):
    def test_run_mujoco_joint_position_bridge_applies_sim_delta_relative_to_robot_start(self) -> None:
        fake_robot = FakeRobot()
        fake_policy_module = ModuleType("berkeley_humanoid_lite_lowlevel.policy")
        fake_policy_module.PolicyController = FakePolicyController
        fake_locomotion_module = ModuleType("berkeley_humanoid_lite_lowlevel.workflows.locomotion")
        fake_locomotion_module.create_locomotion_robot = lambda **_: fake_robot
        fake_environment_module = ModuleType("berkeley_humanoid_lite.environments")
        fake_environment_module.MujocoSimulator = FakeSimulator

        configuration = SimpleNamespace(
            policy_dt=0.02,
            default_joint_positions=[0.0, 0.0],
            action_indices=[0, 1],
            joints=["leg_left_hip_roll_joint", "leg_right_hip_roll_joint"],
        )

        with patch.dict(
            "sys.modules",
            {
                "berkeley_humanoid_lite_lowlevel.policy": fake_policy_module,
                "berkeley_humanoid_lite_lowlevel.workflows.locomotion": fake_locomotion_module,
                "berkeley_humanoid_lite.environments": fake_environment_module,
            },
        ):
            run_mujoco_joint_position_bridge(
                configuration,
                dry_run=True,
                max_delta_radians=1.0,
                max_step_radians=1.0,
            )

        np.testing.assert_allclose(fake_robot.last_targets, np.array([1.2, -1.4], dtype=np.float32))
        self.assertTrue(fake_robot.stop_called)
        self.assertFalse(fake_robot.shutdown_called)
        self.assertEqual(len(FakeSimulator.instances), 1)
        self.assertTrue(FakeSimulator.instances[0].closed)
        FakeSimulator.instances.clear()

    def test_run_mujoco_joint_position_bridge_configures_robot_gains_before_motion(self) -> None:
        recorded_kwargs: dict[str, float] = {}
        fake_robot = FakeRobot()
        fake_robot.actuators = SimpleNamespace(
            configure_damping_mode=lambda **kwargs: recorded_kwargs.update(kwargs),
        )
        fake_policy_module = ModuleType("berkeley_humanoid_lite_lowlevel.policy")
        fake_policy_module.PolicyController = FakePolicyController
        fake_locomotion_module = ModuleType("berkeley_humanoid_lite_lowlevel.workflows.locomotion")
        fake_locomotion_module.create_locomotion_robot = lambda **_: fake_robot
        fake_environment_module = ModuleType("berkeley_humanoid_lite.environments")
        fake_environment_module.MujocoSimulator = FakeSimulator

        configuration = SimpleNamespace(
            policy_dt=0.02,
            default_joint_positions=[0.0, 0.0],
            action_indices=[0, 1],
            joints=["leg_left_hip_roll_joint", "leg_right_hip_roll_joint"],
        )

        with patch.dict(
            "sys.modules",
            {
                "berkeley_humanoid_lite_lowlevel.policy": fake_policy_module,
                "berkeley_humanoid_lite_lowlevel.workflows.locomotion": fake_locomotion_module,
                "berkeley_humanoid_lite.environments": fake_environment_module,
            },
        ):
            run_mujoco_joint_position_bridge(
                configuration,
                dry_run=False,
                position_kp=32.0,
                position_kd=3.0,
                torque_limit=8.0,
                max_delta_radians=1.0,
                max_step_radians=1.0,
            )

        self.assertEqual(
            recorded_kwargs,
            {
                "position_kp": 32.0,
                "position_kd": 3.0,
                "torque_limit": 8.0,
            },
        )
        self.assertTrue(fake_robot.stop_called)
        FakeSimulator.instances.clear()

    def test_run_mujoco_joint_position_bridge_can_override_policy_gate_limit(self) -> None:
        fake_robot = FakeRobot()
        fake_policy_module = ModuleType("berkeley_humanoid_lite_lowlevel.policy")
        fake_policy_module.PolicyController = FakePolicyController
        fake_locomotion_module = ModuleType("berkeley_humanoid_lite_lowlevel.workflows.locomotion")
        fake_locomotion_module.create_locomotion_robot = lambda **_: fake_robot
        fake_environment_module = ModuleType("berkeley_humanoid_lite.environments")
        fake_environment_module.MujocoSimulator = FakeSimulator

        configuration = SimpleNamespace(
            policy_dt=0.02,
            default_joint_positions=[0.0, 0.0],
            action_indices=[0, 1],
            joints=["leg_left_hip_roll_joint", "leg_right_hip_roll_joint"],
        )

        with patch.dict(
            "sys.modules",
            {
                "berkeley_humanoid_lite_lowlevel.policy": fake_policy_module,
                "berkeley_humanoid_lite_lowlevel.workflows.locomotion": fake_locomotion_module,
                "berkeley_humanoid_lite.environments": fake_environment_module,
            },
        ):
            run_mujoco_joint_position_bridge(
                configuration,
                dry_run=True,
                policy_gate_degrees=12.0,
                max_delta_radians=1.0,
                max_step_radians=1.0,
            )

        self.assertEqual(fake_robot._POLICY_ENTRY_GATE_MAX_ABS_DELTA_DEG, 12.0)
        self.assertTrue(fake_robot.stop_called)
        FakeSimulator.instances.clear()

    def test_run_mujoco_joint_position_bridge_closes_startup_failures_with_shutdown(self) -> None:
        class FailingRobot(FakeRobot):
            def reset(self) -> np.ndarray:
                raise RuntimeError("robot reset failed")

        failing_robot = FailingRobot()
        fake_policy_module = ModuleType("berkeley_humanoid_lite_lowlevel.policy")
        fake_policy_module.PolicyController = FakePolicyController
        fake_locomotion_module = ModuleType("berkeley_humanoid_lite_lowlevel.workflows.locomotion")
        fake_locomotion_module.create_locomotion_robot = lambda **_: failing_robot
        fake_environment_module = ModuleType("berkeley_humanoid_lite.environments")
        fake_environment_module.MujocoSimulator = FakeSimulator

        configuration = SimpleNamespace(
            policy_dt=0.02,
            default_joint_positions=[0.0, 0.0],
            action_indices=[0, 1],
            joints=["leg_left_hip_roll_joint", "leg_right_hip_roll_joint"],
        )

        with patch.dict(
            "sys.modules",
            {
                "berkeley_humanoid_lite_lowlevel.policy": fake_policy_module,
                "berkeley_humanoid_lite_lowlevel.workflows.locomotion": fake_locomotion_module,
                "berkeley_humanoid_lite.environments": fake_environment_module,
            },
        ):
            with self.assertRaisesRegex(RuntimeError, "robot reset failed"):
                run_mujoco_joint_position_bridge(
                    configuration,
                    dry_run=True,
                )

        self.assertFalse(failing_robot.stop_called)
        self.assertTrue(failing_robot.shutdown_called)
        self.assertEqual(len(FakeSimulator.instances), 1)
        self.assertTrue(FakeSimulator.instances[0].closed)
        FakeSimulator.instances.clear()


if __name__ == "__main__":
    unittest.main()
