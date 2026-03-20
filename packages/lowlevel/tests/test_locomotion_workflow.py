from __future__ import annotations

import contextlib
import io
import struct
import unittest
from types import ModuleType, SimpleNamespace

import numpy as np
from berkeley_humanoid_lite_lowlevel.robot.command_source import LocomotionCommand
from berkeley_humanoid_lite_lowlevel.robot.control_state import LocomotionControlState
from berkeley_humanoid_lite_lowlevel.workflows import locomotion as locomotion_workflow
from berkeley_humanoid_lite_lowlevel.workflows.locomotion import encode_gamepad_command_packet


class FakePolicyController:
    last_observations: np.ndarray | None = None

    def __init__(self, configuration: object) -> None:
        self.configuration = configuration

    def load_policy(self) -> None:
        return None

    def compute_actions(self, observations: np.ndarray) -> np.ndarray:
        type(self).last_observations = observations.copy()
        return np.array([0.1, -0.2], dtype=np.float32)


class LocomotionWorkflowTests(unittest.TestCase):
    def test_encode_gamepad_command_packet_matches_native_runtime_layout(self) -> None:
        command = LocomotionCommand(
            requested_state=LocomotionControlState.POLICY_CONTROL,
            velocity_x=0.25,
            velocity_y=-0.5,
            velocity_yaw=0.75,
        )

        packet = encode_gamepad_command_packet(command)

        self.assertEqual(len(packet), 13)

        mode, velocity_x, velocity_y, velocity_yaw = struct.unpack("<Bfff", packet)
        self.assertEqual(mode, int(LocomotionControlState.POLICY_CONTROL))
        self.assertAlmostEqual(velocity_x, 0.25)
        self.assertAlmostEqual(velocity_y, -0.5)
        self.assertAlmostEqual(velocity_yaw, 0.75)

    def test_run_policy_inference_smoke_test_prints_actions_and_uses_command_velocity(self) -> None:
        fake_policy_module = ModuleType("berkeley_humanoid_lite_lowlevel.policy.controller")
        fake_policy_module.PolicyController = FakePolicyController
        configuration = SimpleNamespace(
            num_actions=2,
            num_joints=2,
            default_joint_positions=[0.3, -0.4],
        )

        with contextlib.redirect_stdout(io.StringIO()) as stdout:
            with unittest.mock.patch.dict(
                "sys.modules",
                {"berkeley_humanoid_lite_lowlevel.policy.controller": fake_policy_module},
            ):
                actions = locomotion_workflow.run_policy_inference_smoke_test(
                    configuration,
                    command_velocity=(0.4, -0.2, 0.1),
                )

        self.assertTrue(np.allclose(actions, np.array([0.1, -0.2], dtype=np.float32)))
        self.assertIn("Actions:", stdout.getvalue())
        self.assertIn("vx=0.400", stdout.getvalue())
        self.assertIsNotNone(FakePolicyController.last_observations)
        np.testing.assert_allclose(
            FakePolicyController.last_observations[12:15],
            np.array([0.4, -0.2, 0.1], dtype=np.float32),
        )


if __name__ == "__main__":
    unittest.main()
