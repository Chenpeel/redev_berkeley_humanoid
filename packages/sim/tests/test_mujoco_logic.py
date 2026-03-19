from __future__ import annotations

import importlib
import sys
import unittest

import numpy as np
import torch
from berkeley_humanoid_lite.environments.control import compute_pd_torques
from berkeley_humanoid_lite.environments.observations import (
    CommandObservationState,
    SensorLayout,
    build_policy_observation,
    quat_rotate_inverse,
    update_command_state,
)
from berkeley_humanoid_lite_lowlevel.robot.command_source import LocomotionCommand
from berkeley_humanoid_lite_lowlevel.robot.control_state import LocomotionControlState


class MujocoLogicTestCase(unittest.TestCase):
    def test_compute_pd_torques_maps_actions_and_clips_limits(self) -> None:
        torques = compute_pd_torques(
            torch.tensor([1.5, -2.0], dtype=torch.float32),
            num_joints=4,
            action_indices=[1, 3],
            joint_positions=torch.tensor([0.1, 0.2, 0.3, 0.4], dtype=torch.float32),
            joint_velocities=torch.tensor([0.5, -0.5, 1.0, -1.0], dtype=torch.float32),
            joint_kp=torch.tensor([10.0, 20.0, 30.0, 40.0], dtype=torch.float32),
            joint_kd=torch.tensor([1.0, 1.0, 1.0, 1.0], dtype=torch.float32),
            effort_limits=torch.tensor([2.0, 2.0, 2.0, 2.0], dtype=torch.float32),
        )

        expected = torch.tensor([-1.5, 2.0, -2.0, -2.0], dtype=torch.float32)
        self.assertTrue(torch.allclose(torques, expected))

    def test_sensor_layout_and_observation_builder_match_policy_layout(self) -> None:
        layout = SensorLayout(num_joints=4, base_sensor_offset=12)
        sensordata = np.arange(19, dtype=np.float32)

        observation = build_policy_observation(
            base_quat=layout.base_quaternion(sensordata),
            base_ang_vel=layout.base_angular_velocity(sensordata),
            joint_positions=layout.joint_positions(sensordata),
            joint_velocities=layout.joint_velocities(sensordata),
            action_indices=[1, 3],
            command_state=CommandObservationState(
                mode=float(LocomotionControlState.POLICY_CONTROL),
                velocity_x=0.1,
                velocity_y=-0.2,
                velocity_yaw=0.3,
            ),
        )

        expected = torch.tensor(
            [12.0, 13.0, 14.0, 15.0, 16.0, 17.0, 18.0, 1.0, 3.0, 5.0, 7.0, 3.0, 0.1, -0.2, 0.3],
            dtype=torch.float32,
        )
        self.assertTrue(torch.allclose(observation, expected))

    def test_update_command_state_keeps_mode_until_new_request(self) -> None:
        command = LocomotionCommand(
            requested_state=LocomotionControlState.INVALID,
            velocity_x=0.4,
            velocity_y=0.6,
            velocity_yaw=-0.2,
        )

        state = update_command_state(command, current_mode=3.0)

        self.assertEqual(state.mode, float(LocomotionControlState.POLICY_CONTROL))
        self.assertEqual(state.velocity_x, 0.4)
        self.assertEqual(state.velocity_y, 0.3)
        self.assertEqual(state.velocity_yaw, -0.2)

    def test_quat_rotate_inverse_rotates_vector_by_inverse_quaternion(self) -> None:
        quaternion = torch.tensor([0.0, 0.0, 0.0, 1.0], dtype=torch.float32)
        vector = torch.tensor([1.0, 2.0, 3.0], dtype=torch.float32)

        rotated = quat_rotate_inverse(quaternion, vector)

        expected = torch.tensor([-1.0, -2.0, 3.0], dtype=torch.float32)
        self.assertTrue(torch.allclose(rotated, expected))

    def test_environments_package_import_is_lazy(self) -> None:
        package_name = "berkeley_humanoid_lite.environments"
        mujoco_module_name = f"{package_name}.mujoco"

        sys.modules.pop(mujoco_module_name, None)
        sys.modules.pop(package_name, None)

        module = importlib.import_module(package_name)

        self.assertNotIn(mujoco_module_name, sys.modules)
        self.assertIs(module.quat_rotate_inverse, quat_rotate_inverse)


if __name__ == "__main__":
    unittest.main()
