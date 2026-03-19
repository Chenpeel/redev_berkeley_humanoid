from __future__ import annotations

import unittest
from types import SimpleNamespace

import torch
from berkeley_humanoid_lite.environments.initialization import (
    JointControlParameters,
    build_joint_control_parameters,
    build_joint_parameter_tensor,
    build_sensor_layout,
    build_simulator_initialization,
    create_default_command_state,
)
from berkeley_humanoid_lite_lowlevel.robot.control_state import LocomotionControlState


class MujocoInitializationTestCase(unittest.TestCase):
    def test_build_joint_parameter_tensor_validates_joint_count(self) -> None:
        with self.assertRaisesRegex(ValueError, "joint_kp 长度必须为 4"):
            build_joint_parameter_tensor(
                [1.0, 2.0, 3.0],
                num_joints=4,
                parameter_name="joint_kp",
            )

    def test_build_joint_control_parameters_converts_configuration_values(self) -> None:
        configuration = SimpleNamespace(
            num_joints=3,
            joint_kp=[10.0, 20.0, 30.0],
            joint_kd=[1.0, 2.0, 3.0],
            effort_limits=[4.0, 5.0, 6.0],
        )

        parameters = build_joint_control_parameters(configuration)

        self.assertIsInstance(parameters, JointControlParameters)
        self.assertTrue(torch.equal(parameters.joint_kp, torch.tensor([10.0, 20.0, 30.0], dtype=torch.float32)))
        self.assertTrue(torch.equal(parameters.joint_kd, torch.tensor([1.0, 2.0, 3.0], dtype=torch.float32)))
        self.assertTrue(torch.equal(parameters.effort_limits, torch.tensor([4.0, 5.0, 6.0], dtype=torch.float32)))

    def test_build_sensor_layout_uses_three_sensor_blocks_per_actuator(self) -> None:
        layout = build_sensor_layout(num_joints=4, num_actuators=6)

        self.assertEqual(layout.num_joints, 4)
        self.assertEqual(layout.base_sensor_offset, 18)

    def test_create_default_command_state_uses_policy_control_mode(self) -> None:
        command_state = create_default_command_state()

        self.assertEqual(command_state.mode, float(LocomotionControlState.POLICY_CONTROL))
        self.assertEqual(command_state.velocity_x, 0.0)
        self.assertEqual(command_state.velocity_y, 0.0)
        self.assertEqual(command_state.velocity_yaw, 0.0)

    def test_build_simulator_initialization_aggregates_runtime_components(self) -> None:
        configuration = SimpleNamespace(
            num_joints=4,
            policy_dt=0.02,
            physics_dt=0.005,
            joint_kp=[10.0, 20.0, 30.0, 40.0],
            joint_kd=[1.0, 2.0, 3.0, 4.0],
            effort_limits=[5.0, 6.0, 7.0, 8.0],
        )

        initialization = build_simulator_initialization(
            configuration,
            num_actuators=6,
        )

        self.assertEqual(initialization.physics_substeps, 4)
        self.assertEqual(initialization.sensor_layout.base_sensor_offset, 18)
        self.assertEqual(initialization.command_state.mode, float(LocomotionControlState.POLICY_CONTROL))
        self.assertTrue(
            torch.equal(
                initialization.control_parameters.effort_limits,
                torch.tensor([5.0, 6.0, 7.0, 8.0], dtype=torch.float32),
            )
        )


if __name__ == "__main__":
    unittest.main()
