from __future__ import annotations

import contextlib
import io
import struct
import sys
import unittest
from types import ModuleType, SimpleNamespace
from unittest import mock

import numpy as np
from berkeley_humanoid_lite_lowlevel.robot.command_source import LocomotionCommand
from berkeley_humanoid_lite_lowlevel.robot.control_state import LocomotionControlState
from berkeley_humanoid_lite_lowlevel.robot.locomotion_diagnostics import build_locomotion_diagnostic_snapshot
from berkeley_humanoid_lite_lowlevel.workflows import locomotion as locomotion_workflow
from berkeley_humanoid_lite_lowlevel.workflows.imu import ImuStreamConfiguration
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
            with mock.patch.dict(
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

    def test_create_locomotion_robot_supports_custom_leg_buses(self) -> None:
        captured: dict[str, object] = {}

        class FakeRobot:
            def __init__(self, specification=None, **_: object) -> None:
                captured["specification"] = specification

            def check_connection(self) -> None:
                captured["checked"] = True

            def shutdown(self) -> None:
                captured["shutdown"] = True

        fake_robot_module = ModuleType("berkeley_humanoid_lite_lowlevel.robot")
        fake_robot_module.LocomotionRobot = FakeRobot

        with mock.patch.dict(sys.modules, {"berkeley_humanoid_lite_lowlevel.robot": fake_robot_module}):
            robot = locomotion_workflow.create_locomotion_robot(left_leg_bus="can2", right_leg_bus="can3")

        specification = captured["specification"]
        self.assertIsNotNone(robot)
        self.assertIsNotNone(specification)
        self.assertTrue(all(address.bus_name == "can2" for address in specification.joint_addresses[:6]))
        self.assertTrue(all(address.bus_name == "can3" for address in specification.joint_addresses[6:]))

    def test_create_locomotion_robot_passes_explicit_imu_settings(self) -> None:
        captured: dict[str, object] = {}

        class FakeRobot:
            def __init__(self, specification=None, **kwargs: object) -> None:
                captured["specification"] = specification
                captured.update(kwargs)

        fake_robot_module = ModuleType("berkeley_humanoid_lite_lowlevel.robot")
        fake_robot_module.LocomotionRobot = FakeRobot

        with mock.patch.dict(sys.modules, {"berkeley_humanoid_lite_lowlevel.robot": fake_robot_module}):
            locomotion_workflow.create_locomotion_robot(
                left_leg_bus="can2",
                right_leg_bus="can3",
                imu_device="/dev/ttyUSB9",
                imu_baudrate=123,
                imu_timeout=0.2,
                imu_wait_timeout=1.5,
                require_imu_ready=False,
            )

        self.assertEqual(captured["imu_device"], "/dev/ttyUSB9")
        self.assertEqual(captured["imu_baudrate"], 123)
        self.assertEqual(captured["imu_read_timeout"], 0.2)
        self.assertEqual(captured["imu_wait_timeout"], 1.5)
        self.assertFalse(captured["require_imu_ready"])

    def test_resolve_locomotion_imu_configuration_accepts_hiwonder_stream(self) -> None:
        with mock.patch.object(
            locomotion_workflow,
            "resolve_imu_stream_configuration",
            return_value=ImuStreamConfiguration("hiwonder", "/dev/ttyUSB0", 460800),
        ) as resolver:
            configuration = locomotion_workflow.resolve_locomotion_imu_configuration(
                protocol="auto",
                device="auto",
                baudrate="auto",
                timeout=0.02,
                probe_duration=0.6,
            )

        self.assertEqual(configuration, ImuStreamConfiguration("hiwonder", "/dev/ttyUSB0", 460800))
        resolver.assert_called_once_with(
            protocol="auto",
            device="auto",
            baudrate="auto",
            timeout=0.02,
            probe_duration=0.6,
        )

    def test_resolve_locomotion_imu_configuration_rejects_packet_stream(self) -> None:
        with mock.patch.object(
            locomotion_workflow,
            "resolve_imu_stream_configuration",
            return_value=ImuStreamConfiguration("packet", "/dev/ttyACM0", 1_000_000),
        ):
            with self.assertRaisesRegex(ValueError, "HiWonder USB IMU streams"):
                locomotion_workflow.resolve_locomotion_imu_configuration()

    def test_check_locomotion_connection_supports_custom_leg_buses(self) -> None:
        class FakeRobot:
            def check_connection(self) -> None:
                return None

            def shutdown(self) -> None:
                return None

        with mock.patch.object(locomotion_workflow, "create_locomotion_robot", return_value=FakeRobot()) as factory:
            locomotion_workflow.check_locomotion_connection(left_leg_bus="can2", right_leg_bus="can3")

        factory.assert_called_once_with(
            left_leg_bus="can2",
            right_leg_bus="can3",
            enable_imu=False,
            enable_command_source=False,
        )

    def test_run_locomotion_loop_debug_prints_state_command_and_joint_data(self) -> None:
        fake_policy_module = ModuleType("berkeley_humanoid_lite_lowlevel.policy.controller")
        fake_policy_module.PolicyController = FakePolicyController

        class FakeRobot:
            def __init__(self) -> None:
                self.specification = SimpleNamespace(
                    joint_count=2,
                    joint_names=("joint_0", "joint_1"),
                    initialization_positions=np.array([0.25, -0.35], dtype=np.float32),
                )
                self.actuators = SimpleNamespace(
                    joint_position_target=np.array([0.0, 0.0], dtype=np.float32),
                    position_offsets=np.array([1.0, -2.0], dtype=np.float32),
                )
                self.joint_position_measured = np.array([0.0, 0.0], dtype=np.float32)
                self.position_offsets = np.array([1.0, -2.0], dtype=np.float32)
                self.joint_axis_directions = np.array([1.0, -1.0], dtype=np.float32)
                self.state = LocomotionControlState.POLICY_CONTROL
                self.requested_state = LocomotionControlState.POLICY_CONTROL

            def enter_damping_mode(self) -> None:
                return None

            def reset(self) -> np.ndarray:
                return np.zeros((15,), dtype=np.float32)

            def step(self, actions: np.ndarray) -> np.ndarray:
                self.actuators.joint_position_target = actions.copy()
                self.joint_position_measured = np.array([0.05, -0.15], dtype=np.float32)

                observations = np.zeros((15,), dtype=np.float32)
                observations[12:15] = np.array([0.3, -0.1, 0.2], dtype=np.float32)
                return observations

            def create_diagnostic_snapshot(self, observations: np.ndarray, actions: np.ndarray):
                return build_locomotion_diagnostic_snapshot(
                    specification=self.specification,
                    state=self.state,
                    requested_state=self.requested_state,
                    command_velocity=observations[12:15],
                    actions=actions,
                    joint_position_target=self.actuators.joint_position_target,
                    joint_position_measured=self.joint_position_measured,
                    position_offsets=self.position_offsets,
                    joint_axis_directions=self.joint_axis_directions,
                    dry_run=False,
                )

            def create_imu_debug_line(self) -> str | None:
                return None

            def stop(self) -> None:
                return None

        class FakeObservationStream:
            def send_numpy(self, observations: np.ndarray) -> None:
                raise KeyboardInterrupt()

            def stop(self) -> None:
                return None

        configuration = SimpleNamespace(
            policy_dt=0.02,
            num_actions=2,
            num_joints=2,
            default_joint_positions=[0.0, 0.0],
        )

        with contextlib.redirect_stdout(io.StringIO()) as stdout:
            with mock.patch.dict(
                "sys.modules",
                {"berkeley_humanoid_lite_lowlevel.policy.controller": fake_policy_module},
            ):
                with mock.patch.object(locomotion_workflow, "create_locomotion_robot", return_value=FakeRobot()):
                    with mock.patch.object(
                        locomotion_workflow,
                        "create_observation_stream",
                        return_value=FakeObservationStream(),
                    ):
                        locomotion_workflow.run_locomotion_loop(
                            configuration,
                            debug=True,
                            debug_every=1,
                        )

        output = stdout.getvalue()
        self.assertIn("Debug snapshots enabled every 1 policy steps", output)
        self.assertIn("state=POLICY_CONTROL", output)
        self.assertIn("requested=POLICY_CONTROL", output)
        self.assertIn("cmd=(+0.300, -0.100, +0.200)", output)
        self.assertIn("[DEBUG] actions", output)
        self.assertIn("[DEBUG] targets", output)
        self.assertIn("[DEBUG] measured", output)
        self.assertIn("[DEBUG] error", output)
        self.assertIn("[DEBUG] offsets", output)
        self.assertIn("[DEBUG] raw_tgt", output)
        self.assertIn("[DEBUG] raw_meas", output)
        self.assertIn("[DEBUG] init_err", output)
        self.assertIn("[DEBUG] init_raw", output)
        self.assertIn("[DEBUG] risk", output)

    def test_run_locomotion_loop_debug_imu_prints_imu_snapshot(self) -> None:
        fake_policy_module = ModuleType("berkeley_humanoid_lite_lowlevel.policy.controller")
        fake_policy_module.PolicyController = FakePolicyController

        class FakeRobot:
            state = LocomotionControlState.POLICY_CONTROL
            requested_state = LocomotionControlState.POLICY_CONTROL

            def enter_damping_mode(self) -> None:
                return None

            def reset(self) -> np.ndarray:
                return np.zeros((15,), dtype=np.float32)

            def step(self, actions: np.ndarray) -> np.ndarray:
                return np.zeros((15,), dtype=np.float32)

            def create_imu_debug_line(self) -> str | None:
                return (
                    "IMU attitude[deg]: roll=  +1.00 pitch=  -2.00 yaw=  +3.00 | "
                    "gyro[deg/s]: x=  +4.00 y=  -5.00 z=  +6.00 | "
                    "quat[wxyz]: [+1.0000, +0.0000, +0.0000, +0.0000]"
                )

            def stop(self) -> None:
                return None

        class FakeObservationStream:
            def send_numpy(self, observations: np.ndarray) -> None:
                raise KeyboardInterrupt()

            def stop(self) -> None:
                return None

        configuration = SimpleNamespace(
            policy_dt=0.02,
            num_actions=2,
            num_joints=2,
            default_joint_positions=[0.0, 0.0],
        )

        with contextlib.redirect_stdout(io.StringIO()) as stdout:
            with mock.patch.dict(
                "sys.modules",
                {"berkeley_humanoid_lite_lowlevel.policy.controller": fake_policy_module},
            ):
                with mock.patch.object(locomotion_workflow, "create_locomotion_robot", return_value=FakeRobot()):
                    with mock.patch.object(
                        locomotion_workflow,
                        "create_observation_stream",
                        return_value=FakeObservationStream(),
                    ):
                        locomotion_workflow.run_locomotion_loop(
                            configuration,
                            debug_imu=True,
                            debug_imu_every=1,
                        )

        output = stdout.getvalue()
        self.assertIn("IMU debug enabled every 1 policy steps", output)
        self.assertIn("[DEBUG][IMU] step=0 IMU attitude[deg]:", output)
        self.assertIn("gyro[deg/s]:", output)
        self.assertIn("quat[wxyz]:", output)

    def test_run_locomotion_loop_dry_run_skips_motor_mode_setup(self) -> None:
        fake_policy_module = ModuleType("berkeley_humanoid_lite_lowlevel.policy.controller")
        fake_policy_module.PolicyController = FakePolicyController
        enter_damping_mode_called = False

        class FakeRobot:
            dry_run = True
            state = LocomotionControlState.IDLE
            requested_state = LocomotionControlState.INVALID

            def enter_damping_mode(self) -> None:
                nonlocal enter_damping_mode_called
                enter_damping_mode_called = True

            def reset(self) -> np.ndarray:
                return np.zeros((15,), dtype=np.float32)

            def step(self, actions: np.ndarray) -> np.ndarray:
                return np.zeros((15,), dtype=np.float32)

            def create_imu_debug_line(self) -> str | None:
                return None

            def stop(self) -> None:
                return None

        class FakeObservationStream:
            def send_numpy(self, observations: np.ndarray) -> None:
                raise KeyboardInterrupt()

            def stop(self) -> None:
                return None

        configuration = SimpleNamespace(
            policy_dt=0.02,
            num_actions=2,
            num_joints=2,
            default_joint_positions=[0.0, 0.0],
        )

        with contextlib.redirect_stdout(io.StringIO()) as stdout:
            with mock.patch.dict(
                "sys.modules",
                {"berkeley_humanoid_lite_lowlevel.policy.controller": fake_policy_module},
            ):
                with mock.patch.object(locomotion_workflow, "create_locomotion_robot", return_value=FakeRobot()):
                    with mock.patch.object(
                        locomotion_workflow,
                        "create_observation_stream",
                        return_value=FakeObservationStream(),
                    ):
                        locomotion_workflow.run_locomotion_loop(
                            configuration,
                            dry_run=True,
                        )

        self.assertFalse(enter_damping_mode_called)
        self.assertIn("Dry-run enabled", stdout.getvalue())

    def test_run_locomotion_loop_passes_imu_settings_to_robot_factory(self) -> None:
        fake_policy_module = ModuleType("berkeley_humanoid_lite_lowlevel.policy.controller")
        fake_policy_module.PolicyController = FakePolicyController

        class FakeRobot:
            state = LocomotionControlState.IDLE
            requested_state = LocomotionControlState.INVALID

            def enter_damping_mode(self) -> None:
                return None

            def reset(self) -> np.ndarray:
                return np.zeros((15,), dtype=np.float32)

            def step(self, actions: np.ndarray) -> np.ndarray:
                return np.zeros((15,), dtype=np.float32)

            def create_imu_debug_line(self) -> str | None:
                return None

            def stop(self) -> None:
                return None

        class FakeObservationStream:
            def send_numpy(self, observations: np.ndarray) -> None:
                raise KeyboardInterrupt()

            def stop(self) -> None:
                return None

        configuration = SimpleNamespace(
            policy_dt=0.02,
            num_actions=2,
            num_joints=2,
            default_joint_positions=[0.0, 0.0],
        )

        with mock.patch.dict(
            "sys.modules",
            {"berkeley_humanoid_lite_lowlevel.policy.controller": fake_policy_module},
        ):
            with mock.patch.object(
                locomotion_workflow,
                "create_locomotion_robot",
                return_value=FakeRobot(),
            ) as factory:
                with mock.patch.object(
                    locomotion_workflow,
                    "create_observation_stream",
                    return_value=FakeObservationStream(),
                ):
                    locomotion_workflow.run_locomotion_loop(
                        configuration,
                        dry_run=True,
                        imu_device="/dev/ttyUSB9",
                        imu_baudrate=6,
                        imu_timeout=0.2,
                        imu_wait_timeout=1.5,
                        require_imu_ready=False,
                    )

        factory.assert_called_once_with(
            left_leg_bus="can0",
            right_leg_bus="can1",
            dry_run=True,
            imu_device="/dev/ttyUSB9",
            imu_baudrate=6,
            imu_timeout=0.2,
            imu_wait_timeout=1.5,
            require_imu_ready=False,
        )

    def test_run_locomotion_loop_shutdowns_instead_of_waiting_for_second_ctrl_c_on_startup_failure(self) -> None:
        fake_policy_module = ModuleType("berkeley_humanoid_lite_lowlevel.policy.controller")
        fake_policy_module.PolicyController = FakePolicyController

        class FakeRobot:
            def __init__(self) -> None:
                self.enter_damping_mode_called = False
                self.stop_called = False
                self.shutdown_called = False

            def enter_damping_mode(self) -> None:
                self.enter_damping_mode_called = True

            def reset(self) -> np.ndarray:
                raise RuntimeError("IMU did not become ready before locomotion reset")

            def stop(self) -> None:
                self.stop_called = True

            def shutdown(self) -> None:
                self.shutdown_called = True

        class FakeObservationStream:
            def stop(self) -> None:
                return None

        configuration = SimpleNamespace(
            policy_dt=0.02,
            num_actions=2,
            num_joints=2,
            default_joint_positions=[0.0, 0.0],
        )
        fake_robot = FakeRobot()

        with mock.patch.dict(
            "sys.modules",
            {"berkeley_humanoid_lite_lowlevel.policy.controller": fake_policy_module},
        ):
            with mock.patch.object(
                locomotion_workflow,
                "create_locomotion_robot",
                return_value=fake_robot,
            ):
                with mock.patch.object(
                    locomotion_workflow,
                    "create_observation_stream",
                    return_value=FakeObservationStream(),
                ):
                    with self.assertRaisesRegex(RuntimeError, "IMU did not become ready"):
                        locomotion_workflow.run_locomotion_loop(configuration)

        self.assertTrue(fake_robot.enter_damping_mode_called)
        self.assertTrue(fake_robot.shutdown_called)
        self.assertFalse(fake_robot.stop_called)


if __name__ == "__main__":
    unittest.main()
