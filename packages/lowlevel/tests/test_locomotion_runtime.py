from __future__ import annotations

import unittest
from types import SimpleNamespace
from unittest import mock

import berkeley_humanoid_lite_lowlevel.robot.locomotion_runtime as locomotion_runtime_module
import numpy as np
from berkeley_humanoid_lite_lowlevel.robot.locomotion_specification import build_leg_locomotion_robot_specification


class FakeCalibrationStore:
    def load_position_offsets(self, joint_count: int) -> np.ndarray:
        return np.zeros((joint_count,), dtype=np.float32)


class FakePoseAlignmentStore:
    def __init__(self, bias: np.ndarray | None = None) -> None:
        self.bias = np.zeros((0,), dtype=np.float32) if bias is None else np.asarray(bias, dtype=np.float32)

    def load_pose_alignment_bias(self, joint_count: int) -> np.ndarray:
        if self.bias.shape == (0,):
            return np.zeros((joint_count,), dtype=np.float32)
        return self.bias.copy()


class FakeActuatorArray:
    def __init__(self, specification, position_offsets=None) -> None:
        self.specification = specification
        self.joint_interfaces = ()
        self.joint_axis_directions = specification.joint_axis_directions.copy()
        self.position_offsets = np.zeros((specification.joint_count,), dtype=np.float32)
        if position_offsets is not None:
            self.position_offsets[:] = np.asarray(position_offsets, dtype=np.float32)
        self.joint_position_target = np.zeros((specification.joint_count,), dtype=np.float32)
        self.joint_position_measured = np.zeros((specification.joint_count,), dtype=np.float32)
        self.joint_velocity_measured = np.zeros((specification.joint_count,), dtype=np.float32)
        self.measurements_ready = False

    def refresh_measurements(self) -> bool:
        self.measurements_ready = True
        return True

    def shutdown(self) -> None:
        return None

    def set_idle_mode(self) -> None:
        return None

    def set_damping_mode(self) -> None:
        return None

    def configure_damping_mode(self) -> None:
        return None


class FakeSerialImu:
    last_instance: FakeSerialImu | None = None
    ready_default = True
    snapshot_default = SimpleNamespace(
        timestamp=1.25,
        quaternion_wxyz=np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float32),
        angular_velocity_deg_s=np.array([90.0, 0.0, -90.0], dtype=np.float32),
        angle_xyz_deg=np.zeros((3,), dtype=np.float32),
        acceleration_xyz_g=np.zeros((3,), dtype=np.float32),
        quaternion_ready=True,
        angular_velocity_ready=True,
    )

    def __init__(self, port: str, baudrate: int, read_timeout: float, *, verbose: bool = False) -> None:
        self.port = port
        self.baudrate = baudrate
        self.read_timeout = read_timeout
        self.verbose = verbose
        self.ready = type(self).ready_default
        self.snapshot_value = type(self).snapshot_default
        self.wait_calls: list[tuple[float, bool, bool]] = []
        self.run_forever_called = False
        self.stopped = False
        type(self).last_instance = self

    def run_forever(self) -> None:
        self.run_forever_called = True

    def wait_until_ready(
        self,
        *,
        timeout: float,
        require_quaternion: bool = True,
        require_angular_velocity: bool = True,
        poll_interval: float = 0.01,
    ) -> bool:
        self.wait_calls.append((timeout, require_quaternion, require_angular_velocity))
        return self.ready

    def snapshot(self):
        return self.snapshot_value

    def stop(self) -> None:
        self.stopped = True


class LocomotionRuntimeTests(unittest.TestCase):
    def setUp(self) -> None:
        FakeSerialImu.ready_default = True
        FakeSerialImu.snapshot_default = SimpleNamespace(
            timestamp=1.25,
            quaternion_wxyz=np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float32),
            angular_velocity_deg_s=np.array([90.0, 0.0, -90.0], dtype=np.float32),
            angle_xyz_deg=np.zeros((3,), dtype=np.float32),
            acceleration_xyz_g=np.zeros((3,), dtype=np.float32),
            quaternion_ready=True,
            angular_velocity_ready=True,
        )
        FakeSerialImu.last_instance = None

    def test_reset_waits_for_imu_ready_and_uses_snapshot_observations(self) -> None:
        specification = build_leg_locomotion_robot_specification()

        with (
            mock.patch.object(locomotion_runtime_module, "LocomotionActuatorArray", FakeActuatorArray),
            mock.patch.object(locomotion_runtime_module, "SerialImu", FakeSerialImu),
        ):
            robot = locomotion_runtime_module.LocomotionRobot(
                specification=specification,
                calibration_store=FakeCalibrationStore(),
                enable_command_source=False,
                imu_device="/dev/ttyUSB9",
                imu_baudrate=6,
                imu_read_timeout=0.2,
                imu_wait_timeout=0.75,
            )
            observations = robot.reset()
            robot.shutdown()

        imu = FakeSerialImu.last_instance
        self.assertIsNotNone(imu)
        assert imu is not None
        self.assertTrue(imu.run_forever_called)
        self.assertEqual(imu.port, "/dev/ttyUSB9")
        self.assertEqual(imu.baudrate, 6)
        self.assertEqual(imu.read_timeout, 0.2)
        self.assertEqual(imu.wait_calls, [(0.75, True, True)])
        np.testing.assert_allclose(observations[0:4], np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float32))
        np.testing.assert_allclose(
            observations[4:7],
            np.array([np.pi / 2.0, 0.0, -np.pi / 2.0], dtype=np.float32),
        )

    def test_reset_raises_when_imu_never_becomes_ready(self) -> None:
        specification = build_leg_locomotion_robot_specification()
        FakeSerialImu.ready_default = False
        FakeSerialImu.snapshot_default = SimpleNamespace(
            timestamp=0.0,
            quaternion_wxyz=np.zeros((4,), dtype=np.float32),
            angular_velocity_deg_s=np.zeros((3,), dtype=np.float32),
            angle_xyz_deg=np.zeros((3,), dtype=np.float32),
            acceleration_xyz_g=np.zeros((3,), dtype=np.float32),
            quaternion_ready=False,
            angular_velocity_ready=False,
        )

        with (
            mock.patch.object(locomotion_runtime_module, "LocomotionActuatorArray", FakeActuatorArray),
            mock.patch.object(locomotion_runtime_module, "SerialImu", FakeSerialImu),
        ):
            robot = locomotion_runtime_module.LocomotionRobot(
                specification=specification,
                calibration_store=FakeCalibrationStore(),
                enable_command_source=False,
                imu_wait_timeout=0.25,
            )
            with self.assertRaisesRegex(RuntimeError, "IMU did not become ready before locomotion reset"):
                robot.reset()
            robot.shutdown()

    def test_get_observations_latches_last_non_invalid_requested_state(self) -> None:
        specification = build_leg_locomotion_robot_specification()

        with mock.patch.object(locomotion_runtime_module, "LocomotionActuatorArray", FakeActuatorArray):
            robot = locomotion_runtime_module.LocomotionRobot(
                specification=specification,
                calibration_store=FakeCalibrationStore(),
                enable_imu=False,
                enable_command_source=False,
            )
            command_sequence = [
                SimpleNamespace(
                    requested_state=locomotion_runtime_module.LocomotionControlState.POLICY_CONTROL,
                    velocity_x=0.3,
                    velocity_y=-0.2,
                    velocity_yaw=0.1,
                ),
                SimpleNamespace(
                    requested_state=locomotion_runtime_module.LocomotionControlState.INVALID,
                    velocity_x=0.0,
                    velocity_y=0.0,
                    velocity_yaw=0.0,
                ),
            ]

            with mock.patch.object(robot, "_get_command", side_effect=command_sequence):
                first_observations = robot.get_observations().copy()
                second_observations = robot.get_observations().copy()

            robot.shutdown()

        command_mode_index = 7 + specification.joint_count * 2
        self.assertEqual(
            int(first_observations[command_mode_index]),
            int(locomotion_runtime_module.LocomotionControlState.POLICY_CONTROL),
        )
        self.assertEqual(
            int(second_observations[command_mode_index]),
            int(locomotion_runtime_module.LocomotionControlState.POLICY_CONTROL),
        )
        self.assertEqual(
            robot.requested_state,
            locomotion_runtime_module.LocomotionControlState.POLICY_CONTROL,
        )

    def test_reset_prints_calibration_audit(self) -> None:
        specification = build_leg_locomotion_robot_specification()

        with (
            mock.patch.object(locomotion_runtime_module, "LocomotionActuatorArray", FakeActuatorArray),
            mock.patch("builtins.print") as mock_print,
        ):
            robot = locomotion_runtime_module.LocomotionRobot(
                specification=specification,
                calibration_store=FakeCalibrationStore(),
                enable_imu=False,
                enable_command_source=False,
            )
            robot.reset()
            robot.shutdown()

        printed_lines = " ".join(str(call.args[0]) for call in mock_print.call_args_list if call.args)
        self.assertIn("Calibration audit:", printed_lines)
        self.assertIn("standing pose:", printed_lines)
        self.assertIn("initialization pose:", printed_lines)

    def test_get_observations_applies_pose_alignment_bias(self) -> None:
        specification = build_leg_locomotion_robot_specification()
        pose_alignment_bias = np.array([0.05] * specification.joint_count, dtype=np.float32)

        with mock.patch.object(locomotion_runtime_module, "LocomotionActuatorArray", FakeActuatorArray):
            robot = locomotion_runtime_module.LocomotionRobot(
                specification=specification,
                calibration_store=FakeCalibrationStore(),
                pose_alignment_store=FakePoseAlignmentStore(pose_alignment_bias),
                enable_imu=False,
                enable_command_source=False,
            )
            robot.actuators.joint_position_measured[:] = np.array([0.1] * specification.joint_count, dtype=np.float32)

            observations = robot.get_observations()
            robot.shutdown()

        np.testing.assert_allclose(
            observations[7 : 7 + specification.joint_count],
            np.array([0.15] * specification.joint_count, dtype=np.float32),
        )

    def test_step_uses_standing_positions_for_initializing_request(self) -> None:
        specification = build_leg_locomotion_robot_specification()
        captured: dict[str, object] = {}

        def fake_advance(context):
            captured["context"] = context
            return SimpleNamespace(
                state=locomotion_runtime_module.LocomotionControlState.INITIALIZING,
                initialization_progress=0.0,
                starting_positions=np.asarray(context.starting_positions, dtype=np.float32).copy(),
                joint_position_target=np.asarray(context.measured_positions, dtype=np.float32).copy(),
                enter_position_mode=False,
                enter_damping_mode=False,
                messages=(),
            )

        with (
            mock.patch.object(locomotion_runtime_module, "LocomotionActuatorArray", FakeActuatorArray),
            mock.patch.object(locomotion_runtime_module, "advance_locomotion_cycle", side_effect=fake_advance),
        ):
            robot = locomotion_runtime_module.LocomotionRobot(
                specification=specification,
                calibration_store=FakeCalibrationStore(),
                enable_imu=False,
                enable_command_source=False,
                dry_run=True,
            )
            robot.actuators.measurements_ready = True
            robot.requested_state = locomotion_runtime_module.LocomotionControlState.INITIALIZING
            robot.step(np.zeros((specification.joint_count,), dtype=np.float32))
            robot.shutdown()

        context = captured["context"]
        np.testing.assert_allclose(context.initialization_positions, specification.standing_positions)
        self.assertFalse(context.restart_initialization)

    def test_step_restarts_initialization_when_switching_from_standing_hold_to_policy_control(self) -> None:
        specification = build_leg_locomotion_robot_specification()
        captured: dict[str, object] = {}

        def fake_advance(context):
            captured["context"] = context
            return SimpleNamespace(
                state=locomotion_runtime_module.LocomotionControlState.INITIALIZING,
                initialization_progress=0.0,
                starting_positions=np.asarray(context.measured_positions, dtype=np.float32).copy(),
                joint_position_target=np.asarray(context.measured_positions, dtype=np.float32).copy(),
                enter_position_mode=False,
                enter_damping_mode=False,
                messages=(),
            )

        with (
            mock.patch.object(locomotion_runtime_module, "LocomotionActuatorArray", FakeActuatorArray),
            mock.patch.object(locomotion_runtime_module, "advance_locomotion_cycle", side_effect=fake_advance),
        ):
            robot = locomotion_runtime_module.LocomotionRobot(
                specification=specification,
                calibration_store=FakeCalibrationStore(),
                enable_imu=False,
                enable_command_source=False,
                dry_run=True,
            )
            robot.state = locomotion_runtime_module.LocomotionControlState.INITIALIZING
            robot.initialization_progress = 1.0
            robot.active_initialization_positions[:] = specification.standing_positions
            robot.active_initialization_label = "standing"
            robot.actuators.measurements_ready = True
            robot.actuators.joint_position_measured[:] = specification.standing_positions
            robot.requested_state = locomotion_runtime_module.LocomotionControlState.POLICY_CONTROL
            robot.step(np.zeros((specification.joint_count,), dtype=np.float32))
            robot.shutdown()

        context = captured["context"]
        self.assertTrue(context.restart_initialization)
        np.testing.assert_allclose(context.initialization_positions, specification.initialization_positions)

    def test_step_blocks_policy_request_when_too_far_from_standing(self) -> None:
        specification = build_leg_locomotion_robot_specification()
        captured: dict[str, object] = {}

        def fake_advance(context):
            captured["context"] = context
            return SimpleNamespace(
                state=locomotion_runtime_module.LocomotionControlState.IDLE,
                initialization_progress=0.0,
                starting_positions=np.asarray(context.starting_positions, dtype=np.float32).copy(),
                joint_position_target=np.asarray(context.measured_positions, dtype=np.float32).copy(),
                enter_position_mode=False,
                enter_damping_mode=False,
                messages=(),
            )

        with (
            mock.patch.object(locomotion_runtime_module, "LocomotionActuatorArray", FakeActuatorArray),
            mock.patch.object(locomotion_runtime_module, "advance_locomotion_cycle", side_effect=fake_advance),
            mock.patch("builtins.print") as mock_print,
        ):
            robot = locomotion_runtime_module.LocomotionRobot(
                specification=specification,
                calibration_store=FakeCalibrationStore(),
                enable_imu=False,
                enable_command_source=False,
                dry_run=True,
            )
            robot.actuators.measurements_ready = True
            robot.actuators.joint_position_measured[:] = np.array([0.5] * specification.joint_count, dtype=np.float32)
            robot.requested_state = locomotion_runtime_module.LocomotionControlState.POLICY_CONTROL
            robot.step(np.zeros((specification.joint_count,), dtype=np.float32))
            robot.shutdown()

        self.assertEqual(
            captured["context"].requested_state,
            locomotion_runtime_module.LocomotionControlState.INVALID,
        )
        printed_lines = " ".join(str(call.args[0]) for call in mock_print.call_args_list if call.args)
        self.assertIn("Policy gate blocked:", printed_lines)

    def test_step_writes_hardware_targets_after_removing_pose_alignment_bias(self) -> None:
        specification = build_leg_locomotion_robot_specification()
        pose_alignment_bias = np.array([0.05] * specification.joint_count, dtype=np.float32)
        policy_actions = np.array([0.2] * specification.joint_count, dtype=np.float32)

        with mock.patch.object(locomotion_runtime_module, "LocomotionActuatorArray", FakeActuatorArray):
            robot = locomotion_runtime_module.LocomotionRobot(
                specification=specification,
                calibration_store=FakeCalibrationStore(),
                pose_alignment_store=FakePoseAlignmentStore(pose_alignment_bias),
                enable_imu=False,
                enable_command_source=False,
                dry_run=True,
            )
            robot.state = locomotion_runtime_module.LocomotionControlState.POLICY_CONTROL
            robot.requested_state = locomotion_runtime_module.LocomotionControlState.POLICY_CONTROL
            robot.actuators.measurements_ready = True
            robot.step(policy_actions)
            robot.shutdown()

        np.testing.assert_allclose(
            robot.actuators.joint_position_target,
            np.array([0.15] * specification.joint_count, dtype=np.float32),
        )

    def test_get_observations_zeroes_command_during_policy_entry_window(self) -> None:
        specification = build_leg_locomotion_robot_specification()

        with mock.patch.object(locomotion_runtime_module, "LocomotionActuatorArray", FakeActuatorArray):
            robot = locomotion_runtime_module.LocomotionRobot(
                specification=specification,
                calibration_store=FakeCalibrationStore(),
                enable_imu=False,
                enable_command_source=False,
                policy_entry_zero_command_steps=2,
            )
            robot.requested_state = locomotion_runtime_module.LocomotionControlState.POLICY_CONTROL
            robot._policy_entry_zero_command_steps_remaining = 2

            with mock.patch.object(
                robot,
                "_get_command",
                return_value=SimpleNamespace(
                    requested_state=locomotion_runtime_module.LocomotionControlState.POLICY_CONTROL,
                    velocity_x=0.8,
                    velocity_y=-0.4,
                    velocity_yaw=0.2,
                ),
            ):
                first_observations = robot.get_observations().copy()
                second_observations = robot.get_observations().copy()
                third_observations = robot.get_observations().copy()

            robot.shutdown()

        command_start = 7 + specification.joint_count * 2 + 1
        np.testing.assert_allclose(
            first_observations[command_start: command_start + 3],
            np.zeros((3,), dtype=np.float32),
        )
        np.testing.assert_allclose(
            second_observations[command_start: command_start + 3],
            np.zeros((3,), dtype=np.float32),
        )
        np.testing.assert_allclose(
            third_observations[command_start: command_start + 3],
            np.array([0.8, -0.4, 0.2], dtype=np.float32),
        )

    def test_step_arms_zero_command_window_when_entering_policy_control(self) -> None:
        specification = build_leg_locomotion_robot_specification()

        with mock.patch.object(locomotion_runtime_module, "LocomotionActuatorArray", FakeActuatorArray):
            robot = locomotion_runtime_module.LocomotionRobot(
                specification=specification,
                calibration_store=FakeCalibrationStore(),
                enable_imu=False,
                enable_command_source=False,
                dry_run=True,
                policy_entry_zero_command_steps=3,
            )
            robot.state = locomotion_runtime_module.LocomotionControlState.INITIALIZING
            robot.initialization_progress = 1.0
            robot.active_initialization_positions[:] = specification.initialization_positions
            robot.active_initialization_label = "policy_entry"
            robot.actuators.measurements_ready = True
            robot.requested_state = locomotion_runtime_module.LocomotionControlState.POLICY_CONTROL

            with mock.patch.object(
                robot,
                "_get_command",
                return_value=SimpleNamespace(
                    requested_state=locomotion_runtime_module.LocomotionControlState.POLICY_CONTROL,
                    velocity_x=0.6,
                    velocity_y=-0.3,
                    velocity_yaw=0.1,
                ),
            ):
                observations = robot.step(np.zeros((specification.joint_count,), dtype=np.float32))

            robot.shutdown()

        command_start = 7 + specification.joint_count * 2 + 1
        self.assertEqual(robot.state, locomotion_runtime_module.LocomotionControlState.POLICY_CONTROL)
        np.testing.assert_allclose(observations[command_start: command_start + 3], np.zeros((3,), dtype=np.float32))
        self.assertEqual(robot._policy_entry_zero_command_steps_remaining, 2)


if __name__ == "__main__":
    unittest.main()
