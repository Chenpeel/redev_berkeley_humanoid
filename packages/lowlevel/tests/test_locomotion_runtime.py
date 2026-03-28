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


if __name__ == "__main__":
    unittest.main()
