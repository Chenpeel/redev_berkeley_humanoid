from __future__ import annotations

import math
import unittest
from unittest import mock

import numpy as np
from berkeley_humanoid_lite_lowlevel.robot.joint_transport import LocomotionActuatorArray
from berkeley_humanoid_lite_lowlevel.robot.locomotion_specification import (
    JointTransportAddress,
    LocomotionRobotSpecification,
)


class _FakeBus:
    def __init__(
        self,
        bus_name: str,
        *,
        read_positions: dict[int, list[float | None]] | None = None,
    ) -> None:
        self.bus_name = bus_name
        self.read_position_timeouts: list[float | None] = []
        self.read_velocity_timeouts: list[float | None] = []
        self._read_positions = {
            device_id: list(values)
            for device_id, values in (read_positions or {}).items()
        }
        self._receive_positions = {
            device_id: list(values)
            for device_id, values in (read_positions or {}).items()
        }

    def read_position_measured(self, device_id: int, timeout: float | None = None) -> float | None:
        self.read_position_timeouts.append(timeout)
        values = self._read_positions[device_id]
        if len(values) > 1:
            return values.pop(0)
        return values[0]

    def read_velocity_measured(self, device_id: int, timeout: float | None = None) -> float:
        self.read_velocity_timeouts.append(timeout)
        return 0.0

    def transmit_pdo_2(self, device_id: int, *, position_target: float, velocity_target: float) -> None:
        return None

    def receive_pdo_2(self, device_id: int) -> tuple[float, float]:
        values = self._receive_positions[device_id]
        if len(values) > 1:
            return values.pop(0), 0.0
        return values[0], 0.0

    def set_mode(self, device_id: int, mode: object) -> None:
        return None

    def feed(self, device_id: int) -> None:
        return None

    def write_position_kp(self, device_id: int, value: float) -> None:
        return None

    def write_position_kd(self, device_id: int, value: float) -> None:
        return None

    def write_torque_limit(self, device_id: int, value: float) -> None:
        return None

    def ping(self, device_id: int) -> bool:
        return True

    def stop(self) -> None:
        return None


class JointTransportTestCase(unittest.TestCase):
    def _build_specification(self) -> LocomotionRobotSpecification:
        return LocomotionRobotSpecification(
            joint_addresses=(
                JointTransportAddress("can0", 1, "joint_1"),
                JointTransportAddress("can0", 2, "joint_2"),
            ),
            mirrored_joint_pairs=((0, 1),),
            joint_axis_directions=np.array([1.0, 1.0], dtype=np.float32),
            initialization_positions=np.zeros((2,), dtype=np.float32),
            standing_positions=np.zeros((2,), dtype=np.float32),
            calibration_reference_positions=np.zeros((2,), dtype=np.float32),
            calibration_limit_selectors=("min", "min"),
        )

    def test_read_positions_unwraps_across_2pi_boundary(self) -> None:
        fake_bus = _FakeBus(
            "can0",
            read_positions={
                1: [0.1, (2.0 * math.pi) - 0.2],
                2: [0.0, 0.0],
            },
        )

        with mock.patch(
            "berkeley_humanoid_lite_lowlevel.robot.joint_transport.recoil.Bus",
            return_value=fake_bus,
        ):
            actuators = LocomotionActuatorArray(self._build_specification())

        first_positions = actuators.read_positions()
        second_positions = actuators.read_positions()

        self.assertAlmostEqual(float(first_positions[0]), 0.1, places=5)
        self.assertAlmostEqual(float(second_positions[0]), -0.2, places=5)

    def test_synchronize_unwraps_feedback_before_joint_space_conversion(self) -> None:
        fake_bus = _FakeBus(
            "can0",
            read_positions={
                1: [0.1, (2.0 * math.pi) - 0.2],
                2: [0.0, 0.0],
            },
        )

        with mock.patch(
            "berkeley_humanoid_lite_lowlevel.robot.joint_transport.recoil.Bus",
            return_value=fake_bus,
        ):
            actuators = LocomotionActuatorArray(self._build_specification())

        actuators.synchronize()
        self.assertAlmostEqual(float(actuators.joint_position_measured[0]), 0.1, places=5)

        actuators.synchronize()
        self.assertAlmostEqual(float(actuators.joint_position_measured[0]), -0.2, places=5)

    def test_refresh_measurements_primes_joint_measurements_without_position_commands(self) -> None:
        fake_bus = _FakeBus(
            "can0",
            read_positions={
                1: [0.25],
                2: [-0.5],
            },
        )

        with mock.patch(
            "berkeley_humanoid_lite_lowlevel.robot.joint_transport.recoil.Bus",
            return_value=fake_bus,
        ):
            actuators = LocomotionActuatorArray(self._build_specification())

        updated = actuators.refresh_measurements()

        self.assertTrue(updated)
        self.assertTrue(actuators.measurements_ready)
        np.testing.assert_allclose(
            actuators.joint_position_measured,
            np.array([0.25, -0.5], dtype=np.float32),
        )

    def test_refresh_measurements_uses_bounded_timeout_reads(self) -> None:
        fake_bus = _FakeBus(
            "can0",
            read_positions={
                1: [0.25],
                2: [-0.5],
            },
        )

        with mock.patch(
            "berkeley_humanoid_lite_lowlevel.robot.joint_transport.recoil.Bus",
            return_value=fake_bus,
        ):
            actuators = LocomotionActuatorArray(self._build_specification())

        actuators.refresh_measurements()

        self.assertEqual(
            fake_bus.read_position_timeouts,
            [actuators._MEASUREMENT_READ_TIMEOUT_S, actuators._MEASUREMENT_READ_TIMEOUT_S],
        )
        self.assertEqual(
            fake_bus.read_velocity_timeouts,
            [actuators._MEASUREMENT_READ_TIMEOUT_S, actuators._MEASUREMENT_READ_TIMEOUT_S],
        )

    def test_read_positions_uses_bounded_timeout_reads(self) -> None:
        fake_bus = _FakeBus(
            "can0",
            read_positions={
                1: [0.25],
                2: [-0.5],
            },
        )

        with mock.patch(
            "berkeley_humanoid_lite_lowlevel.robot.joint_transport.recoil.Bus",
            return_value=fake_bus,
        ):
            actuators = LocomotionActuatorArray(self._build_specification())

        actuators.read_positions()

        self.assertEqual(
            fake_bus.read_position_timeouts,
            [actuators._MEASUREMENT_READ_TIMEOUT_S, actuators._MEASUREMENT_READ_TIMEOUT_S],
        )

    def test_read_positions_reports_missing_measurements_without_marking_complete(self) -> None:
        fake_bus = _FakeBus(
            "can0",
            read_positions={
                1: [None],
                2: [0.5],
            },
        )

        with (
            mock.patch(
                "berkeley_humanoid_lite_lowlevel.robot.joint_transport.recoil.Bus",
                return_value=fake_bus,
            ),
            mock.patch("builtins.print") as mock_print,
        ):
            actuators = LocomotionActuatorArray(self._build_specification())
            positions = actuators.read_positions()

        np.testing.assert_allclose(positions, np.array([0.0, 0.5], dtype=np.float32))
        self.assertTrue(actuators.measurements_ready)
        self.assertFalse(actuators.position_measurements_complete)
        self.assertEqual(actuators.missing_position_measurement_names, ("joint_1",))
        mock_print.assert_called_once()
        self.assertIn("timed out reading joint positions", mock_print.call_args.args[0])
        self.assertIn("joint_1", mock_print.call_args.args[0])

    def test_joint_and_raw_position_helpers_share_same_transform_formula(self) -> None:
        fake_bus = _FakeBus(
            "can0",
            read_positions={
                1: [0.0],
                2: [0.0],
            },
        )

        with mock.patch(
            "berkeley_humanoid_lite_lowlevel.robot.joint_transport.recoil.Bus",
            return_value=fake_bus,
        ):
            actuators = LocomotionActuatorArray(
                self._build_specification(),
                position_offsets=np.array([1.0, -2.0], dtype=np.float32),
            )

        joint_positions = np.array([0.5, -0.25], dtype=np.float32)
        raw_positions = actuators.joint_to_raw_positions(joint_positions)

        np.testing.assert_allclose(raw_positions, np.array([1.5, -2.25], dtype=np.float32))
        np.testing.assert_allclose(actuators.raw_to_joint_positions(raw_positions), joint_positions)


if __name__ == "__main__":
    unittest.main()
