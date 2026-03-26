from __future__ import annotations

import unittest
from unittest import mock

from berkeley_humanoid_lite_lowlevel.robot.locomotion_specification import build_leg_locomotion_robot_specification
from berkeley_humanoid_lite_lowlevel.workflows import actuator as actuator_workflow


class _FakeBus:
    def __init__(self, channel: str) -> None:
        self.channel = channel
        self.stored_device_ids: list[int] = []
        self.stopped = False

    def store_settings_to_flash(self, device_id: int) -> None:
        self.stored_device_ids.append(device_id)

    def stop(self) -> None:
        self.stopped = True


class ActuatorCalibrationWorkflowTests(unittest.TestCase):
    def test_run_leg_actuator_calibration_uses_leg_spec_order_and_stores_flash(self) -> None:
        created_buses: dict[str, _FakeBus] = {}
        ping_calls: list[tuple[str, int]] = []
        calibration_calls: list[tuple[str, int, float]] = []

        def fake_create_actuator_bus(channel: str, bitrate: int) -> _FakeBus:
            self.assertEqual(bitrate, 123456)
            bus = _FakeBus(channel)
            created_buses[channel] = bus
            return bus

        def fake_ping_actuator(bus: _FakeBus, device_id: int) -> bool:
            ping_calls.append((bus.channel, device_id))
            return True

        def fake_calibrate(bus: _FakeBus, device_id: int, *, wait_seconds: float) -> None:
            calibration_calls.append((bus.channel, device_id, wait_seconds))

        with (
            mock.patch.object(actuator_workflow, "create_actuator_bus", side_effect=fake_create_actuator_bus),
            mock.patch.object(actuator_workflow, "ping_actuator", side_effect=fake_ping_actuator),
            mock.patch.object(
                actuator_workflow,
                "calibrate_actuator_electrical_offset",
                side_effect=fake_calibrate,
            ),
            mock.patch.object(actuator_workflow.time, "sleep"),
        ):
            actuator_workflow.run_leg_actuator_calibration(
                left_leg_bus="can2",
                right_leg_bus="can3",
                bitrate=123456,
                wait_seconds=7.5,
                flash_settle_seconds=0.0,
                inter_joint_delay_seconds=0.0,
            )

        specification = build_leg_locomotion_robot_specification(
            left_leg_bus="can2",
            right_leg_bus="can3",
        )
        expected_channels_and_ids = [
            (address.bus_name, address.device_id)
            for address in specification.joint_addresses
        ]

        self.assertEqual(ping_calls, expected_channels_and_ids)
        self.assertEqual(
            calibration_calls,
            [(channel, device_id, 7.5) for channel, device_id in expected_channels_and_ids],
        )
        self.assertEqual(
            created_buses["can2"].stored_device_ids,
            [address.device_id for address in specification.joint_addresses[:6]],
        )
        self.assertEqual(
            created_buses["can3"].stored_device_ids,
            [address.device_id for address in specification.joint_addresses[6:]],
        )
        self.assertTrue(all(bus.stopped for bus in created_buses.values()))

    def test_run_leg_actuator_calibration_supports_skip_flash(self) -> None:
        fake_bus = _FakeBus("can0")

        with (
            mock.patch.object(actuator_workflow, "create_actuator_bus", return_value=fake_bus),
            mock.patch.object(actuator_workflow, "ping_actuator", return_value=True),
            mock.patch.object(actuator_workflow, "calibrate_actuator_electrical_offset"),
            mock.patch.object(actuator_workflow.time, "sleep"),
        ):
            actuator_workflow.run_leg_actuator_calibration(
                flash_settle_seconds=0.0,
                inter_joint_delay_seconds=0.0,
                store_to_flash=False,
            )

        self.assertEqual(fake_bus.stored_device_ids, [])


if __name__ == "__main__":
    unittest.main()
