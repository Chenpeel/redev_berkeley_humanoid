from __future__ import annotations

import unittest

from berkeley_humanoid_lite_lowlevel.robot.configuration_io import write_robot_configuration


class _FakeBus:
    def __init__(self) -> None:
        self.calls: list[str] = []

    def __getattr__(self, name: str):
        def _method(*_args, **_kwargs):
            self.calls.append(name)
            return 0

        return _method


class _FakeJoint:
    def __init__(self, joint_name: str, device_id: int, bus: _FakeBus) -> None:
        self.joint_name = joint_name
        self.device_id = device_id
        self.bus = bus


class _FakeRobot:
    def __init__(self, joint_interfaces: list[_FakeJoint]) -> None:
        self.joint_interfaces = joint_interfaces


class ConfigurationIoTests(unittest.TestCase):
    def test_write_robot_configuration_rejects_missing_device_id(self) -> None:
        bus = _FakeBus()
        robot = _FakeRobot([_FakeJoint("left_hip_roll_joint", 5, bus)])

        with self.assertRaisesRegex(ValueError, "missing required device_id"):
            write_robot_configuration(
                robot,
                {
                    "left_hip_roll_joint": {
                        "fast_frame_frequency": 0,
                    }
                },
            )

        self.assertEqual(bus.calls, [])

    def test_write_robot_configuration_rejects_device_id_mismatch_before_bus_io(self) -> None:
        bus = _FakeBus()
        robot = _FakeRobot([_FakeJoint("left_hip_roll_joint", 5, bus)])

        with self.assertRaisesRegex(ValueError, "device_id mismatch"):
            write_robot_configuration(
                robot,
                {
                    "left_hip_roll_joint": {
                        "device_id": 1,
                    }
                },
            )

        self.assertEqual(bus.calls, [])


if __name__ == "__main__":
    unittest.main()
