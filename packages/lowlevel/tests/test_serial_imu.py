from __future__ import annotations

import struct
import unittest
from unittest import mock

import numpy as np
from berkeley_humanoid_lite_lowlevel.robot.imu import Baudrate, FrameType, SerialImu


def _build_hiwonder_frame(
    frame_type: int,
    *,
    data1: int = 0,
    data2: int = 0,
    data3: int = 0,
    data4: int = 0,
    checksum_delta: int = 0,
) -> bytes:
    payload = struct.pack("<Bhhhh", frame_type, data1, data2, data3, data4)
    checksum = (0x55 + sum(payload) + checksum_delta) & 0xFF
    return bytes((0x55,)) + payload + bytes((checksum,))


class FakeSerialPort:
    def __init__(self, data: bytes) -> None:
        self._buffer = bytearray(data)
        self.is_open = True

    def read(self, size: int = 1) -> bytes:
        if size <= 0 or not self._buffer:
            return b""
        chunk = bytes(self._buffer[:size])
        del self._buffer[:size]
        return chunk

    def close(self) -> None:
        self.is_open = False

    def write(self, data: bytes) -> int:
        return len(data)


class SerialImuParserTests(unittest.TestCase):
    def test_read_frame_type_resynchronizes_after_noise_and_bad_checksum(self) -> None:
        bad_gyro_frame = _build_hiwonder_frame(
            FrameType.ANGULAR_VELOCITY,
            data1=123,
            data2=456,
            data3=789,
            checksum_delta=1,
        )
        good_gyro_frame = _build_hiwonder_frame(
            FrameType.ANGULAR_VELOCITY,
            data1=16384,
            data2=-16384,
            data3=0,
        )
        fake_serial = FakeSerialPort(b"\x00\x01\x02" + bad_gyro_frame + good_gyro_frame)

        with mock.patch(
            "berkeley_humanoid_lite_lowlevel.robot.imu.serial.Serial",
            return_value=fake_serial,
        ):
            imu = SerialImu(port="/dev/null", baudrate=Baudrate.BAUD_460800, read_timeout=0.01)
            frame_type = imu.read_frame_type()

        self.assertEqual(frame_type, FrameType.ANGULAR_VELOCITY)
        np.testing.assert_allclose(
            imu.snapshot().angular_velocity_deg_s,
            np.array([1000.0, -1000.0, 0.0], dtype=np.float32),
        )

    def test_read_frame_type_skips_unknown_frame_type_with_valid_checksum(self) -> None:
        unknown_frame = _build_hiwonder_frame(0x5B, data1=1, data2=2, data3=3, data4=4)
        good_angle_frame = _build_hiwonder_frame(
            FrameType.ANGLE,
            data1=16384,
            data2=0,
            data3=-16384,
        )
        fake_serial = FakeSerialPort(unknown_frame + good_angle_frame)

        with mock.patch(
            "berkeley_humanoid_lite_lowlevel.robot.imu.serial.Serial",
            return_value=fake_serial,
        ):
            imu = SerialImu(port="/dev/null", baudrate=Baudrate.BAUD_460800, read_timeout=0.01)
            frame_type = imu.read_frame_type()

        self.assertEqual(frame_type, FrameType.ANGLE)
        np.testing.assert_allclose(
            imu.snapshot().angle_xyz_deg,
            np.array([90.0, 0.0, -90.0], dtype=np.float32),
        )


if __name__ == "__main__":
    unittest.main()
