from __future__ import annotations

from io import BytesIO
import math
import struct
import unittest

from berkeley_humanoid_lite_lowlevel.sensors import OrientationSample, read_orientation_sample


def _build_frame(*values: float, payload_size: int = 28) -> bytes:
    payload = struct.pack("<7f", *values)
    if payload_size != 28:
        payload = payload[:payload_size]
    return b"\x75\x65" + payload_size.to_bytes(2, byteorder="little") + payload


class OrientationStreamTests(unittest.TestCase):
    def test_read_orientation_sample_parses_valid_frame(self) -> None:
        stream = BytesIO(_build_frame(1.0, 0.0, 0.0, 0.0, 0.1, -0.2, 0.3))

        sample = read_orientation_sample(stream, timestamp=12.5)

        assert sample is not None
        self.assertEqual(sample.quaternion_wxyz, (1.0, 0.0, 0.0, 0.0))
        self.assertEqual(sample.angular_velocity_xyz, (0.10000000149011612, -0.20000000298023224, 0.30000001192092896))
        self.assertEqual(sample.timestamp, 12.5)

    def test_read_orientation_sample_rejects_invalid_sync_word(self) -> None:
        stream = BytesIO(b"\x00\x65" + (28).to_bytes(2, byteorder="little") + b"\x00" * 28)

        sample = read_orientation_sample(stream)

        self.assertIsNone(sample)

    def test_read_orientation_sample_rejects_unexpected_payload_size(self) -> None:
        stream = BytesIO(_build_frame(1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, payload_size=24))

        sample = read_orientation_sample(stream)

        self.assertIsNone(sample)

    def test_orientation_sample_converts_quaternion_to_euler_degrees(self) -> None:
        sample = OrientationSample(
            quaternion_wxyz=(math.cos(math.pi / 4.0), 0.0, 0.0, math.sin(math.pi / 4.0)),
            angular_velocity_xyz=(0.0, 0.0, 0.0),
            timestamp=0.0,
        )

        roll, pitch, yaw = sample.to_euler_degrees()

        self.assertAlmostEqual(roll, 0.0, places=6)
        self.assertAlmostEqual(pitch, 0.0, places=6)
        self.assertAlmostEqual(yaw, 90.0, places=6)


if __name__ == "__main__":
    unittest.main()
