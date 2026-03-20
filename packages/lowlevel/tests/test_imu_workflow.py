from __future__ import annotations

import unittest
from unittest import mock

from berkeley_humanoid_lite_lowlevel.workflows.imu import (
    ImuStreamConfiguration,
    _build_probe_configurations,
    _probe_hiwonder,
    detect_imu_stream,
    normalize_hiwonder_baudrate,
    parse_baudrate_argument,
    resolve_imu_stream_configuration,
)
from berkeley_humanoid_lite_lowlevel.robot.imu import Baudrate


class ImuWorkflowTests(unittest.TestCase):
    def test_parse_baudrate_argument_accepts_auto(self) -> None:
        self.assertIsNone(parse_baudrate_argument("auto"))

    def test_parse_baudrate_argument_parses_numeric_string(self) -> None:
        self.assertEqual(parse_baudrate_argument("460800"), 460800)

    def test_normalize_hiwonder_baudrate_accepts_serial_rate(self) -> None:
        self.assertEqual(normalize_hiwonder_baudrate(460800), Baudrate.BAUD_460800)

    def test_normalize_hiwonder_baudrate_accepts_enum_code(self) -> None:
        self.assertEqual(normalize_hiwonder_baudrate(Baudrate.BAUD_460800), Baudrate.BAUD_460800)

    def test_build_probe_configurations_prefers_hiwonder_candidates(self) -> None:
        with mock.patch(
            "berkeley_humanoid_lite_lowlevel.workflows.imu.discover_orientation_devices",
            return_value=["/dev/ttyUSB0"],
        ):
            configurations = _build_probe_configurations(
                protocol="auto",
                device="auto",
                baudrate=None,
            )

        self.assertEqual(
            configurations[:8],
            [
                ImuStreamConfiguration("hiwonder", "/dev/ttyUSB0", 460800),
                ImuStreamConfiguration("hiwonder", "/dev/ttyUSB0", 115200),
                ImuStreamConfiguration("hiwonder", "/dev/ttyUSB0", 230400),
                ImuStreamConfiguration("hiwonder", "/dev/ttyUSB0", 9600),
                ImuStreamConfiguration("hiwonder", "/dev/ttyUSB0", 19200),
                ImuStreamConfiguration("hiwonder", "/dev/ttyUSB0", 38400),
                ImuStreamConfiguration("hiwonder", "/dev/ttyUSB0", 57600),
                ImuStreamConfiguration("packet", "/dev/ttyUSB0", 1_000_000),
            ],
        )

    def test_detect_imu_stream_returns_first_working_candidate(self) -> None:
        with mock.patch(
            "berkeley_humanoid_lite_lowlevel.workflows.imu.discover_orientation_devices",
            return_value=["/dev/ttyUSB0"],
        ), mock.patch(
            "berkeley_humanoid_lite_lowlevel.workflows.imu._probe_hiwonder",
            side_effect=[False, True],
        ), mock.patch(
            "berkeley_humanoid_lite_lowlevel.workflows.imu._probe_packet",
            return_value=False,
        ):
            configuration = detect_imu_stream()

        self.assertEqual(configuration, ImuStreamConfiguration("hiwonder", "/dev/ttyUSB0", 115200))

    def test_probe_hiwonder_accepts_valid_zero_frame(self) -> None:
        imu = mock.Mock()
        imu.read_frame.side_effect = [True]

        with mock.patch(
            "berkeley_humanoid_lite_lowlevel.workflows.imu.SerialImu",
            return_value=imu,
        ), mock.patch(
            "berkeley_humanoid_lite_lowlevel.workflows.imu.time.perf_counter",
            side_effect=[0.0, 0.0],
        ):
            self.assertTrue(
                _probe_hiwonder(
                    "/dev/ttyUSB0",
                    baudrate=460800,
                    timeout=0.01,
                    probe_duration=0.5,
                )
            )

        imu.close.assert_called_once()

    def test_resolve_imu_stream_configuration_skips_detection_when_explicit(self) -> None:
        with mock.patch(
            "berkeley_humanoid_lite_lowlevel.workflows.imu.detect_imu_stream",
            side_effect=AssertionError("detect_imu_stream should not be called"),
        ):
            configuration = resolve_imu_stream_configuration(
                protocol="hiwonder",
                device="/dev/ttyUSB0",
                baudrate="460800",
            )

        self.assertEqual(configuration, ImuStreamConfiguration("hiwonder", "/dev/ttyUSB0", 460800))


if __name__ == "__main__":
    unittest.main()
