from __future__ import annotations

import unittest
from unittest import mock

from berkeley_humanoid_lite_lowlevel.robot.imu import Baudrate, FrameType, SamplingRate
from berkeley_humanoid_lite_lowlevel.workflows.imu import (
    ImuStreamConfiguration,
    _build_probe_configurations,
    _probe_hiwonder,
    configure_hiwonder_output,
    detect_imu_stream,
    normalize_hiwonder_baudrate,
    normalize_hiwonder_sampling_rate,
    parse_baudrate_argument,
    resolve_hiwonder_output_content,
    resolve_imu_stream_configuration,
)


class ImuWorkflowTests(unittest.TestCase):
    def test_parse_baudrate_argument_accepts_auto(self) -> None:
        self.assertIsNone(parse_baudrate_argument("auto"))

    def test_parse_baudrate_argument_parses_numeric_string(self) -> None:
        self.assertEqual(parse_baudrate_argument("460800"), 460800)

    def test_normalize_hiwonder_baudrate_accepts_serial_rate(self) -> None:
        self.assertEqual(normalize_hiwonder_baudrate(460800), Baudrate.BAUD_460800)

    def test_normalize_hiwonder_baudrate_accepts_enum_code(self) -> None:
        self.assertEqual(normalize_hiwonder_baudrate(Baudrate.BAUD_460800), Baudrate.BAUD_460800)

    def test_normalize_hiwonder_sampling_rate_accepts_supported_rate(self) -> None:
        self.assertEqual(normalize_hiwonder_sampling_rate(10), SamplingRate.RATE_10_HZ)

    def test_resolve_hiwonder_output_content_uses_profile_defaults(self) -> None:
        output_content = resolve_hiwonder_output_content(profile="control")

        self.assertTrue(output_content["angular_velocity"])
        self.assertTrue(output_content["angle"])
        self.assertTrue(output_content["quaternion"])
        self.assertFalse(output_content["acceleration"])

    def test_resolve_hiwonder_output_content_allows_explicit_overrides(self) -> None:
        output_content = resolve_hiwonder_output_content(
            profile="control",
            acceleration_output=True,
            quaternion_output=False,
        )

        self.assertTrue(output_content["acceleration"])
        self.assertFalse(output_content["quaternion"])

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

    def test_probe_hiwonder_accepts_meaningful_sensor_frame(self) -> None:
        imu = mock.Mock()
        imu.read_frame_type.side_effect = [FrameType.ANGLE]

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

    def test_probe_hiwonder_ignores_time_frame_until_sensor_frame_arrives(self) -> None:
        imu = mock.Mock()
        imu.read_frame_type.side_effect = [FrameType.TIME, FrameType.ANGULAR_VELOCITY]

        with mock.patch(
            "berkeley_humanoid_lite_lowlevel.workflows.imu.SerialImu",
            return_value=imu,
        ), mock.patch(
            "berkeley_humanoid_lite_lowlevel.workflows.imu.time.perf_counter",
            side_effect=[0.0, 0.0, 0.1, 0.1],
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

    def test_probe_hiwonder_rejects_time_only_stream(self) -> None:
        imu = mock.Mock()
        imu.read_frame_type.side_effect = [FrameType.TIME, None]

        with mock.patch(
            "berkeley_humanoid_lite_lowlevel.workflows.imu.SerialImu",
            return_value=imu,
        ), mock.patch(
            "berkeley_humanoid_lite_lowlevel.workflows.imu.time.perf_counter",
            side_effect=[0.0, 0.0, 0.2, 0.6],
        ):
            self.assertFalse(
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

    def test_configure_hiwonder_output_applies_baudrate_sampling_and_output_content(self) -> None:
        imu = mock.Mock()

        with mock.patch(
            "berkeley_humanoid_lite_lowlevel.workflows.imu.resolve_imu_stream_configuration",
            return_value=ImuStreamConfiguration("hiwonder", "/dev/ttyUSB0", 9600),
        ), mock.patch(
            "berkeley_humanoid_lite_lowlevel.workflows.imu.SerialImu",
            return_value=imu,
        ), mock.patch("berkeley_humanoid_lite_lowlevel.workflows.imu.time.sleep"):
            configuration = configure_hiwonder_output(
                device="/dev/ttyUSB0",
                baudrate="9600",
                target_baudrate=460800,
                rate_hz=10.0,
                profile="control",
                save=True,
            )

        imu.unlock.assert_called_once_with()
        imu.set_baudrate.assert_called_once_with(Baudrate.BAUD_460800)
        imu.set_sampling_rate.assert_called_once_with(SamplingRate.RATE_10_HZ)
        imu.set_output_content.assert_called_once_with(
            time=False,
            acceleration=False,
            angular_velocity=True,
            angle=True,
            magnetic_field=False,
            port_status=False,
            pressure=False,
            gps=False,
            velocity=False,
            quaternion=True,
            gps_position_accuracy=False,
        )
        imu.save.assert_called_once_with()
        imu.close.assert_called_once_with()
        self.assertEqual(configuration, ImuStreamConfiguration("hiwonder", "/dev/ttyUSB0", 460800))


if __name__ == "__main__":
    unittest.main()
