import argparse

from berkeley_humanoid_lite_lowlevel.workflows import stream_orientation
from berkeley_humanoid_lite_lowlevel.workflows.imu import (
    DEFAULT_IMU_BAUDRATE,
    DEFAULT_IMU_PROBE_DURATION,
    DEFAULT_IMU_PROTOCOL,
    DEFAULT_IMU_SERIAL_DEVICE,
    DEFAULT_IMU_TIMEOUT,
    SUPPORTED_IMU_PROTOCOLS,
)


def main() -> None:
    parser = argparse.ArgumentParser(description="Stream data from the serial IMU with optional auto detection.")
    parser.add_argument(
        "--protocol",
        type=str,
        choices=SUPPORTED_IMU_PROTOCOLS,
        default=DEFAULT_IMU_PROTOCOL,
        help="IMU protocol to use: auto, hiwonder, or packet",
    )
    parser.add_argument(
        "--device",
        type=str,
        default=DEFAULT_IMU_SERIAL_DEVICE,
        help="Serial device path or 'auto' to probe common IMU ports",
    )
    parser.add_argument(
        "--baudrate",
        type=str,
        default=DEFAULT_IMU_BAUDRATE,
        help="Serial baudrate or 'auto' to probe common values",
    )
    parser.add_argument("--timeout", type=float, default=DEFAULT_IMU_TIMEOUT, help="Serial read timeout in seconds")
    parser.add_argument(
        "--probe-duration",
        type=float,
        default=DEFAULT_IMU_PROBE_DURATION,
        help="Probe time in seconds for each auto-detection candidate",
    )
    arguments = parser.parse_args()

    stream_orientation(
        protocol=arguments.protocol,
        device=arguments.device,
        baudrate=arguments.baudrate,
        timeout=arguments.timeout,
        probe_duration=arguments.probe_duration,
    )


if __name__ == "__main__":
    main()
