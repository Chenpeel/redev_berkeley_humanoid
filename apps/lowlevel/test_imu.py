import argparse

from berkeley_humanoid_lite_lowlevel.workflows import stream_orientation
from berkeley_humanoid_lite_lowlevel.workflows.imu import (
    DEFAULT_IMU_BAUDRATE,
    DEFAULT_IMU_SERIAL_DEVICE,
    DEFAULT_IMU_TIMEOUT,
)


def main() -> None:
    parser = argparse.ArgumentParser(description="Stream orientation data from the serial IMU.")
    parser.add_argument(
        "--device",
        type=str,
        default=DEFAULT_IMU_SERIAL_DEVICE,
        help="Serial device path or 'auto' to probe common IMU ports",
    )
    parser.add_argument("--baudrate", type=int, default=DEFAULT_IMU_BAUDRATE, help="Serial baudrate")
    parser.add_argument("--timeout", type=float, default=DEFAULT_IMU_TIMEOUT, help="Serial read timeout in seconds")
    arguments = parser.parse_args()

    stream_orientation(
        device=arguments.device,
        baudrate=arguments.baudrate,
        timeout=arguments.timeout,
    )


if __name__ == "__main__":
    main()
