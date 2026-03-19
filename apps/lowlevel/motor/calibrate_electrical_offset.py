# Copyright (c) 2025, The Berkeley Humanoid Lite Project Developers.

import argparse

from berkeley_humanoid_lite_lowlevel.actuator import DEFAULT_ACTUATOR_BITRATE
from berkeley_humanoid_lite_lowlevel.workflows import run_actuator_calibration


def main() -> None:
    parser = argparse.ArgumentParser(description="Run actuator electrical offset calibration")
    parser.add_argument("--channel", type=str, default="can0", help="CAN transport channel")
    parser.add_argument("--id", type=int, default=1, help="Actuator device identifier")
    parser.add_argument("--bitrate", type=int, default=DEFAULT_ACTUATOR_BITRATE, help="CAN transport bitrate")
    parser.add_argument(
        "--wait-seconds",
        type=float,
        default=20.0,
        help="Seconds to wait for calibration sequence completion",
    )
    arguments = parser.parse_args()

    run_actuator_calibration(
        channel=arguments.channel,
        device_id=arguments.id,
        bitrate=arguments.bitrate,
        wait_seconds=arguments.wait_seconds,
    )


if __name__ == "__main__":
    main()
