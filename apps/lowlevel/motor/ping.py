# Copyright (c) 2025, The Berkeley Humanoid Lite Project Developers.

import argparse

from berkeley_humanoid_lite_lowlevel.actuator import DEFAULT_ACTUATOR_BITRATE
from berkeley_humanoid_lite_lowlevel.workflows import check_actuator_connection


def main() -> None:
    parser = argparse.ArgumentParser(description="Check actuator connectivity")
    parser.add_argument("--channel", type=str, default="can0", help="CAN transport channel")
    parser.add_argument("--id", type=int, default=1, help="Actuator device identifier")
    parser.add_argument("--bitrate", type=int, default=DEFAULT_ACTUATOR_BITRATE, help="CAN transport bitrate")
    arguments = parser.parse_args()

    is_online = check_actuator_connection(
        channel=arguments.channel,
        device_id=arguments.id,
        bitrate=arguments.bitrate,
    )
    print("Motor is online" if is_online else "Motor is offline")


if __name__ == "__main__":
    main()
