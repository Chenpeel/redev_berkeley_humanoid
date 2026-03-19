# Copyright (c) 2025, The Berkeley Humanoid Lite Project Developers.

import argparse

from berkeley_humanoid_lite_lowlevel.actuator import DEFAULT_ACTUATOR_BITRATE
from berkeley_humanoid_lite_lowlevel.runtime_paths import get_hardware_config_path
from berkeley_humanoid_lite_lowlevel.workflows import export_actuator_configuration


def main() -> None:
    parser = argparse.ArgumentParser(description="Export actuator configuration")
    parser.add_argument("--channel", type=str, default="can0", help="CAN transport channel")
    parser.add_argument("--id", type=int, default=1, help="Actuator device identifier")
    parser.add_argument("--bitrate", type=int, default=DEFAULT_ACTUATOR_BITRATE, help="CAN transport bitrate")
    parser.add_argument(
        "--output",
        type=str,
        default=str(get_hardware_config_path("motor_configuration.json")),
        help="Path to the exported actuator configuration file",
    )
    arguments = parser.parse_args()

    output_path = export_actuator_configuration(
        channel=arguments.channel,
        device_id=arguments.id,
        bitrate=arguments.bitrate,
        output_path=arguments.output,
    )
    print(f"Saved configuration to {output_path}")


if __name__ == "__main__":
    main()
