# Copyright (c) 2025, The Berkeley Humanoid Lite Project Developers.

import argparse

from berkeley_humanoid_lite_lowlevel.runtime_paths import get_hardware_config_path
from berkeley_humanoid_lite_lowlevel.workflows import apply_robot_configuration


def main() -> None:
    parser = argparse.ArgumentParser(description="Apply robot configuration")
    parser.add_argument(
        "--config",
        type=str,
        default=str(get_hardware_config_path("robot_configuration.json")),
        help="Path to the configuration file to apply",
    )
    parser.add_argument(
        "--no-store-to-flash",
        action="store_true",
        help="Skip persisting parameters to actuator flash",
    )
    arguments = parser.parse_args()

    apply_robot_configuration(
        arguments.config,
        store_to_flash=not arguments.no_store_to_flash,
    )


if __name__ == "__main__":
    main()
