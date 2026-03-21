# Copyright (c) 2025, The Berkeley Humanoid Lite Project Developers.

import argparse

from berkeley_humanoid_lite_lowlevel.cli import add_leg_bus_arguments, run_with_friendly_lowlevel_errors
from berkeley_humanoid_lite_lowlevel.runtime_paths import get_hardware_config_path


def main() -> None:
    from berkeley_humanoid_lite_lowlevel.workflows import apply_robot_configuration

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
    add_leg_bus_arguments(parser)
    arguments = parser.parse_args()

    apply_robot_configuration(
        arguments.config,
        store_to_flash=not arguments.no_store_to_flash,
        left_leg_bus=arguments.left_leg_bus,
        right_leg_bus=arguments.right_leg_bus,
    )


if __name__ == "__main__":
    run_with_friendly_lowlevel_errors(main)
