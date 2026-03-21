# Copyright (c) 2025, The Berkeley Humanoid Lite Project Developers.

import argparse

from berkeley_humanoid_lite_lowlevel.cli import add_leg_bus_arguments, run_with_friendly_lowlevel_errors
from berkeley_humanoid_lite_lowlevel.runtime_paths import get_hardware_config_path


def main() -> None:
    from berkeley_humanoid_lite_lowlevel.workflows import export_robot_configuration

    parser = argparse.ArgumentParser(description="Export robot configuration")
    parser.add_argument(
        "--output",
        type=str,
        default=str(get_hardware_config_path("robot_configuration.json")),
        help="Path to the exported configuration file",
    )
    add_leg_bus_arguments(parser)
    arguments = parser.parse_args()

    output_path = export_robot_configuration(
        arguments.output,
        left_leg_bus=arguments.left_leg_bus,
        right_leg_bus=arguments.right_leg_bus,
    )
    print(f"Saved configuration to {output_path}")


if __name__ == "__main__":
    run_with_friendly_lowlevel_errors(main)
