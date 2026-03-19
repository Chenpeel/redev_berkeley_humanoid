# Copyright (c) 2025, The Berkeley Humanoid Lite Project Developers.

import argparse

from berkeley_humanoid_lite_lowlevel.runtime_paths import get_hardware_config_path
from berkeley_humanoid_lite_lowlevel.workflows import export_robot_configuration


def main() -> None:
    parser = argparse.ArgumentParser(description="Export robot configuration")
    parser.add_argument(
        "--output",
        type=str,
        default=str(get_hardware_config_path("robot_configuration.json")),
        help="Path to the exported configuration file",
    )
    arguments = parser.parse_args()

    output_path = export_robot_configuration(arguments.output)
    print(f"Saved configuration to {output_path}")


if __name__ == "__main__":
    main()
