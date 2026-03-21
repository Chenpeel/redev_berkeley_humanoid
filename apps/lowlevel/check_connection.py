# Copyright (c) 2025, The Berkeley Humanoid Lite Project Developers.

import argparse

from berkeley_humanoid_lite_lowlevel.cli import add_leg_bus_arguments, run_with_friendly_lowlevel_errors


def main() -> None:
    from berkeley_humanoid_lite_lowlevel.workflows import check_locomotion_connection

    parser = argparse.ArgumentParser(description="Check lower-body actuator connectivity")
    add_leg_bus_arguments(parser)
    arguments = parser.parse_args()

    check_locomotion_connection(
        left_leg_bus=arguments.left_leg_bus,
        right_leg_bus=arguments.right_leg_bus,
    )


if __name__ == "__main__":
    run_with_friendly_lowlevel_errors(main)
