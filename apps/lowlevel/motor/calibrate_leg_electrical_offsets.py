# Copyright (c) 2025, The Berkeley Humanoid Lite Project Developers.

import argparse

from berkeley_humanoid_lite_lowlevel.cli import add_leg_bus_arguments, run_with_friendly_lowlevel_errors


def main() -> None:
    from berkeley_humanoid_lite_lowlevel.actuator import DEFAULT_ACTUATOR_BITRATE
    from berkeley_humanoid_lite_lowlevel.workflows import run_leg_actuator_calibration

    parser = argparse.ArgumentParser(
        description="Run electrical offset calibration for all lower-body actuators and optionally store to flash",
    )
    add_leg_bus_arguments(parser)
    parser.add_argument("--bitrate", type=int, default=DEFAULT_ACTUATOR_BITRATE, help="CAN transport bitrate")
    parser.add_argument(
        "--wait-seconds",
        type=float,
        default=20.0,
        help="Seconds to wait for each actuator calibration sequence to complete",
    )
    parser.add_argument(
        "--flash-settle-seconds",
        type=float,
        default=0.5,
        help="Seconds to wait between electrical calibration and store-to-flash",
    )
    parser.add_argument(
        "--inter-joint-delay-seconds",
        type=float,
        default=0.5,
        help="Seconds to wait before moving on to the next actuator",
    )
    parser.add_argument(
        "--no-store-to-flash",
        action="store_true",
        help="Skip persisting the freshly calibrated settings to actuator flash",
    )
    arguments = parser.parse_args()

    run_leg_actuator_calibration(
        left_leg_bus=arguments.left_leg_bus,
        right_leg_bus=arguments.right_leg_bus,
        bitrate=arguments.bitrate,
        wait_seconds=arguments.wait_seconds,
        store_to_flash=not arguments.no_store_to_flash,
        flash_settle_seconds=arguments.flash_settle_seconds,
        inter_joint_delay_seconds=arguments.inter_joint_delay_seconds,
    )


if __name__ == "__main__":
    run_with_friendly_lowlevel_errors(main)
