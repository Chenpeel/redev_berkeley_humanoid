"""
calibrate_joints.py

Copyright (c) 2025, The Berkeley Humanoid Lite Project Developers.

Run this script after each power cycle to calibrate the encoder offset of each joint.

Important:
This calibration records the raw joint readings for the specification-defined
calibration reference pose. It does not ask the operator to push each joint to
its farthest mechanical limit.
"""

import argparse

from berkeley_humanoid_lite_lowlevel.cli import add_leg_bus_arguments, run_with_friendly_gamepad_errors


def main() -> None:
    from berkeley_humanoid_lite_lowlevel.workflows import run_joint_calibration

    parser = argparse.ArgumentParser(description="Calibrate lower-body joint offsets")
    add_leg_bus_arguments(parser)
    arguments = parser.parse_args()

    print("Joint calibration note:")
    print("  This workflow records the raw readings for the predefined calibration reference pose.")
    print("  Do not force each joint to its farthest mechanical limit.")
    print("  Hold the robot at the intended calibration pose, then switch the gamepad back to IDLE to save.")

    run_joint_calibration(
        left_leg_bus=arguments.left_leg_bus,
        right_leg_bus=arguments.right_leg_bus,
    )


if __name__ == "__main__":
    run_with_friendly_gamepad_errors(main)
