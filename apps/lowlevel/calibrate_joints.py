"""
calibrate_joints.py

Copyright (c) 2025, The Berkeley Humanoid Lite Project Developers.

Run this script after each power cycle to calibrate the encoder offset of each joint.
"""

from berkeley_humanoid_lite_lowlevel.cli import run_with_friendly_gamepad_errors


def main() -> None:
    from berkeley_humanoid_lite_lowlevel.workflows import run_joint_calibration

    run_joint_calibration()


if __name__ == "__main__":
    run_with_friendly_gamepad_errors(main)
