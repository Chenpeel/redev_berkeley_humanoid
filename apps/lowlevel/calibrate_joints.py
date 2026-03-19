"""
calibrate_joints.py

Copyright (c) 2025, The Berkeley Humanoid Lite Project Developers.

Run this script after each power cycle to calibrate the encoder offset of each joint.
"""

from berkeley_humanoid_lite_lowlevel.workflows import run_joint_calibration


def main() -> None:
    run_joint_calibration()


if __name__ == "__main__":
    main()
