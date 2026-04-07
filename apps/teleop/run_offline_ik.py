# Copyright (c) 2025, The Berkeley Humanoid Lite Project Developers.

from __future__ import annotations

import argparse

from berkeley_humanoid_lite_lowlevel.teleoperation import SUPPORTED_FAKE_TELEOP_PATTERNS
from berkeley_humanoid_lite_lowlevel.workflows import run_offline_teleoperation_ik


def main() -> None:
    parser = argparse.ArgumentParser(description="Run offline teleoperation IK validation with fake controller data")
    parser.add_argument(
        "--pattern",
        choices=SUPPORTED_FAKE_TELEOP_PATTERNS,
        default="dual_arm_circle",
        help="Fake controller motion pattern",
    )
    parser.add_argument("--frequency", type=float, default=100.0, help="Update frequency in Hz")
    parser.add_argument(
        "--duration",
        type=float,
        default=10.0,
        help="Run duration in seconds, use 0 or a negative value to run until interrupted",
    )
    parser.add_argument(
        "--left-enable",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="Enable left arm clutch input",
    )
    parser.add_argument(
        "--right-enable",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="Enable right arm clutch input",
    )
    parser.add_argument(
        "--with-gripper",
        action="store_true",
        help="Drive gripper triggers with a synthetic waveform",
    )
    parser.add_argument(
        "--sweep-arm",
        choices=("auto", "left", "right"),
        default="auto",
        help="Arm to animate for the single_arm_sweep pattern",
    )
    parser.add_argument(
        "--measurement-alpha",
        type=float,
        default=1.0,
        help="Observation write-back factor in (0, 1], lower values add first-order lag",
    )
    parser.add_argument("--print-every", type=int, default=10, help="Print one status line every N control steps")
    arguments = parser.parse_args()

    duration_seconds = None if arguments.duration <= 0.0 else arguments.duration
    run_offline_teleoperation_ik(
        pattern=arguments.pattern,
        frequency=arguments.frequency,
        duration_seconds=duration_seconds,
        left_enable=arguments.left_enable,
        right_enable=arguments.right_enable,
        with_gripper=arguments.with_gripper,
        sweep_arm=arguments.sweep_arm,
        measurement_alpha=arguments.measurement_alpha,
        print_every=arguments.print_every,
    )


if __name__ == "__main__":
    main()
