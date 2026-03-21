# Copyright (c) 2025, The Berkeley Humanoid Lite Project Developers.

import argparse

from berkeley_humanoid_lite_lowlevel.cli import run_with_friendly_lowlevel_errors


def main() -> None:
    from berkeley_humanoid_lite_lowlevel.actuator import DEFAULT_ACTUATOR_BITRATE
    from berkeley_humanoid_lite_lowlevel.workflows import run_actuator_angle_test

    parser = argparse.ArgumentParser(description="Move one actuator to a target angle with a linear ramp.")
    parser.add_argument("--channel", type=str, default="can0", help="CAN transport channel")
    parser.add_argument("--id", type=int, default=1, help="Actuator device identifier")
    parser.add_argument("--bitrate", type=int, default=DEFAULT_ACTUATOR_BITRATE, help="CAN transport bitrate")

    target_group = parser.add_mutually_exclusive_group(required=True)
    target_group.add_argument("--target-rad", type=float, help="Target angle in radians")
    target_group.add_argument("--target-deg", type=float, help="Target angle in degrees")

    return_group = parser.add_mutually_exclusive_group()
    return_group.add_argument("--return-rad", type=float, help="Return angle in radians for cyclic tests")
    return_group.add_argument("--return-deg", type=float, help="Return angle in degrees for cyclic tests")

    parser.add_argument("--kp", type=float, default=0.2, help="Position proportional gain")
    parser.add_argument("--kd", type=float, default=0.005, help="Position derivative gain")
    parser.add_argument("--torque-limit", type=float, default=0.2, help="Torque limit")
    parser.add_argument(
        "--max-speed-deg",
        type=float,
        default=30.0,
        help="Maximum ramp speed in degrees per second",
    )
    parser.add_argument(
        "--hold-seconds",
        type=float,
        default=2.0,
        help="How long to hold the target before moving to the next stage",
    )
    parser.add_argument(
        "--control-rate",
        type=float,
        default=200.0,
        help="Position command update frequency in Hz",
    )
    parser.add_argument(
        "--position-tolerance-deg",
        type=float,
        default=2.0,
        help="Maximum allowed position error during the hold evaluation phase",
    )
    parser.add_argument(
        "--velocity-tolerance-deg",
        type=float,
        default=10.0,
        help="Maximum allowed speed magnitude during the hold evaluation phase",
    )
    parser.add_argument(
        "--settle-timeout-seconds",
        type=float,
        default=2.0,
        help="Maximum time allowed for the actuator to settle after the ramp finishes",
    )
    parser.add_argument(
        "--stable-samples",
        type=int,
        default=20,
        help="Number of consecutive in-tolerance samples required before hold evaluation starts",
    )
    parser.add_argument(
        "--cycles",
        type=int,
        default=None,
        help=(
            "Number of target to return cycles. "
            "If omitted, perform a single move to the target."
        ),
    )
    arguments = parser.parse_args()

    run_actuator_angle_test(
        channel=arguments.channel,
        device_id=arguments.id,
        bitrate=arguments.bitrate,
        target_angle_radians=arguments.target_rad,
        target_angle_degrees=arguments.target_deg,
        return_angle_radians=arguments.return_rad,
        return_angle_degrees=arguments.return_deg,
        position_kp=arguments.kp,
        position_kd=arguments.kd,
        torque_limit=arguments.torque_limit,
        max_speed_degrees_per_second=arguments.max_speed_deg,
        hold_seconds=arguments.hold_seconds,
        control_frequency_hz=arguments.control_rate,
        position_tolerance_degrees=arguments.position_tolerance_deg,
        velocity_tolerance_degrees_per_second=arguments.velocity_tolerance_deg,
        settle_timeout_seconds=arguments.settle_timeout_seconds,
        required_stable_samples=arguments.stable_samples,
        cycles=arguments.cycles,
    )


if __name__ == "__main__":
    run_with_friendly_lowlevel_errors(main)
