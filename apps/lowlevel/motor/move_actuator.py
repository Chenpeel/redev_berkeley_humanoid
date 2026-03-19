# Copyright (c) 2025, The Berkeley Humanoid Lite Project Developers.

import argparse

from berkeley_humanoid_lite_lowlevel.actuator import DEFAULT_ACTUATOR_BITRATE
from berkeley_humanoid_lite_lowlevel.workflows import run_actuator_motion_demo


def main() -> None:
    parser = argparse.ArgumentParser(description="Run actuator sine motion demo")
    parser.add_argument("--channel", type=str, default="can0", help="CAN transport channel")
    parser.add_argument("--id", type=int, default=1, help="Actuator device identifier")
    parser.add_argument("--bitrate", type=int, default=DEFAULT_ACTUATOR_BITRATE, help="CAN transport bitrate")
    parser.add_argument("--kp", type=float, default=0.2, help="Position controller proportional gain")
    parser.add_argument("--kd", type=float, default=0.005, help="Position controller derivative gain")
    parser.add_argument("--torque-limit", type=float, default=0.2, help="Torque limit")
    parser.add_argument("--motion-frequency-hz", type=float, default=1.0, help="Sine motion frequency in Hz")
    parser.add_argument(
        "--motion-amplitude-radians",
        type=float,
        default=1.0,
        help="Sine motion amplitude in radians",
    )
    parser.add_argument(
        "--control-frequency-hz",
        type=float,
        default=200.0,
        help="Command loop frequency in Hz",
    )
    arguments = parser.parse_args()

    run_actuator_motion_demo(
        channel=arguments.channel,
        device_id=arguments.id,
        bitrate=arguments.bitrate,
        position_kp=arguments.kp,
        position_kd=arguments.kd,
        torque_limit=arguments.torque_limit,
        motion_frequency_hz=arguments.motion_frequency_hz,
        motion_amplitude_radians=arguments.motion_amplitude_radians,
        control_frequency_hz=arguments.control_frequency_hz,
    )


if __name__ == "__main__":
    main()
