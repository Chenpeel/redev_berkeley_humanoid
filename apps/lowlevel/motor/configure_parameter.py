# Copyright (c) 2025, The Berkeley Humanoid Lite Project Developers.

import argparse

from berkeley_humanoid_lite_lowlevel.cli import run_with_friendly_lowlevel_errors


def main() -> None:
    from berkeley_humanoid_lite_lowlevel.actuator import DEFAULT_ACTUATOR_BITRATE
    from berkeley_humanoid_lite_lowlevel.workflows import configure_actuator

    parser = argparse.ArgumentParser(description="Configure actuator parameters")
    parser.add_argument("--channel", type=str, default="can0", help="CAN transport channel")
    parser.add_argument("--id", type=int, default=1, help="Actuator device identifier")
    parser.add_argument("--bitrate", type=int, default=DEFAULT_ACTUATOR_BITRATE, help="CAN transport bitrate")
    parser.add_argument("--fast-frame-frequency", type=int, default=None, help="Fast frame frequency")
    parser.add_argument("--gear-ratio", type=float, default=None, help="Gear ratio")
    parser.add_argument("--kp", type=float, default=None, help="Position controller proportional gain")
    parser.add_argument("--kd", type=float, default=None, help="Position controller derivative gain")
    parser.add_argument("--torque-limit", type=float, default=None, help="Torque limit")
    parser.add_argument("--phase-order", type=int, default=None, help="Motor phase order")
    parser.add_argument("--position-limit-lower", type=float, default=None, help="Lower position limit")
    parser.add_argument("--position-limit-upper", type=float, default=None, help="Upper position limit")
    parser.add_argument("--current-bandwidth-hz", type=float, default=None, help="Current loop bandwidth")
    parser.add_argument("--phase-resistance", type=float, default=None, help="Motor phase resistance")
    parser.add_argument("--phase-inductance", type=float, default=None, help="Motor phase inductance")
    parser.add_argument("--store-to-flash", action="store_true", help="Persist updated settings to flash")
    arguments = parser.parse_args()

    if not any(
        value is not None
        for value in (
            arguments.fast_frame_frequency,
            arguments.gear_ratio,
            arguments.kp,
            arguments.kd,
            arguments.torque_limit,
            arguments.phase_order,
            arguments.position_limit_lower,
            arguments.position_limit_upper,
            arguments.current_bandwidth_hz,
        )
    ) and not arguments.store_to_flash:
        raise SystemExit("No parameter overrides provided.")

    configure_actuator(
        channel=arguments.channel,
        device_id=arguments.id,
        bitrate=arguments.bitrate,
        fast_frame_frequency=arguments.fast_frame_frequency,
        gear_ratio=arguments.gear_ratio,
        position_kp=arguments.kp,
        position_kd=arguments.kd,
        torque_limit=arguments.torque_limit,
        phase_order=arguments.phase_order,
        position_limit_lower=arguments.position_limit_lower,
        position_limit_upper=arguments.position_limit_upper,
        current_bandwidth_hz=arguments.current_bandwidth_hz,
        phase_resistance=arguments.phase_resistance,
        phase_inductance=arguments.phase_inductance,
        store_to_flash=arguments.store_to_flash,
    )


if __name__ == "__main__":
    run_with_friendly_lowlevel_errors(main)
