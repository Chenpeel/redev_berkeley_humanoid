# Copyright (c) 2025, The Berkeley Humanoid Lite Project Developers.

import argparse

from berkeley_humanoid_lite_lowlevel.cli import add_leg_bus_arguments, run_with_friendly_gamepad_errors
from berkeley_humanoid_lite_lowlevel.runtime_paths import get_policy_config_path
from berkeley_humanoid_lite_lowlevel.workflows.imu import (
    DEFAULT_IMU_BAUDRATE,
    DEFAULT_IMU_PROBE_DURATION,
    DEFAULT_IMU_PROTOCOL,
    DEFAULT_IMU_SERIAL_DEVICE,
    DEFAULT_IMU_TIMEOUT,
    normalize_hiwonder_baudrate,
)
from berkeley_humanoid_lite_lowlevel.workflows.locomotion import (
    DEFAULT_LOCOMOTION_IMU_WAIT_TIMEOUT,
    resolve_locomotion_imu_configuration,
)

SUPPORTED_LOCOMOTION_IMU_PROTOCOLS = ("auto", "hiwonder")


def main() -> None:
    from berkeley_humanoid_lite_lowlevel.policy.configuration import load_policy_deployment_configuration
    from berkeley_humanoid_lite_lowlevel.workflows import run_locomotion_loop

    parser = argparse.ArgumentParser(description="Run lower-body locomotion loop")
    parser.add_argument(
        "--config",
        type=str,
        default=str(get_policy_config_path("policy_biped_50hz.yaml")),
        help="Path to the deployment configuration file",
    )
    add_leg_bus_arguments(parser)
    parser.add_argument(
        "--debug",
        action="store_true",
        help="Print locomotion state, command, action, and joint snapshots while running",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Read sensors and compute targets without sending motor position commands",
    )
    parser.add_argument(
        "--debug-every",
        type=int,
        default=25,
        help="Print a debug snapshot every N policy steps when --debug is enabled",
    )
    parser.add_argument(
        "--debug-imu",
        action="store_true",
        help="Print IMU attitude and angular velocity snapshots while running",
    )
    parser.add_argument(
        "--debug-imu-every",
        type=int,
        default=25,
        help="Print an IMU debug snapshot every N policy steps when --debug-imu is enabled",
    )
    parser.add_argument(
        "--imu-protocol",
        type=str,
        choices=SUPPORTED_LOCOMOTION_IMU_PROTOCOLS,
        default=DEFAULT_IMU_PROTOCOL,
        help="IMU protocol to use for lowlevel locomotion runtime",
    )
    parser.add_argument(
        "--imu-device",
        type=str,
        default=DEFAULT_IMU_SERIAL_DEVICE,
        help="Serial device path or 'auto' to probe common IMU ports",
    )
    parser.add_argument(
        "--imu-baudrate",
        type=str,
        default=DEFAULT_IMU_BAUDRATE,
        help="Serial baudrate or 'auto' to probe common values",
    )
    parser.add_argument(
        "--imu-timeout",
        type=float,
        default=DEFAULT_IMU_TIMEOUT,
        help="Serial read timeout in seconds",
    )
    parser.add_argument(
        "--imu-probe-duration",
        type=float,
        default=DEFAULT_IMU_PROBE_DURATION,
        help="Probe time in seconds for each auto-detection candidate",
    )
    parser.add_argument(
        "--imu-wait-timeout",
        type=float,
        default=DEFAULT_LOCOMOTION_IMU_WAIT_TIMEOUT,
        help="Maximum time to wait for valid quaternion and gyro frames before reset",
    )
    parser.add_argument(
        "--skip-imu-ready-check",
        action="store_true",
        help="Start locomotion without waiting for the IMU quaternion/gyro stream to become ready",
    )
    arguments = parser.parse_args()

    configuration = load_policy_deployment_configuration(arguments.config)
    imu_configuration = resolve_locomotion_imu_configuration(
        protocol=arguments.imu_protocol,
        device=arguments.imu_device,
        baudrate=arguments.imu_baudrate,
        timeout=arguments.imu_timeout,
        probe_duration=arguments.imu_probe_duration,
    )
    print(
        "Resolved IMU config:",
        f"protocol={imu_configuration.protocol}",
        f"device={imu_configuration.device}",
        f"baudrate={imu_configuration.baudrate}",
    )
    run_locomotion_loop(
        configuration,
        left_leg_bus=arguments.left_leg_bus,
        right_leg_bus=arguments.right_leg_bus,
        dry_run=arguments.dry_run,
        debug=arguments.debug,
        debug_every=arguments.debug_every,
        debug_imu=arguments.debug_imu,
        debug_imu_every=arguments.debug_imu_every,
        imu_device=imu_configuration.device,
        imu_baudrate=normalize_hiwonder_baudrate(imu_configuration.baudrate),
        imu_timeout=arguments.imu_timeout,
        imu_wait_timeout=arguments.imu_wait_timeout,
        require_imu_ready=not arguments.skip_imu_ready_check,
    )


if __name__ == "__main__":
    run_with_friendly_gamepad_errors(main)
