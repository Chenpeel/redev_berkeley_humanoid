# Copyright (c) 2025, The Berkeley Humanoid Lite Project Developers.

from __future__ import annotations

import argparse
import math

from berkeley_humanoid_lite.workflows import run_mujoco_joint_position_bridge
from berkeley_humanoid_lite_lowlevel.cli import add_leg_bus_arguments, run_with_friendly_gamepad_errors
from berkeley_humanoid_lite_lowlevel.policy import load_policy_deployment_configuration
from berkeley_humanoid_lite_lowlevel.runtime_paths import get_policy_config_path


def main() -> None:
    parser = argparse.ArgumentParser(description="Bridge MuJoCo joint deltas to real robot position targets")
    parser.add_argument(
        "--config",
        type=str,
        default=str(get_policy_config_path("policy_biped_50hz.yaml")),
        help="Path to the deployment configuration file",
    )
    add_leg_bus_arguments(parser)
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Compute bridge targets without sending motor position commands",
    )
    parser.add_argument(
        "--debug",
        action="store_true",
        help="Print MuJoCo delta, measured joint positions, and bridge targets while running",
    )
    parser.add_argument(
        "--debug-every",
        type=int,
        default=25,
        help="Print a bridge debug snapshot every N policy steps when --debug is enabled",
    )
    parser.add_argument(
        "--scale",
        "--bridge-scale",
        dest="bridge_scale",
        type=float,
        default=1.0,
        help="Uniform scaling factor applied to MuJoCo joint deltas before sending to the robot",
    )
    parser.add_argument(
        "--max-delta-deg",
        type=float,
        default=30.0,
        help="Clamp each bridged joint delta to +/- this many degrees",
    )
    parser.add_argument(
        "--max-step-deg",
        type=float,
        default=3.0,
        help="Clamp each control-step target update to +/- this many degrees",
    )
    parser.add_argument(
        "--position-kp",
        type=float,
        default=20.0,
        help="Position proportional gain written to the robot before bridge startup",
    )
    parser.add_argument(
        "--position-kd",
        type=float,
        default=2.0,
        help="Position derivative gain written to the robot before bridge startup",
    )
    parser.add_argument(
        "--torque-limit",
        type=float,
        default=4.0,
        help="Torque limit written to the robot before bridge startup",
    )
    parser.add_argument(
        "--policy-gate-deg",
        type=float,
        default=None,
        help="Override the lowlevel policy-entry standing-pose gate threshold in degrees",
    )
    parser.add_argument(
        "--enable-imu",
        action="store_true",
        help="Keep the lowlevel IMU enabled while the bridge is running",
    )
    arguments = parser.parse_args()

    configuration = load_policy_deployment_configuration(arguments.config)
    run_mujoco_joint_position_bridge(
        configuration,
        left_leg_bus=arguments.left_leg_bus,
        right_leg_bus=arguments.right_leg_bus,
        dry_run=arguments.dry_run,
        debug=arguments.debug,
        debug_every=arguments.debug_every,
        bridge_scale=arguments.bridge_scale,
        max_delta_radians=math.radians(arguments.max_delta_deg),
        max_step_radians=math.radians(arguments.max_step_deg),
        position_kp=arguments.position_kp,
        position_kd=arguments.position_kd,
        torque_limit=arguments.torque_limit,
        policy_gate_degrees=arguments.policy_gate_deg,
        enable_imu=arguments.enable_imu,
    )


if __name__ == "__main__":
    run_with_friendly_gamepad_errors(main)
