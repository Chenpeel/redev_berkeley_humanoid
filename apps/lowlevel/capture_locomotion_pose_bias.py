# Copyright (c) 2025, The Berkeley Humanoid Lite Project Developers.

import argparse

from berkeley_humanoid_lite_lowlevel.cli import add_leg_bus_arguments, run_with_friendly_gamepad_errors
from berkeley_humanoid_lite_lowlevel.runtime_paths import get_policy_config_path, get_pose_alignment_path


def main() -> None:
    from berkeley_humanoid_lite_lowlevel.policy.configuration import load_policy_deployment_configuration
    from berkeley_humanoid_lite_lowlevel.workflows import run_locomotion_pose_alignment_capture

    parser = argparse.ArgumentParser(description="Capture locomotion nominal pose alignment bias")
    parser.add_argument(
        "--config",
        type=str,
        default=str(get_policy_config_path("policy_biped_50hz.yaml")),
        help="Path to the deployment configuration file",
    )
    add_leg_bus_arguments(parser)
    parser.add_argument(
        "--pose-alignment-path",
        type=str,
        default=str(get_pose_alignment_path()),
        help="Path to the locomotion pose-alignment file",
    )
    parser.add_argument(
        "--polling-interval",
        type=float,
        default=0.05,
        help="Polling interval in seconds while previewing the taught pose",
    )
    parser.add_argument(
        "--capture-window-size",
        type=int,
        default=20,
        help="Number of recent samples to average when A+LB requests capture",
    )
    parser.add_argument(
        "--max-stddev-deg",
        type=float,
        default=1.0,
        help="Maximum joint sample stddev in degrees allowed before saving",
    )
    parser.add_argument(
        "--write",
        action="store_true",
        help="Persist the captured bias to the pose-alignment file",
    )
    arguments = parser.parse_args()

    configuration = load_policy_deployment_configuration(arguments.config)
    run_locomotion_pose_alignment_capture(
        configuration,
        left_leg_bus=arguments.left_leg_bus,
        right_leg_bus=arguments.right_leg_bus,
        pose_alignment_path=arguments.pose_alignment_path,
        polling_interval_seconds=arguments.polling_interval,
        capture_window_size=arguments.capture_window_size,
        max_stddev_deg=arguments.max_stddev_deg,
        write=arguments.write,
    )


if __name__ == "__main__":
    run_with_friendly_gamepad_errors(main)
