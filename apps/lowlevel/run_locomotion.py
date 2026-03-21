# Copyright (c) 2025, The Berkeley Humanoid Lite Project Developers.

import argparse

from berkeley_humanoid_lite_lowlevel.cli import add_leg_bus_arguments, run_with_friendly_gamepad_errors
from berkeley_humanoid_lite_lowlevel.runtime_paths import get_policy_config_path


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
    arguments = parser.parse_args()

    configuration = load_policy_deployment_configuration(arguments.config)
    run_locomotion_loop(
        configuration,
        left_leg_bus=arguments.left_leg_bus,
        right_leg_bus=arguments.right_leg_bus,
    )


if __name__ == "__main__":
    run_with_friendly_gamepad_errors(main)
