# Copyright (c) 2025, The Berkeley Humanoid Lite Project Developers.

from berkeley_humanoid_lite.workflows import run_mujoco_policy_loop
from berkeley_humanoid_lite_lowlevel.cli import run_with_friendly_gamepad_errors
from berkeley_humanoid_lite_lowlevel.policy import load_policy_deployment_configuration_from_cli


def main() -> None:
    configuration = load_policy_deployment_configuration_from_cli()
    run_mujoco_policy_loop(configuration)


if __name__ == "__main__":
    run_with_friendly_gamepad_errors(main)
