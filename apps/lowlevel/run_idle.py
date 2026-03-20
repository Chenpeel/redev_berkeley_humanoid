# Copyright (c) 2025, The Berkeley Humanoid Lite Project Developers.

from berkeley_humanoid_lite_lowlevel.cli import run_with_friendly_gamepad_errors
from berkeley_humanoid_lite_lowlevel.policy.configuration import load_policy_deployment_configuration_from_cli
from berkeley_humanoid_lite_lowlevel.workflows import run_idle_stream


def main() -> None:
    configuration = load_policy_deployment_configuration_from_cli()
    run_idle_stream(configuration)


if __name__ == "__main__":
    run_with_friendly_gamepad_errors(main)
