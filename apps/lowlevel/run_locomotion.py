# Copyright (c) 2025, The Berkeley Humanoid Lite Project Developers.

from berkeley_humanoid_lite_lowlevel.policy.configuration import load_policy_deployment_configuration_from_cli
from berkeley_humanoid_lite_lowlevel.workflows import run_locomotion_loop


def main() -> None:
    configuration = load_policy_deployment_configuration_from_cli()
    run_locomotion_loop(configuration)


if __name__ == "__main__":
    main()
