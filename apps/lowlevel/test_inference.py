import argparse

from berkeley_humanoid_lite_lowlevel.policy.configuration import load_policy_deployment_configuration
from berkeley_humanoid_lite_lowlevel.runtime_paths import get_policy_config_path
from berkeley_humanoid_lite_lowlevel.workflows import run_policy_inference_smoke_test


def main() -> None:
    parser = argparse.ArgumentParser(description="Policy inference smoke test")
    parser.add_argument(
        "--config",
        type=str,
        default=str(get_policy_config_path("policy_humanoid.yaml")),
        help="Path to the deployment configuration file",
    )
    arguments = parser.parse_args()

    configuration = load_policy_deployment_configuration(arguments.config)
    run_policy_inference_smoke_test(configuration)


if __name__ == "__main__":
    main()
