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
    parser.add_argument("--vx", type=float, default=0.0, help="Synthetic forward command velocity for the smoke test")
    parser.add_argument("--vy", type=float, default=0.0, help="Synthetic lateral command velocity for the smoke test")
    parser.add_argument("--vyaw", type=float, default=0.0, help="Synthetic yaw command velocity for the smoke test")
    arguments = parser.parse_args()

    configuration = load_policy_deployment_configuration(arguments.config)
    run_policy_inference_smoke_test(
        configuration,
        command_velocity=(arguments.vx, arguments.vy, arguments.vyaw),
    )


if __name__ == "__main__":
    main()
