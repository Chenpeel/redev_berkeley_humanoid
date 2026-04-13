# Copyright (c) 2025, The Berkeley Humanoid Lite Project Developers.

from __future__ import annotations

import argparse

from berkeley_humanoid_lite.workflows import run_quest_webxr_observation_visualizer
from berkeley_humanoid_lite_lowlevel.policy import load_policy_deployment_configuration
from berkeley_humanoid_lite_lowlevel.runtime_paths import get_policy_config_path


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Serve a Quest WebXR page and drive the sim2real MuJoCo visualizer from controller motion"
    )
    parser.add_argument(
        "--config",
        type=str,
        default=str(get_policy_config_path("policy_humanoid.yaml")),
        help="Path to the deployment configuration file",
    )
    parser.add_argument("--host", type=str, default="0.0.0.0", help="Bind host for HTTPS and WSS")
    parser.add_argument("--https-port", type=int, default=8443, help="HTTPS port for the Quest page")
    parser.add_argument("--ws-port", type=int, default=8442, help="WebSocket port for controller streaming")
    parser.add_argument(
        "--position-scale",
        type=float,
        default=1.0,
        help="Uniform scale applied to WebXR controller translation before IK",
    )
    parser.add_argument(
        "--frequency",
        type=float,
        default=60.0,
        help="Visualizer update frequency in Hz",
    )
    parser.add_argument(
        "--measurement-alpha",
        type=float,
        default=0.35,
        help="Observation write-back factor in (0, 1]",
    )
    parser.add_argument(
        "--cert-file",
        type=str,
        default=None,
        help="Optional certificate path for HTTPS/WSS",
    )
    parser.add_argument(
        "--key-file",
        type=str,
        default=None,
        help="Optional private key path for HTTPS/WSS",
    )
    arguments = parser.parse_args()

    configuration = load_policy_deployment_configuration(arguments.config)
    run_quest_webxr_observation_visualizer(
        configuration,
        host=arguments.host,
        https_port=arguments.https_port,
        websocket_port=arguments.ws_port,
        position_scale=arguments.position_scale,
        frequency=arguments.frequency,
        measurement_alpha=arguments.measurement_alpha,
        certificate_path=arguments.cert_file,
        private_key_path=arguments.key_file,
    )


if __name__ == "__main__":
    main()
