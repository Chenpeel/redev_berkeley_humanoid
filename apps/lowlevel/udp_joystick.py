import argparse

from berkeley_humanoid_lite_lowlevel.cli import run_with_friendly_gamepad_errors
from berkeley_humanoid_lite_lowlevel.workflows import broadcast_gamepad_commands
from berkeley_humanoid_lite_lowlevel.workflows.locomotion import (
    DEFAULT_GAMEPAD_UDP_HOST,
    DEFAULT_GAMEPAD_UDP_PORT,
    DEFAULT_GAMEPAD_UDP_RATE_HZ,
)


def main() -> None:
    parser = argparse.ArgumentParser(description="Broadcast gamepad commands to the native runtime over UDP.")
    parser.add_argument(
        "--host",
        type=str,
        default=DEFAULT_GAMEPAD_UDP_HOST,
        help="Target host for joystick UDP packets",
    )
    parser.add_argument(
        "--port",
        type=int,
        default=DEFAULT_GAMEPAD_UDP_PORT,
        help="Target UDP port for joystick packets",
    )
    parser.add_argument(
        "--rate-hz",
        type=float,
        default=DEFAULT_GAMEPAD_UDP_RATE_HZ,
        help="Broadcast rate in Hz",
    )
    arguments = parser.parse_args()

    broadcast_gamepad_commands(
        host=arguments.host,
        port=arguments.port,
        rate_hz=arguments.rate_hz,
    )


if __name__ == "__main__":
    run_with_friendly_gamepad_errors(main)
