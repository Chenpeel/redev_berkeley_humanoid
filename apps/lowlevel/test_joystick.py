"""
test_joystick.py

Copyright (c) 2025, The Berkeley Humanoid Lite Project Developers.

Stream the latest locomotion command decoded from the gamepad.
"""

from berkeley_humanoid_lite_lowlevel.workflows import stream_gamepad_commands


def main() -> None:
    stream_gamepad_commands()


if __name__ == "__main__":
    main()
