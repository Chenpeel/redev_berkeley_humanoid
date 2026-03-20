from __future__ import annotations

from collections.abc import Callable

from berkeley_humanoid_lite_lowlevel.robot.command_source import GamepadInputError


def run_with_friendly_gamepad_errors(callback: Callable[[], None]) -> None:
    try:
        callback()
    except GamepadInputError as error:
        raise SystemExit(str(error)) from error
