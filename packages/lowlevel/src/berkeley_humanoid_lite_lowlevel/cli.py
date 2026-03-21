from __future__ import annotations

from collections.abc import Callable

from berkeley_humanoid_lite_lowlevel.robot.command_source import GamepadInputError

_LOWLEVEL_RUNTIME_SYNC_COMMAND = "uv sync --all-packages --group dev --group lowlevel-runtime"
_MISSING_DEPENDENCY_HINTS = {
    "can": ("python-can", _LOWLEVEL_RUNTIME_SYNC_COMMAND),
    "inputs": ("inputs", _LOWLEVEL_RUNTIME_SYNC_COMMAND),
    "onnxruntime": ("onnxruntime", _LOWLEVEL_RUNTIME_SYNC_COMMAND),
    "pkg_resources": ("setuptools<81", _LOWLEVEL_RUNTIME_SYNC_COMMAND),
    "serial": ("pyserial", _LOWLEVEL_RUNTIME_SYNC_COMMAND),
}


def run_with_friendly_lowlevel_errors(callback: Callable[[], None]) -> None:
    try:
        callback()
    except GamepadInputError as error:
        raise SystemExit(str(error)) from error
    except ModuleNotFoundError as error:
        dependency_name = error.name
        if dependency_name not in _MISSING_DEPENDENCY_HINTS:
            raise

        package_name, install_command = _MISSING_DEPENDENCY_HINTS[dependency_name]
        raise SystemExit(
            f"缺少 lowlevel 运行时依赖 `{package_name}`。"
            f"请先执行 `{install_command}`，然后再重试。"
        ) from error


def run_with_friendly_gamepad_errors(callback: Callable[[], None]) -> None:
    run_with_friendly_lowlevel_errors(callback)
