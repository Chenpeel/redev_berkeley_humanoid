from __future__ import annotations

import argparse
import errno
from collections.abc import Callable

from berkeley_humanoid_lite_lowlevel.robot.command_source import GamepadInputError
from berkeley_humanoid_lite_lowlevel.robot.locomotion_specification import (
    DEFAULT_LEFT_LEG_BUS,
    DEFAULT_RIGHT_LEG_BUS,
)

_LOWLEVEL_RUNTIME_SYNC_COMMAND = "uv sync --all-packages --group dev --group lowlevel-runtime"
_MISSING_DEPENDENCY_HINTS = {
    "can": ("python-can", _LOWLEVEL_RUNTIME_SYNC_COMMAND),
    "inputs": ("inputs", _LOWLEVEL_RUNTIME_SYNC_COMMAND),
    "onnxruntime": ("onnxruntime", _LOWLEVEL_RUNTIME_SYNC_COMMAND),
    "pkg_resources": ("setuptools<81", _LOWLEVEL_RUNTIME_SYNC_COMMAND),
    "serial": ("pyserial", _LOWLEVEL_RUNTIME_SYNC_COMMAND),
}


def add_leg_bus_arguments(parser: argparse.ArgumentParser) -> None:
    parser.add_argument(
        "--left-leg-bus",
        type=str,
        default=DEFAULT_LEFT_LEG_BUS,
        help="CAN bus name for the left leg",
    )
    parser.add_argument(
        "--right-leg-bus",
        type=str,
        default=DEFAULT_RIGHT_LEG_BUS,
        help="CAN bus name for the right leg",
    )


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
    except OSError as error:
        if error.errno != errno.ENODEV:
            raise

        raise SystemExit(
            "未找到可用的 CAN 设备。"
            "请先执行 `bash apps/lowlevel/start_can_transports.sh` 拉起对应的 can 接口"
            "（例如 can0-can3），"
            "然后再重试。"
        ) from error


def run_with_friendly_gamepad_errors(callback: Callable[[], None]) -> None:
    run_with_friendly_lowlevel_errors(callback)
