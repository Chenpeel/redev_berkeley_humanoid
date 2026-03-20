from __future__ import annotations

import glob
import os
import re
import sys
import threading
from dataclasses import dataclass

try:
    from inputs import GamePad, UnpluggedError, devices, get_gamepad
except ImportError as import_error:  # pragma: no cover - 依赖缺失时在真实运行环境暴露
    GamePad = None
    UnpluggedError = None
    devices = None
    get_gamepad = None
    _GAMEPAD_IMPORT_ERROR = import_error
else:  # pragma: no cover - 仅在真实硬件环境下执行
    _GAMEPAD_IMPORT_ERROR = None

from .control_state import LocomotionControlState


class XInputCode:
    AXIS_X_L = "ABS_X"
    AXIS_Y_L = "ABS_Y"
    AXIS_X_R = "ABS_RX"

    BTN_A = "BTN_SOUTH"
    BTN_X = "BTN_NORTH"
    BTN_BUMPER_L = "BTN_TL"
    BTN_BUMPER_R = "BTN_TR"
    BTN_THUMB_L = "BTN_THUMBL"
    BTN_THUMB_R = "BTN_THUMBR"


class GamepadInputError(RuntimeError):
    """手柄输入不可用时抛出的统一异常。"""


class GamepadDependencyError(GamepadInputError):
    """缺少手柄输入依赖。"""


class GamepadUnavailableError(GamepadInputError):
    """未检测到可用手柄，或运行中断开连接。"""


AXIS_CODES = (
    XInputCode.AXIS_X_L,
    XInputCode.AXIS_Y_L,
    XInputCode.AXIS_X_R,
)


def _build_dependency_error() -> GamepadDependencyError:
    return GamepadDependencyError(
        "缺少 `inputs` 依赖，无法读取手柄输入。请先安装依赖后再重试。"
    )


def _build_unavailable_error() -> GamepadUnavailableError:
    return GamepadUnavailableError(
        "未检测到可用手柄。请先连接手柄，并确认系统已经识别该设备后再重试。"
    )


def _is_linux() -> bool:
    return sys.platform.startswith("linux")


def _sanitize_device_identifier(name: str) -> str:
    identifier = re.sub(r"[^A-Za-z0-9._-]+", "_", name.strip()).strip("_")
    return identifier or "gamepad"


def _read_linux_gamepad_name(js_sysfs_path: str) -> str:
    candidate_paths = (
        os.path.join(js_sysfs_path, "device", "name"),
        os.path.join(js_sysfs_path, "device", "device", "name"),
    )
    for candidate_path in candidate_paths:
        try:
            with open(candidate_path, encoding="utf-8") as device_name_file:
                device_name = device_name_file.read().strip()
        except OSError:
            continue
        if device_name:
            return device_name
    return os.path.basename(js_sysfs_path)


def _find_linux_gamepad_event(js_sysfs_path: str) -> str | None:
    candidate_patterns = (
        os.path.join(js_sysfs_path, "device", "event*"),
        os.path.join(js_sysfs_path, "device", "input*", "event*"),
        os.path.join(js_sysfs_path, "device", "device", "event*"),
        os.path.join(js_sysfs_path, "device", "device", "input*", "event*"),
    )
    for candidate_pattern in candidate_patterns:
        event_paths = sorted(glob.glob(candidate_pattern))
        if event_paths:
            return event_paths[0]
    return None


def _build_fallback_device_path(device_name: str) -> str:
    identifier = _sanitize_device_identifier(device_name)
    return f"/dev/input/by-id/manual-{identifier}-event-joystick"


def _discover_linux_gamepads() -> list[object]:
    if not _is_linux() or devices is None or GamePad is None:
        return []

    known_char_names = {
        gamepad.get_char_name()
        for gamepad in getattr(devices, "gamepads", [])
        if hasattr(gamepad, "get_char_name")
    }
    discovered_gamepads: list[object] = []

    for js_sysfs_path in sorted(glob.glob("/sys/class/input/js*")):
        event_sysfs_path = _find_linux_gamepad_event(js_sysfs_path)
        if event_sysfs_path is None:
            continue

        event_name = os.path.basename(event_sysfs_path)
        if event_name in known_char_names:
            continue

        character_device_path = os.path.join("/dev/input", event_name)
        if not os.path.exists(character_device_path):
            continue

        device_name = _read_linux_gamepad_name(js_sysfs_path)
        # inputs 在 Linux 上只扫描 by-id/by-path，这里为 js/event 设备构造一个可解析的伪路径。
        fallback_device_path = _build_fallback_device_path(device_name)
        try:
            discovered_gamepads.append(
                GamePad(
                    devices,
                    fallback_device_path,
                    char_path_override=character_device_path,
                )
            )
        except Exception:  # pragma: no cover - 依赖真实设备环境触发
            continue
        known_char_names.add(event_name)

    return discovered_gamepads


def _register_discovered_gamepads() -> None:
    if devices is None:
        return

    discovered_gamepads = _discover_linux_gamepads()
    if not discovered_gamepads:
        return

    devices.gamepads.extend(discovered_gamepads)
    update_all_devices = getattr(devices, "_update_all_devices", None)
    if callable(update_all_devices):
        update_all_devices()
    elif hasattr(devices, "all_devices"):
        devices.all_devices.extend(discovered_gamepads)


def _normalize_axis_value(
    raw_value: int,
    *,
    mode: str | None,
    stick_sensitivity: float,
    dead_zone: float,
) -> float:
    if mode == "unsigned":
        value = ((32768.0 - raw_value) / 32768.0) * stick_sensitivity
    else:
        value = (raw_value / -32768.0) * stick_sensitivity

    if abs(value) < dead_zone:
        return 0.0
    return max(-1.0, min(1.0, value))


@dataclass(frozen=True)
class LocomotionCommand:
    requested_state: LocomotionControlState
    velocity_x: float
    velocity_y: float
    velocity_yaw: float

    @classmethod
    def zero(cls) -> LocomotionCommand:
        return cls(
            requested_state=LocomotionControlState.INVALID,
            velocity_x=0.0,
            velocity_y=0.0,
            velocity_yaw=0.0,
        )


def build_command_from_states(
    states: dict[str, int],
    *,
    stick_sensitivity: float = 1.0,
    dead_zone: float = 0.01,
    axis_modes: dict[str, str] | None = None,
) -> LocomotionCommand:
    def normalize(code: str) -> float:
        raw_value = states.get(code, 0)
        mode = None if axis_modes is None else axis_modes.get(code)
        if mode is None and raw_value > 32767:
            mode = "unsigned"
        return _normalize_axis_value(
            raw_value,
            mode=mode,
            stick_sensitivity=stick_sensitivity,
            dead_zone=dead_zone,
        )

    requested_state = LocomotionControlState.INVALID
    if states.get(XInputCode.BTN_A) and states.get(XInputCode.BTN_BUMPER_R):
        requested_state = LocomotionControlState.POLICY_CONTROL
    elif states.get(XInputCode.BTN_A) and states.get(XInputCode.BTN_BUMPER_L):
        requested_state = LocomotionControlState.INITIALIZING
    elif (
        states.get(XInputCode.BTN_X)
        or states.get(XInputCode.BTN_THUMB_L)
        or states.get(XInputCode.BTN_THUMB_R)
    ):
        requested_state = LocomotionControlState.IDLE

    return LocomotionCommand(
        requested_state=requested_state,
        velocity_x=normalize(XInputCode.AXIS_Y_L),
        velocity_y=normalize(XInputCode.AXIS_X_R),
        velocity_yaw=normalize(XInputCode.AXIS_X_L),
    )


class GamepadCommandSource:
    """将游戏手柄输入包装成稳定的命令读取接口。"""

    def __init__(
        self,
        *,
        stick_sensitivity: float = 1.0,
        dead_zone: float = 0.01,
    ) -> None:
        self.stick_sensitivity = stick_sensitivity
        self.dead_zone = dead_zone
        self._stopped = threading.Event()
        self._thread: threading.Thread | None = None
        self._failure: GamepadInputError | None = None
        self._states = self._create_initial_states()
        self._axis_modes: dict[str, str] = {}
        self.command = LocomotionCommand.zero()

    @staticmethod
    def _create_initial_states() -> dict[str, int]:
        return {
            XInputCode.AXIS_X_L: 0,
            XInputCode.AXIS_Y_L: 0,
            XInputCode.AXIS_X_R: 0,
            XInputCode.BTN_A: 0,
            XInputCode.BTN_X: 0,
            XInputCode.BTN_BUMPER_L: 0,
            XInputCode.BTN_BUMPER_R: 0,
            XInputCode.BTN_THUMB_L: 0,
            XInputCode.BTN_THUMB_R: 0,
        }

    def start(self) -> None:
        if self._thread is not None and self._thread.is_alive():
            self.raise_if_failed()
            return
        self._ensure_available()
        self._stopped.clear()
        self._failure = None
        self._thread = threading.Thread(
            target=self.run_forever,
            daemon=True,
            name="gamepad-command-source",
        )
        self._thread.start()

    def stop(self) -> None:
        self._stopped.set()

    def run_forever(self) -> None:
        while not self._stopped.is_set():
            try:
                self.advance()
            except GamepadInputError as error:
                self._failure = error
                self._stopped.set()
            except Exception as error:  # pragma: no cover - 只在真实设备异常时触发
                self._failure = GamepadInputError(f"读取手柄输入失败：{error}")
                self._stopped.set()

    def advance(self) -> None:
        if get_gamepad is None:  # pragma: no cover - 依赖缺失只在真实运行时触发
            raise _build_dependency_error() from _GAMEPAD_IMPORT_ERROR

        if devices is not None and not devices.gamepads:
            _register_discovered_gamepads()

        try:
            events = get_gamepad()
        except Exception as error:  # pragma: no cover - 依赖真实设备触发
            if UnpluggedError is not None and isinstance(error, UnpluggedError):
                raise _build_unavailable_error() from error
            raise

        for event in events:
            self._states[event.code] = event.state
            if event.code in AXIS_CODES:
                if event.state > 32767:
                    self._axis_modes[event.code] = "unsigned"
                elif event.state < 0:
                    self._axis_modes[event.code] = "signed"

        self.command = build_command_from_states(
            self._states,
            stick_sensitivity=self.stick_sensitivity,
            dead_zone=self.dead_zone,
            axis_modes=self._axis_modes,
        )

    def snapshot(self) -> LocomotionCommand:
        self.raise_if_failed()
        return self.command

    def raise_if_failed(self) -> None:
        if self._failure is not None:
            raise self._failure

    def _ensure_available(self) -> None:
        if get_gamepad is None:  # pragma: no cover - 依赖缺失只在真实运行时触发
            raise _build_dependency_error() from _GAMEPAD_IMPORT_ERROR

        if devices is None:  # pragma: no cover - 防御性分支
            raise _build_unavailable_error()

        if not devices.gamepads:
            _register_discovered_gamepads()

        if not devices.gamepads:
            raise _build_unavailable_error()
