from __future__ import annotations

from dataclasses import dataclass
from glob import glob
import math
import os
import struct
import time
from typing import BinaryIO

import serial


_SYNC_1 = b"\x75"
_SYNC_2 = b"\x65"
_EXPECTED_FLOAT_COUNT = 7
_EXPECTED_PAYLOAD_SIZE = _EXPECTED_FLOAT_COUNT * 4
AUTO_DETECT_SERIAL_DEVICE = "auto"
_PREFERRED_DEVICE_CANDIDATES = (
    "/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0",
    "/dev/serial/by-path/pci-0000:00:14.0-usb-0:4.1:1.0",
    "/dev/serial/by-path/pci-0000:00:14.0-usb-0:4:1.0",
)
_DEVICE_GLOB_PATTERNS = (
    "/dev/serial/by-id/*",
    "/dev/serial/by-path/*",
    "/dev/ttyUSB*",
    "/dev/ttyACM*",
)


@dataclass(frozen=True)
class OrientationSample:
    """一次姿态串流采样。"""

    quaternion_wxyz: tuple[float, float, float, float]
    angular_velocity_xyz: tuple[float, float, float]
    timestamp: float

    def to_euler_degrees(self) -> tuple[float, float, float]:
        """将四元数转换为 XYZ 欧拉角。"""
        w, x, y, z = self.quaternion_wxyz

        sin_roll = 2.0 * (w * x + y * z)
        cos_roll = 1.0 - 2.0 * (x * x + y * y)
        roll = math.atan2(sin_roll, cos_roll)

        sin_pitch = 2.0 * (w * y - z * x)
        if abs(sin_pitch) >= 1.0:
            pitch = math.copysign(math.pi / 2.0, sin_pitch)
        else:
            pitch = math.asin(sin_pitch)

        sin_yaw = 2.0 * (w * z + x * y)
        cos_yaw = 1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(sin_yaw, cos_yaw)

        return tuple(math.degrees(angle) for angle in (roll, pitch, yaw))


def _discover_orientation_devices() -> list[str]:
    """按优先级收集当前可用的 IMU 串口路径。"""
    devices: list[str] = []
    seen_targets: set[str] = set()

    for candidate in _PREFERRED_DEVICE_CANDIDATES:
        if not os.path.exists(candidate):
            continue

        resolved = os.path.realpath(candidate)
        if resolved in seen_targets:
            continue

        seen_targets.add(resolved)
        devices.append(candidate)

    for pattern in _DEVICE_GLOB_PATTERNS:
        for candidate in sorted(glob(pattern)):
            resolved = os.path.realpath(candidate)
            if resolved in seen_targets:
                continue

            seen_targets.add(resolved)
            devices.append(candidate)

    return devices


def discover_orientation_devices() -> list[str]:
    """返回当前可用的 IMU 串口候选列表。"""
    return _discover_orientation_devices()


def resolve_orientation_device(device: str | None) -> str:
    """解析 IMU 串口路径，支持自动探测。"""
    if device and device != AUTO_DETECT_SERIAL_DEVICE:
        return device

    devices = discover_orientation_devices()
    if len(devices) == 1:
        return devices[0]

    if not devices:
        searched_patterns = ", ".join(_DEVICE_GLOB_PATTERNS)
        raise serial.SerialException(
            "No IMU serial device found. "
            f"Checked common paths and patterns: {searched_patterns}. "
            "Pass --device to specify the serial port explicitly."
        )

    available_devices = ", ".join(devices)
    raise serial.SerialException(
        "Multiple serial devices found. "
        f"Pass --device to select one explicitly: {available_devices}"
    )


def read_orientation_sample(
    stream: BinaryIO,
    *,
    timestamp: float | None = None,
) -> OrientationSample | None:
    """从字节流读取一帧姿态数据。"""
    sync_1 = stream.read(1)
    if sync_1 != _SYNC_1:
        return None

    sync_2 = stream.read(1)
    if sync_2 != _SYNC_2:
        return None

    payload_size_bytes = stream.read(2)
    if len(payload_size_bytes) != 2:
        return None

    payload_size = int.from_bytes(payload_size_bytes, byteorder="little", signed=False)
    payload = stream.read(payload_size)
    if len(payload) != payload_size or payload_size != _EXPECTED_PAYLOAD_SIZE:
        return None

    sample_values = struct.unpack("<7f", payload)
    quaternion = tuple(float(value) for value in sample_values[:4])
    angular_velocity = tuple(float(value) for value in sample_values[4:])
    return OrientationSample(
        quaternion_wxyz=quaternion,
        angular_velocity_xyz=angular_velocity,
        timestamp=time.perf_counter() if timestamp is None else timestamp,
    )


class SerialOrientationStream:
    """串口姿态串流读取器。"""

    def __init__(self, device: str | None, *, baudrate: int, timeout: float) -> None:
        self.device = resolve_orientation_device(device)
        self.baudrate = baudrate
        self.timeout = timeout
        self._serial = serial.Serial(self.device, baudrate, timeout=timeout)

    def read_sample(self) -> OrientationSample | None:
        """读取一帧姿态样本。"""
        return read_orientation_sample(self._serial)

    def close(self) -> None:
        """关闭串口。"""
        self._serial.close()

    def __enter__(self) -> SerialOrientationStream:
        return self

    def __exit__(self, exc_type: object, exc: object, exc_tb: object) -> None:
        self.close()
