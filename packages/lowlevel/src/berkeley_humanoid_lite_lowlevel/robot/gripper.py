from __future__ import annotations

import struct

import serial


GRIPPER_OPEN_RAW_VALUE = 0.2
GRIPPER_CLOSED_RAW_RANGE = 0.6
GRIPPER_COMMAND_CODE = 0x0C


def normalize_gripper_target(target: float) -> float:
    return max(0.0, min(1.0, float(target)))


def normalized_gripper_target_to_raw_value(target: float) -> float:
    return GRIPPER_OPEN_RAW_VALUE + normalize_gripper_target(target) * GRIPPER_CLOSED_RAW_RANGE


class SerialGripper:
    def __init__(self, port: str = "/dev/ttyUSB0", baudrate: int = 115200) -> None:
        self.port = port
        self.baudrate = baudrate
        self._serial = serial.Serial(self.port, self.baudrate)

    def write_targets(self, left_target: float, right_target: float) -> None:
        data = struct.pack(
            "<ffb",
            normalized_gripper_target_to_raw_value(left_target),
            normalized_gripper_target_to_raw_value(right_target),
            GRIPPER_COMMAND_CODE,
        )
        self._serial.write(data)

    def readline(self) -> bytes:
        return self._serial.readline()

    def close(self) -> None:
        self._serial.close()
