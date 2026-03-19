from __future__ import annotations

import time
from pathlib import Path
from typing import Protocol

import numpy as np
from omegaconf import OmegaConf

from berkeley_humanoid_lite_lowlevel.runtime_paths import (
    ensure_parent_directory,
    get_calibration_path,
)

from .control_state import LocomotionControlState
from .locomotion_specification import LocomotionRobotSpecification


class CommandSource(Protocol):
    def snapshot(self) -> object:
        ...


class ActuatorArray(Protocol):
    def read_positions(self) -> np.ndarray:
        ...


class CalibrationStore:
    def __init__(self, calibration_path: str | Path | None = None) -> None:
        self.calibration_path = get_calibration_path() if calibration_path is None else Path(calibration_path)

    def load_position_offsets(self, joint_count: int) -> np.ndarray:
        if not self.calibration_path.exists():
            return np.zeros((joint_count,), dtype=np.float32)

        raw_config = OmegaConf.load(self.calibration_path)
        position_offsets = np.asarray(raw_config.get("position_offsets", []), dtype=np.float32)
        if position_offsets.shape != (joint_count,):
            raise ValueError("标定文件中的 position_offsets 长度与关节数不一致。")
        return position_offsets

    def save_position_offsets(self, position_offsets: np.ndarray) -> Path:
        output_path = ensure_parent_directory(self.calibration_path)
        OmegaConf.save(
            config=OmegaConf.create({"position_offsets": [float(offset) for offset in position_offsets]}),
            f=output_path,
        )
        return output_path


def update_limit_readings(
    limit_readings: np.ndarray,
    joint_readings: np.ndarray,
    calibration_limit_selectors: tuple[str, ...] | np.ndarray,
) -> np.ndarray:
    if limit_readings.shape != joint_readings.shape:
        raise ValueError("极限读数和当前读数的形状必须一致。")

    selector_array = np.asarray(calibration_limit_selectors)
    if selector_array.shape != limit_readings.shape:
        raise ValueError("搜索方向的长度必须与关节数一致。")

    updated_limit_readings = limit_readings.copy()
    minimum_mask = selector_array == "min"
    maximum_mask = selector_array == "max"
    updated_limit_readings[minimum_mask] = np.minimum(
        updated_limit_readings[minimum_mask],
        joint_readings[minimum_mask],
    )
    updated_limit_readings[maximum_mask] = np.maximum(
        updated_limit_readings[maximum_mask],
        joint_readings[maximum_mask],
    )
    return updated_limit_readings


def compute_position_offsets(
    limit_readings: np.ndarray,
    reference_positions: np.ndarray,
) -> np.ndarray:
    if limit_readings.shape != reference_positions.shape:
        raise ValueError("极限读数和参考位姿的形状必须一致。")
    return (limit_readings - reference_positions).astype(np.float32, copy=False)


def capture_calibration_offsets(
    specification: LocomotionRobotSpecification,
    actuator_array: ActuatorArray,
    command_source: CommandSource,
    polling_interval_seconds: float = 0.05,
) -> np.ndarray:
    limit_readings = actuator_array.read_positions()
    print("initial readings:")
    print([f"{reading:.2f}" for reading in limit_readings])

    while getattr(command_source.snapshot(), "requested_state", LocomotionControlState.INVALID) != LocomotionControlState.IDLE:
        joint_readings = actuator_array.read_positions()
        limit_readings = update_limit_readings(
            limit_readings,
            joint_readings,
            specification.calibration_limit_selectors,
        )
        print(time.time(), [f"{reading:.2f}" for reading in limit_readings])
        time.sleep(polling_interval_seconds)

    offsets = compute_position_offsets(
        limit_readings,
        specification.calibration_reference_positions,
    )

    print("final readings at the limits:")
    print([f"{limit_reading:.4f}" for limit_reading in limit_readings])

    print("offsets:")
    print([f"{offset:.4f}" for offset in offsets])

    return offsets
