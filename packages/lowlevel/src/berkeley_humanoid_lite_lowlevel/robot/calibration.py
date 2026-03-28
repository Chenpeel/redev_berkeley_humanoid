from __future__ import annotations

import time
from collections import deque
from dataclasses import dataclass
from datetime import UTC, datetime
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


@dataclass(frozen=True)
class CalibrationCaptureResult:
    position_offsets: np.ndarray
    averaged_readings: np.ndarray
    stddev_readings: np.ndarray
    reference_delta_deg: np.ndarray
    stddev_deg: np.ndarray
    sample_count: int
    worst_reference_joint_name: str
    max_abs_reference_delta_deg: float
    least_stable_joint_name: str
    max_stddev_deg: float

    def build_metadata(self, specification: LocomotionRobotSpecification) -> dict[str, object]:
        return {
            "schema_version": 2,
            "joint_names": list(specification.joint_names),
            "reference_positions": [float(value) for value in specification.calibration_reference_positions],
            "capture": {
                "sample_count": int(self.sample_count),
                "max_abs_reference_delta_deg": float(self.max_abs_reference_delta_deg),
                "max_stddev_deg": float(self.max_stddev_deg),
                "worst_reference_joint": self.worst_reference_joint_name,
                "least_stable_joint": self.least_stable_joint_name,
                "created_at": datetime.now(UTC).isoformat(),
            },
        }


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

    def save_position_offsets(
        self,
        position_offsets: np.ndarray,
        *,
        metadata: dict[str, object] | None = None,
    ) -> Path:
        output_path = ensure_parent_directory(self.calibration_path)
        config = {"position_offsets": [float(offset) for offset in position_offsets]}
        if metadata is not None:
            config.update(metadata)
        OmegaConf.save(
            config=OmegaConf.create(config),
            f=output_path,
        )
        return output_path


def update_limit_readings(
    limit_readings: np.ndarray,
    joint_readings: np.ndarray,
    calibration_limit_selectors: tuple[str, ...] | np.ndarray,
) -> np.ndarray:
    """
    Track the signed reading chosen for each joint's calibration reference pose.

    The historical selector names are ``min`` / ``max``, but here they do not
    mean "drive the joint to its farthest mechanical hard stop". They only
    describe which signed reading should be retained while the operator moves
    the robot into the predefined calibration pose.
    """
    if limit_readings.shape != joint_readings.shape:
        raise ValueError("标定参考位姿读数和当前读数的形状必须一致。")

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
    # 标定的本质是把“校准参考位姿下读到的原始关节值”映射回 spec 定义的参考角。
    # 因此这里的 reference_positions 是 calibration reference pose，不是机械极限位。
    if limit_readings.shape != reference_positions.shape:
        raise ValueError("标定参考位姿读数和参考位姿的形状必须一致。")
    return (limit_readings - reference_positions).astype(np.float32, copy=False)


def _ensure_initial_position_measurements(actuator_array: ActuatorArray) -> None:
    position_measurements_complete = getattr(actuator_array, "position_measurements_complete", None)
    if position_measurements_complete is None:
        measurements_ready = getattr(actuator_array, "measurements_ready", None)
        if measurements_ready is False:
            raise RuntimeError("Failed to read any initial joint measurements before calibration capture.")
        return

    if bool(position_measurements_complete):
        return

    missing_names = getattr(actuator_array, "missing_position_measurement_names", ())
    if callable(missing_names):
        missing_names = missing_names()
    if not missing_names:
        missing_names = ("unknown joints",)
    missing_text = ", ".join(str(name) for name in missing_names)
    raise RuntimeError(
        "Failed to read all initial joint measurements before calibration capture. "
        f"Missing joints: {missing_text}."
    )


def _build_capture_result(
    specification: LocomotionRobotSpecification,
    sample_window: deque[np.ndarray],
) -> CalibrationCaptureResult:
    sample_matrix = np.stack(tuple(sample_window), axis=0).astype(np.float32, copy=False)
    averaged_readings = sample_matrix.mean(axis=0)
    stddev_readings = sample_matrix.std(axis=0)
    reference_delta_deg = np.rad2deg(averaged_readings - specification.calibration_reference_positions).astype(
        np.float32,
        copy=False,
    )
    stddev_deg = np.rad2deg(stddev_readings).astype(np.float32, copy=False)
    if specification.joint_count > 0:
        worst_reference_index = int(np.argmax(np.abs(reference_delta_deg)))
        worst_reference_joint_name = specification.joint_names[worst_reference_index]
        least_stable_index = int(np.argmax(stddev_deg))
        least_stable_joint_name = specification.joint_names[least_stable_index]
    else:
        worst_reference_joint_name = ""
        least_stable_joint_name = ""

    return CalibrationCaptureResult(
        position_offsets=compute_position_offsets(
            averaged_readings,
            specification.calibration_reference_positions,
        ),
        averaged_readings=averaged_readings.astype(np.float32, copy=False),
        stddev_readings=stddev_readings.astype(np.float32, copy=False),
        reference_delta_deg=reference_delta_deg,
        stddev_deg=stddev_deg,
        sample_count=int(sample_matrix.shape[0]),
        worst_reference_joint_name=worst_reference_joint_name,
        max_abs_reference_delta_deg=float(np.max(np.abs(reference_delta_deg))) if reference_delta_deg.size else 0.0,
        least_stable_joint_name=least_stable_joint_name,
        max_stddev_deg=float(np.max(stddev_deg)) if stddev_deg.size else 0.0,
    )


def _validate_capture_result(
    result: CalibrationCaptureResult,
    *,
    required_sample_count: int,
    max_stddev_deg: float,
) -> None:
    if result.sample_count < required_sample_count:
        raise RuntimeError(
            "Calibration capture ended before the stable sample window filled. "
            f"Need {required_sample_count} samples, got {result.sample_count}."
        )
    if result.max_stddev_deg > max_stddev_deg:
        raise RuntimeError(
            "Calibration pose is not stable enough to save. "
            f"least_stable_joint={result.least_stable_joint_name} "
            f"max_stddev_deg={result.max_stddev_deg:.2f} "
            f"limit={max_stddev_deg:.2f}"
        )


def capture_calibration_result(
    specification: LocomotionRobotSpecification,
    actuator_array: ActuatorArray,
    command_source: CommandSource,
    polling_interval_seconds: float = 0.05,
    capture_window_size: int = 20,
    max_stddev_deg: float = 1.0,
) -> CalibrationCaptureResult:
    # 这个流程记录的是“校准参考位姿”下的原始读数。
    # 操作者应该把机器人摆到期望的 calibration reference pose，
    # 而不是反复去找每个关节能到达的最大机械行程。
    if capture_window_size <= 0:
        raise ValueError("capture_window_size 必须为正数。")

    initial_readings = actuator_array.read_positions()
    _ensure_initial_position_measurements(actuator_array)
    sample_window: deque[np.ndarray] = deque(maxlen=int(capture_window_size))
    sample_window.append(np.asarray(initial_readings, dtype=np.float32).copy())

    print("Calibration reference pose guide:")
    for joint_name, reference_position, selector in zip(
        specification.joint_names,
        specification.calibration_reference_positions,
        specification.calibration_limit_selectors,
        strict=True,
    ):
        print(
            f"  {joint_name}: target={np.rad2deg(reference_position):+.2f} deg, "
            f"selector={selector}"
        )
    print("Selector note: min/max only selects the signed reading to retain for that reference pose.")
    print("Do not search for the farthest mechanical limit.")
    print("Switch the gamepad back to IDLE only after the robot is holding the intended calibration pose.")
    print(
        "Calibration save gate:",
        f"samples={capture_window_size}",
        f"max_stddev_deg<={max_stddev_deg:.2f}",
    )
    print("Note: large candidate offsets are expected before calibration and do not imply a bad pose by themselves.")

    print("initial readings before moving to calibration reference poses:")
    print([f"{reading:.2f}" for reading in initial_readings])

    while (
        getattr(command_source.snapshot(), "requested_state", LocomotionControlState.INVALID)
        != LocomotionControlState.IDLE
    ):
        joint_readings = actuator_array.read_positions()
        sample_window.append(np.asarray(joint_readings, dtype=np.float32).copy())
        capture_result = _build_capture_result(
            specification,
            sample_window,
        )
        print(
            f"{time.time():.3f}",
            f"samples={capture_result.sample_count}/{capture_window_size}",
            f"worst_reference_joint={capture_result.worst_reference_joint_name}",
            f"max_abs_reference_delta_deg={capture_result.max_abs_reference_delta_deg:.2f}",
            f"least_stable_joint={capture_result.least_stable_joint_name}",
            f"max_stddev_deg={capture_result.max_stddev_deg:.2f}",
        )
        print("candidate offset delta[deg]:", [f"{value:+.2f}" for value in capture_result.reference_delta_deg])
        time.sleep(polling_interval_seconds)

    capture_result = _build_capture_result(
        specification,
        sample_window,
    )
    _validate_capture_result(
        capture_result,
        required_sample_count=int(capture_window_size),
        max_stddev_deg=float(max_stddev_deg),
    )

    print("final averaged readings captured for calibration reference poses:")
    print([f"{reading:.4f}" for reading in capture_result.averaged_readings])
    print("final stddev[deg]:")
    print([f"{value:.4f}" for value in capture_result.stddev_deg])
    print("final candidate offset delta[deg]:")
    print([f"{value:+.4f}" for value in capture_result.reference_delta_deg])

    print("offsets:")
    print([f"{offset:.4f}" for offset in capture_result.position_offsets])

    return capture_result


def capture_calibration_offsets(
    specification: LocomotionRobotSpecification,
    actuator_array: ActuatorArray,
    command_source: CommandSource,
    polling_interval_seconds: float = 0.05,
    capture_window_size: int = 20,
    max_stddev_deg: float = 1.0,
) -> np.ndarray:
    return capture_calibration_result(
        specification,
        actuator_array,
        command_source,
        polling_interval_seconds=polling_interval_seconds,
        capture_window_size=capture_window_size,
        max_stddev_deg=max_stddev_deg,
    ).position_offsets
