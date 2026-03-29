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
    get_pose_alignment_path,
)

from .control_state import LocomotionControlState
from .locomotion_specification import LocomotionRobotSpecification


class CommandSource(Protocol):
    def snapshot(self) -> object:
        ...


class ActuatorArray(Protocol):
    measurements_ready: bool
    joint_position_measured: np.ndarray

    def read_positions(self) -> np.ndarray:
        ...


@dataclass(frozen=True)
class PoseAlignmentCaptureResult:
    pose_alignment_bias: np.ndarray
    taught_positions: np.ndarray
    stddev_readings: np.ndarray
    stddev_deg: np.ndarray
    sample_count: int
    least_stable_joint_name: str
    max_stddev_deg: float

    def build_metadata(
        self,
        specification: LocomotionRobotSpecification,
        *,
        reference_positions: np.ndarray,
    ) -> dict[str, object]:
        return {
            "schema_version": 1,
            "joint_names": list(specification.joint_names),
            "reference_positions": [float(value) for value in np.asarray(reference_positions, dtype=np.float32)],
            "capture": {
                "sample_count": int(self.sample_count),
                "max_stddev_deg": float(self.max_stddev_deg),
                "least_stable_joint": self.least_stable_joint_name,
                "created_at": datetime.now(UTC).isoformat(),
            },
        }


class PoseAlignmentStore:
    def __init__(self, pose_alignment_path: str | Path | None = None) -> None:
        self.pose_alignment_path = (
            get_pose_alignment_path()
            if pose_alignment_path is None
            else Path(pose_alignment_path)
        )

    def load_pose_alignment_bias(self, joint_count: int) -> np.ndarray:
        if not self.pose_alignment_path.exists():
            return np.zeros((joint_count,), dtype=np.float32)

        raw_config = OmegaConf.load(self.pose_alignment_path)
        pose_alignment_bias = np.asarray(
            raw_config.get("pose_alignment_bias", []),
            dtype=np.float32,
        )
        if pose_alignment_bias.shape != (joint_count,):
            raise ValueError("pose alignment 文件中的 pose_alignment_bias 长度与关节数不一致。")
        return pose_alignment_bias

    def save_pose_alignment_bias(
        self,
        pose_alignment_bias: np.ndarray,
        *,
        metadata: dict[str, object] | None = None,
    ) -> Path:
        output_path = ensure_parent_directory(self.pose_alignment_path)
        config = {
            "pose_alignment_bias": [
                float(offset) for offset in np.asarray(pose_alignment_bias, dtype=np.float32)
            ]
        }
        if metadata is not None:
            config.update(metadata)
        OmegaConf.save(
            config=OmegaConf.create(config),
            f=output_path,
        )
        return output_path


def compute_pose_alignment_bias(
    reference_positions: np.ndarray,
    taught_positions: np.ndarray,
) -> np.ndarray:
    reference = np.asarray(reference_positions, dtype=np.float32)
    taught = np.asarray(taught_positions, dtype=np.float32)
    if reference.shape != taught.shape:
        raise ValueError("reference_positions 与 taught_positions 的形状必须一致。")
    return (reference - taught).astype(np.float32, copy=False)


def apply_pose_alignment_bias(
    hardware_positions: np.ndarray,
    pose_alignment_bias: np.ndarray,
) -> np.ndarray:
    hardware = np.asarray(hardware_positions, dtype=np.float32)
    bias = np.asarray(pose_alignment_bias, dtype=np.float32)
    if hardware.shape != bias.shape:
        raise ValueError("hardware_positions 与 pose_alignment_bias 的形状必须一致。")
    return (hardware + bias).astype(np.float32, copy=False)


def remove_pose_alignment_bias(
    policy_positions: np.ndarray,
    pose_alignment_bias: np.ndarray,
) -> np.ndarray:
    policy = np.asarray(policy_positions, dtype=np.float32)
    bias = np.asarray(pose_alignment_bias, dtype=np.float32)
    if policy.shape != bias.shape:
        raise ValueError("policy_positions 与 pose_alignment_bias 的形状必须一致。")
    return (policy - bias).astype(np.float32, copy=False)


def _ensure_initial_position_measurements(actuator_array: ActuatorArray) -> None:
    measurements_ready = getattr(actuator_array, "measurements_ready", None)
    if measurements_ready is False:
        raise RuntimeError("Failed to read initial joint measurements before pose alignment capture.")


def _build_capture_result(
    specification: LocomotionRobotSpecification,
    *,
    reference_positions: np.ndarray,
    sample_window: deque[np.ndarray],
) -> PoseAlignmentCaptureResult:
    sample_matrix = np.stack(tuple(sample_window), axis=0).astype(np.float32, copy=False)
    taught_positions = sample_matrix.mean(axis=0)
    stddev_readings = sample_matrix.std(axis=0)
    stddev_deg = np.rad2deg(stddev_readings).astype(np.float32, copy=False)
    least_stable_index = int(np.argmax(stddev_deg)) if specification.joint_count > 0 else 0
    least_stable_joint_name = (
        specification.joint_names[least_stable_index]
        if specification.joint_count > 0
        else ""
    )
    return PoseAlignmentCaptureResult(
        pose_alignment_bias=compute_pose_alignment_bias(reference_positions, taught_positions),
        taught_positions=taught_positions.astype(np.float32, copy=False),
        stddev_readings=stddev_readings.astype(np.float32, copy=False),
        stddev_deg=stddev_deg,
        sample_count=int(sample_matrix.shape[0]),
        least_stable_joint_name=least_stable_joint_name,
        max_stddev_deg=float(np.max(stddev_deg)) if stddev_deg.size else 0.0,
    )


def capture_pose_alignment_result(
    specification: LocomotionRobotSpecification,
    actuator_array: ActuatorArray,
    command_source: CommandSource,
    *,
    reference_positions: np.ndarray,
    polling_interval_seconds: float = 0.05,
    capture_window_size: int = 20,
    max_stddev_deg: float = 1.0,
    capture_state: LocomotionControlState = LocomotionControlState.INITIALIZING,
) -> PoseAlignmentCaptureResult:
    if capture_window_size <= 0:
        raise ValueError("capture_window_size 必须为正数。")

    reference = np.asarray(reference_positions, dtype=np.float32)
    if reference.shape != (specification.joint_count,):
        raise ValueError("reference_positions 的长度必须与关节数一致。")

    initial_readings = np.asarray(actuator_array.read_positions(), dtype=np.float32)
    _ensure_initial_position_measurements(actuator_array)

    sample_window: deque[np.ndarray] = deque(maxlen=int(capture_window_size))
    sample_window.append(initial_readings.copy())

    print("Locomotion nominal pose teach guide:")
    print("  1. 先让机器人进入 damping。")
    print("  2. 手动把整机摆到你认为合理的 nominal / align 姿态。")
    print("  3. 按 A+LB 触发采集。")
    print(
        "Capture save gate:",
        f"samples={capture_window_size}",
        f"max_stddev_deg<={max_stddev_deg:.2f}",
    )
    print("Reference pose from policy default_joint_positions[rad]:")
    print([f"{value:+.4f}" for value in reference])

    while True:
        joint_readings = np.asarray(actuator_array.read_positions(), dtype=np.float32)
        sample_window.append(joint_readings.copy())
        capture_result = _build_capture_result(
            specification,
            reference_positions=reference,
            sample_window=sample_window,
        )
        capture_requested = (
            getattr(command_source.snapshot(), "requested_state", LocomotionControlState.INVALID)
            == capture_state
        )

        print(
            f"{time.time():.3f}",
            f"samples={capture_result.sample_count}/{capture_window_size}",
            f"least_stable_joint={capture_result.least_stable_joint_name}",
            f"max_stddev_deg={capture_result.max_stddev_deg:.2f}",
            f"capture_requested={int(capture_requested)}",
        )
        print("current taught pose[rad]:", [f"{value:+.4f}" for value in capture_result.taught_positions])
        print(
            "candidate pose_alignment_bias[deg]:",
            [f"{value:+.2f}" for value in np.rad2deg(capture_result.pose_alignment_bias)],
        )

        if (
            capture_requested
            and capture_result.sample_count >= capture_window_size
            and capture_result.max_stddev_deg <= max_stddev_deg
        ):
            break

        time.sleep(polling_interval_seconds)

    print("final taught pose[rad]:")
    print([f"{value:+.4f}" for value in capture_result.taught_positions])
    print("final stddev[deg]:")
    print([f"{value:.4f}" for value in capture_result.stddev_deg])
    print("final pose_alignment_bias[deg]:")
    print([f"{value:+.4f}" for value in np.rad2deg(capture_result.pose_alignment_bias)])
    print("preview pose_alignment_bias[rad]:")
    print([f"{value:+.4f}" for value in capture_result.pose_alignment_bias])
    print("preview pose_alignment_bias[deg]:")
    print([f"{value:+.4f}" for value in np.rad2deg(capture_result.pose_alignment_bias)])

    return capture_result
