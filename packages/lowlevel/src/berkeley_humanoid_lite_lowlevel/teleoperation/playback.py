from __future__ import annotations

import json
from collections.abc import Mapping, Sequence
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np

IK_RESULT_SCHEMA = "offline_teleoperation_ik_results/v1"

BIMANUAL_SOLVER_JOINT_NAMES = (
    "left_shoulder_pitch_joint",
    "left_shoulder_roll_joint",
    "left_shoulder_yaw_joint",
    "left_elbow_joint",
    "left_wrist_yaw_joint",
    "right_shoulder_pitch_joint",
    "right_shoulder_roll_joint",
    "right_shoulder_yaw_joint",
    "right_elbow_joint",
    "right_wrist_yaw_joint",
)

IK_RESULT_TO_BIMANUAL_JOINT_NAME = {
    "arm_left_shoulder_pitch_joint": "left_shoulder_pitch_joint",
    "arm_left_shoulder_roll_joint": "left_shoulder_roll_joint",
    "arm_left_shoulder_yaw_joint": "left_shoulder_yaw_joint",
    "arm_left_elbow_pitch_joint": "left_elbow_joint",
    "arm_left_elbow_roll_joint": "left_wrist_yaw_joint",
    "arm_right_shoulder_pitch_joint": "right_shoulder_pitch_joint",
    "arm_right_shoulder_roll_joint": "right_shoulder_roll_joint",
    "arm_right_shoulder_yaw_joint": "right_shoulder_yaw_joint",
    "arm_right_elbow_pitch_joint": "right_elbow_joint",
    "arm_right_elbow_roll_joint": "right_wrist_yaw_joint",
}

_BIMANUAL_JOINT_INDEX = {
    joint_name: index for index, joint_name in enumerate(BIMANUAL_SOLVER_JOINT_NAMES)
}


@dataclass
class TeleoperationPlaybackDeltaBridge:
    ik_start_positions: np.ndarray
    real_start_positions: np.ndarray
    previous_targets: np.ndarray
    bridge_scale: np.ndarray
    max_delta_radians: float | None = None
    max_step_radians: float | None = None

    def compute_target_positions(
        self,
        ik_joint_positions: Sequence[float] | np.ndarray,
    ) -> tuple[np.ndarray, np.ndarray]:
        joint_positions = np.asarray(ik_joint_positions, dtype=np.float32)
        if joint_positions.shape != self.ik_start_positions.shape:
            raise ValueError("ik_joint_positions 的长度必须与桥接关节数一致。")

        joint_delta = (joint_positions - self.ik_start_positions) * self.bridge_scale
        if self.max_delta_radians is not None:
            joint_delta = np.clip(
                joint_delta,
                -self.max_delta_radians,
                self.max_delta_radians,
            )

        unclipped_targets = self.real_start_positions + joint_delta
        if self.max_step_radians is None:
            targets = unclipped_targets
        else:
            step_delta = np.clip(
                unclipped_targets - self.previous_targets,
                -self.max_step_radians,
                self.max_step_radians,
            )
            targets = self.previous_targets + step_delta

        self.previous_targets[:] = targets
        return targets.astype(np.float32, copy=True), joint_delta.astype(np.float32, copy=True)


def load_ik_result_payload(path: str | Path) -> dict[str, Any]:
    payload_path = Path(path)
    payload = json.loads(payload_path.read_text(encoding="utf-8"))
    if payload.get("schema") != IK_RESULT_SCHEMA:
        raise ValueError(
            f"Unsupported IK result schema {payload.get('schema')!r}. "
            f"Expected {IK_RESULT_SCHEMA!r}."
        )
    return payload


def list_ik_result_cases(payload: Mapping[str, Any]) -> list[tuple[int, str, str]]:
    cases = payload.get("cases")
    if not isinstance(cases, list):
        raise ValueError("IK result payload is missing a valid cases list.")

    listed_cases: list[tuple[int, str, str]] = []
    for index, case in enumerate(cases):
        pattern = str(case.get("pattern", ""))
        sweep_arm = str(case.get("sweep_arm", "auto"))
        listed_cases.append((index, pattern, sweep_arm))
    return listed_cases


def resolve_ik_result_case(
    payload: Mapping[str, Any],
    *,
    pattern: str | None = None,
    sweep_arm: str = "auto",
    case_index: int | None = None,
) -> dict[str, Any]:
    cases = payload.get("cases")
    if not isinstance(cases, list) or not cases:
        raise ValueError("IK result payload does not contain any cases.")

    if case_index is not None:
        if not 0 <= case_index < len(cases):
            raise ValueError(f"case_index {case_index} 超出范围，当前共有 {len(cases)} 个 case。")
        return dict(cases[case_index])

    if pattern is None:
        if len(cases) == 1:
            return dict(cases[0])
        raise ValueError("IK result payload contains multiple cases; please specify a pattern or case index.")

    matched_cases = [
        dict(case)
        for case in cases
        if str(case.get("pattern")) == pattern and str(case.get("sweep_arm", "auto")) == sweep_arm
    ]
    if len(matched_cases) == 1:
        return matched_cases[0]
    if not matched_cases:
        raise ValueError(f"未找到 pattern={pattern!r}, sweep_arm={sweep_arm!r} 对应的 case。")
    raise ValueError(f"存在多个 pattern={pattern!r}, sweep_arm={sweep_arm!r} 的 case。")


def map_ik_result_joint_targets_to_bimanual(
    joint_targets: Mapping[str, float],
) -> np.ndarray:
    mapped_targets = np.zeros((len(BIMANUAL_SOLVER_JOINT_NAMES),), dtype=np.float32)
    missing_joint_names: list[str] = []
    for ik_joint_name, bimanual_joint_name in IK_RESULT_TO_BIMANUAL_JOINT_NAME.items():
        if ik_joint_name not in joint_targets:
            missing_joint_names.append(ik_joint_name)
            continue
        mapped_targets[_BIMANUAL_JOINT_INDEX[bimanual_joint_name]] = float(joint_targets[ik_joint_name])

    if missing_joint_names:
        joined_names = ", ".join(missing_joint_names)
        raise ValueError(f"IK result sample is missing these joints: {joined_names}")

    return mapped_targets


def build_bimanual_action(
    *,
    solver_joint_targets: Sequence[float] | np.ndarray,
    joint_axis_directions: Sequence[float] | np.ndarray,
    gripper_targets: Mapping[str, float] | None = None,
) -> np.ndarray:
    solver_targets = np.asarray(solver_joint_targets, dtype=np.float32)
    axis_directions = np.asarray(joint_axis_directions, dtype=np.float32)
    if solver_targets.shape != (10,):
        raise ValueError("solver_joint_targets 必须是长度为 10 的向量。")
    if axis_directions.shape[0] < 12:
        raise ValueError("joint_axis_directions 必须至少包含 12 个元素。")

    actions = np.zeros((12,), dtype=np.float32)
    actions[:10] = solver_targets * axis_directions[:10]
    if gripper_targets is not None:
        actions[10] = float(gripper_targets.get("left", 0.5))
        actions[11] = float(gripper_targets.get("right", 0.5))
    return actions


def create_teleoperation_playback_delta_bridge(
    *,
    ik_joint_positions: Sequence[float] | np.ndarray,
    real_joint_positions: Sequence[float] | np.ndarray,
    bridge_scale: float | Sequence[float] = 1.0,
    max_delta_radians: float | None = None,
    max_step_radians: float | None = None,
) -> TeleoperationPlaybackDeltaBridge:
    ik_positions = np.asarray(ik_joint_positions, dtype=np.float32)
    real_positions = np.asarray(real_joint_positions, dtype=np.float32)
    if ik_positions.shape != (10,):
        raise ValueError("ik_joint_positions 必须是长度为 10 的向量。")
    if real_positions.shape != (10,):
        raise ValueError("real_joint_positions 必须是长度为 10 的向量。")

    scale_array = np.asarray(bridge_scale, dtype=np.float32)
    if scale_array.ndim == 0:
        if not np.isfinite(scale_array.item()):
            raise ValueError("bridge_scale must be finite")
        scale_array = np.full((10,), float(scale_array.item()), dtype=np.float32)
    elif scale_array.shape != (10,):
        raise ValueError("bridge_scale 的长度必须与双臂关节数一致。")
    elif not np.all(np.isfinite(scale_array)):
        raise ValueError("bridge_scale must contain only finite values")

    if max_delta_radians is not None and max_delta_radians <= 0.0:
        raise ValueError("max_delta_radians must be positive when provided")
    if max_step_radians is not None and max_step_radians <= 0.0:
        raise ValueError("max_step_radians must be positive when provided")

    return TeleoperationPlaybackDeltaBridge(
        ik_start_positions=ik_positions.astype(np.float32, copy=True),
        real_start_positions=real_positions.astype(np.float32, copy=True),
        previous_targets=real_positions.astype(np.float32, copy=True),
        bridge_scale=scale_array.astype(np.float32, copy=True),
        max_delta_radians=max_delta_radians,
        max_step_radians=max_step_radians,
    )
