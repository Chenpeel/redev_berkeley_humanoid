from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from ..sensors.orientation import OrientationSample
from .control_state import LocomotionControlState
from .locomotion_specification import LocomotionRobotSpecification


@dataclass(frozen=True)
class LocomotionDiagnosticSnapshot:
    state: LocomotionControlState
    requested_state: LocomotionControlState
    command_velocity: np.ndarray
    actions: np.ndarray
    targets: np.ndarray
    measured: np.ndarray
    offsets: np.ndarray
    raw_targets: np.ndarray
    raw_measured: np.ndarray
    delta_to_standing: np.ndarray
    raw_delta_to_standing: np.ndarray
    delta_to_initialization: np.ndarray
    raw_delta_to_initialization: np.ndarray
    dry_run: bool
    risk_joint_name: str
    risk_raw_delta_to_initialization: float
    standing_risk_joint_name: str
    risk_raw_delta_to_standing: float


def _build_raw_pose_delta(
    target_positions: np.ndarray,
    measured: np.ndarray,
    offsets: np.ndarray,
    directions: np.ndarray,
) -> np.ndarray:
    raw_targets = (target_positions + offsets) * directions
    raw_measured = (measured + offsets) * directions
    return raw_targets - raw_measured


def _find_risk_joint_name(
    specification: LocomotionRobotSpecification,
    raw_delta: np.ndarray,
) -> tuple[str, float]:
    if specification.joint_count == 0:
        return "", 0.0

    risk_index = int(np.argmax(np.abs(raw_delta)))
    return specification.joint_names[risk_index], float(raw_delta[risk_index])


def format_imu_debug_line(
    quaternion_wxyz: np.ndarray,
    angular_velocity_deg_s: np.ndarray,
) -> str:
    quaternion = np.asarray(quaternion_wxyz, dtype=np.float32)
    angular_velocity = np.asarray(angular_velocity_deg_s, dtype=np.float32)
    sample = OrientationSample(
        quaternion_wxyz=tuple(float(value) for value in quaternion),
        angular_velocity_xyz=tuple(float(value) for value in angular_velocity),
        timestamp=0.0,
    )
    roll, pitch, yaw = sample.to_euler_degrees()
    return (
        "IMU attitude[deg]: "
        f"roll={roll:+7.2f} "
        f"pitch={pitch:+7.2f} "
        f"yaw={yaw:+7.2f} | "
        "gyro[deg/s]: "
        f"x={angular_velocity[0]:+7.2f} "
        f"y={angular_velocity[1]:+7.2f} "
        f"z={angular_velocity[2]:+7.2f} | "
        "quat[wxyz]: "
        f"[{quaternion[0]:+.4f}, {quaternion[1]:+.4f}, {quaternion[2]:+.4f}, {quaternion[3]:+.4f}]"
    )


def build_locomotion_diagnostic_snapshot(
    *,
    specification: LocomotionRobotSpecification,
    state: LocomotionControlState,
    requested_state: LocomotionControlState,
    command_velocity: np.ndarray,
    actions: np.ndarray,
    joint_position_target: np.ndarray,
    joint_position_measured: np.ndarray,
    position_offsets: np.ndarray,
    joint_axis_directions: np.ndarray,
    dry_run: bool,
) -> LocomotionDiagnosticSnapshot:
    actions_array = np.asarray(actions, dtype=np.float32)
    targets = np.asarray(joint_position_target, dtype=np.float32)
    measured = np.asarray(joint_position_measured, dtype=np.float32)
    offsets = np.asarray(position_offsets, dtype=np.float32)
    directions = np.asarray(joint_axis_directions, dtype=np.float32)
    standing_positions = np.asarray(specification.standing_positions, dtype=np.float32)
    initialization_positions = np.asarray(specification.initialization_positions, dtype=np.float32)
    velocity_command = np.asarray(command_velocity, dtype=np.float32)

    raw_targets = (targets + offsets) * directions
    raw_measured = (measured + offsets) * directions
    delta_to_standing = standing_positions - measured
    raw_delta_to_standing = _build_raw_pose_delta(
        standing_positions,
        measured,
        offsets,
        directions,
    )
    delta_to_initialization = initialization_positions - measured
    raw_delta_to_initialization = _build_raw_pose_delta(
        initialization_positions,
        measured,
        offsets,
        directions,
    )
    risk_joint_name, risk_raw_delta = _find_risk_joint_name(
        specification,
        raw_delta_to_initialization,
    )
    standing_risk_joint_name, standing_risk_raw_delta = _find_risk_joint_name(
        specification,
        raw_delta_to_standing,
    )

    return LocomotionDiagnosticSnapshot(
        state=state,
        requested_state=requested_state,
        command_velocity=velocity_command,
        actions=actions_array,
        targets=targets,
        measured=measured,
        offsets=offsets,
        raw_targets=raw_targets,
        raw_measured=raw_measured,
        delta_to_standing=delta_to_standing,
        raw_delta_to_standing=raw_delta_to_standing,
        delta_to_initialization=delta_to_initialization,
        raw_delta_to_initialization=raw_delta_to_initialization,
        dry_run=dry_run,
        risk_joint_name=risk_joint_name,
        risk_raw_delta_to_initialization=risk_raw_delta,
        standing_risk_joint_name=standing_risk_joint_name,
        risk_raw_delta_to_standing=standing_risk_raw_delta,
    )
