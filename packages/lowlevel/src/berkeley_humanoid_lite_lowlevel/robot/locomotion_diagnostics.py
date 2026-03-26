from __future__ import annotations

from dataclasses import dataclass

import numpy as np

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
    delta_to_initialization: np.ndarray
    raw_delta_to_initialization: np.ndarray
    dry_run: bool
    risk_joint_name: str
    risk_raw_delta_to_initialization: float


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
    initialization_positions = np.asarray(specification.initialization_positions, dtype=np.float32)
    velocity_command = np.asarray(command_velocity, dtype=np.float32)

    raw_targets = (targets + offsets) * directions
    raw_measured = (measured + offsets) * directions
    delta_to_initialization = initialization_positions - measured
    raw_initialization_targets = (initialization_positions + offsets) * directions
    raw_delta_to_initialization = raw_initialization_targets - raw_measured

    if specification.joint_count > 0:
        risk_index = int(np.argmax(np.abs(raw_delta_to_initialization)))
        risk_joint_name = specification.joint_names[risk_index]
        risk_raw_delta = float(raw_delta_to_initialization[risk_index])
    else:
        risk_joint_name = ""
        risk_raw_delta = 0.0

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
        delta_to_initialization=delta_to_initialization,
        raw_delta_to_initialization=raw_delta_to_initialization,
        dry_run=dry_run,
        risk_joint_name=risk_joint_name,
        risk_raw_delta_to_initialization=risk_raw_delta,
    )
