from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .control_state import LocomotionControlState


def linear_interpolate(start: np.ndarray, end: np.ndarray, percentage: float) -> np.ndarray:
    percentage = min(max(percentage, 0.0), 1.0)
    return start * (1.0 - percentage) + end * percentage


@dataclass(frozen=True)
class LocomotionCycleContext:
    state: LocomotionControlState
    requested_state: LocomotionControlState
    initialization_progress: float
    initialization_step: float
    starting_positions: np.ndarray
    measured_positions: np.ndarray
    policy_actions: np.ndarray
    initialization_positions: np.ndarray
    restart_initialization: bool = False


@dataclass(frozen=True)
class LocomotionCycleResult:
    state: LocomotionControlState
    initialization_progress: float
    starting_positions: np.ndarray
    joint_position_target: np.ndarray
    enter_position_mode: bool
    enter_damping_mode: bool
    messages: tuple[str, ...]


def advance_locomotion_cycle(context: LocomotionCycleContext) -> LocomotionCycleResult:
    measured_positions = np.asarray(context.measured_positions, dtype=np.float32).copy()
    starting_positions = np.asarray(context.starting_positions, dtype=np.float32).copy()
    policy_actions = np.asarray(context.policy_actions, dtype=np.float32).copy()
    initialization_positions = np.asarray(context.initialization_positions, dtype=np.float32).copy()

    if context.restart_initialization:
        return LocomotionCycleResult(
            state=LocomotionControlState.INITIALIZING,
            initialization_progress=0.0,
            starting_positions=measured_positions.copy(),
            joint_position_target=measured_positions.copy(),
            enter_position_mode=False,
            enter_damping_mode=False,
            messages=("Restarting initialization towards requested pose",),
        )

    if context.state == LocomotionControlState.IDLE:
        if context.requested_state in (
            LocomotionControlState.INITIALIZING,
            LocomotionControlState.POLICY_CONTROL,
        ):
            return LocomotionCycleResult(
                state=LocomotionControlState.INITIALIZING,
                initialization_progress=0.0,
                starting_positions=measured_positions.copy(),
                joint_position_target=measured_positions.copy(),
                enter_position_mode=True,
                enter_damping_mode=False,
                messages=("Switching to initialization mode",),
            )

        return LocomotionCycleResult(
            state=LocomotionControlState.IDLE,
            initialization_progress=context.initialization_progress,
            starting_positions=starting_positions,
            joint_position_target=measured_positions.copy(),
            enter_position_mode=False,
            enter_damping_mode=False,
            messages=(),
        )

    if context.state == LocomotionControlState.INITIALIZING:
        if context.initialization_progress < 1.0:
            next_progress = min(context.initialization_progress + context.initialization_step, 1.0)
            return LocomotionCycleResult(
                state=LocomotionControlState.INITIALIZING,
                initialization_progress=next_progress,
                starting_positions=starting_positions,
                joint_position_target=linear_interpolate(
                    starting_positions,
                    initialization_positions,
                    next_progress,
                ),
                enter_position_mode=False,
                enter_damping_mode=False,
                messages=(f"init: {context.initialization_progress:.2f}",),
            )

        if context.requested_state == LocomotionControlState.POLICY_CONTROL:
            return LocomotionCycleResult(
                state=LocomotionControlState.POLICY_CONTROL,
                initialization_progress=context.initialization_progress,
                starting_positions=starting_positions,
                joint_position_target=initialization_positions.copy(),
                enter_position_mode=False,
                enter_damping_mode=False,
                messages=("Switching to policy control mode",),
            )

        if context.requested_state == LocomotionControlState.IDLE:
            return LocomotionCycleResult(
                state=LocomotionControlState.IDLE,
                initialization_progress=context.initialization_progress,
                starting_positions=starting_positions,
                joint_position_target=initialization_positions.copy(),
                enter_position_mode=False,
                enter_damping_mode=True,
                messages=("Switching to idle mode",),
            )

        return LocomotionCycleResult(
            state=LocomotionControlState.INITIALIZING,
            initialization_progress=context.initialization_progress,
            starting_positions=starting_positions,
            joint_position_target=initialization_positions.copy(),
            enter_position_mode=False,
            enter_damping_mode=False,
            messages=(),
        )

    if context.state == LocomotionControlState.POLICY_CONTROL:
        if context.requested_state == LocomotionControlState.IDLE:
            return LocomotionCycleResult(
                state=LocomotionControlState.IDLE,
                initialization_progress=context.initialization_progress,
                starting_positions=starting_positions,
                joint_position_target=policy_actions.copy(),
                enter_position_mode=False,
                enter_damping_mode=True,
                messages=("Switching to idle mode",),
            )

        return LocomotionCycleResult(
            state=LocomotionControlState.POLICY_CONTROL,
            initialization_progress=context.initialization_progress,
            starting_positions=starting_positions,
            joint_position_target=policy_actions.copy(),
            enter_position_mode=False,
            enter_damping_mode=False,
            messages=(),
        )

    return LocomotionCycleResult(
        state=LocomotionControlState.IDLE,
        initialization_progress=context.initialization_progress,
        starting_positions=starting_positions,
        joint_position_target=measured_positions.copy(),
        enter_position_mode=False,
        enter_damping_mode=False,
        messages=(),
    )
