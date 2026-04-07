# Copyright (c) 2025, The Berkeley Humanoid Lite Project Developers.

from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np
from berkeley_humanoid_lite_lowlevel.teleoperation import (
    SUPPORTED_FAKE_TELEOP_PATTERNS,
    TeleoperationIkSolver,
    build_fake_bridge_data,
)
from berkeley_humanoid_lite_lowlevel.workflows.teleoperation import (
    build_minimal_teleoperation_observations,
    update_offline_joint_observations,
)


def _resolve_joint_names(solver: TeleoperationIkSolver) -> list[str]:
    model = solver.robot.model
    joint_names: list[str] = []
    for q_index in range(7, 17):
        joint_name = None
        for joint_index, candidate_name in enumerate(model.names):
            nq = model.nqs[joint_index]
            idx_q = model.idx_qs[joint_index]
            if nq and idx_q <= q_index < idx_q + nq:
                joint_name = candidate_name
                break
        if joint_name is None:
            raise ValueError(f"Unable to resolve teleoperation joint name for q index {q_index}.")
        joint_names.append(joint_name)
    return joint_names


def _build_case_results(
    *,
    pattern: str,
    sweep_arm: str,
    frequency: float,
    duration: float,
    measurement_alpha: float,
    with_gripper: bool,
    left_enable: bool,
    right_enable: bool,
) -> dict[str, object]:
    solver = TeleoperationIkSolver(enable_visualizer=False)
    observations = build_minimal_teleoperation_observations(solver.robot.model.nq)
    joint_names = _resolve_joint_names(solver)
    step_count = max(1, int(round(duration * frequency)))
    step_duration = 1.0 / frequency
    samples: list[dict[str, object]] = []
    joint_series: dict[str, list[float]] = {name: [] for name in joint_names}

    for step in range(step_count):
        timestamp = step * step_duration
        bridge_data = build_fake_bridge_data(
            timestamp,
            pattern=pattern,
            left_enable=left_enable,
            right_enable=right_enable,
            with_gripper=with_gripper,
            sweep_arm=sweep_arm,
        )
        solver.update_controller(bridge_data)
        joint_targets, gripper_targets = solver.update(observations)
        measured_targets = update_offline_joint_observations(
            observations,
            joint_targets,
            alpha=measurement_alpha,
        )

        target_mapping = {
            joint_name: float(joint_targets[index])
            for index, joint_name in enumerate(joint_names)
        }
        measured_mapping = {
            joint_name: float(measured_targets[index])
            for index, joint_name in enumerate(joint_names)
        }
        for joint_name, value in target_mapping.items():
            joint_series[joint_name].append(value)

        samples.append(
            {
                "step": step,
                "timestamp": timestamp,
                "joint_targets": target_mapping,
                "joint_measurements": measured_mapping,
                "gripper_targets": {
                    "left": float(gripper_targets[0]),
                    "right": float(gripper_targets[1]),
                },
            }
        )

    joint_summary = {
        joint_name: {
            "min": float(np.min(values)),
            "max": float(np.max(values)),
            "mean": float(np.mean(values)),
            "final": float(values[-1]),
        }
        for joint_name, values in joint_series.items()
    }

    return {
        "pattern": pattern,
        "sweep_arm": sweep_arm,
        "step_count": step_count,
        "frequency": frequency,
        "duration": duration,
        "measurement_alpha": measurement_alpha,
        "joint_names": joint_names,
        "joint_summary": joint_summary,
        "samples": samples,
    }


def main() -> None:
    parser = argparse.ArgumentParser(description="Export offline teleoperation IK results to a JSON file")
    parser.add_argument(
        "--patterns",
        nargs="+",
        choices=SUPPORTED_FAKE_TELEOP_PATTERNS,
        default=list(SUPPORTED_FAKE_TELEOP_PATTERNS),
        help="Fake teleoperation patterns to export",
    )
    parser.add_argument("--frequency", type=float, default=20.0, help="Sampling frequency in Hz")
    parser.add_argument("--duration", type=float, default=2.0, help="Duration in seconds for each pattern")
    parser.add_argument(
        "--measurement-alpha",
        type=float,
        default=1.0,
        help="Observation write-back factor in (0, 1]",
    )
    parser.add_argument(
        "--with-gripper",
        action="store_true",
        help="Enable synthetic gripper waveform export",
    )
    parser.add_argument(
        "--left-enable",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="Enable left arm clutch input",
    )
    parser.add_argument(
        "--right-enable",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="Enable right arm clutch input",
    )
    parser.add_argument(
        "--output",
        type=Path,
        required=True,
        help="Output JSON path",
    )
    arguments = parser.parse_args()

    if arguments.frequency <= 0.0:
        raise ValueError("--frequency must be positive")
    if arguments.duration <= 0.0:
        raise ValueError("--duration must be positive")
    if not 0.0 < arguments.measurement_alpha <= 1.0:
        raise ValueError("--measurement-alpha must be in (0, 1]")

    cases: list[dict[str, object]] = []
    for pattern in arguments.patterns:
        if pattern == "single_arm_sweep":
            cases.append(
                _build_case_results(
                    pattern=pattern,
                    sweep_arm="left",
                    frequency=arguments.frequency,
                    duration=arguments.duration,
                    measurement_alpha=arguments.measurement_alpha,
                    with_gripper=arguments.with_gripper,
                    left_enable=arguments.left_enable,
                    right_enable=arguments.right_enable,
                )
            )
            cases.append(
                _build_case_results(
                    pattern=pattern,
                    sweep_arm="right",
                    frequency=arguments.frequency,
                    duration=arguments.duration,
                    measurement_alpha=arguments.measurement_alpha,
                    with_gripper=arguments.with_gripper,
                    left_enable=arguments.left_enable,
                    right_enable=arguments.right_enable,
                )
            )
            continue

        cases.append(
            _build_case_results(
                pattern=pattern,
                sweep_arm="auto",
                frequency=arguments.frequency,
                duration=arguments.duration,
                measurement_alpha=arguments.measurement_alpha,
                with_gripper=arguments.with_gripper,
                left_enable=arguments.left_enable,
                right_enable=arguments.right_enable,
            )
        )

    payload = {
        "schema": "offline_teleoperation_ik_results/v1",
        "frequency": arguments.frequency,
        "duration": arguments.duration,
        "measurement_alpha": arguments.measurement_alpha,
        "with_gripper": arguments.with_gripper,
        "left_enable": arguments.left_enable,
        "right_enable": arguments.right_enable,
        "cases": cases,
    }

    arguments.output.parent.mkdir(parents=True, exist_ok=True)
    arguments.output.write_text(
        json.dumps(payload, indent=2),
        encoding="utf-8",
    )
    print(arguments.output)


if __name__ == "__main__":
    main()
