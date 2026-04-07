# Copyright (c) 2025, The Berkeley Humanoid Lite Project Developers.

from __future__ import annotations

import argparse
import math
from pathlib import Path

from berkeley_humanoid_lite_lowlevel.teleoperation.playback import (
    list_ik_result_cases,
    load_ik_result_payload,
)
from berkeley_humanoid_lite_lowlevel.workflows import run_teleoperation_ik_result_playback


def main() -> None:
    parser = argparse.ArgumentParser(description="Play exported teleoperation IK results through the bimanual robot")
    parser.add_argument(
        "--input",
        type=Path,
        required=True,
        help="Path to an offline teleoperation IK result JSON file",
    )
    parser.add_argument(
        "--pattern",
        type=str,
        default=None,
        help="Pattern name to play, for example static_pose or dual_arm_circle",
    )
    parser.add_argument(
        "--sweep-arm",
        choices=("auto", "left", "right"),
        default="auto",
        help="Sweep-arm selector used when resolving single_arm_sweep cases",
    )
    parser.add_argument(
        "--case-index",
        type=int,
        default=None,
        help="Play a case by numeric index instead of pattern/sweep-arm",
    )
    parser.add_argument(
        "--list-cases",
        action="store_true",
        help="Print the available cases in the IK result file and exit",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Preview playback targets without opening robot buses or sending motor commands",
    )
    parser.add_argument(
        "--delta-bridge",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="Replay samples using real-start-relative delta bridging instead of absolute targets",
    )
    parser.add_argument(
        "--scale",
        "--bridge-scale",
        dest="bridge_scale",
        type=float,
        default=1.0,
        help="Scaling factor applied to IK joint deltas in delta-bridge mode",
    )
    parser.add_argument(
        "--max-delta-deg",
        type=float,
        default=30.0,
        help="Clamp each bridged joint delta to +/- this many degrees in delta-bridge mode",
    )
    parser.add_argument(
        "--max-step-deg",
        type=float,
        default=3.0,
        help="Clamp each playback step update to +/- this many degrees in delta-bridge mode",
    )
    parser.add_argument(
        "--position-kp",
        type=float,
        default=20.0,
        help="Position proportional gain written to the robot before playback starts",
    )
    parser.add_argument(
        "--position-kd",
        type=float,
        default=2.0,
        help="Position derivative gain written to the robot before playback starts",
    )
    parser.add_argument(
        "--torque-limit",
        type=float,
        default=2.0,
        help="Torque limit written to the robot before playback starts",
    )
    parser.add_argument(
        "--print-every",
        type=int,
        default=10,
        help="Print one playback status line every N samples",
    )
    arguments = parser.parse_args()

    if arguments.list_cases:
        payload = load_ik_result_payload(arguments.input)
        for index, pattern, sweep_arm in list_ik_result_cases(payload):
            print(f"{index}: pattern={pattern}, sweep_arm={sweep_arm}")
        return

    run_teleoperation_ik_result_playback(
        input_path=str(arguments.input),
        pattern=arguments.pattern,
        sweep_arm=arguments.sweep_arm,
        case_index=arguments.case_index,
        dry_run=arguments.dry_run,
        delta_bridge=arguments.delta_bridge,
        bridge_scale=arguments.bridge_scale,
        max_delta_radians=math.radians(arguments.max_delta_deg),
        max_step_radians=math.radians(arguments.max_step_deg),
        position_kp=arguments.position_kp,
        position_kd=arguments.position_kd,
        torque_limit=arguments.torque_limit,
        print_every=arguments.print_every,
    )


if __name__ == "__main__":
    main()
