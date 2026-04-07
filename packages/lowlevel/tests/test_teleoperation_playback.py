from __future__ import annotations

import tempfile
import unittest
from pathlib import Path

import numpy as np
from berkeley_humanoid_lite_lowlevel.teleoperation.playback import (
    BIMANUAL_SOLVER_JOINT_NAMES,
    build_bimanual_action,
    create_teleoperation_playback_delta_bridge,
    list_ik_result_cases,
    load_ik_result_payload,
    map_ik_result_joint_targets_to_bimanual,
    resolve_ik_result_case,
)


def _build_payload() -> dict[str, object]:
    return {
        "schema": "offline_teleoperation_ik_results/v1",
        "cases": [
            {
                "pattern": "static_pose",
                "sweep_arm": "auto",
                "frequency": 20.0,
                "samples": [
                    {
                        "step": 0,
                        "timestamp": 0.0,
                        "joint_targets": {
                            "arm_left_shoulder_pitch_joint": 0.1,
                            "arm_left_shoulder_roll_joint": 0.2,
                            "arm_left_shoulder_yaw_joint": 0.3,
                            "arm_left_elbow_pitch_joint": 0.4,
                            "arm_left_elbow_roll_joint": 0.5,
                            "arm_right_shoulder_pitch_joint": 0.6,
                            "arm_right_shoulder_roll_joint": 0.7,
                            "arm_right_shoulder_yaw_joint": 0.8,
                            "arm_right_elbow_pitch_joint": 0.9,
                            "arm_right_elbow_roll_joint": 1.0,
                        },
                        "gripper_targets": {
                            "left": 0.25,
                            "right": 0.75,
                        },
                    }
                ],
            },
            {
                "pattern": "single_arm_sweep",
                "sweep_arm": "left",
                "frequency": 20.0,
                "samples": [],
            },
        ],
    }


class TeleoperationPlaybackTests(unittest.TestCase):
    def test_list_ik_result_cases_returns_index_pattern_and_sweep_arm(self) -> None:
        payload = _build_payload()

        listed_cases = list_ik_result_cases(payload)

        self.assertEqual(
            listed_cases,
            [
                (0, "static_pose", "auto"),
                (1, "single_arm_sweep", "left"),
            ],
        )

    def test_load_ik_result_payload_validates_schema(self) -> None:
        payload_path = Path(tempfile.mkdtemp()) / "ik.json"
        payload_path.write_text('{"schema":"offline_teleoperation_ik_results/v1","cases":[]}', encoding="utf-8")

        payload = load_ik_result_payload(payload_path)

        self.assertEqual(payload["schema"], "offline_teleoperation_ik_results/v1")

    def test_resolve_ik_result_case_supports_pattern_and_index(self) -> None:
        payload = _build_payload()

        by_pattern = resolve_ik_result_case(payload, pattern="static_pose", sweep_arm="auto")
        by_index = resolve_ik_result_case(payload, case_index=1)

        self.assertEqual(by_pattern["pattern"], "static_pose")
        self.assertEqual(by_index["sweep_arm"], "left")

    def test_map_ik_result_joint_targets_to_bimanual_uses_explicit_order(self) -> None:
        payload = _build_payload()
        sample = payload["cases"][0]["samples"][0]

        mapped_targets = map_ik_result_joint_targets_to_bimanual(sample["joint_targets"])

        np.testing.assert_allclose(
            mapped_targets,
            np.array([0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0], dtype=np.float32),
        )
        self.assertEqual(
            BIMANUAL_SOLVER_JOINT_NAMES,
            (
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
            ),
        )

    def test_build_bimanual_action_applies_axis_directions_and_grippers(self) -> None:
        action = build_bimanual_action(
            solver_joint_targets=np.array([0.1] * 10, dtype=np.float32),
            joint_axis_directions=np.array([1, -1, 1, -1, 1, -1, 1, -1, 1, -1, 1, 1], dtype=np.float32),
            gripper_targets={"left": 0.2, "right": 0.8},
        )

        np.testing.assert_allclose(
            action,
            np.array([0.1, -0.1, 0.1, -0.1, 0.1, -0.1, 0.1, -0.1, 0.1, -0.1, 0.2, 0.8], dtype=np.float32),
        )

    def test_delta_bridge_replays_relative_to_robot_start(self) -> None:
        bridge = create_teleoperation_playback_delta_bridge(
            ik_joint_positions=np.array([0.2, -0.4, 0.1, 0.0, 0.3, 0.2, -0.1, 0.4, 0.0, 0.5], dtype=np.float32),
            real_joint_positions=np.array([1.0] * 10, dtype=np.float32),
            bridge_scale=1.5,
            max_delta_radians=0.5,
            max_step_radians=0.2,
        )

        targets, delta = bridge.compute_target_positions(
            np.array([0.4, -0.2, 0.3, 0.1, 0.5, 0.0, -0.3, 0.2, -0.1, 0.7], dtype=np.float32),
        )

        np.testing.assert_allclose(
            delta,
            np.array([0.3, 0.3, 0.3, 0.15, 0.3, -0.3, -0.3, -0.3, -0.15, 0.3], dtype=np.float32),
        )
        np.testing.assert_allclose(
            targets,
            np.array([1.2, 1.2, 1.2, 1.15, 1.2, 0.8, 0.8, 0.8, 0.85, 1.2], dtype=np.float32),
        )


if __name__ == "__main__":
    unittest.main()
