from __future__ import annotations

import unittest
from types import SimpleNamespace

import numpy as np
from berkeley_humanoid_lite.workflows.sim2real import (
    build_quest_teleoperation_visualizer_observation,
    resolve_visualizer_default_joint_positions,
    resolve_visualizer_joint_names,
)


class QuestWebXrVisualizerHelpersTestCase(unittest.TestCase):
    def test_resolve_visualizer_joint_names_uses_configuration_when_lengths_match(self) -> None:
        configuration = SimpleNamespace(
            num_joints=3,
            joints=["joint_a", "joint_b", "joint_c"],
        )

        self.assertEqual(
            resolve_visualizer_joint_names(configuration),
            ("joint_a", "joint_b", "joint_c"),
        )

    def test_resolve_visualizer_default_joint_positions_falls_back_to_variant_defaults(self) -> None:
        configuration = SimpleNamespace(
            num_joints=22,
            joints=[
                "arm_left_shoulder_pitch_joint",
                "arm_left_shoulder_roll_joint",
                "arm_left_shoulder_yaw_joint",
                "arm_left_elbow_pitch_joint",
                "arm_left_elbow_roll_joint",
                "arm_right_shoulder_pitch_joint",
                "arm_right_shoulder_roll_joint",
                "arm_right_shoulder_yaw_joint",
                "arm_right_elbow_pitch_joint",
                "arm_right_elbow_roll_joint",
                "leg_left_hip_roll_joint",
                "leg_left_hip_yaw_joint",
                "leg_left_hip_pitch_joint",
                "leg_left_knee_pitch_joint",
                "leg_left_ankle_pitch_joint",
                "leg_left_ankle_roll_joint",
                "leg_right_hip_roll_joint",
                "leg_right_hip_yaw_joint",
                "leg_right_hip_pitch_joint",
                "leg_right_knee_pitch_joint",
                "leg_right_ankle_pitch_joint",
                "leg_right_ankle_roll_joint",
            ],
            default_joint_positions=[0.0] * 24,
        )

        default_positions = resolve_visualizer_default_joint_positions(configuration)

        self.assertEqual(default_positions.shape, (22,))
        np.testing.assert_allclose(
            default_positions[10:22],
            np.array([0.0, 0.0, -0.2, 0.4, -0.3, 0.0, 0.0, 0.0, -0.2, 0.4, -0.3, 0.0], dtype=np.float32),
        )

    def test_build_quest_teleoperation_visualizer_observation_injects_arm_targets(self) -> None:
        configuration = SimpleNamespace(
            num_joints=22,
            joints=[
                "arm_left_shoulder_pitch_joint",
                "arm_left_shoulder_roll_joint",
                "arm_left_shoulder_yaw_joint",
                "arm_left_elbow_pitch_joint",
                "arm_left_elbow_roll_joint",
                "arm_right_shoulder_pitch_joint",
                "arm_right_shoulder_roll_joint",
                "arm_right_shoulder_yaw_joint",
                "arm_right_elbow_pitch_joint",
                "arm_right_elbow_roll_joint",
                "leg_left_hip_roll_joint",
                "leg_left_hip_yaw_joint",
                "leg_left_hip_pitch_joint",
                "leg_left_knee_pitch_joint",
                "leg_left_ankle_pitch_joint",
                "leg_left_ankle_roll_joint",
                "leg_right_hip_roll_joint",
                "leg_right_hip_yaw_joint",
                "leg_right_hip_pitch_joint",
                "leg_right_knee_pitch_joint",
                "leg_right_ankle_pitch_joint",
                "leg_right_ankle_roll_joint",
            ],
            default_joint_positions=[
                *([0.0] * 10),
                0.0,
                0.0,
                -0.2,
                0.4,
                -0.3,
                0.0,
                0.0,
                0.0,
                -0.2,
                0.4,
                -0.3,
                0.0,
            ],
        )
        arm_joint_positions = np.linspace(-0.45, 0.45, 10, dtype=np.float32)

        observation, next_positions = build_quest_teleoperation_visualizer_observation(
            configuration,
            arm_joint_positions=arm_joint_positions,
            previous_joint_positions=np.zeros((22,), dtype=np.float32),
            dt=0.05,
        )

        np.testing.assert_allclose(observation[0:4], np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float32))
        np.testing.assert_allclose(observation[7:17], arm_joint_positions)
        np.testing.assert_allclose(next_positions[0:10], arm_joint_positions)
        np.testing.assert_allclose(next_positions[10:22], np.asarray(configuration.default_joint_positions[10:22]))
        np.testing.assert_allclose(observation[29:39], arm_joint_positions / 0.05)


if __name__ == "__main__":
    unittest.main()
