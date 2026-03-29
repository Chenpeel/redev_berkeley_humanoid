from __future__ import annotations

import tempfile
import unittest
from pathlib import Path

import numpy as np

from berkeley_humanoid_lite_lowlevel.robot.pose_alignment import (
    PoseAlignmentStore,
    apply_pose_alignment_bias,
    compute_pose_alignment_bias,
    remove_pose_alignment_bias,
)


class PoseAlignmentTests(unittest.TestCase):
    def test_compute_pose_alignment_bias_matches_reference_minus_taught(self) -> None:
        reference = np.array([0.0, -0.2, 0.4], dtype=np.float32)
        taught = np.array([0.1, -0.1, 0.25], dtype=np.float32)

        bias = compute_pose_alignment_bias(reference, taught)

        np.testing.assert_allclose(bias, np.array([-0.1, -0.1, 0.15], dtype=np.float32))

    def test_apply_and_remove_pose_alignment_bias_are_inverse(self) -> None:
        hardware_positions = np.array([0.1, -0.3, 0.2], dtype=np.float32)
        bias = np.array([0.05, -0.02, 0.1], dtype=np.float32)

        policy_positions = apply_pose_alignment_bias(hardware_positions, bias)

        np.testing.assert_allclose(remove_pose_alignment_bias(policy_positions, bias), hardware_positions)

    def test_pose_alignment_store_round_trip(self) -> None:
        with tempfile.TemporaryDirectory() as temporary_directory:
            store_path = Path(temporary_directory) / "locomotion_pose_alignment.yaml"
            store = PoseAlignmentStore(store_path)
            expected = np.array([0.01, -0.02, 0.03], dtype=np.float32)

            store.save_pose_alignment_bias(expected, metadata={"schema_version": 1})

            loaded = store.load_pose_alignment_bias(3)

        np.testing.assert_allclose(loaded, expected)

    def test_pose_alignment_store_returns_zeros_when_missing(self) -> None:
        with tempfile.TemporaryDirectory() as temporary_directory:
            store = PoseAlignmentStore(Path(temporary_directory) / "missing.yaml")
            loaded = store.load_pose_alignment_bias(4)

        np.testing.assert_allclose(loaded, np.zeros((4,), dtype=np.float32))


if __name__ == "__main__":
    unittest.main()
