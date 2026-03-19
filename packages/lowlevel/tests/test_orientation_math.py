from __future__ import annotations

import unittest

import numpy as np
import torch
from berkeley_humanoid_lite_lowlevel.robot.orientation_math import (
    compute_projected_gravity,
    create_gravity_vector,
    quat_rotate_inverse,
)


class OrientationMathTestCase(unittest.TestCase):
    def test_create_gravity_vector_matches_numpy_default(self) -> None:
        gravity_vector = create_gravity_vector()

        np.testing.assert_allclose(gravity_vector, np.array([0.0, 0.0, -1.0], dtype=np.float32))

    def test_create_gravity_vector_matches_torch_reference_type(self) -> None:
        reference = torch.ones(4, dtype=torch.float32)

        gravity_vector = create_gravity_vector(reference=reference)

        self.assertIsInstance(gravity_vector, torch.Tensor)
        self.assertTrue(torch.equal(gravity_vector, torch.tensor([0.0, 0.0, -1.0], dtype=torch.float32)))

    def test_quat_rotate_inverse_supports_numpy_inputs(self) -> None:
        quaternion = np.array([0.0, 0.0, 0.0, 1.0], dtype=np.float32)
        vector = np.array([1.0, 2.0, 3.0], dtype=np.float32)

        rotated = quat_rotate_inverse(quaternion, vector)

        np.testing.assert_allclose(rotated, np.array([-1.0, -2.0, 3.0], dtype=np.float32))

    def test_quat_rotate_inverse_supports_torch_inputs(self) -> None:
        quaternion = torch.tensor([0.0, 0.0, 0.0, 1.0], dtype=torch.float32)
        vector = torch.tensor([1.0, 2.0, 3.0], dtype=torch.float32)

        rotated = quat_rotate_inverse(quaternion, vector)

        self.assertIsInstance(rotated, torch.Tensor)
        self.assertTrue(torch.allclose(rotated, torch.tensor([-1.0, -2.0, 3.0], dtype=torch.float32)))

    def test_compute_projected_gravity_uses_default_gravity_vector(self) -> None:
        quaternion = np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float32)

        projected_gravity = compute_projected_gravity(quaternion)

        np.testing.assert_allclose(projected_gravity, np.array([0.0, 0.0, -1.0], dtype=np.float32))


if __name__ == "__main__":
    unittest.main()
