from __future__ import annotations

import unittest

import numpy as np
from berkeley_humanoid_lite_lowlevel.robot.control_state import LocomotionControlState
from berkeley_humanoid_lite_lowlevel.robot.locomotion_diagnostics import (
    build_locomotion_diagnostic_snapshot,
    format_imu_debug_line,
)
from berkeley_humanoid_lite_lowlevel.robot.locomotion_specification import build_leg_locomotion_robot_specification


class LocomotionDiagnosticsTests(unittest.TestCase):
    def test_format_imu_debug_line_includes_euler_angles_gyro_and_quaternion(self) -> None:
        output = format_imu_debug_line(
            quaternion_wxyz=np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float32),
            angular_velocity_deg_s=np.array([12.5, -3.0, 1.25], dtype=np.float32),
        )

        self.assertIn("IMU attitude[deg]:", output)
        self.assertIn("roll=  +0.00", output)
        self.assertIn("pitch=  +0.00", output)
        self.assertIn("yaw=  +0.00", output)
        self.assertIn("gyro[deg/s]:", output)
        self.assertIn("x= +12.50", output)
        self.assertIn("y=  -3.00", output)
        self.assertIn("z=  +1.25", output)
        self.assertIn("quat[wxyz]: [+1.0000, +0.0000, +0.0000, +0.0000]", output)

    def test_build_locomotion_diagnostic_snapshot_includes_standing_delta(self) -> None:
        specification = build_leg_locomotion_robot_specification()
        snapshot = build_locomotion_diagnostic_snapshot(
            specification=specification,
            state=LocomotionControlState.INITIALIZING,
            requested_state=LocomotionControlState.INITIALIZING,
            command_velocity=np.zeros((3,), dtype=np.float32),
            actions=np.zeros((specification.joint_count,), dtype=np.float32),
            joint_position_target=np.zeros((specification.joint_count,), dtype=np.float32),
            joint_position_measured=np.zeros((specification.joint_count,), dtype=np.float32),
            position_offsets=np.zeros((specification.joint_count,), dtype=np.float32),
            joint_axis_directions=specification.joint_axis_directions,
            dry_run=True,
        )

        np.testing.assert_allclose(snapshot.delta_to_standing, specification.standing_positions)
        np.testing.assert_allclose(snapshot.delta_to_initialization, specification.initialization_positions)
        self.assertEqual(snapshot.delta_to_standing.shape, (specification.joint_count,))
        self.assertEqual(snapshot.raw_delta_to_standing.shape, (specification.joint_count,))
        self.assertTrue(snapshot.standing_risk_joint_name.endswith("_joint"))


if __name__ == "__main__":
    unittest.main()
