from __future__ import annotations

import unittest

import numpy as np
from berkeley_humanoid_lite_lowlevel.robot.locomotion_diagnostics import format_imu_debug_line


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


if __name__ == "__main__":
    unittest.main()
