from __future__ import annotations

import struct
import unittest

from berkeley_humanoid_lite_lowlevel.robot.command_source import LocomotionCommand
from berkeley_humanoid_lite_lowlevel.robot.control_state import LocomotionControlState
from berkeley_humanoid_lite_lowlevel.workflows.locomotion import encode_gamepad_command_packet


class LocomotionWorkflowTests(unittest.TestCase):
    def test_encode_gamepad_command_packet_matches_native_runtime_layout(self) -> None:
        command = LocomotionCommand(
            requested_state=LocomotionControlState.POLICY_CONTROL,
            velocity_x=0.25,
            velocity_y=-0.5,
            velocity_yaw=0.75,
        )

        packet = encode_gamepad_command_packet(command)

        self.assertEqual(len(packet), 13)

        mode, velocity_x, velocity_y, velocity_yaw = struct.unpack("<Bfff", packet)
        self.assertEqual(mode, int(LocomotionControlState.POLICY_CONTROL))
        self.assertAlmostEqual(velocity_x, 0.25)
        self.assertAlmostEqual(velocity_y, -0.5)
        self.assertAlmostEqual(velocity_yaw, 0.75)


if __name__ == "__main__":
    unittest.main()
