from __future__ import annotations

import unittest
from types import ModuleType, SimpleNamespace
from unittest.mock import Mock, patch

import berkeley_humanoid_lite_lowlevel.robot.command_source as command_source_module
from berkeley_humanoid_lite_lowlevel.cli import run_with_friendly_gamepad_errors
from berkeley_humanoid_lite_lowlevel.robot.command_source import GamepadCommandSource, GamepadUnavailableError
from berkeley_humanoid_lite_lowlevel.workflows import locomotion as locomotion_workflow


class FakePolicyController:
    def __init__(self, configuration: object) -> None:
        self.configuration = configuration

    def load_policy(self) -> None:
        return None


class GamepadInputTests(unittest.TestCase):
    def test_command_source_start_fails_fast_when_no_gamepad_is_detected(self) -> None:
        command_source = GamepadCommandSource()

        with (
            patch.object(command_source_module, "devices", SimpleNamespace(gamepads=[])),
            patch.object(command_source_module, "get_gamepad", Mock()),
        ):
            with self.assertRaisesRegex(GamepadUnavailableError, "未检测到可用手柄"):
                command_source.start()

        self.assertIsNone(command_source._thread)

    def test_cli_wrapper_converts_gamepad_error_to_friendly_exit(self) -> None:
        def raise_unavailable_error() -> None:
            raise GamepadUnavailableError("未检测到可用手柄。")

        with self.assertRaises(SystemExit) as raised:
            run_with_friendly_gamepad_errors(raise_unavailable_error)

        self.assertEqual(str(raised.exception), "未检测到可用手柄。")

    def test_broadcast_gamepad_commands_skips_udp_setup_when_start_fails(self) -> None:
        with (
            patch.object(locomotion_workflow, "create_gamepad_command_stream") as create_gamepad_command_stream,
            patch.object(
                locomotion_workflow.GamepadCommandSource,
                "start",
                side_effect=GamepadUnavailableError("未检测到可用手柄。"),
            ),
        ):
            with self.assertRaises(GamepadUnavailableError):
                locomotion_workflow.broadcast_gamepad_commands()

        create_gamepad_command_stream.assert_not_called()

    def test_run_locomotion_loop_delays_observation_stream_until_robot_is_ready(self) -> None:
        fake_policy_module = ModuleType("berkeley_humanoid_lite_lowlevel.policy.controller")
        fake_policy_module.PolicyController = FakePolicyController
        fake_robot_module = ModuleType("berkeley_humanoid_lite_lowlevel.robot")
        fake_robot_module.LocomotionRobot = Mock(
            side_effect=GamepadUnavailableError("未检测到可用手柄。")
        )
        configuration = SimpleNamespace(
            policy_dt=0.02,
            ip_policy_obs_port=10001,
            ip_host_addr="127.0.0.1",
        )

        with (
            patch.dict(
                "sys.modules",
                {
                    "berkeley_humanoid_lite_lowlevel.policy.controller": fake_policy_module,
                    "berkeley_humanoid_lite_lowlevel.robot": fake_robot_module,
                },
            ),
            patch.object(locomotion_workflow, "create_observation_stream") as create_observation_stream,
        ):
            with self.assertRaises(GamepadUnavailableError):
                locomotion_workflow.run_locomotion_loop(configuration)

        create_observation_stream.assert_not_called()


if __name__ == "__main__":
    unittest.main()
