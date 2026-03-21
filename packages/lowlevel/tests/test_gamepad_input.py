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
    def test_discover_linux_gamepads_reads_js_devices_when_inputs_enumeration_is_empty(self) -> None:
        fake_devices = SimpleNamespace(gamepads=[], all_devices=[])
        gamepad_factory = Mock(return_value="gamepad")
        glob_results = {
            "/sys/class/input/js*": ["/sys/class/input/js0"],
            "/sys/class/input/js0/device/event*": ["/sys/class/input/js0/device/event15"],
        }

        with (
            patch.object(command_source_module, "devices", fake_devices),
            patch.object(command_source_module, "GamePad", gamepad_factory),
            patch.object(command_source_module, "_is_linux", return_value=True),
            patch.object(
                command_source_module.glob,
                "glob",
                side_effect=lambda pattern: glob_results.get(pattern, []),
            ),
            patch.object(command_source_module.os.path, "exists", return_value=True),
            patch.object(
                command_source_module,
                "_read_linux_gamepad_name",
                return_value="Xbox Wireless Controller",
            ),
        ):
            discovered_gamepads = command_source_module._discover_linux_gamepads()

        self.assertEqual(discovered_gamepads, ["gamepad"])
        gamepad_factory.assert_called_once_with(
            fake_devices,
            "/dev/input/by-id/manual-Xbox_Wireless_Controller-event-joystick",
            char_path_override="/dev/input/event15",
        )

    def test_command_source_uses_linux_js_fallback_when_inputs_gamepads_are_empty(self) -> None:
        command_source = GamepadCommandSource()
        fake_gamepad = object()
        fake_devices = SimpleNamespace(gamepads=[], all_devices=[])

        with (
            patch.object(command_source_module, "devices", fake_devices),
            patch.object(command_source_module, "get_gamepad", Mock()),
            patch.object(command_source_module, "_discover_linux_gamepads", return_value=[fake_gamepad]),
        ):
            command_source._ensure_available()

        self.assertEqual(fake_devices.gamepads, [fake_gamepad])
        self.assertEqual(fake_devices.all_devices, [fake_gamepad])

    def test_command_source_advance_retries_after_registering_linux_fallback(self) -> None:
        command_source = GamepadCommandSource()
        fake_devices = SimpleNamespace(gamepads=[])
        fake_event = SimpleNamespace(code="ABS_X", state=1234)

        def register_fallback() -> None:
            fake_devices.gamepads.append(object())

        with (
            patch.object(command_source_module, "devices", fake_devices),
            patch.object(
                command_source_module,
                "_register_discovered_gamepads",
                side_effect=register_fallback,
            ) as register_discovered_gamepads,
            patch.object(command_source_module, "get_gamepad", return_value=[fake_event]),
        ):
            command_source.advance()

        register_discovered_gamepads.assert_called_once_with()
        self.assertAlmostEqual(command_source.snapshot().velocity_yaw, -1234 / 32768.0)

    def test_command_source_tracks_unsigned_axis_modes_for_bluetooth_gamepads(self) -> None:
        command_source = GamepadCommandSource(dead_zone=0.0)
        fake_devices = SimpleNamespace(gamepads=[object()])
        events = [
            SimpleNamespace(code="ABS_X", state=32768),
            SimpleNamespace(code="ABS_Y", state=32768),
            SimpleNamespace(code="ABS_RX", state=32768),
        ]

        with (
            patch.object(command_source_module, "devices", fake_devices),
            patch.object(command_source_module, "get_gamepad", return_value=events),
        ):
            command_source.advance()

        self.assertEqual(
            command_source._axis_modes,
            {"ABS_X": "unsigned", "ABS_Y": "unsigned", "ABS_RX": "unsigned"},
        )
        self.assertEqual(command_source.snapshot().velocity_x, 0.0)
        self.assertEqual(command_source.snapshot().velocity_y, 0.0)
        self.assertEqual(command_source.snapshot().velocity_yaw, 0.0)

    def test_command_source_start_fails_fast_when_no_gamepad_is_detected(self) -> None:
        command_source = GamepadCommandSource()

        with (
            patch.object(command_source_module, "devices", SimpleNamespace(gamepads=[])),
            patch.object(command_source_module, "get_gamepad", Mock()),
            patch.object(command_source_module, "_discover_linux_gamepads", return_value=[]),
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

    def test_cli_wrapper_converts_missing_runtime_dependency_to_friendly_exit(self) -> None:
        def raise_missing_can_dependency() -> None:
            error = ModuleNotFoundError("No module named 'can'")
            error.name = "can"
            raise error

        with self.assertRaises(SystemExit) as raised:
            run_with_friendly_gamepad_errors(raise_missing_can_dependency)

        self.assertIn("python-can", str(raised.exception))
        self.assertIn("uv sync --all-packages --group dev --group lowlevel-runtime", str(raised.exception))

    def test_cli_wrapper_converts_missing_setuptools_dependency_to_friendly_exit(self) -> None:
        def raise_missing_pkg_resources_dependency() -> None:
            error = ModuleNotFoundError("No module named 'pkg_resources'")
            error.name = "pkg_resources"
            raise error

        with self.assertRaises(SystemExit) as raised:
            run_with_friendly_gamepad_errors(raise_missing_pkg_resources_dependency)

        self.assertIn("setuptools", str(raised.exception))
        self.assertIn("uv sync --all-packages --group dev --group lowlevel-runtime", str(raised.exception))

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
