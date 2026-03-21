from __future__ import annotations

import math
import unittest
from unittest.mock import patch

from berkeley_humanoid_lite_lowlevel import actuator
from berkeley_humanoid_lite_lowlevel.workflows import actuator as actuator_workflow


class ActuatorOperationTests(unittest.TestCase):
    def test_resolve_angle_radians_prefers_radians_over_degrees(self) -> None:
        self.assertEqual(
            actuator.resolve_angle_radians(1.25, 90.0),
            1.25,
        )

    def test_resolve_angle_radians_converts_degrees_when_needed(self) -> None:
        self.assertAlmostEqual(
            actuator.resolve_angle_radians(None, 180.0),
            math.pi,
        )

    def test_build_actuator_angle_sequence_defaults_to_single_target_stage(self) -> None:
        self.assertEqual(
            actuator.build_actuator_angle_sequence(target_angle_radians=0.75),
            [("target", 0.75)],
        )

    def test_build_actuator_angle_sequence_uses_zero_return_when_omitted(self) -> None:
        self.assertEqual(
            actuator.build_actuator_angle_sequence(
                target_angle_radians=0.5,
                cycles=2,
            ),
            [
                ("cycle 1/2: target", 0.5),
                ("cycle 1/2: return", 0.0),
                ("cycle 2/2: target", 0.5),
                ("cycle 2/2: return", 0.0),
            ],
        )

    def test_build_actuator_angle_sequence_rejects_non_positive_cycles(self) -> None:
        with self.assertRaisesRegex(ValueError, "positive integer"):
            actuator.build_actuator_angle_sequence(target_angle_radians=0.5, cycles=0)

    def test_sample_within_tolerance_requires_error_and_velocity_both_in_range(self) -> None:
        self.assertTrue(
            actuator.sample_within_tolerance(
                position_error_radians=math.radians(0.5),
                velocity_radians_per_second=math.radians(2.0),
                position_tolerance_radians=math.radians(1.0),
                velocity_tolerance_radians_per_second=math.radians(5.0),
            )
        )
        self.assertFalse(
            actuator.sample_within_tolerance(
                position_error_radians=math.radians(1.5),
                velocity_radians_per_second=math.radians(2.0),
                position_tolerance_radians=math.radians(1.0),
                velocity_tolerance_radians_per_second=math.radians(5.0),
            )
        )
        self.assertFalse(
            actuator.sample_within_tolerance(
                position_error_radians=math.radians(0.5),
                velocity_radians_per_second=math.radians(6.0),
                position_tolerance_radians=math.radians(1.0),
                velocity_tolerance_radians_per_second=math.radians(5.0),
            )
        )


class ActuatorWorkflowTests(unittest.TestCase):
    def test_run_actuator_angle_test_requires_cycles_for_return_angle(self) -> None:
        with self.assertRaisesRegex(ValueError, "--return-rad/--return-deg requires --cycles"):
            actuator_workflow.run_actuator_angle_test(
                channel="can0",
                device_id=1,
                target_angle_degrees=30.0,
                return_angle_degrees=0.0,
            )

    def test_run_actuator_angle_test_converts_arguments_and_stops_bus(self) -> None:
        captured: dict[str, object] = {}

        class FakeBus:
            def stop(self) -> None:
                captured["stopped"] = True

        fake_bus = FakeBus()

        with (
            patch.object(actuator_workflow, "create_actuator_bus", return_value=fake_bus),
            patch.object(actuator_workflow, "run_actuator_angle_sequence") as run_actuator_angle_sequence,
            patch.object(actuator_workflow, "print"),
        ):
            actuator_workflow.run_actuator_angle_test(
                channel="can2",
                device_id=3,
                bitrate=500_000,
                target_angle_degrees=90.0,
                return_angle_degrees=-30.0,
                position_kp=0.4,
                position_kd=0.02,
                torque_limit=0.7,
                max_speed_degrees_per_second=45.0,
                hold_seconds=1.5,
                control_frequency_hz=150.0,
                position_tolerance_degrees=1.5,
                velocity_tolerance_degrees_per_second=8.0,
                settle_timeout_seconds=3.0,
                required_stable_samples=12,
                cycles=2,
            )

        self.assertTrue(captured.get("stopped"))
        run_actuator_angle_sequence.assert_called_once()
        args, kwargs = run_actuator_angle_sequence.call_args
        self.assertIs(args[0], fake_bus)
        self.assertEqual(args[1], 3)
        self.assertAlmostEqual(kwargs["target_angle_radians"], math.pi / 2.0)
        self.assertAlmostEqual(kwargs["return_angle_radians"], -math.pi / 6.0)
        self.assertEqual(kwargs["cycles"], 2)
        self.assertEqual(kwargs["position_kp"], 0.4)
        self.assertEqual(kwargs["position_kd"], 0.02)
        self.assertEqual(kwargs["torque_limit"], 0.7)
        self.assertAlmostEqual(kwargs["max_speed_radians_per_second"], math.pi / 4.0)
        self.assertEqual(kwargs["hold_seconds"], 1.5)
        self.assertEqual(kwargs["control_frequency_hz"], 150.0)
        self.assertAlmostEqual(kwargs["position_tolerance_radians"], math.radians(1.5))
        self.assertAlmostEqual(kwargs["velocity_tolerance_radians_per_second"], math.radians(8.0))
        self.assertEqual(kwargs["settle_timeout_seconds"], 3.0)
        self.assertEqual(kwargs["required_stable_samples"], 12)


if __name__ == "__main__":
    unittest.main()
