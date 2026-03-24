from __future__ import annotations

import re
import sys
import types
import unittest
from pathlib import Path
from types import SimpleNamespace

import torch
from berkeley_humanoid_lite.training.export import (
    _resolve_actuator_scalar,
    build_policy_deployment_configuration,
)


def _install_fake_isaaclab_string_module() -> None:
    isaaclab_module = types.ModuleType("isaaclab")
    isaaclab_utils_module = types.ModuleType("isaaclab.utils")
    isaaclab_string_module = types.ModuleType("isaaclab.utils.string")

    def resolve_matching_names_values(expression_dict, candidate_names, preserve_order=True):
        indices: list[int] = []
        matched_names: list[str] = []
        matched_values: list[object] = []

        for expression, value in expression_dict.items():
            pattern = re.compile(expression)
            for index, candidate_name in enumerate(candidate_names):
                if pattern.fullmatch(candidate_name):
                    indices.append(index)
                    matched_names.append(candidate_name)
                    matched_values.append(value)

        return indices, matched_names, matched_values

    isaaclab_string_module.resolve_matching_names_values = resolve_matching_names_values
    isaaclab_utils_module.string = isaaclab_string_module
    isaaclab_module.utils = isaaclab_utils_module
    sys.modules["isaaclab"] = isaaclab_module
    sys.modules["isaaclab.utils"] = isaaclab_utils_module
    sys.modules["isaaclab.utils.string"] = isaaclab_string_module


class ExportConfigurationTestCase(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        _install_fake_isaaclab_string_module()

    def test_resolve_actuator_scalar_falls_back_to_effort_limit_sim(self) -> None:
        actuator_group = SimpleNamespace(effort_limit=None, effort_limit_sim=6.0)

        resolved_limit = _resolve_actuator_scalar(
            "legs",
            actuator_group,
            primary_name="effort_limit",
            fallback_names=("effort_limit_sim",),
        )

        self.assertEqual(resolved_limit, 6.0)

    def test_build_policy_deployment_configuration_uses_effort_limit_sim_when_old_field_is_none(self) -> None:
        env_cfg = SimpleNamespace(
            scene=SimpleNamespace(
                robot=SimpleNamespace(
                    init_state=SimpleNamespace(
                        joint_pos={"joint_a": 0.1, "joint_b": -0.2},
                        pos=(0.0, 0.0, 0.0),
                    ),
                    actuators={
                        "legs": SimpleNamespace(
                            joint_names_expr=["joint_.*"],
                            stiffness=20.0,
                            damping=2.0,
                            effort_limit=None,
                            effort_limit_sim=6.0,
                        ),
                    },
                )
            ),
            actions=SimpleNamespace(
                joint_pos=SimpleNamespace(
                    joint_names=["joint_.*"],
                    scale=0.25,
                )
            ),
            sim=SimpleNamespace(dt=0.005),
            decimation=8,
            observations=SimpleNamespace(
                policy=SimpleNamespace(
                    actions=SimpleNamespace(history_length=3),
                    velocity_commands=SimpleNamespace(
                        func=lambda env, command_name: torch.tensor([[0.0, 0.0, 0.0]], dtype=torch.float32),
                        params={"command_name": "base_velocity"},
                    ),
                )
            ),
        )
        env = SimpleNamespace(
            unwrapped=SimpleNamespace(device="cpu"),
            observation_space={"policy": SimpleNamespace(shape=(75,))},
            action_space=SimpleNamespace(shape=(2,)),
        )

        configuration = build_policy_deployment_configuration(
            env_cfg=env_cfg,
            env=env,
            export_directory=Path("artifacts/tmp/exported"),
        )

        self.assertEqual(configuration["effort_limits"], [6.0, 6.0])
        self.assertEqual(configuration["joint_kp"], [20.0, 20.0])
        self.assertEqual(configuration["joint_kd"], [2.0, 2.0])
        self.assertEqual(configuration["action_indices"], [0, 1])

    def test_resolve_actuator_scalar_raises_clear_error_when_limit_is_missing(self) -> None:
        actuator_group = SimpleNamespace(effort_limit=None, effort_limit_sim=None)

        with self.assertRaisesRegex(
            ValueError,
            "Actuator group 'legs' does not define any of: effort_limit, effort_limit_sim",
        ):
            _resolve_actuator_scalar(
                "legs",
                actuator_group,
                primary_name="effort_limit",
                fallback_names=("effort_limit_sim",),
            )


if __name__ == "__main__":
    unittest.main()
