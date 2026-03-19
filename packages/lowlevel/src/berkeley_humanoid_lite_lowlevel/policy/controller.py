from __future__ import annotations

from abc import ABC, abstractmethod

import numpy as np
import onnxruntime as ort
import torch

from berkeley_humanoid_lite_lowlevel.policy.configuration import PolicyDeploymentConfiguration
from berkeley_humanoid_lite_lowlevel.robot.orientation_math import compute_projected_gravity, create_gravity_vector
from berkeley_humanoid_lite_lowlevel.runtime_paths import resolve_workspace_path


class Policy(ABC):
    @abstractmethod
    def forward(self, observations: np.ndarray) -> np.ndarray:
        raise NotImplementedError


class TorchPolicy(Policy):
    def __init__(self, checkpoint_path: str, device: str = "cpu"):
        self.device = device
        self.model: torch.nn.Module = torch.load(checkpoint_path, map_location=self.device)
        self.model.eval()

    def forward(self, observations: np.ndarray) -> np.ndarray:
        observations_tensor = torch.from_numpy(observations).unsqueeze(0).to(self.device)
        actions_tensor = self.model(observations_tensor)
        return actions_tensor.detach().cpu().squeeze(0).numpy()


class OnnxPolicy(Policy):
    def __init__(self, checkpoint_path: str):
        self.model: ort.InferenceSession = ort.InferenceSession(checkpoint_path)

        input_shape = self.model.get_inputs()[0].shape
        try:
            self.model.run(None, {"obs": np.zeros(input_shape, dtype=np.float32)})[0]
            self.input_key = "obs"
        except Exception as error:
            print(error)
            self.input_key = "onnx::Gemm_0"

    def forward(self, observations: np.ndarray) -> np.ndarray:
        return np.array(self.model.run(None, {self.input_key: observations})[0])


class PolicyController:
    def __init__(self, configuration: PolicyDeploymentConfiguration):
        self.configuration = configuration

        self.command_velocity = np.array(self.configuration.command_velocity, dtype=np.float32)
        if self.configuration.num_actions == self.configuration.num_joints:
            self.default_joint_positions = np.array(self.configuration.default_joint_positions, dtype=np.float32)
        else:
            self.default_joint_positions = np.array(self.configuration.default_joint_positions[10:], dtype=np.float32)

        self.gravity_vector = create_gravity_vector()
        self.policy_observations = np.zeros(
            (1, self.configuration.num_observations * (self.configuration.history_length + 1)),
            dtype=np.float32,
        )
        self.policy_actions = np.zeros((1, self.configuration.num_actions), dtype=np.float32)
        self.previous_actions = np.zeros((self.configuration.num_actions,), dtype=np.float32)

    def load_policy(self) -> None:
        checkpoint_path = resolve_workspace_path(self.configuration.policy_checkpoint_path)

        if ".pt" in str(checkpoint_path):
            torch.set_printoptions(precision=2)
            self.policy = TorchPolicy(str(checkpoint_path))
            print("Using Torch runner")
            return

        if ".onnx" in str(checkpoint_path):
            self.policy = OnnxPolicy(str(checkpoint_path))
            print("Using ONNX runner")
            return

        raise ValueError("Unrecognized policy format")

    def compute_actions(self, robot_observations: np.ndarray) -> np.ndarray:
        robot_base_quaternion = robot_observations[0:4]
        robot_base_angular_velocity = robot_observations[4:7]
        robot_joint_positions = (
            robot_observations[7 : 7 + self.configuration.num_actions] - self.default_joint_positions
        )
        robot_joint_velocities = robot_observations[
            7 + self.configuration.num_actions : 7 + self.configuration.num_actions * 2
        ]
        command_velocity = robot_observations[
            7 + self.configuration.num_actions * 2 + 1 : 7 + self.configuration.num_actions * 2 + 4
        ]

        projected_gravity = compute_projected_gravity(robot_base_quaternion, self.gravity_vector)
        self.policy_observations[:] = np.concatenate(
            [
                self.policy_observations[0, self.configuration.num_observations :],
                command_velocity,
                robot_base_angular_velocity,
                projected_gravity,
                robot_joint_positions,
                robot_joint_velocities,
                self.previous_actions,
            ],
            axis=0,
        )

        self.policy_actions[:] = self.policy.forward(self.policy_observations)

        clipped_actions = np.clip(
            self.policy_actions[0],
            self.configuration.action_limit_lower,
            self.configuration.action_limit_upper,
        )
        self.previous_actions[:] = clipped_actions

        return clipped_actions * self.configuration.action_scale + self.default_joint_positions
