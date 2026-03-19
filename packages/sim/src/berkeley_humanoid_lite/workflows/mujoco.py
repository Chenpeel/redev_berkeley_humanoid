from __future__ import annotations

import numpy as np
import torch

def run_mujoco_policy_loop(configuration: object) -> None:
    """在 MuJoCo 中运行策略推理闭环。"""
    from berkeley_humanoid_lite_lowlevel.policy import PolicyController

    from ..environments import MujocoSimulator

    simulator = MujocoSimulator(configuration)
    observations = simulator.reset()

    policy_controller = PolicyController(configuration)
    policy_controller.load_policy()

    default_actions = np.array(configuration.default_joint_positions, dtype=np.float32)[simulator.cfg.action_indices]

    while True:
        actions = policy_controller.compute_actions(observations.numpy())
        if actions is None:
            actions = default_actions
        observations = simulator.step(torch.as_tensor(actions, dtype=torch.float32))


def run_mujoco_visualization(configuration: object) -> None:
    """在 MuJoCo 中可视化 real-to-sim 观测流。"""
    from berkeley_humanoid_lite_lowlevel.policy import create_policy_deployment_configuration

    from ..environments import MujocoVisualizer
    from ..streams import UdpObservationReceiver

    visualizer_configuration = create_policy_deployment_configuration(
        {
            "num_joints": configuration.num_joints,
            "physics_dt": configuration.physics_dt,
        }
    )
    visualizer = MujocoVisualizer(visualizer_configuration)

    observation_receiver = UdpObservationReceiver(
        listen_host="0.0.0.0",
        remote_host=str(configuration.ip_host_addr),
        port=int(configuration.ip_policy_obs_port),
        observation_size=compute_policy_observation_size(configuration.num_joints),
    )
    observation_receiver.start()

    while True:
        visualizer.step(observation_receiver.buffer)


def compute_policy_observation_size(num_joints: int) -> int:
    """返回策略观测向量长度。"""
    return 7 + num_joints * 2 + 1 + 3
