from __future__ import annotations

import unittest
from types import ModuleType, SimpleNamespace
from unittest.mock import patch

from berkeley_humanoid_lite.environments.mujoco import MujocoEnv, MujocoSimulator
from berkeley_humanoid_lite.environments.session import MujocoSession
from berkeley_humanoid_lite.streams.udp_observation import UdpObservationReceiver
from berkeley_humanoid_lite.workflows.mujoco import run_mujoco_policy_loop, run_mujoco_visualization


class FakeViewer:
    def __init__(self) -> None:
        self.close_calls = 0

    def close(self) -> None:
        self.close_calls += 1


class FakeCommandSource:
    def __init__(self) -> None:
        self.stop_calls = 0

    def stop(self) -> None:
        self.stop_calls += 1


class FakeUdp:
    def __init__(self) -> None:
        self.stop_calls = 0

    def stop(self) -> None:
        self.stop_calls += 1


class FakeSimulator:
    instances: list[FakeSimulator] = []

    def __init__(self, configuration: object) -> None:
        self.cfg = configuration
        self.closed = False
        self.action_indices = configuration.action_indices
        self._step_calls = 0
        self.__class__.instances.append(self)

    def reset(self) -> object:
        return SimpleNamespace(numpy=lambda: [0.0] * 10)

    def step(self, actions: object) -> object:
        self._step_calls += 1
        raise RuntimeError("stop policy loop")

    def close(self) -> None:
        self.closed = True


class FakePolicyController:
    def __init__(self, configuration: object) -> None:
        self.configuration = configuration

    def load_policy(self) -> None:
        return None

    def compute_actions(self, observations: object) -> object:
        return [0.0 for _ in range(len(observations))]


class FakeVisualizer:
    instances: list[FakeVisualizer] = []

    def __init__(self, configuration: object) -> None:
        self.configuration = configuration
        self.closed = False
        self.__class__.instances.append(self)

    def step(self, observations: object) -> None:
        raise RuntimeError("stop visualization loop")

    def close(self) -> None:
        self.closed = True


class FakeObservationReceiver:
    instances: list[FakeObservationReceiver] = []

    def __init__(self, **kwargs: object) -> None:
        self.kwargs = kwargs
        self.buffer = [0.0] * kwargs["observation_size"]
        self.closed = False
        self.started = False
        self.__class__.instances.append(self)

    def start(self) -> None:
        self.started = True

    def close(self) -> None:
        self.closed = True


class MujocoLifecycleTestCase(unittest.TestCase):
    def test_mujoco_env_close_is_idempotent(self) -> None:
        viewer = FakeViewer()
        env = object.__new__(MujocoEnv)
        env.session = MujocoSession(model="model", data="data", viewer=viewer)
        env._closed = False

        env.close()
        env.close()

        self.assertEqual(viewer.close_calls, 1)

    def test_mujoco_simulator_close_stops_command_source_and_closes_session(self) -> None:
        viewer = FakeViewer()
        simulator = object.__new__(MujocoSimulator)
        simulator.session = MujocoSession(model="model", data="data", viewer=viewer)
        simulator.command_source = FakeCommandSource()
        simulator._closed = False

        simulator.close()
        simulator.close()

        self.assertEqual(simulator.command_source.stop_calls, 1)
        self.assertEqual(viewer.close_calls, 1)

    def test_udp_observation_receiver_close_stops_udp_transport(self) -> None:
        receiver = UdpObservationReceiver(
            listen_host="0.0.0.0",
            remote_host="127.0.0.1",
            port=5000,
            observation_size=4,
        )
        fake_udp = FakeUdp()
        receiver._udp = fake_udp

        receiver.close()

        self.assertTrue(receiver._stopped.is_set())
        self.assertEqual(fake_udp.stop_calls, 1)

    def test_run_mujoco_policy_loop_closes_simulator_on_failure(self) -> None:
        fake_policy_module = ModuleType("berkeley_humanoid_lite_lowlevel.policy")
        fake_policy_module.PolicyController = FakePolicyController
        fake_environment_module = ModuleType("berkeley_humanoid_lite.environments")
        fake_environment_module.MujocoSimulator = FakeSimulator

        configuration = SimpleNamespace(
            default_joint_positions=[0.0, 0.0],
            action_indices=[0, 1],
        )

        with patch.dict(
            "sys.modules",
            {
                "berkeley_humanoid_lite_lowlevel.policy": fake_policy_module,
                "berkeley_humanoid_lite.environments": fake_environment_module,
            },
        ):
            with self.assertRaisesRegex(RuntimeError, "stop policy loop"):
                run_mujoco_policy_loop(configuration)

        self.assertEqual(len(FakeSimulator.instances), 1)
        self.assertTrue(FakeSimulator.instances[0].closed)
        FakeSimulator.instances.clear()

    def test_run_mujoco_visualization_closes_receiver_and_visualizer_on_failure(self) -> None:
        fake_policy_module = ModuleType("berkeley_humanoid_lite_lowlevel.policy")
        fake_policy_module.create_policy_deployment_configuration = lambda values: SimpleNamespace(**values)
        fake_environment_module = ModuleType("berkeley_humanoid_lite.environments")
        fake_environment_module.MujocoVisualizer = FakeVisualizer
        fake_stream_module = ModuleType("berkeley_humanoid_lite.streams")
        fake_stream_module.UdpObservationReceiver = FakeObservationReceiver

        configuration = SimpleNamespace(
            num_joints=12,
            physics_dt=0.005,
            ip_host_addr="127.0.0.1",
            ip_policy_obs_port=5000,
        )

        with patch.dict(
            "sys.modules",
            {
                "berkeley_humanoid_lite_lowlevel.policy": fake_policy_module,
                "berkeley_humanoid_lite.environments": fake_environment_module,
                "berkeley_humanoid_lite.streams": fake_stream_module,
            },
        ):
            with self.assertRaisesRegex(RuntimeError, "stop visualization loop"):
                run_mujoco_visualization(configuration)

        self.assertEqual(len(FakeVisualizer.instances), 1)
        self.assertEqual(len(FakeObservationReceiver.instances), 1)
        self.assertTrue(FakeVisualizer.instances[0].closed)
        self.assertTrue(FakeObservationReceiver.instances[0].closed)
        FakeVisualizer.instances.clear()
        FakeObservationReceiver.instances.clear()


if __name__ == "__main__":
    unittest.main()
