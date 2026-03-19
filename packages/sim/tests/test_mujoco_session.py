from __future__ import annotations

import unittest
from dataclasses import dataclass
from pathlib import Path
from types import SimpleNamespace

from berkeley_humanoid_lite.environments.session import (
    MujocoSession,
    close_mujoco_session,
    create_mujoco_session,
    resolve_mjcf_scene_path,
    step_mujoco_session,
    sync_mujoco_viewer,
)


@dataclass
class FakeViewer:
    sync_calls: int = 0
    close_calls: int = 0

    def sync(self) -> None:
        self.sync_calls += 1

    def close(self) -> None:
        self.close_calls += 1


class FakeModel:
    def __init__(self) -> None:
        self.opt = SimpleNamespace(timestep=0.0)


class MujocoSessionTestCase(unittest.TestCase):
    def test_resolve_mjcf_scene_path_points_to_existing_xml(self) -> None:
        scene_path = resolve_mjcf_scene_path(12)

        self.assertIsInstance(scene_path, Path)
        self.assertEqual(scene_path.suffix, ".xml")
        self.assertTrue(scene_path.exists())

    def test_create_mujoco_session_uses_factories_and_sets_timestep(self) -> None:
        recorded: dict[str, object] = {}

        def fake_model_loader(path: str) -> FakeModel:
            recorded["path"] = path
            return FakeModel()

        def fake_data_factory(model: FakeModel) -> object:
            recorded["model"] = model
            return {"model": model}

        def fake_viewer_factory(model: FakeModel, data: object) -> FakeViewer:
            recorded["viewer_input"] = (model, data)
            return FakeViewer()

        session = create_mujoco_session(
            num_joints=12,
            physics_dt=0.005,
            model_loader=fake_model_loader,
            data_factory=fake_data_factory,
            viewer_factory=fake_viewer_factory,
        )

        self.assertIsInstance(session, MujocoSession)
        self.assertTrue(str(recorded["path"]).endswith("bhl_biped_scene.xml"))
        self.assertIs(recorded["model"], session.model)
        self.assertEqual(recorded["viewer_input"], (session.model, session.data))
        self.assertEqual(session.model.opt.timestep, 0.005)

    def test_step_mujoco_session_delegates_to_step_function(self) -> None:
        session = MujocoSession(model="model", data="data", viewer=FakeViewer())
        step_calls: list[tuple[object, object]] = []

        step_mujoco_session(
            session,
            step_fn=lambda model, data: step_calls.append((model, data)),
        )

        self.assertEqual(step_calls, [("model", "data")])

    def test_sync_mujoco_viewer_calls_viewer_sync(self) -> None:
        viewer = FakeViewer()
        session = MujocoSession(model="model", data="data", viewer=viewer)

        sync_mujoco_viewer(session)

        self.assertEqual(viewer.sync_calls, 1)

    def test_close_mujoco_session_calls_viewer_close_when_available(self) -> None:
        viewer = FakeViewer()
        session = MujocoSession(model="model", data="data", viewer=viewer)

        close_mujoco_session(session)

        self.assertEqual(viewer.close_calls, 1)

    def test_close_mujoco_session_tolerates_viewer_without_close(self) -> None:
        session = MujocoSession(model="model", data="data", viewer=SimpleNamespace(sync=lambda: None))

        close_mujoco_session(session)


if __name__ == "__main__":
    unittest.main()
