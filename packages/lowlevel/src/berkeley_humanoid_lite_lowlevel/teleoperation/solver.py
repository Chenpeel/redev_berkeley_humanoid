# Copyright (c) 2025, The Berkeley Humanoid Lite Project Developers.

from __future__ import annotations

from typing import Any
import os
import time

from berkeley_humanoid_lite_assets.paths import get_urdf_path
import meshcat_shapes
import numpy as np
import pink
from pink import solve_ik
from pink.tasks import FrameTask
from pink.visualization import start_meshcat_visualizer
import pinocchio as pin
import qpsolvers


class TeleoperationIkSolver:
    def __init__(
        self,
        urdf_path: str = str(get_urdf_path()),
    ) -> None:
        urdf_package_path = os.path.dirname(urdf_path)
        self.robot = pin.RobotWrapper.BuildFromURDF(
            urdf_path,
            package_dirs=[urdf_package_path],
            root_joint=pin.JointModelFreeFlyer(),
        )

        self.visualizer = start_meshcat_visualizer(self.robot)
        self.viewer = self.visualizer.viewer

        self.end_effectors = ["arm_left_elbow_roll", "arm_right_elbow_roll"]
        self.end_effector_ids = [self.robot.model.getFrameId(name) for name in self.end_effectors]
        self.num_end_effectors = len(self.end_effectors)

        for name in self.end_effectors:
            meshcat_shapes.frame(self.viewer[f"{name}_target"], opacity=0.5)
            meshcat_shapes.frame(self.viewer[f"{name}_vive"], opacity=0.25)
            meshcat_shapes.frame(self.viewer[name], opacity=1.0)

        self.end_effector_tasks = [
            FrameTask(
                name,
                position_cost=1.0,
                orientation_cost=0.1,
                lm_damping=1.0,
            )
            for name in self.end_effectors
        ]
        self.base_tasks = [
            FrameTask(
                "base",
                position_cost=1000.0,
                orientation_cost=1000.0,
                lm_damping=1.0,
            )
        ]

        if any(end_effector_id < 0 or end_effector_id >= len(self.robot.model.frames) for end_effector_id in self.end_effector_ids):
            raise ValueError("End effector name not found in model")

        self.solver = qpsolvers.available_solvers[0]
        if "quadprog" in qpsolvers.available_solvers:
            self.solver = "quadprog"

        self.last_update_time = time.perf_counter()
        self.first_run = True
        self.last_button_data = (False, False)
        self.vive_last_poses = [pin.SE3.Identity() for _ in self.end_effector_ids]
        self.robot_last_poses = [pin.SE3.Identity() for _ in self.end_effector_ids]
        self.pose_data = (np.eye(4), np.eye(4))
        self.button_data = (False, False)
        self.trigger_data = (0.0, 0.0)

    def update_controller(self, bridge_data: dict[str, Any]) -> None:
        self.pose_data[0][:, :] = np.array(bridge_data["left"]["pose"])
        self.pose_data[1][:, :] = np.array(bridge_data["right"]["pose"])
        self.button_data = (bridge_data["left"]["button_pressed"], bridge_data["right"]["button_pressed"])
        self.trigger_data = (bridge_data["left"]["trigger"], bridge_data["right"]["trigger"])

    def update(self, observations: np.ndarray) -> tuple[np.ndarray, tuple[float, float]]:
        vive_poses = [
            pin.SE3(
                quat=pin.Quaternion(R=self.pose_data[index][0:3, 0:3]),
                translation=self.pose_data[index][0:3, -1],
            )
            for index in range(self.num_end_effectors)
        ]

        if observations.shape != (self.robot.model.nq,):
            raise ValueError("Unexpected teleoperation observation shape.")

        data = self.robot.data.copy()
        configuration_vector = observations.copy()
        configuration_vector[3:7] = configuration_vector[[4, 5, 6, 3]]
        pin.forwardKinematics(self.robot.model, data, configuration_vector)
        pin.framesForwardKinematics(self.robot.model, data, configuration_vector)
        pin.updateFramePlacements(self.robot.model, data)

        if self.first_run:
            self.first_run = False
            self.last_update_time = time.perf_counter()
            self.last_button_data = self.button_data

        delta_poses = [pin.SE3.Identity() for _ in range(self.num_end_effectors)]
        for index in range(self.num_end_effectors):
            if self.button_data[index] and not self.last_button_data[index]:
                self.vive_last_poses[index] = vive_poses[index].copy()
            if not self.button_data[index] and self.last_button_data[index]:
                self.robot_last_poses[index] = data.oMf[self.end_effector_ids[index]].copy()
            if self.button_data[index]:
                delta_poses[index].rotation = (vive_poses[index] * self.vive_last_poses[index].inverse()).rotation
                delta_poses[index].translation = (
                    vive_poses[index].translation - self.vive_last_poses[index].translation
                )
        self.last_button_data = self.button_data

        desired_poses = [
            pin.SE3(
                delta_poses[index].rotation @ self.robot_last_poses[index].rotation,
                self.robot_last_poses[index].translation + delta_poses[index].translation,
            )
            for index in range(self.num_end_effectors)
        ]

        configuration = pink.Configuration(self.robot.model, self.robot.data, configuration_vector)
        for index, name in enumerate(self.end_effectors):
            self.viewer[f"{name}_target"].set_transform(desired_poses[index].np)
            self.viewer[f"{name}_vive"].set_transform(vive_poses[index].np)
            self.viewer[name].set_transform(configuration.get_transform_frame_to_world(self.end_effector_tasks[index].frame).np)
            self.end_effector_tasks[index].transform_target_to_world = desired_poses[index]

        self.base_tasks[0].transform_target_to_world = pin.SE3(pin.Quaternion.Identity(), np.array([0.0, 0.0, 0.5]))
        tasks = self.end_effector_tasks + self.base_tasks

        delta_time = time.perf_counter() - self.last_update_time
        self.last_update_time = time.perf_counter()

        velocity = solve_ik(configuration, tasks, delta_time, solver=self.solver, safety_break=False)
        configuration.integrate_inplace(velocity, delta_time)
        self.visualizer.display(configuration.q)

        return configuration.q[7:17], self.trigger_data
