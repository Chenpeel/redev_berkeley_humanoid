
import time

import numpy as np
import torch
from berkeley_humanoid_lite_lowlevel.policy.configuration import PolicyDeploymentConfiguration
from berkeley_humanoid_lite_lowlevel.robot import GamepadCommandSource

from .control import compute_pd_torques
from .initialization import build_simulator_initialization
from .observations import (
    build_policy_observation,
    update_command_state,
)
from .runtime import (
    apply_visualizer_observation,
    pace_policy_step,
    reset_simulator_state,
    reset_visualizer_state,
)
from .session import close_mujoco_session, create_mujoco_session, step_mujoco_session, sync_mujoco_viewer


class MujocoEnv:
    def __init__(self, cfg: PolicyDeploymentConfiguration):
        self.cfg = cfg
        self.session = create_mujoco_session(
            num_joints=int(cfg.num_joints),
            physics_dt=float(self.cfg.physics_dt),
        )
        self.mj_model = self.session.model
        self.mj_data = self.session.data
        self.mj_viewer = self.session.viewer
        self._closed = False

    def close(self) -> None:
        if self._closed:
            return
        close_mujoco_session(self.session)
        self._closed = True

    def __enter__(self) -> "MujocoEnv":
        return self

    def __exit__(self, exc_type: object, exc: object, exc_tb: object) -> None:
        self.close()


class MujocoVisualizer(MujocoEnv):
    """MuJoCo simulation environment for the Berkeley Humanoid Lite robot.

    This class handles the physics simulation, state observation, and control
    of the robot in the MuJoCo environment.

    Args:
        cfg (PolicyDeploymentConfiguration): Configuration object containing simulation parameters
    """
    def __init__(self, cfg: PolicyDeploymentConfiguration):
        super().__init__(cfg)

        self.num_dofs = self.mj_model.nu
        print(f"Number of DOFs: {self.num_dofs}")

    def reset(self) -> None:
        """Reset the simulation environment to initial state.

        Returns:
            torch.Tensor: Initial observations after reset
        """
        reset_visualizer_state(self.mj_data, num_dofs=self.num_dofs)

    def step(self, robot_observations: np.array) -> None:
        """Execute one simulation step with the given actions.

        Args:
            actions (torch.Tensor): Joint position targets for controlled joints

        Returns:
            torch.Tensor: Updated observations after executing the action
        """
        apply_visualizer_observation(
            self.mj_data,
            robot_observations,
            num_dofs=self.num_dofs,
        )

        step_mujoco_session(self.session)
        sync_mujoco_viewer(self.session)


class MujocoSimulator(MujocoEnv):
    """MuJoCo simulation environment for the Berkeley Humanoid Lite robot.

    This class handles the physics simulation, state observation, and control
    of the robot in the MuJoCo environment.

    Args:
        cfg (PolicyDeploymentConfiguration): Configuration object containing simulation parameters
    """
    def __init__(self, cfg: PolicyDeploymentConfiguration):
        super().__init__(cfg)
        initialization = build_simulator_initialization(
            self.cfg,
            num_actuators=self.mj_model.nu,
        )
        self.physics_substeps = initialization.physics_substeps
        self.sensor_layout = initialization.sensor_layout
        self.control_parameters = initialization.control_parameters
        self.command_state = initialization.command_state

        self.n_steps = 0

        print("Policy frequency: ", 1 / self.cfg.policy_dt)
        print("Physics frequency: ", 1 / self.cfg.physics_dt)
        print("Physics substeps: ", self.physics_substeps)

        self.command_source = GamepadCommandSource()
        self.command_source.start()

    def close(self) -> None:
        if self._closed:
            return
        command_source = getattr(self, "command_source", None)
        if command_source is not None:
            command_source.stop()
        super().close()

    def reset(self) -> torch.Tensor:
        """Reset the simulation environment to initial state.

        Returns:
            torch.Tensor: Initial observations after reset
        """
        reset_simulator_state(
            self.mj_data,
            default_base_position=self.cfg.default_base_position,
            default_joint_positions=self.cfg.default_joint_positions,
        )

        observations = self._get_observations()
        return observations

    def step(self, actions: torch.Tensor) -> torch.Tensor:
        """Execute one simulation step with the given actions.

        Args:
            actions (torch.Tensor): Joint position targets for controlled joints

        Returns:
            torch.Tensor: Updated observations after executing the action
        """
        step_start_time = time.perf_counter()

        for _ in range(self.physics_substeps):
            self._apply_actions(actions)
            step_mujoco_session(self.session)

        sync_mujoco_viewer(self.session)
        observations = self._get_observations()

        # Maintain real-time simulation
        pace_policy_step(self.cfg.policy_dt, step_start_time)

        self.n_steps += 1
        return observations

    def _apply_actions(self, actions: torch.Tensor):
        """Apply control actions to the robot.

        Implements PD control with torque limits and filtering.

        Args:
            actions (torch.Tensor): Target joint positions for controlled joints
        """
        output_torques = compute_pd_torques(
            actions,
            num_joints=self.cfg.num_joints,
            action_indices=self.cfg.action_indices,
            joint_positions=self._get_joint_pos(),
            joint_velocities=self._get_joint_vel(),
            joint_kp=self.control_parameters.joint_kp,
            joint_kd=self.control_parameters.joint_kd,
            effort_limits=self.control_parameters.effort_limits,
        )
        self.mj_data.ctrl[:] = output_torques.numpy()

    def _get_base_pos(self) -> torch.Tensor:
        """Get base position of the robot.

        Returns:
            torch.Tensor: Base position [x, y, z]
        """
        return torch.tensor(self.mj_data.qpos[:3], dtype=torch.float32)

    def _get_base_quat(self) -> torch.Tensor:
        """Get base orientation quaternion from sensors.

        Returns:
            torch.Tensor: Base orientation quaternion [w, x, y, z]
        """
        return self.sensor_layout.base_quaternion(self.mj_data.sensordata)

    def _get_base_ang_vel(self) -> torch.Tensor:
        """Get base angular velocity from sensors.

        Returns:
            torch.Tensor: Base angular velocity [wx, wy, wz]
        """
        return self.sensor_layout.base_angular_velocity(self.mj_data.sensordata)

    def _get_joint_pos(self) -> torch.Tensor:
        """Get joint positions from sensors.

        Returns:
            torch.Tensor: Joint positions
        """
        return self.sensor_layout.joint_positions(self.mj_data.sensordata)

    def _get_joint_vel(self) -> torch.Tensor:
        """Get joint velocities from sensors.

        Returns:
            torch.Tensor: Joint velocities
        """
        return self.sensor_layout.joint_velocities(self.mj_data.sensordata)

    def _get_observations(self) -> torch.Tensor:
        """Get complete observation vector for the policy.

        Returns:
            torch.Tensor: Concatenated observation vector containing base orientation,
                         angular velocity, joint positions, velocities, and command state
        """
        command = self.command_source.snapshot()
        self.command_state = update_command_state(command, current_mode=self.command_state.mode)

        return build_policy_observation(
            base_quat=self._get_base_quat(),
            base_ang_vel=self._get_base_ang_vel(),
            joint_positions=self._get_joint_pos(),
            joint_velocities=self._get_joint_vel(),
            action_indices=self.cfg.action_indices,
            command_state=self.command_state,
        )
