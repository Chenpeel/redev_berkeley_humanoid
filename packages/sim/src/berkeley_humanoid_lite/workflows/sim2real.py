from __future__ import annotations

from dataclasses import dataclass
from typing import Sequence

import numpy as np
import torch


def compute_policy_observation_size(num_joints: int) -> int:
    return 7 + num_joints * 2 + 1 + 3


def _format_array(values: Sequence[float] | np.ndarray) -> str:
    return np.array2string(
        np.asarray(values, dtype=np.float32),
        precision=3,
        suppress_small=True,
        floatmode="fixed",
    )


def _format_bridge_scale(value: float | Sequence[float]) -> str:
    scale_array = np.asarray(value, dtype=np.float32)
    if scale_array.ndim == 0:
        return f"{float(scale_array.item()):.3f}"
    return _format_array(scale_array)


def _canonicalize_joint_name(name: str) -> str:
    canonical_name = str(name).strip()
    if canonical_name.startswith("leg_"):
        return canonical_name[len("leg_") :]
    return canonical_name


def resolve_mujoco_bridge_joint_indices(
    sim_joint_names: Sequence[str],
    robot_joint_names: Sequence[str],
) -> np.ndarray:
    sim_index_by_name = {
        str(name): index
        for index, name in enumerate(sim_joint_names)
    }
    canonical_sim_index_by_name: dict[str, int] = {}
    for index, name in enumerate(sim_joint_names):
        canonical_name = _canonicalize_joint_name(str(name))
        existing_index = canonical_sim_index_by_name.get(canonical_name)
        if existing_index is not None and existing_index != index:
            raise ValueError(f"MuJoCo 关节名归一化后重复: {canonical_name}")
        canonical_sim_index_by_name[canonical_name] = index

    resolved_indices: list[int] = []
    missing_joint_names: list[str] = []
    for robot_joint_name in robot_joint_names:
        exact_index = sim_index_by_name.get(str(robot_joint_name))
        if exact_index is not None:
            resolved_indices.append(exact_index)
            continue

        canonical_name = _canonicalize_joint_name(str(robot_joint_name))
        canonical_index = canonical_sim_index_by_name.get(canonical_name)
        if canonical_index is None:
            missing_joint_names.append(str(robot_joint_name))
            continue
        resolved_indices.append(canonical_index)

    if missing_joint_names:
        joined_names = ", ".join(missing_joint_names)
        raise ValueError(f"MuJoCo 配置缺少这些真实机器人关节: {joined_names}")

    return np.asarray(resolved_indices, dtype=np.int64)


def _coerce_bridge_scale(
    bridge_scale: float | Sequence[float],
    *,
    joint_count: int,
) -> np.ndarray:
    scale_array = np.asarray(bridge_scale, dtype=np.float32)
    if scale_array.ndim == 0:
        if not np.isfinite(scale_array.item()):
            raise ValueError("bridge_scale must be finite")
        return np.full((joint_count,), float(scale_array.item()), dtype=np.float32)
    if scale_array.shape != (joint_count,):
        raise ValueError("bridge_scale 的长度必须与桥接关节数一致。")
    if not np.all(np.isfinite(scale_array)):
        raise ValueError("bridge_scale must contain only finite values")
    return scale_array.astype(np.float32, copy=True)


@dataclass
class MujocoJointDeltaBridgeState:
    sim_joint_indices: np.ndarray
    sim_start_positions: np.ndarray
    real_start_positions: np.ndarray
    previous_targets: np.ndarray
    bridge_scale: np.ndarray
    max_delta_radians: float | None = None
    max_step_radians: float | None = None

    def project_sim_joint_positions(self, sim_joint_positions: Sequence[float] | np.ndarray) -> np.ndarray:
        sim_positions = np.asarray(sim_joint_positions, dtype=np.float32)
        if sim_positions.ndim != 1:
            raise ValueError("sim_joint_positions must be a 1-D array")
        return sim_positions[self.sim_joint_indices].astype(np.float32, copy=True)

    def compute_target_positions(
        self,
        sim_joint_positions: Sequence[float] | np.ndarray,
    ) -> tuple[np.ndarray, np.ndarray]:
        projected_sim_positions = self.project_sim_joint_positions(sim_joint_positions)
        sim_delta = (projected_sim_positions - self.sim_start_positions) * self.bridge_scale
        if self.max_delta_radians is not None:
            sim_delta = np.clip(sim_delta, -self.max_delta_radians, self.max_delta_radians)

        unclipped_targets = self.real_start_positions + sim_delta
        if self.max_step_radians is None:
            targets = unclipped_targets
        else:
            step_delta = np.clip(
                unclipped_targets - self.previous_targets,
                -self.max_step_radians,
                self.max_step_radians,
            )
            targets = self.previous_targets + step_delta

        self.previous_targets[:] = targets
        return targets.astype(np.float32, copy=True), sim_delta.astype(np.float32, copy=True)


def create_mujoco_joint_delta_bridge(
    *,
    sim_joint_positions: Sequence[float] | np.ndarray,
    real_joint_positions: Sequence[float] | np.ndarray,
    sim_joint_indices: Sequence[int] | np.ndarray,
    bridge_scale: float | Sequence[float] = 1.0,
    max_delta_radians: float | None = None,
    max_step_radians: float | None = None,
) -> MujocoJointDeltaBridgeState:
    sim_positions = np.asarray(sim_joint_positions, dtype=np.float32)
    real_positions = np.asarray(real_joint_positions, dtype=np.float32)
    indices = np.asarray(sim_joint_indices, dtype=np.int64)
    if sim_positions.ndim != 1:
        raise ValueError("sim_joint_positions must be a 1-D array")
    if real_positions.ndim != 1:
        raise ValueError("real_joint_positions must be a 1-D array")
    if indices.ndim != 1:
        raise ValueError("sim_joint_indices must be a 1-D array")
    if real_positions.shape != indices.shape:
        raise ValueError("real_joint_positions 的长度必须与桥接关节数一致。")
    if np.any(indices < 0) or np.any(indices >= sim_positions.shape[0]):
        raise ValueError("sim_joint_indices 包含越界索引。")
    if max_delta_radians is not None and max_delta_radians <= 0.0:
        raise ValueError("max_delta_radians must be positive when provided")
    if max_step_radians is not None and max_step_radians <= 0.0:
        raise ValueError("max_step_radians must be positive when provided")

    scale_array = _coerce_bridge_scale(
        bridge_scale,
        joint_count=int(indices.shape[0]),
    )
    start_positions = sim_positions[indices].astype(np.float32, copy=True)
    return MujocoJointDeltaBridgeState(
        sim_joint_indices=indices.astype(np.int64, copy=True),
        sim_start_positions=start_positions,
        real_start_positions=real_positions.astype(np.float32, copy=True),
        previous_targets=real_positions.astype(np.float32, copy=True),
        bridge_scale=scale_array,
        max_delta_radians=max_delta_radians,
        max_step_radians=max_step_radians,
    )


def _as_numpy_array(values: object) -> np.ndarray:
    if isinstance(values, np.ndarray):
        return values.astype(np.float32, copy=False)
    if torch.is_tensor(values):
        return values.detach().cpu().numpy().astype(np.float32, copy=False)
    if hasattr(values, "numpy") and callable(values.numpy):
        return np.asarray(values.numpy(), dtype=np.float32)
    return np.asarray(values, dtype=np.float32)


def _read_mujoco_joint_positions(simulator: object) -> np.ndarray:
    get_joint_positions = getattr(simulator, "get_joint_positions", None)
    if callable(get_joint_positions):
        return _as_numpy_array(get_joint_positions())

    if hasattr(simulator, "_get_joint_pos") and callable(simulator._get_joint_pos):
        return _as_numpy_array(simulator._get_joint_pos())

    raise AttributeError("MuJoCo simulator does not expose joint position accessors")


def print_mujoco_joint_delta_bridge_debug_snapshot(
    *,
    step_index: int,
    projected_sim_positions: np.ndarray,
    sim_delta: np.ndarray,
    real_joint_positions: np.ndarray,
    target_positions: np.ndarray,
) -> None:
    tracking_error = target_positions - real_joint_positions
    print(f"[DEBUG][BRIDGE] step={step_index}")
    print("[DEBUG][BRIDGE] sim      =", _format_array(projected_sim_positions))
    print("[DEBUG][BRIDGE] delta    =", _format_array(sim_delta))
    print("[DEBUG][BRIDGE] measured =", _format_array(real_joint_positions))
    print("[DEBUG][BRIDGE] target   =", _format_array(target_positions))
    print("[DEBUG][BRIDGE] error    =", _format_array(tracking_error))


def run_observation_visualizer(configuration: object) -> None:
    from .mujoco import run_mujoco_visualization

    run_mujoco_visualization(configuration)


def run_mujoco_joint_position_bridge(
    configuration: object,
    *,
    left_leg_bus: str = "can0",
    right_leg_bus: str = "can1",
    dry_run: bool = False,
    debug: bool = False,
    debug_every: int = 25,
    bridge_scale: float | Sequence[float] = 1.0,
    max_delta_radians: float | None = np.deg2rad(30.0),
    max_step_radians: float | None = np.deg2rad(3.0),
    position_kp: float = 20.0,
    position_kd: float = 2.0,
    torque_limit: float = 4.0,
    policy_gate_degrees: float | None = None,
    enable_imu: bool = False,
) -> None:
    from berkeley_humanoid_lite.environments import MujocoSimulator
    from berkeley_humanoid_lite_lowlevel.policy import PolicyController
    from berkeley_humanoid_lite_lowlevel.robot.control_state import LocomotionControlState
    from berkeley_humanoid_lite_lowlevel.workflows.locomotion import create_locomotion_robot

    if debug_every <= 0:
        raise ValueError("debug_every must be positive")

    print(f"Policy frequency: {1 / configuration.policy_dt} Hz")
    if dry_run:
        print("Dry-run enabled: bridge targets will be computed without sending motor position commands")
    if debug:
        print(f"Bridge debug snapshots enabled every {debug_every} policy steps")
    print(f"Bridge scale: {_format_bridge_scale(bridge_scale)}")
    if max_delta_radians is not None:
        print(f"Max bridge delta: {np.rad2deg(max_delta_radians):.2f} deg")
    if max_step_radians is not None:
        print(f"Max bridge step: {np.rad2deg(max_step_radians):.2f} deg")
    if not dry_run:
        print(
            "Bridge position gains:",
            f"kp={position_kp:.2f}",
            f"kd={position_kd:.2f}",
            f"torque_limit={torque_limit:.2f}",
        )
    if policy_gate_degrees is not None:
        print(f"Bridge policy gate limit: {policy_gate_degrees:.2f} deg")

    simulator = None
    robot = None
    startup_completed = False
    try:
        simulator = MujocoSimulator(configuration)
        observations = simulator.reset()
        initial_sim_positions = _read_mujoco_joint_positions(simulator)

        controller = PolicyController(configuration)
        controller.load_policy()

        default_actions = np.asarray(
            configuration.default_joint_positions,
            dtype=np.float32,
        )[list(simulator.cfg.action_indices)]

        robot = create_locomotion_robot(
            left_leg_bus=left_leg_bus,
            right_leg_bus=right_leg_bus,
            enable_imu=enable_imu,
            enable_command_source=False,
            dry_run=dry_run,
            require_imu_ready=enable_imu,
        )
        if policy_gate_degrees is not None:
            robot._POLICY_ENTRY_GATE_MAX_ABS_DELTA_DEG = float(policy_gate_degrees)
        if not dry_run:
            robot.actuators.configure_damping_mode(
                position_kp=position_kp,
                position_kd=position_kd,
                torque_limit=torque_limit,
            )
        robot.reset()
        startup_completed = True

        sim_joint_indices = resolve_mujoco_bridge_joint_indices(
            configuration.joints,
            robot.specification.joint_names,
        )
        preparation_targets = initial_sim_positions[sim_joint_indices].astype(np.float32, copy=True)
        preparation_phase = "standing"
        bridge_state = None
        step_index = 0

        print("Bridge preparation: moving robot to standing pose")
        while True:
            if bridge_state is None:
                if preparation_phase == "standing":
                    robot.requested_state = LocomotionControlState.INITIALIZING
                    robot.step(preparation_targets)
                    if (
                        robot.state == LocomotionControlState.INITIALIZING
                        and float(robot.initialization_progress) >= 1.0
                    ):
                        preparation_phase = "policy_entry"
                        print("Bridge preparation: moving robot to policy-entry pose")
                    continue

                robot.requested_state = LocomotionControlState.POLICY_CONTROL
                robot.step(preparation_targets)
                if robot.state != LocomotionControlState.POLICY_CONTROL:
                    continue

                bridge_state = create_mujoco_joint_delta_bridge(
                    sim_joint_positions=initial_sim_positions,
                    real_joint_positions=robot.joint_position_measured,
                    sim_joint_indices=sim_joint_indices,
                    bridge_scale=bridge_scale,
                    max_delta_radians=max_delta_radians,
                    max_step_radians=max_step_radians,
                )
                print("Bridge armed: MuJoCo joint deltas now drive the real robot position targets")

            actions = controller.compute_actions(_as_numpy_array(observations))
            if actions is None:
                actions = default_actions
            observations = simulator.step(torch.as_tensor(actions, dtype=torch.float32))
            sim_joint_positions = _read_mujoco_joint_positions(simulator)
            target_positions, sim_delta = bridge_state.compute_target_positions(sim_joint_positions)
            robot.requested_state = LocomotionControlState.POLICY_CONTROL
            robot.step(target_positions)

            if debug and step_index % debug_every == 0:
                print_mujoco_joint_delta_bridge_debug_snapshot(
                    step_index=step_index,
                    projected_sim_positions=bridge_state.project_sim_joint_positions(sim_joint_positions),
                    sim_delta=sim_delta,
                    real_joint_positions=robot.joint_position_measured,
                    target_positions=target_positions,
                )
            step_index += 1
    except KeyboardInterrupt:
        print("Stopping MuJoCo joint delta bridge.")
    finally:
        if robot is not None:
            if startup_completed:
                robot.stop()
            else:
                robot.shutdown()
        if simulator is not None:
            simulator.close()
