from __future__ import annotations

from collections.abc import Sequence
from dataclasses import dataclass

import torch
from berkeley_humanoid_lite_lowlevel.robot.control_state import LocomotionControlState

from .observations import CommandObservationState, SensorLayout
from .runtime import compute_physics_substeps


@dataclass(frozen=True)
class JointControlParameters:
    joint_kp: torch.Tensor
    joint_kd: torch.Tensor
    effort_limits: torch.Tensor


@dataclass(frozen=True)
class SimulatorInitialization:
    physics_substeps: int
    sensor_layout: SensorLayout
    control_parameters: JointControlParameters
    command_state: CommandObservationState


def build_joint_parameter_tensor(
    values: Sequence[float] | torch.Tensor,
    *,
    num_joints: int,
    parameter_name: str,
) -> torch.Tensor:
    """把部署配置里的关节参数转换成定长 float32 tensor。"""
    tensor = torch.as_tensor(values, dtype=torch.float32)
    expected_shape = (num_joints,)
    if tensor.shape != expected_shape:
        raise ValueError(f"{parameter_name} 长度必须为 {num_joints}，实际为 {tuple(tensor.shape)}。")
    return tensor.clone()


def build_joint_control_parameters(configuration: object) -> JointControlParameters:
    """从部署配置构建 PD 控制参数。"""
    num_joints = int(configuration.num_joints)
    return JointControlParameters(
        joint_kp=build_joint_parameter_tensor(
            configuration.joint_kp,
            num_joints=num_joints,
            parameter_name="joint_kp",
        ),
        joint_kd=build_joint_parameter_tensor(
            configuration.joint_kd,
            num_joints=num_joints,
            parameter_name="joint_kd",
        ),
        effort_limits=build_joint_parameter_tensor(
            configuration.effort_limits,
            num_joints=num_joints,
            parameter_name="effort_limits",
        ),
    )


def build_sensor_layout(
    *,
    num_joints: int,
    num_actuators: int,
) -> SensorLayout:
    """根据 MuJoCo actuator 数量构建传感器布局。"""
    return SensorLayout(
        num_joints=num_joints,
        base_sensor_offset=3 * num_actuators,
    )


def create_default_command_state(
    *,
    default_mode: float = float(LocomotionControlState.POLICY_CONTROL),
) -> CommandObservationState:
    """创建 simulator 初始命令状态。"""
    return CommandObservationState(
        mode=default_mode,
        velocity_x=0.0,
        velocity_y=0.0,
        velocity_yaw=0.0,
    )


def build_simulator_initialization(
    configuration: object,
    *,
    num_actuators: int,
) -> SimulatorInitialization:
    """汇总 simulator 启动阶段需要的纯配置对象。"""
    return SimulatorInitialization(
        physics_substeps=compute_physics_substeps(configuration.policy_dt, configuration.physics_dt),
        sensor_layout=build_sensor_layout(
            num_joints=int(configuration.num_joints),
            num_actuators=num_actuators,
        ),
        control_parameters=build_joint_control_parameters(configuration),
        command_state=create_default_command_state(),
    )
