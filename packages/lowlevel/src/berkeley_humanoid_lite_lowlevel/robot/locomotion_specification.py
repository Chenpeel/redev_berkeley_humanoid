from __future__ import annotations

from dataclasses import dataclass
from typing import Literal

import numpy as np


DEFAULT_IMU_BAUDRATE = 0x08
CalibrationLimitSelector = Literal["min", "max"]


@dataclass(frozen=True)
class JointTransportAddress:
    bus_name: str
    device_id: int
    joint_name: str


@dataclass(frozen=True)
class LocomotionRobotSpecification:
    joint_addresses: tuple[JointTransportAddress, ...]
    mirrored_joint_pairs: tuple[tuple[int, int], ...]
    joint_axis_directions: np.ndarray
    initialization_positions: np.ndarray
    calibration_reference_positions: np.ndarray
    calibration_limit_selectors: tuple[CalibrationLimitSelector, ...]
    imu_baudrate: int = DEFAULT_IMU_BAUDRATE

    def __post_init__(self) -> None:
        joint_count = len(self.joint_addresses)
        if self.joint_axis_directions.shape != (joint_count,):
            raise ValueError("joint_axis_directions 的长度必须与关节数一致。")
        if self.initialization_positions.shape != (joint_count,):
            raise ValueError("initialization_positions 的长度必须与关节数一致。")
        if self.calibration_reference_positions.shape != (joint_count,):
            raise ValueError("calibration_reference_positions 的长度必须与关节数一致。")
        if len(self.calibration_limit_selectors) != joint_count:
            raise ValueError("calibration_limit_selectors 的长度必须与关节数一致。")

    @property
    def joint_count(self) -> int:
        return len(self.joint_addresses)

    @property
    def observation_size(self) -> int:
        return 4 + 3 + self.joint_count + self.joint_count + 1 + 3

    @property
    def joint_names(self) -> tuple[str, ...]:
        return tuple(address.joint_name for address in self.joint_addresses)


def build_default_locomotion_robot_specification() -> LocomotionRobotSpecification:
    return LocomotionRobotSpecification(
        joint_addresses=(
            JointTransportAddress("can0", 1, "left_hip_roll_joint"),
            JointTransportAddress("can0", 3, "left_hip_yaw_joint"),
            JointTransportAddress("can0", 5, "left_hip_pitch_joint"),
            JointTransportAddress("can0", 7, "left_knee_pitch_joint"),
            JointTransportAddress("can0", 11, "left_ankle_pitch_joint"),
            JointTransportAddress("can0", 13, "left_ankle_roll_joint"),
            JointTransportAddress("can1", 2, "right_hip_roll_joint"),
            JointTransportAddress("can1", 4, "right_hip_yaw_joint"),
            JointTransportAddress("can1", 6, "right_hip_pitch_joint"),
            JointTransportAddress("can1", 8, "right_knee_pitch_joint"),
            JointTransportAddress("can1", 12, "right_ankle_pitch_joint"),
            JointTransportAddress("can1", 14, "right_ankle_roll_joint"),
        ),
        mirrored_joint_pairs=((0, 6), (1, 7), (2, 8), (3, 9), (4, 10), (5, 11)),
        joint_axis_directions=np.array(
            [-1, 1, -1, -1, -1, 1, -1, 1, 1, 1, 1, 1],
            dtype=np.float32,
        ),
        initialization_positions=np.array(
            [
                0.0,
                0.0,
                -0.2,
                0.4,
                -0.3,
                0.0,
                0.0,
                0.0,
                -0.2,
                0.4,
                -0.3,
                0.0,
            ],
            dtype=np.float32,
        ),
        calibration_reference_positions=np.array(
            [
                np.deg2rad(-10.0),
                np.deg2rad(33.75),
                np.deg2rad(56.25),
                np.deg2rad(0.0),
                np.deg2rad(-45.0),
                np.deg2rad(-15.0),
                np.deg2rad(10.0),
                np.deg2rad(-33.75),
                np.deg2rad(56.25),
                np.deg2rad(0.0),
                np.deg2rad(-45.0),
                np.deg2rad(15.0),
            ],
            dtype=np.float32,
        ),
        calibration_limit_selectors=(
            "min",
            "max",
            "max",
            "min",
            "min",
            "min",
            "max",
            "min",
            "max",
            "min",
            "min",
            "max",
        ),
    )
