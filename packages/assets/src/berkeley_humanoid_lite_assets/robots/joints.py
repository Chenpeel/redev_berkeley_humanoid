"""机器人关节名称与默认姿态定义。"""

from __future__ import annotations


LEG_JOINT_NAMES: tuple[str, ...] = (
    "leg_left_hip_roll_joint",
    "leg_left_hip_yaw_joint",
    "leg_left_hip_pitch_joint",
    "leg_left_knee_pitch_joint",
    "leg_left_ankle_pitch_joint",
    "leg_left_ankle_roll_joint",
    "leg_right_hip_roll_joint",
    "leg_right_hip_yaw_joint",
    "leg_right_hip_pitch_joint",
    "leg_right_knee_pitch_joint",
    "leg_right_ankle_pitch_joint",
    "leg_right_ankle_roll_joint",
)

ARM_JOINT_NAMES: tuple[str, ...] = (
    "arm_left_shoulder_pitch_joint",
    "arm_left_shoulder_roll_joint",
    "arm_left_shoulder_yaw_joint",
    "arm_left_elbow_pitch_joint",
    "arm_left_elbow_roll_joint",
    "arm_right_shoulder_pitch_joint",
    "arm_right_shoulder_roll_joint",
    "arm_right_shoulder_yaw_joint",
    "arm_right_elbow_pitch_joint",
    "arm_right_elbow_roll_joint",
)

FULL_BODY_JOINT_NAMES = ARM_JOINT_NAMES + LEG_JOINT_NAMES

LEG_DEFAULT_JOINT_POSITIONS = {
    "leg_left_hip_roll_joint": 0.0,
    "leg_left_hip_yaw_joint": 0.0,
    "leg_left_hip_pitch_joint": -0.2,
    "leg_left_knee_pitch_joint": 0.4,
    "leg_left_ankle_pitch_joint": -0.3,
    "leg_left_ankle_roll_joint": 0.0,
    "leg_right_hip_roll_joint": 0.0,
    "leg_right_hip_yaw_joint": 0.0,
    "leg_right_hip_pitch_joint": -0.2,
    "leg_right_knee_pitch_joint": 0.4,
    "leg_right_ankle_pitch_joint": -0.3,
    "leg_right_ankle_roll_joint": 0.0,
}

ARM_DEFAULT_JOINT_POSITIONS = {
    "arm_left_shoulder_pitch_joint": 0.0,
    "arm_left_shoulder_roll_joint": 0.0,
    "arm_left_shoulder_yaw_joint": 0.0,
    "arm_left_elbow_pitch_joint": 0.0,
    "arm_left_elbow_roll_joint": 0.0,
    "arm_right_shoulder_pitch_joint": 0.0,
    "arm_right_shoulder_roll_joint": 0.0,
    "arm_right_shoulder_yaw_joint": 0.0,
    "arm_right_elbow_pitch_joint": 0.0,
    "arm_right_elbow_roll_joint": 0.0,
}

FULL_BODY_DEFAULT_JOINT_POSITIONS = ARM_DEFAULT_JOINT_POSITIONS | LEG_DEFAULT_JOINT_POSITIONS
