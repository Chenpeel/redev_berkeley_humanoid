"""机器人变体与执行器分组定义。"""

from __future__ import annotations

from dataclasses import dataclass

from .joints import (
    FULL_BODY_DEFAULT_JOINT_POSITIONS,
    FULL_BODY_JOINT_NAMES,
    LEG_DEFAULT_JOINT_POSITIONS,
    LEG_JOINT_NAMES,
)


@dataclass(frozen=True)
class ActuatorGroupSpecification:
    """执行器分组的标准化定义。"""

    name: str
    joint_name_patterns: tuple[str, ...]
    effort_limit: float
    velocity_limit: float
    stiffness: float
    damping: float
    armature: float


@dataclass(frozen=True)
class RobotVariantSpecification:
    """机器人变体的标准化定义。"""

    name: str
    usd_file_name: str
    mjcf_scene_file_name: str
    joint_names: tuple[str, ...]
    initial_joint_positions: dict[str, float]
    actuator_groups: tuple[ActuatorGroupSpecification, ...]
    soft_joint_position_limit_factor: float = 0.9

    @property
    def joint_count(self) -> int:
        return len(self.joint_names)


LEG_ACTUATOR_GROUPS = (
    ActuatorGroupSpecification(
        name="legs",
        joint_name_patterns=(
            "leg_.*_hip_yaw_joint",
            "leg_.*_hip_roll_joint",
            "leg_.*_hip_pitch_joint",
            "leg_.*_knee_pitch_joint",
        ),
        effort_limit=6.0,
        velocity_limit=10.0,
        stiffness=20.0,
        damping=2.0,
        armature=0.007,
    ),
    ActuatorGroupSpecification(
        name="ankles",
        joint_name_patterns=(
            "leg_.*_ankle_pitch_joint",
            "leg_.*_ankle_roll_joint",
        ),
        effort_limit=6.0,
        velocity_limit=10.0,
        stiffness=20.0,
        damping=2.0,
        armature=0.002,
    ),
)

FULL_BODY_ACTUATOR_GROUPS = (
    ActuatorGroupSpecification(
        name="arms",
        joint_name_patterns=(
            "arm_.*_shoulder_pitch_joint",
            "arm_.*_shoulder_roll_joint",
            "arm_.*_shoulder_yaw_joint",
            "arm_.*_elbow_pitch_joint",
            "arm_.*_elbow_roll_joint",
        ),
        effort_limit=4.0,
        velocity_limit=10.0,
        stiffness=10.0,
        damping=2.0,
        armature=0.002,
    ),
) + LEG_ACTUATOR_GROUPS

BIPED_VARIANT = RobotVariantSpecification(
    name="biped",
    usd_file_name="berkeley_humanoid_lite_biped.usd",
    mjcf_scene_file_name="bhl_biped_scene.xml",
    joint_names=tuple(LEG_JOINT_NAMES),
    initial_joint_positions=dict(LEG_DEFAULT_JOINT_POSITIONS),
    actuator_groups=LEG_ACTUATOR_GROUPS,
)

FULL_BODY_VARIANT = RobotVariantSpecification(
    name="full_body",
    usd_file_name="berkeley_humanoid_lite.usd",
    mjcf_scene_file_name="bhl_scene.xml",
    joint_names=tuple(FULL_BODY_JOINT_NAMES),
    initial_joint_positions=dict(FULL_BODY_DEFAULT_JOINT_POSITIONS),
    actuator_groups=FULL_BODY_ACTUATOR_GROUPS,
)

ROBOT_VARIANTS = {
    BIPED_VARIANT.name: BIPED_VARIANT,
    FULL_BODY_VARIANT.name: FULL_BODY_VARIANT,
}


def get_variant(name: str) -> RobotVariantSpecification:
    """按标准名称获取机器人变体。"""

    try:
        return ROBOT_VARIANTS[name]
    except KeyError as error:
        available_names = ", ".join(sorted(ROBOT_VARIANTS))
        raise KeyError(f"未知机器人变体: {name}. 可选值: {available_names}") from error


def get_variant_for_joint_count(joint_count: int) -> RobotVariantSpecification:
    """按关节数量推断机器人变体。"""

    for variant in ROBOT_VARIANTS.values():
        if variant.joint_count == joint_count:
            return variant
    raise ValueError(f"未找到 joint_count={joint_count} 对应的机器人变体")
