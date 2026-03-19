"""机器人规格与 Isaac Lab 配置导出。"""

from .joints import (
    ARM_DEFAULT_JOINT_POSITIONS,
    ARM_JOINT_NAMES,
    FULL_BODY_DEFAULT_JOINT_POSITIONS,
    FULL_BODY_JOINT_NAMES,
    LEG_DEFAULT_JOINT_POSITIONS,
    LEG_JOINT_NAMES,
)
from .variants import (
    BIPED_VARIANT,
    FULL_BODY_ACTUATOR_GROUPS,
    FULL_BODY_VARIANT,
    LEG_ACTUATOR_GROUPS,
    ROBOT_VARIANTS,
    ActuatorGroupSpecification,
    RobotVariantSpecification,
    get_variant,
    get_variant_for_joint_count,
)

__all__ = [
    "ARM_DEFAULT_JOINT_POSITIONS",
    "ARM_JOINT_NAMES",
    "ActuatorGroupSpecification",
    "BIPED_VARIANT",
    "FULL_BODY_ACTUATOR_GROUPS",
    "FULL_BODY_DEFAULT_JOINT_POSITIONS",
    "FULL_BODY_JOINT_NAMES",
    "FULL_BODY_VARIANT",
    "LEG_ACTUATOR_GROUPS",
    "LEG_DEFAULT_JOINT_POSITIONS",
    "LEG_JOINT_NAMES",
    "ROBOT_VARIANTS",
    "RobotVariantSpecification",
    "get_variant",
    "get_variant_for_joint_count",
    "build_articulation_cfg",
    "BIPED_ARTICULATION_CFG",
    "FULL_BODY_ARTICULATION_CFG",
]


def __getattr__(name: str):
    if name in {
        "build_articulation_cfg",
        "BIPED_ARTICULATION_CFG",
        "FULL_BODY_ARTICULATION_CFG",
    }:
        from .isaaclab import (
            BIPED_ARTICULATION_CFG,
            FULL_BODY_ARTICULATION_CFG,
            build_articulation_cfg,
        )

        exports = {
            "build_articulation_cfg": build_articulation_cfg,
            "BIPED_ARTICULATION_CFG": BIPED_ARTICULATION_CFG,
            "FULL_BODY_ARTICULATION_CFG": FULL_BODY_ARTICULATION_CFG,
        }
        return exports[name]

    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
