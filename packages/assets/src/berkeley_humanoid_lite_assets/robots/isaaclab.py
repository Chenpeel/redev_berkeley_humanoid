"""Isaac Lab 机器人配置构建函数。"""

from __future__ import annotations

import isaaclab.sim as sim_utils
from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.assets.articulation import ArticulationCfg

from berkeley_humanoid_lite_assets.paths import get_usd_path

from .variants import BIPED_VARIANT, FULL_BODY_VARIANT, RobotVariantSpecification


def _build_actuator_group(
    actuator_group,
) -> ImplicitActuatorCfg:
    return ImplicitActuatorCfg(
        joint_names_expr=list(actuator_group.joint_name_patterns),
        effort_limit_sim=actuator_group.effort_limit,
        velocity_limit_sim=actuator_group.velocity_limit,
        stiffness=actuator_group.stiffness,
        damping=actuator_group.damping,
        armature=actuator_group.armature,
    )


def build_articulation_cfg(robot_variant: RobotVariantSpecification) -> ArticulationCfg:
    """根据标准化变体定义生成 Isaac Lab 配置。"""

    return ArticulationCfg(
        spawn=sim_utils.UsdFileCfg(
            usd_path=str(get_usd_path(robot_variant.usd_file_name)),
            activate_contact_sensors=True,
            rigid_props=sim_utils.RigidBodyPropertiesCfg(
                disable_gravity=False,
                retain_accelerations=False,
                linear_damping=0.0,
                angular_damping=0.0,
                max_linear_velocity=1000.0,
                max_angular_velocity=1000.0,
                max_depenetration_velocity=1.0,
            ),
            articulation_props=sim_utils.ArticulationRootPropertiesCfg(
                enabled_self_collisions=False,
                solver_position_iteration_count=8,
                solver_velocity_iteration_count=4,
            ),
        ),
        init_state=ArticulationCfg.InitialStateCfg(
            pos=(0.0, 0.0, 0.0),
            joint_pos=dict(robot_variant.initial_joint_positions),
            joint_vel={".*": 0.0},
        ),
        soft_joint_pos_limit_factor=robot_variant.soft_joint_position_limit_factor,
        actuators={
            actuator_group.name: _build_actuator_group(actuator_group)
            for actuator_group in robot_variant.actuator_groups
        },
    )


BIPED_ARTICULATION_CFG = build_articulation_cfg(BIPED_VARIANT)
FULL_BODY_ARTICULATION_CFG = build_articulation_cfg(FULL_BODY_VARIANT)
