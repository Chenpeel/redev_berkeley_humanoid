from __future__ import annotations

import math
from dataclasses import dataclass

from isaaclab.managers import EventTermCfg as EventTerm
from isaaclab.managers import ObservationGroupCfg as ObsGroup
from isaaclab.managers import ObservationTermCfg as ObsTerm
from isaaclab.managers import RewardTermCfg as RewTerm
from isaaclab.managers import SceneEntityCfg
from isaaclab.managers import TerminationTermCfg as DoneTerm
from isaaclab.utils import configclass
from isaaclab.utils.noise import AdditiveUniformNoiseCfg as Unoise

import berkeley_humanoid_lite.tasks.locomotion.velocity.mdp as mdp
from berkeley_humanoid_lite.tasks.locomotion.velocity.velocity_env_cfg import LocomotionVelocityEnvCfg


@dataclass(frozen=True)
class CommandRange:
    """速度指令采样范围。"""

    linear_velocity_x: tuple[float, float]
    linear_velocity_y: tuple[float, float]
    angular_velocity_z: tuple[float, float]


@dataclass(frozen=True)
class JointDeviationPenalty:
    """关节偏离默认姿态的惩罚配置。"""

    attribute_name: str
    joint_patterns: tuple[str, ...]
    weight: float


@dataclass(frozen=True)
class RewardProfile:
    """速度任务奖励超参数。"""

    tracking_standard_deviation: float
    flat_orientation_weight: float
    action_rate_weight: float
    torque_weight: float
    acceleration_weight: float
    feet_air_time_threshold: float
    feet_air_time_weight: float
    undesired_contact_body_patterns: tuple[str, ...]
    joint_deviation_penalties: tuple[JointDeviationPenalty, ...]


@dataclass(frozen=True)
class VelocityTaskProfile:
    """某个速度任务变体的公共配置。"""

    class_name: str
    joint_names: tuple[str, ...]
    robot_cfg: object
    command_range: CommandRange
    reward_profile: RewardProfile


def build_commands_cfg(profile: VelocityTaskProfile):
    """构建指令配置类。"""

    command_range = profile.command_range

    @configclass
    class CommandsCfg:
        """Command specifications for the MDP."""

        base_velocity = mdp.UniformVelocityCommandCfg(
            resampling_time_range=(10.0, 10.0),
            debug_vis=True,
            asset_name="robot",
            heading_command=True,
            heading_control_stiffness=0.5,
            rel_standing_envs=0.02,
            rel_heading_envs=1.0,
            ranges=mdp.UniformVelocityCommandCfg.Ranges(
                lin_vel_x=command_range.linear_velocity_x,
                lin_vel_y=command_range.linear_velocity_y,
                ang_vel_z=command_range.angular_velocity_z,
                heading=(-math.pi, math.pi),
            ),
        )

    CommandsCfg.__name__ = f"{profile.class_name}CommandsCfg"
    return CommandsCfg


def build_observations_cfg(profile: VelocityTaskProfile):
    """构建观测配置类。"""

    joint_names = list(profile.joint_names)

    @configclass
    class PolicyCfg(ObsGroup):
        """Observations for policy group."""

        velocity_commands = ObsTerm(func=mdp.generated_commands, params={"command_name": "base_velocity"})
        base_ang_vel = ObsTerm(func=mdp.base_ang_vel, noise=Unoise(n_min=-0.3, n_max=0.3))
        projected_gravity = ObsTerm(func=mdp.projected_gravity, noise=Unoise(n_min=-0.05, n_max=0.05))
        joint_pos = ObsTerm(
            func=mdp.joint_pos_rel,
            params={"asset_cfg": SceneEntityCfg("robot", joint_names=joint_names, preserve_order=True)},
            noise=Unoise(n_min=-0.05, n_max=0.05),
        )
        joint_vel = ObsTerm(
            func=mdp.joint_vel_rel,
            params={"asset_cfg": SceneEntityCfg("robot", joint_names=joint_names, preserve_order=True)},
            noise=Unoise(n_min=-2.0, n_max=2.0),
        )
        actions = ObsTerm(func=mdp.last_action)

        def __post_init__(self):
            self.enable_corruption = True

    @configclass
    class CriticCfg(PolicyCfg):
        """Observations for critic group."""

        base_lin_vel = ObsTerm(func=mdp.base_lin_vel)

        def __post_init__(self):
            self.enable_corruption = False

    @configclass
    class ObservationsCfg:
        """Observation specifications for the MDP."""

        policy: PolicyCfg = PolicyCfg()
        critic: CriticCfg = CriticCfg()

    PolicyCfg.__name__ = f"{profile.class_name}PolicyObservationsCfg"
    CriticCfg.__name__ = f"{profile.class_name}CriticObservationsCfg"
    ObservationsCfg.__name__ = f"{profile.class_name}ObservationsCfg"
    return ObservationsCfg


def build_actions_cfg(profile: VelocityTaskProfile):
    """构建动作配置类。"""

    joint_names = list(profile.joint_names)

    @configclass
    class ActionsCfg:
        """Action specifications for the MDP."""

        joint_pos = mdp.JointPositionActionCfg(
            asset_name="robot",
            joint_names=joint_names,
            scale=0.25,
            preserve_order=True,
            use_default_offset=True,
        )

    ActionsCfg.__name__ = f"{profile.class_name}ActionsCfg"
    return ActionsCfg


def build_rewards_cfg(profile: VelocityTaskProfile):
    """构建奖励配置类。"""

    reward_profile = profile.reward_profile
    joint_names = list(profile.joint_names)

    rewards_cfg = {
        "__doc__": "Reward terms for the MDP.",
        "__module__": __name__,
        "track_lin_vel_xy_exp": RewTerm(
            func=mdp.track_lin_vel_xy_yaw_frame_exp,
            params={"command_name": "base_velocity", "std": reward_profile.tracking_standard_deviation},
            weight=2.0,
        ),
        "track_ang_vel_z_exp": RewTerm(
            func=mdp.track_ang_vel_z_world_exp,
            params={"command_name": "base_velocity", "std": reward_profile.tracking_standard_deviation},
            weight=1.0,
        ),
        "termination_penalty": RewTerm(func=mdp.is_terminated, weight=-10.0),
        "lin_vel_z_l2": RewTerm(func=mdp.lin_vel_z_l2, weight=-0.1),
        "ang_vel_xy_l2": RewTerm(func=mdp.ang_vel_xy_l2, weight=-0.05),
        "flat_orientation_l2": RewTerm(func=mdp.flat_orientation_l2, weight=reward_profile.flat_orientation_weight),
        "action_rate_l2": RewTerm(func=mdp.action_rate_l2, weight=reward_profile.action_rate_weight),
        "dof_torques_l2": RewTerm(
            func=mdp.joint_torques_l2,
            params={"asset_cfg": SceneEntityCfg("robot", joint_names=joint_names)},
            weight=reward_profile.torque_weight,
        ),
        "dof_acc_l2": RewTerm(
            func=mdp.joint_acc_l2,
            params={"asset_cfg": SceneEntityCfg("robot", joint_names=joint_names)},
            weight=reward_profile.acceleration_weight,
        ),
        "dof_pos_limits": RewTerm(func=mdp.joint_pos_limits, weight=-1.0),
        "feet_air_time": RewTerm(
            func=mdp.feet_air_time_positive_biped,
            params={
                "command_name": "base_velocity",
                "sensor_cfg": SceneEntityCfg("contact_forces", body_names=".*_ankle_roll"),
                "threshold": reward_profile.feet_air_time_threshold,
            },
            weight=reward_profile.feet_air_time_weight,
        ),
        "feet_slide": RewTerm(
            func=mdp.feet_slide,
            params={
                "sensor_cfg": SceneEntityCfg("contact_forces", body_names=".*_ankle_roll"),
                "asset_cfg": SceneEntityCfg("robot", body_names=".*_ankle_roll"),
            },
            weight=-0.1,
        ),
        "undesired_contacts": RewTerm(
            func=mdp.undesired_contacts,
            params={
                "sensor_cfg": SceneEntityCfg(
                    "contact_forces",
                    body_names=list(reward_profile.undesired_contact_body_patterns),
                ),
                "threshold": 1.0,
            },
            weight=-1.0,
        ),
    }

    for penalty in reward_profile.joint_deviation_penalties:
        rewards_cfg[penalty.attribute_name] = RewTerm(
            func=mdp.joint_deviation_l1,
            params={"asset_cfg": SceneEntityCfg("robot", joint_names=list(penalty.joint_patterns))},
            weight=penalty.weight,
        )

    rewards_cls = configclass(type(f"{profile.class_name}RewardsCfg", (), rewards_cfg))
    return rewards_cls


@configclass
class TerminationsCfg:
    """Termination terms for the MDP."""

    time_out = DoneTerm(func=mdp.time_out, time_out=True)
    base_orientation = DoneTerm(
        func=mdp.bad_orientation,
        params={"limit_angle": 0.78, "asset_cfg": SceneEntityCfg("robot", body_names="base")},
    )


@configclass
class EventsCfg:
    """Configuration for events."""

    physics_material = EventTerm(
        func=mdp.randomize_rigid_body_material,
        params={
            "asset_cfg": SceneEntityCfg("robot", body_names=".*"),
            "static_friction_range": (0.4, 1.2),
            "dynamic_friction_range": (0.4, 1.2),
            "restitution_range": (0.0, 0.0),
            "num_buckets": 64,
        },
        mode="startup",
    )
    add_base_mass = EventTerm(
        func=mdp.randomize_rigid_body_mass,
        params={
            "asset_cfg": SceneEntityCfg("robot", body_names="base"),
            "mass_distribution_params": (-1.0, 2.0),
            "operation": "add",
        },
        mode="startup",
    )
    add_all_joint_default_pos = EventTerm(
        func=mdp.randomize_joint_default_pos,
        params={
            "asset_cfg": SceneEntityCfg("robot", joint_names=[".*"]),
            "pos_distribution_params": (-0.05, 0.05),
            "operation": "add",
        },
        mode="startup",
    )
    scale_all_actuator_torque_constant = EventTerm(
        func=mdp.randomize_actuator_gains,
        params={
            "asset_cfg": SceneEntityCfg("robot", joint_names=[".*"]),
            "stiffness_distribution_params": (0.8, 1.2),
            "damping_distribution_params": (0.8, 1.2),
            "operation": "scale",
        },
        mode="startup",
    )
    reset_base = EventTerm(
        func=mdp.reset_root_state_uniform,
        params={
            "pose_range": {"x": (-0.5, 0.5), "y": (-0.5, 0.5), "yaw": (-3.14, 3.14)},
            "velocity_range": {
                "x": (-0.5, 0.5),
                "y": (-0.5, 0.5),
                "z": (0.0, 0.0),
                "roll": (-0.5, 0.5),
                "pitch": (-0.5, 0.5),
                "yaw": (-0.5, 0.5),
            },
        },
        mode="reset",
    )
    reset_robot_joints = EventTerm(
        func=mdp.reset_joints_by_scale,
        params={"position_range": (0.5, 1.5), "velocity_range": (0.0, 0.0)},
        mode="reset",
    )
    base_external_force_torque = EventTerm(
        func=mdp.apply_external_force_torque,
        params={
            "asset_cfg": SceneEntityCfg("robot", body_names="base"),
            "force_range": (-2.0, 2.0),
            "torque_range": (-2.0, 2.0),
        },
        mode="reset",
    )


@configclass
class CurriculumsCfg:
    """Curriculum terms for the MDP."""

    pass


def build_environment_cfg(
    class_name: str,
    profile: VelocityTaskProfile,
    commands_cfg_cls,
    observations_cfg_cls,
    actions_cfg_cls,
    rewards_cfg_cls,
):
    """构建环境配置类。"""

    @configclass
    class VelocityTaskEnvCfg(LocomotionVelocityEnvCfg):
        commands: commands_cfg_cls = commands_cfg_cls()
        observations: observations_cfg_cls = observations_cfg_cls()
        actions: actions_cfg_cls = actions_cfg_cls()
        rewards: rewards_cfg_cls = rewards_cfg_cls()
        terminations: TerminationsCfg = TerminationsCfg()
        events: EventsCfg = EventsCfg()
        curriculums: CurriculumsCfg = CurriculumsCfg()

        def __post_init__(self):
            super().__post_init__()
            self.decimation = 8
            self.sim.render_interval = self.decimation
            self.scene.robot = profile.robot_cfg.replace(prim_path="{ENV_REGEX_NS}/robot")

    VelocityTaskEnvCfg.__name__ = class_name
    return VelocityTaskEnvCfg
