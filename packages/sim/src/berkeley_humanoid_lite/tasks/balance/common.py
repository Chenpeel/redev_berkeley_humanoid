from __future__ import annotations

from dataclasses import dataclass

from isaaclab.managers import EventTermCfg as EventTerm
from isaaclab.managers import RewardTermCfg as RewTerm
from isaaclab.managers import SceneEntityCfg
from isaaclab.managers import TerminationTermCfg as DoneTerm
from isaaclab.utils import configclass

import berkeley_humanoid_lite.tasks.balance.mdp as mdp
from berkeley_humanoid_lite.tasks.locomotion.velocity.config.common import (
    CommandRange,
    JointDeviationPenalty,
)
from berkeley_humanoid_lite.tasks.locomotion.velocity.config.common import (
    build_actions_cfg as build_locomotion_actions_cfg,
)
from berkeley_humanoid_lite.tasks.locomotion.velocity.config.common import (
    build_observations_cfg as build_locomotion_observations_cfg,
)
from berkeley_humanoid_lite.tasks.locomotion.velocity.velocity_env_cfg import LocomotionVelocityEnvCfg


@dataclass(frozen=True)
class ResetProfile:
    """Reset distribution for standing and recovery tasks."""

    pose_range: dict[str, tuple[float, float]]
    velocity_range: dict[str, tuple[float, float]]
    joint_position_range: tuple[float, float]
    joint_velocity_range: tuple[float, float]


@dataclass(frozen=True)
class DisturbanceProfile:
    """Interval disturbance configuration for recovery tasks."""

    interval_range_s: tuple[float, float]
    velocity_range: dict[str, tuple[float, float]]


@dataclass(frozen=True)
class TerminationProfile:
    """Termination thresholds for balance tasks."""

    orientation_limit: float
    contact_force_threshold: float
    body_contact_patterns: tuple[str, ...]


@dataclass(frozen=True)
class BalanceRewardProfile:
    """Reward weights for standing and push-recovery tasks."""

    tracking_standard_deviation: float
    linear_velocity_tracking_weight: float
    angular_velocity_tracking_weight: float
    flat_orientation_weight: float
    action_rate_weight: float
    torque_weight: float
    acceleration_weight: float
    termination_penalty_weight: float
    undesired_contact_weight: float
    undesired_contact_body_patterns: tuple[str, ...]
    joint_deviation_penalties: tuple[JointDeviationPenalty, ...]
    feet_body_patterns: tuple[str, ...] = (".*_ankle_roll",)
    feet_slide_weight: float = 0.0
    feet_air_time_penalty_weight: float = 0.0


@dataclass(frozen=True)
class BalanceTaskProfile:
    """Shared profile for balance-oriented tasks."""

    class_name: str
    joint_names: tuple[str, ...]
    robot_cfg: object
    command_range: CommandRange
    reward_profile: BalanceRewardProfile
    reset_profile: ResetProfile
    termination_profile: TerminationProfile
    disturbance_profile: DisturbanceProfile | None = None
    episode_length_s: float = 20.0
    decimation: int = 8


def build_commands_cfg(profile: BalanceTaskProfile):
    """Build zero or near-zero velocity commands for standing tasks."""

    command_range = profile.command_range

    @configclass
    class CommandsCfg:
        """Command specifications for the balance MDP."""

        base_velocity = mdp.UniformVelocityCommandCfg(
            resampling_time_range=(profile.episode_length_s, profile.episode_length_s),
            debug_vis=True,
            asset_name="robot",
            heading_command=False,
            rel_standing_envs=1.0,
            ranges=mdp.UniformVelocityCommandCfg.Ranges(
                lin_vel_x=command_range.linear_velocity_x,
                lin_vel_y=command_range.linear_velocity_y,
                ang_vel_z=command_range.angular_velocity_z,
                heading=(0.0, 0.0),
            ),
        )

    CommandsCfg.__name__ = f"{profile.class_name}CommandsCfg"
    return CommandsCfg


def build_observations_cfg(profile: BalanceTaskProfile):
    """Reuse the existing locomotion observation layout for deployment compatibility."""

    return build_locomotion_observations_cfg(profile)


def build_actions_cfg(profile: BalanceTaskProfile):
    """Reuse the existing locomotion action layout."""

    return build_locomotion_actions_cfg(profile)


def build_rewards_cfg(profile: BalanceTaskProfile):
    """Build reward terms specialized for standing and push recovery."""

    reward_profile = profile.reward_profile
    joint_names = list(profile.joint_names)
    feet_body_names = list(reward_profile.feet_body_patterns)

    rewards_cfg = {
        "__doc__": "Reward terms for balance-oriented tasks.",
        "__module__": __name__,
        "track_lin_vel_xy_exp": RewTerm(
            func=mdp.track_lin_vel_xy_yaw_frame_exp,
            params={"command_name": "base_velocity", "std": reward_profile.tracking_standard_deviation},
            weight=reward_profile.linear_velocity_tracking_weight,
        ),
        "track_ang_vel_z_exp": RewTerm(
            func=mdp.track_ang_vel_z_world_exp,
            params={"command_name": "base_velocity", "std": reward_profile.tracking_standard_deviation},
            weight=reward_profile.angular_velocity_tracking_weight,
        ),
        "termination_penalty": RewTerm(func=mdp.is_terminated, weight=reward_profile.termination_penalty_weight),
        "lin_vel_z_l2": RewTerm(func=mdp.lin_vel_z_l2, weight=-0.2),
        "ang_vel_xy_l2": RewTerm(func=mdp.ang_vel_xy_l2, weight=-0.1),
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
        "undesired_contacts": RewTerm(
            func=mdp.undesired_contacts,
            params={
                "sensor_cfg": SceneEntityCfg(
                    "contact_forces",
                    body_names=list(reward_profile.undesired_contact_body_patterns),
                ),
                "threshold": 1.0,
            },
            weight=reward_profile.undesired_contact_weight,
        ),
    }

    if reward_profile.feet_slide_weight != 0.0:
        rewards_cfg["feet_slide"] = RewTerm(
            func=mdp.feet_slide,
            params={
                "sensor_cfg": SceneEntityCfg("contact_forces", body_names=feet_body_names),
                "asset_cfg": SceneEntityCfg("robot", body_names=feet_body_names),
            },
            weight=reward_profile.feet_slide_weight,
        )

    if reward_profile.feet_air_time_penalty_weight != 0.0:
        rewards_cfg["feet_air_time_penalty"] = RewTerm(
            func=mdp.feet_air_time_penalty,
            params={
                "sensor_cfg": SceneEntityCfg("contact_forces", body_names=feet_body_names),
            },
            weight=reward_profile.feet_air_time_penalty_weight,
        )

    for penalty in reward_profile.joint_deviation_penalties:
        rewards_cfg[penalty.attribute_name] = RewTerm(
            func=mdp.joint_deviation_l1,
            params={"asset_cfg": SceneEntityCfg("robot", joint_names=list(penalty.joint_patterns))},
            weight=penalty.weight,
        )

    rewards_cls = configclass(type(f"{profile.class_name}RewardsCfg", (), rewards_cfg))
    return rewards_cls


def build_terminations_cfg(profile: BalanceTaskProfile):
    """Build termination terms for balance tasks."""

    termination_profile = profile.termination_profile

    terminations_cfg = {
        "__doc__": "Termination terms for balance-oriented tasks.",
        "__module__": __name__,
        "time_out": DoneTerm(func=mdp.time_out, time_out=True),
        "base_orientation": DoneTerm(
            func=mdp.bad_orientation,
            params={
                "limit_angle": termination_profile.orientation_limit,
                "asset_cfg": SceneEntityCfg("robot", body_names="base"),
            },
        ),
        "body_contact": DoneTerm(
            func=mdp.body_contact,
            params={
                "sensor_cfg": SceneEntityCfg(
                    "contact_forces",
                    body_names=list(termination_profile.body_contact_patterns),
                ),
                "threshold": termination_profile.contact_force_threshold,
            },
        ),
    }

    terminations_cls = configclass(type(f"{profile.class_name}TerminationsCfg", (), terminations_cfg))
    return terminations_cls


def build_events_cfg(profile: BalanceTaskProfile):
    """Build reset/randomization events for balance tasks."""

    reset_profile = profile.reset_profile
    events_cfg = {
        "__doc__": "Configuration for balance-task events.",
        "__module__": __name__,
        "physics_material": EventTerm(
            func=mdp.randomize_rigid_body_material,
            params={
                "asset_cfg": SceneEntityCfg("robot", body_names=".*"),
                "static_friction_range": (0.4, 1.2),
                "dynamic_friction_range": (0.4, 1.2),
                "restitution_range": (0.0, 0.0),
                "num_buckets": 64,
            },
            mode="startup",
        ),
        "add_base_mass": EventTerm(
            func=mdp.randomize_rigid_body_mass,
            params={
                "asset_cfg": SceneEntityCfg("robot", body_names="base"),
                "mass_distribution_params": (-1.0, 2.0),
                "operation": "add",
            },
            mode="startup",
        ),
        "add_all_joint_default_pos": EventTerm(
            func=mdp.randomize_joint_default_pos,
            params={
                "asset_cfg": SceneEntityCfg("robot", joint_names=[".*"]),
                "pos_distribution_params": (-0.05, 0.05),
                "operation": "add",
            },
            mode="startup",
        ),
        "scale_all_actuator_torque_constant": EventTerm(
            func=mdp.randomize_actuator_gains,
            params={
                "asset_cfg": SceneEntityCfg("robot", joint_names=[".*"]),
                "stiffness_distribution_params": (0.8, 1.2),
                "damping_distribution_params": (0.8, 1.2),
                "operation": "scale",
            },
            mode="startup",
        ),
        "reset_base": EventTerm(
            func=mdp.reset_root_state_uniform,
            params={
                "pose_range": reset_profile.pose_range,
                "velocity_range": reset_profile.velocity_range,
            },
            mode="reset",
        ),
        "reset_robot_joints": EventTerm(
            func=mdp.reset_joints_by_scale,
            params={
                "position_range": reset_profile.joint_position_range,
                "velocity_range": reset_profile.joint_velocity_range,
            },
            mode="reset",
        ),
    }

    if profile.disturbance_profile is not None:
        events_cfg["push_robot"] = EventTerm(
            func=mdp.push_by_setting_velocity,
            params={"velocity_range": profile.disturbance_profile.velocity_range},
            mode="interval",
            interval_range_s=profile.disturbance_profile.interval_range_s,
        )

    events_cls = configclass(type(f"{profile.class_name}EventsCfg", (), events_cfg))
    return events_cls


def build_curriculums_cfg(profile: BalanceTaskProfile):
    """Build an empty curriculum section so later task-specific schedules can plug in cleanly."""

    curriculums_cfg = {
        "__doc__": "Curriculum terms for balance-oriented tasks.",
        "__module__": __name__,
    }
    curriculums_cls = configclass(type(f"{profile.class_name}CurriculumsCfg", (), curriculums_cfg))
    return curriculums_cls


def build_environment_cfg(
    class_name: str,
    profile: BalanceTaskProfile,
    commands_cfg_cls,
    observations_cfg_cls,
    actions_cfg_cls,
    rewards_cfg_cls,
    terminations_cfg_cls,
    events_cfg_cls,
    curriculums_cfg_cls,
):
    """Build the final environment config class."""

    @configclass
    class BalanceTaskEnvCfg(LocomotionVelocityEnvCfg):
        commands: commands_cfg_cls = commands_cfg_cls()
        observations: observations_cfg_cls = observations_cfg_cls()
        actions: actions_cfg_cls = actions_cfg_cls()
        rewards: rewards_cfg_cls = rewards_cfg_cls()
        terminations: terminations_cfg_cls = terminations_cfg_cls()
        events: events_cfg_cls = events_cfg_cls()
        curriculums: curriculums_cfg_cls = curriculums_cfg_cls()

        def __post_init__(self):
            super().__post_init__()
            self.decimation = profile.decimation
            self.episode_length_s = profile.episode_length_s
            self.sim.render_interval = self.decimation
            self.scene.robot = profile.robot_cfg.replace(prim_path="{ENV_REGEX_NS}/robot")

    BalanceTaskEnvCfg.__name__ = class_name
    return BalanceTaskEnvCfg
