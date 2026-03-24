from __future__ import annotations

from dataclasses import dataclass

from isaaclab.managers import EventTermCfg as EventTerm
from isaaclab.managers import ObservationGroupCfg as ObsGroup
from isaaclab.managers import ObservationTermCfg as ObsTerm
from isaaclab.managers import RewardTermCfg as RewTerm
from isaaclab.managers import SceneEntityCfg
from isaaclab.managers import TerminationTermCfg as DoneTerm
from isaaclab.utils import configclass
from isaaclab.utils.noise import AdditiveUniformNoiseCfg as Unoise

import berkeley_humanoid_lite.tasks.recovery.mdp as mdp
from berkeley_humanoid_lite.tasks.locomotion.velocity.config.common import (
    CommandRange,
    JointDeviationPenalty,
)
from berkeley_humanoid_lite.tasks.locomotion.velocity.config.common import (
    build_actions_cfg as build_locomotion_actions_cfg,
)
from berkeley_humanoid_lite.tasks.locomotion.velocity.velocity_env_cfg import LocomotionVelocityEnvCfg


@dataclass(frozen=True)
class PoseBucketProfile:
    """Reset distribution and contact semantics for one recovery bucket."""

    name: str
    pose_range: dict[str, tuple[float, float]]
    velocity_range: dict[str, tuple[float, float]]
    joint_position_range: tuple[float, float]
    joint_velocity_range: tuple[float, float]
    allowed_contact_patterns: tuple[str, ...] = ()
    sampling_weight: float = 1.0


@dataclass(frozen=True)
class SuccessHoldProfile:
    """Terminal hold condition used to hand off recovery to standing."""

    duration_s: float
    minimum_height: float
    maximum_tilt_l2: float


@dataclass(frozen=True)
class PhaseScheduleProfile:
    """High-level phase naming reserved for later stage-specific curricula."""

    names: tuple[str, ...]


@dataclass(frozen=True)
class RecoveryTerminationProfile:
    """Termination thresholds for recovery-oriented tasks."""

    orientation_limit: float
    minimum_base_height: float
    contact_force_threshold: float
    dangerous_contact_patterns: tuple[str, ...]


@dataclass(frozen=True)
class RecoveryRewardProfile:
    """Reward weights for posture recovery and get-up tasks."""

    target_base_height: float
    upright_weight: float
    height_weight: float
    height_progress_weight: float
    upright_progress_weight: float
    success_hold_bonus_weight: float
    support_transition_weight: float
    action_rate_weight: float
    torque_weight: float
    acceleration_weight: float
    termination_penalty_weight: float
    dangerous_contact_weight: float
    dangerous_contact_body_patterns: tuple[str, ...]
    joint_deviation_penalties: tuple[JointDeviationPenalty, ...]


@dataclass(frozen=True)
class RecoveryTaskProfile:
    """Shared profile for recovery-oriented tasks."""

    class_name: str
    joint_names: tuple[str, ...]
    robot_cfg: object
    command_range: CommandRange
    reward_profile: RecoveryRewardProfile
    pose_buckets: tuple[PoseBucketProfile, ...]
    success_hold_profile: SuccessHoldProfile
    termination_profile: RecoveryTerminationProfile
    contact_observation_patterns: tuple[str, ...] = ()
    phase_schedule_profile: PhaseScheduleProfile | None = None
    episode_length_s: float = 60.0
    decimation: int = 8


def build_commands_cfg(profile: RecoveryTaskProfile):
    """Build near-zero commands for recovery tasks."""

    command_range = profile.command_range

    @configclass
    class CommandsCfg:
        """Command specifications for the recovery MDP."""

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


def build_observations_cfg(profile: RecoveryTaskProfile):
    """Build recovery observations with bucket and progress context."""

    joint_names = list(profile.joint_names)
    bucket_names = tuple(bucket.name for bucket in profile.pose_buckets)
    contact_patterns = list(profile.contact_observation_patterns)

    policy_terms = {
        "__doc__": "Observations for policy group.",
        "__module__": __name__,
        "velocity_commands": ObsTerm(func=mdp.generated_commands, params={"command_name": "base_velocity"}),
        "base_ang_vel": ObsTerm(func=mdp.base_ang_vel, noise=Unoise(n_min=-0.3, n_max=0.3)),
        "projected_gravity": ObsTerm(func=mdp.projected_gravity, noise=Unoise(n_min=-0.05, n_max=0.05)),
        "joint_pos": ObsTerm(
            func=mdp.joint_pos_rel,
            params={"asset_cfg": SceneEntityCfg("robot", joint_names=joint_names, preserve_order=True)},
            noise=Unoise(n_min=-0.05, n_max=0.05),
        ),
        "joint_vel": ObsTerm(
            func=mdp.joint_vel_rel,
            params={"asset_cfg": SceneEntityCfg("robot", joint_names=joint_names, preserve_order=True)},
            noise=Unoise(n_min=-2.0, n_max=2.0),
        ),
        "recovery_progress": ObsTerm(
            func=mdp.recovery_phase_progress,
            params={
                "target_height": profile.reward_profile.target_base_height,
                "minimum_height": profile.termination_profile.minimum_base_height,
            },
        ),
        "recovery_bucket": ObsTerm(func=mdp.recovery_bucket_one_hot, params={"bucket_names": bucket_names}),
        "actions": ObsTerm(func=mdp.last_action),
    }
    if contact_patterns:
        policy_terms["contact_map"] = ObsTerm(
            func=mdp.contact_state_map,
            params={
                "sensor_cfg": SceneEntityCfg("contact_forces", body_names=contact_patterns),
                "threshold": 1.0,
            },
        )

    @configclass
    class PolicyCfg(ObsGroup):
        """Observations for policy group."""

        velocity_commands = policy_terms["velocity_commands"]
        base_ang_vel = policy_terms["base_ang_vel"]
        projected_gravity = policy_terms["projected_gravity"]
        joint_pos = policy_terms["joint_pos"]
        joint_vel = policy_terms["joint_vel"]
        recovery_progress = policy_terms["recovery_progress"]
        recovery_bucket = policy_terms["recovery_bucket"]
        if "contact_map" in policy_terms:
            contact_map = policy_terms["contact_map"]
        actions = policy_terms["actions"]

        def __post_init__(self):
            self.enable_corruption = True

    @configclass
    class CriticCfg(PolicyCfg):
        """Observations for critic group."""

        base_lin_vel = ObsTerm(func=mdp.base_lin_vel)
        success_hold = ObsTerm(
            func=mdp.success_hold_progress,
            params={
                "bucket_names": bucket_names,
                "hold_duration_s": profile.success_hold_profile.duration_s,
            },
        )

        def __post_init__(self):
            self.enable_corruption = False

    @configclass
    class ObservationsCfg:
        """Observation specifications for the recovery MDP."""

        policy: PolicyCfg = PolicyCfg()
        critic: CriticCfg = CriticCfg()

    PolicyCfg.__name__ = f"{profile.class_name}PolicyObservationsCfg"
    CriticCfg.__name__ = f"{profile.class_name}CriticObservationsCfg"
    ObservationsCfg.__name__ = f"{profile.class_name}ObservationsCfg"
    return ObservationsCfg


def build_actions_cfg(profile: RecoveryTaskProfile):
    """Reuse the existing action layout for deployment compatibility."""

    return build_locomotion_actions_cfg(profile)


def build_rewards_cfg(profile: RecoveryTaskProfile):
    """Build reward terms for recovery-oriented tasks."""

    reward_profile = profile.reward_profile
    bucket_names = tuple(bucket.name for bucket in profile.pose_buckets)
    joint_names = list(profile.joint_names)
    dangerous_patterns = list(reward_profile.dangerous_contact_body_patterns)
    support_patterns = sorted(
        {
            pattern
            for bucket in profile.pose_buckets
            for pattern in bucket.allowed_contact_patterns
        }
    )

    rewards_cfg = {
        "__doc__": "Reward terms for recovery-oriented tasks.",
        "__module__": __name__,
        "uprightness": RewTerm(func=mdp.upright_score, weight=reward_profile.upright_weight),
        "base_height_error_l1": RewTerm(
            func=mdp.base_height_error_l1,
            params={"target_height": reward_profile.target_base_height},
            weight=reward_profile.height_weight,
        ),
        "height_progress": RewTerm(
            func=mdp.base_height_progress,
            params={"bucket_names": bucket_names},
            weight=reward_profile.height_progress_weight,
        ),
        "upright_progress": RewTerm(
            func=mdp.upright_progress,
            params={"bucket_names": bucket_names},
            weight=reward_profile.upright_progress_weight,
        ),
        "success_hold_bonus": RewTerm(
            func=mdp.success_hold_bonus,
            params={
                "bucket_names": bucket_names,
                "hold_duration_s": profile.success_hold_profile.duration_s,
                "minimum_height": profile.success_hold_profile.minimum_height,
                "maximum_tilt_l2": profile.success_hold_profile.maximum_tilt_l2,
            },
            weight=reward_profile.success_hold_bonus_weight,
        ),
        "termination_penalty": RewTerm(func=mdp.is_terminated, weight=reward_profile.termination_penalty_weight),
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
        "dangerous_contacts": RewTerm(
            func=mdp.undesired_contacts,
            params={
                "sensor_cfg": SceneEntityCfg("contact_forces", body_names=dangerous_patterns),
                "threshold": 1.0,
            },
            weight=reward_profile.dangerous_contact_weight,
        ),
    }

    if support_patterns and reward_profile.support_transition_weight != 0.0:
        rewards_cfg["support_contact_transition"] = RewTerm(
            func=mdp.support_contact_transition,
            params={
                "sensor_cfg": SceneEntityCfg("contact_forces", body_names=support_patterns),
                "allowed_contact_patterns": tuple(support_patterns),
                "threshold": 1.0,
            },
            weight=reward_profile.support_transition_weight,
        )

    for penalty in reward_profile.joint_deviation_penalties:
        rewards_cfg[penalty.attribute_name] = RewTerm(
            func=mdp.joint_deviation_l1,
            params={"asset_cfg": SceneEntityCfg("robot", joint_names=list(penalty.joint_patterns))},
            weight=penalty.weight,
        )

    rewards_cls = configclass(type(f"{profile.class_name}RewardsCfg", (), rewards_cfg))
    return rewards_cls


def build_terminations_cfg(profile: RecoveryTaskProfile):
    """Build termination terms for recovery-oriented tasks."""

    bucket_names = tuple(bucket.name for bucket in profile.pose_buckets)
    allowed_patterns = tuple(bucket.allowed_contact_patterns for bucket in profile.pose_buckets)
    termination_profile = profile.termination_profile

    terminations_cfg = {
        "__doc__": "Termination terms for recovery-oriented tasks.",
        "__module__": __name__,
        "time_out": DoneTerm(func=mdp.time_out, time_out=True),
        "base_orientation": DoneTerm(
            func=mdp.bad_orientation,
            params={
                "limit_angle": termination_profile.orientation_limit,
                "asset_cfg": SceneEntityCfg("robot", body_names="base"),
            },
        ),
        "base_height": DoneTerm(
            func=mdp.base_height_below_threshold,
            params={"minimum_height": termination_profile.minimum_base_height},
        ),
        "dangerous_body_contact": DoneTerm(
            func=mdp.dangerous_body_contact,
            params={
                "sensor_cfg": SceneEntityCfg(
                    "contact_forces",
                    body_names=list(termination_profile.dangerous_contact_patterns),
                ),
                "threshold": termination_profile.contact_force_threshold,
            },
        ),
        "disallowed_body_contact": DoneTerm(
            func=mdp.disallowed_body_contact,
            params={
                "sensor_cfg": SceneEntityCfg("contact_forces", body_names=".*"),
                "threshold": termination_profile.contact_force_threshold,
                "bucket_names": bucket_names,
                "bucket_allowed_contact_patterns": allowed_patterns,
            },
        ),
    }

    terminations_cls = configclass(type(f"{profile.class_name}TerminationsCfg", (), terminations_cfg))
    return terminations_cls


def build_events_cfg(profile: RecoveryTaskProfile):
    """Build startup randomization and bucket-aware reset events."""

    events_cfg = {
        "__doc__": "Configuration for recovery-task events.",
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
        "reset_pose_bucket": EventTerm(
            func=mdp.reset_to_pose_buckets,
            params={"buckets": profile.pose_buckets},
            mode="reset",
        ),
    }

    events_cls = configclass(type(f"{profile.class_name}EventsCfg", (), events_cfg))
    return events_cls


def build_curriculums_cfg(profile: RecoveryTaskProfile):
    """Build an empty curriculum section for future bucket schedules."""

    phase_names = profile.phase_schedule_profile.names if profile.phase_schedule_profile is not None else ()
    curriculums_cfg = {
        "__doc__": "Curriculum terms for recovery-oriented tasks.",
        "__module__": __name__,
        "phase_names": phase_names,
    }
    curriculums_cls = configclass(type(f"{profile.class_name}CurriculumsCfg", (), curriculums_cfg))
    return curriculums_cls


def build_environment_cfg(
    class_name: str,
    profile: RecoveryTaskProfile,
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
    class RecoveryTaskEnvCfg(LocomotionVelocityEnvCfg):
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

    RecoveryTaskEnvCfg.__name__ = class_name
    return RecoveryTaskEnvCfg
