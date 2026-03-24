from berkeley_humanoid_lite_assets.robots import BIPED_ARTICULATION_CFG, LEG_JOINT_NAMES

from berkeley_humanoid_lite.tasks.locomotion.velocity.config.common import CommandRange, JointDeviationPenalty
from berkeley_humanoid_lite.tasks.recovery.common import (
    PhaseScheduleProfile,
    PoseBucketProfile,
    RecoveryRewardProfile,
    RecoveryTaskProfile,
    RecoveryTerminationProfile,
    SuccessHoldProfile,
    build_actions_cfg,
    build_commands_cfg,
    build_curriculums_cfg,
    build_environment_cfg,
    build_events_cfg,
    build_observations_cfg,
    build_rewards_cfg,
    build_terminations_cfg,
)

_SAFE_EMPTY_PATTERN = (".^",)

_BIPED_POSTURE_RECOVERY_PROFILE = RecoveryTaskProfile(
    class_name="BerkeleyHumanoidLiteBipedPostureRecovery",
    joint_names=tuple(LEG_JOINT_NAMES),
    robot_cfg=BIPED_ARTICULATION_CFG,
    command_range=CommandRange(
        linear_velocity_x=(0.0, 0.0),
        linear_velocity_y=(0.0, 0.0),
        angular_velocity_z=(0.0, 0.0),
    ),
    reward_profile=RecoveryRewardProfile(
        target_base_height=0.66,
        upright_weight=2.5,
        height_weight=-1.2,
        height_progress_weight=4.0,
        upright_progress_weight=3.0,
        success_hold_bonus_weight=8.0,
        support_transition_weight=0.25,
        action_rate_weight=-0.002,
        torque_weight=-2.0e-3,
        acceleration_weight=-1.0e-6,
        termination_penalty_weight=-20.0,
        dangerous_contact_weight=0.0,
        dangerous_contact_body_patterns=_SAFE_EMPTY_PATTERN,
        joint_deviation_penalties=(
            JointDeviationPenalty("joint_deviation_hip", (".*_hip_yaw_joint", ".*_hip_roll_joint"), -0.20),
            JointDeviationPenalty("joint_deviation_ankle_roll", (".*_ankle_roll_joint",), -0.20),
        ),
    ),
    pose_buckets=(
        PoseBucketProfile(
            name="kneeling",
            pose_range={
                "x": (-0.05, 0.05),
                "y": (-0.05, 0.05),
                "z": (-0.18, -0.10),
                "roll": (-0.08, 0.08),
                "pitch": (-0.38, -0.18),
                "yaw": (-0.25, 0.25),
            },
            velocity_range={
                "x": (-0.08, 0.08),
                "y": (-0.08, 0.08),
                "z": (-0.10, 0.0),
                "roll": (-0.15, 0.15),
                "pitch": (-0.15, 0.15),
                "yaw": (-0.15, 0.15),
            },
            joint_position_range=(0.65, 1.05),
            joint_velocity_range=(-0.30, 0.30),
            allowed_contact_patterns=(".*_ankle_roll", ".*_knee_.*"),
        ),
        PoseBucketProfile(
            name="crouched",
            pose_range={
                "x": (-0.05, 0.05),
                "y": (-0.05, 0.05),
                "z": (-0.10, -0.04),
                "roll": (-0.08, 0.08),
                "pitch": (-0.22, -0.05),
                "yaw": (-0.25, 0.25),
            },
            velocity_range={
                "x": (-0.06, 0.06),
                "y": (-0.06, 0.06),
                "z": (-0.05, 0.0),
                "roll": (-0.12, 0.12),
                "pitch": (-0.12, 0.12),
                "yaw": (-0.12, 0.12),
            },
            joint_position_range=(0.82, 1.08),
            joint_velocity_range=(-0.25, 0.25),
            allowed_contact_patterns=(".*_ankle_roll",),
        ),
        PoseBucketProfile(
            name="seated",
            pose_range={
                "x": (-0.08, 0.08),
                "y": (-0.08, 0.08),
                "z": (-0.22, -0.14),
                "roll": (-0.10, 0.10),
                "pitch": (-0.10, 0.10),
                "yaw": (-0.30, 0.30),
            },
            velocity_range={
                "x": (-0.05, 0.05),
                "y": (-0.05, 0.05),
                "z": (-0.05, 0.0),
                "roll": (-0.10, 0.10),
                "pitch": (-0.10, 0.10),
                "yaw": (-0.10, 0.10),
            },
            joint_position_range=(0.55, 1.12),
            joint_velocity_range=(-0.20, 0.20),
            allowed_contact_patterns=("base", ".*_hip_.*", ".*_ankle_roll"),
        ),
        PoseBucketProfile(
            name="half_prone",
            pose_range={
                "x": (-0.08, 0.08),
                "y": (-0.08, 0.08),
                "z": (-0.22, -0.14),
                "roll": (-0.22, 0.22),
                "pitch": (-0.85, -0.50),
                "yaw": (-0.30, 0.30),
            },
            velocity_range={
                "x": (-0.08, 0.08),
                "y": (-0.08, 0.08),
                "z": (-0.10, 0.0),
                "roll": (-0.15, 0.15),
                "pitch": (-0.15, 0.15),
                "yaw": (-0.15, 0.15),
            },
            joint_position_range=(0.45, 1.18),
            joint_velocity_range=(-0.30, 0.30),
            allowed_contact_patterns=("base", ".*_knee_.*", ".*_ankle_roll"),
        ),
        PoseBucketProfile(
            name="half_supine",
            pose_range={
                "x": (-0.08, 0.08),
                "y": (-0.08, 0.08),
                "z": (-0.22, -0.14),
                "roll": (-0.22, 0.22),
                "pitch": (0.50, 0.85),
                "yaw": (-0.30, 0.30),
            },
            velocity_range={
                "x": (-0.08, 0.08),
                "y": (-0.08, 0.08),
                "z": (-0.10, 0.0),
                "roll": (-0.15, 0.15),
                "pitch": (-0.15, 0.15),
                "yaw": (-0.15, 0.15),
            },
            joint_position_range=(0.45, 1.18),
            joint_velocity_range=(-0.30, 0.30),
            allowed_contact_patterns=("base", ".*_ankle_roll"),
        ),
    ),
    success_hold_profile=SuccessHoldProfile(duration_s=0.75, minimum_height=0.62, maximum_tilt_l2=0.18),
    termination_profile=RecoveryTerminationProfile(
        orientation_limit=1.25,
        minimum_base_height=0.16,
        contact_force_threshold=1.0,
        dangerous_contact_patterns=_SAFE_EMPTY_PATTERN,
    ),
    contact_observation_patterns=("base", ".*_knee_.*", ".*_ankle_roll"),
    phase_schedule_profile=PhaseScheduleProfile(names=("grounded", "supported", "upright")),
    episode_length_s=50.0,
)

CommandsCfg = build_commands_cfg(_BIPED_POSTURE_RECOVERY_PROFILE)
ObservationsCfg = build_observations_cfg(_BIPED_POSTURE_RECOVERY_PROFILE)
ActionsCfg = build_actions_cfg(_BIPED_POSTURE_RECOVERY_PROFILE)
RewardsCfg = build_rewards_cfg(_BIPED_POSTURE_RECOVERY_PROFILE)
TerminationsCfg = build_terminations_cfg(_BIPED_POSTURE_RECOVERY_PROFILE)
EventsCfg = build_events_cfg(_BIPED_POSTURE_RECOVERY_PROFILE)
CurriculumsCfg = build_curriculums_cfg(_BIPED_POSTURE_RECOVERY_PROFILE)
BerkeleyHumanoidLiteBipedPostureRecoveryEnvCfg = build_environment_cfg(
    class_name="BerkeleyHumanoidLiteBipedPostureRecoveryEnvCfg",
    profile=_BIPED_POSTURE_RECOVERY_PROFILE,
    commands_cfg_cls=CommandsCfg,
    observations_cfg_cls=ObservationsCfg,
    actions_cfg_cls=ActionsCfg,
    rewards_cfg_cls=RewardsCfg,
    terminations_cfg_cls=TerminationsCfg,
    events_cfg_cls=EventsCfg,
    curriculums_cfg_cls=CurriculumsCfg,
)
