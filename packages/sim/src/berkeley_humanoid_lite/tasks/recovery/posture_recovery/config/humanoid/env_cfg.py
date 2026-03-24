from berkeley_humanoid_lite_assets.robots import FULL_BODY_ARTICULATION_CFG, FULL_BODY_JOINT_NAMES

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

_HUMANOID_POSTURE_RECOVERY_PROFILE = RecoveryTaskProfile(
    class_name="BerkeleyHumanoidLitePostureRecovery",
    joint_names=tuple(FULL_BODY_JOINT_NAMES),
    robot_cfg=FULL_BODY_ARTICULATION_CFG,
    command_range=CommandRange(
        linear_velocity_x=(0.0, 0.0),
        linear_velocity_y=(0.0, 0.0),
        angular_velocity_z=(0.0, 0.0),
    ),
    reward_profile=RecoveryRewardProfile(
        target_base_height=0.72,
        upright_weight=2.5,
        height_weight=-1.0,
        height_progress_weight=4.0,
        upright_progress_weight=3.0,
        success_hold_bonus_weight=8.0,
        support_transition_weight=0.25,
        action_rate_weight=-0.001,
        torque_weight=-2.0e-5,
        acceleration_weight=-1.0e-7,
        termination_penalty_weight=-20.0,
        dangerous_contact_weight=0.0,
        dangerous_contact_body_patterns=_SAFE_EMPTY_PATTERN,
        joint_deviation_penalties=(
            JointDeviationPenalty("joint_deviation_hip", (".*_hip_yaw_joint", ".*_hip_roll_joint"), -0.35),
            JointDeviationPenalty("joint_deviation_ankle_roll", (".*_ankle_roll_joint",), -0.35),
            JointDeviationPenalty(
                "joint_deviation_shoulder",
                (".*_shoulder_pitch_joint", ".*_shoulder_roll_joint", ".*_shoulder_yaw_joint"),
                -0.15,
            ),
            JointDeviationPenalty("joint_deviation_elbow", (".*_elbow_pitch_joint", ".*_elbow_roll_joint"), -0.15),
        ),
    ),
    pose_buckets=(
        PoseBucketProfile(
            name="kneeling",
            pose_range={
                "x": (-0.05, 0.05),
                "y": (-0.05, 0.05),
                "z": (-0.20, -0.12),
                "roll": (-0.08, 0.08),
                "pitch": (-0.40, -0.20),
                "yaw": (-0.30, 0.30),
            },
            velocity_range={
                "x": (-0.08, 0.08),
                "y": (-0.08, 0.08),
                "z": (-0.10, 0.0),
                "roll": (-0.20, 0.20),
                "pitch": (-0.20, 0.20),
                "yaw": (-0.20, 0.20),
            },
            joint_position_range=(0.65, 1.05),
            joint_velocity_range=(-0.35, 0.35),
            allowed_contact_patterns=(".*_ankle_roll", ".*_knee_.*"),
        ),
        PoseBucketProfile(
            name="crouched",
            pose_range={
                "x": (-0.05, 0.05),
                "y": (-0.05, 0.05),
                "z": (-0.12, -0.06),
                "roll": (-0.10, 0.10),
                "pitch": (-0.25, -0.05),
                "yaw": (-0.30, 0.30),
            },
            velocity_range={
                "x": (-0.06, 0.06),
                "y": (-0.06, 0.06),
                "z": (-0.05, 0.0),
                "roll": (-0.15, 0.15),
                "pitch": (-0.15, 0.15),
                "yaw": (-0.15, 0.15),
            },
            joint_position_range=(0.80, 1.10),
            joint_velocity_range=(-0.25, 0.25),
            allowed_contact_patterns=(".*_ankle_roll",),
        ),
        PoseBucketProfile(
            name="seated",
            pose_range={
                "x": (-0.08, 0.08),
                "y": (-0.08, 0.08),
                "z": (-0.25, -0.16),
                "roll": (-0.12, 0.12),
                "pitch": (-0.12, 0.12),
                "yaw": (-0.35, 0.35),
            },
            velocity_range={
                "x": (-0.05, 0.05),
                "y": (-0.05, 0.05),
                "z": (-0.05, 0.0),
                "roll": (-0.12, 0.12),
                "pitch": (-0.12, 0.12),
                "yaw": (-0.12, 0.12),
            },
            joint_position_range=(0.55, 1.15),
            joint_velocity_range=(-0.20, 0.20),
            allowed_contact_patterns=("base", ".*_hip_.*", ".*_ankle_roll"),
        ),
        PoseBucketProfile(
            name="half_prone",
            pose_range={
                "x": (-0.08, 0.08),
                "y": (-0.08, 0.08),
                "z": (-0.25, -0.16),
                "roll": (-0.25, 0.25),
                "pitch": (-0.90, -0.55),
                "yaw": (-0.35, 0.35),
            },
            velocity_range={
                "x": (-0.08, 0.08),
                "y": (-0.08, 0.08),
                "z": (-0.10, 0.0),
                "roll": (-0.18, 0.18),
                "pitch": (-0.18, 0.18),
                "yaw": (-0.18, 0.18),
            },
            joint_position_range=(0.45, 1.20),
            joint_velocity_range=(-0.35, 0.35),
            allowed_contact_patterns=("base", ".*_knee_.*", ".*_elbow_.*", ".*_ankle_roll"),
        ),
        PoseBucketProfile(
            name="half_supine",
            pose_range={
                "x": (-0.08, 0.08),
                "y": (-0.08, 0.08),
                "z": (-0.25, -0.16),
                "roll": (-0.25, 0.25),
                "pitch": (0.55, 0.90),
                "yaw": (-0.35, 0.35),
            },
            velocity_range={
                "x": (-0.08, 0.08),
                "y": (-0.08, 0.08),
                "z": (-0.10, 0.0),
                "roll": (-0.18, 0.18),
                "pitch": (-0.18, 0.18),
                "yaw": (-0.18, 0.18),
            },
            joint_position_range=(0.45, 1.20),
            joint_velocity_range=(-0.35, 0.35),
            allowed_contact_patterns=("base", ".*_elbow_.*", ".*_ankle_roll"),
        ),
    ),
    success_hold_profile=SuccessHoldProfile(duration_s=0.75, minimum_height=0.68, maximum_tilt_l2=0.20),
    termination_profile=RecoveryTerminationProfile(
        orientation_limit=1.35,
        minimum_base_height=0.18,
        contact_force_threshold=1.0,
        dangerous_contact_patterns=_SAFE_EMPTY_PATTERN,
    ),
    contact_observation_patterns=("base", ".*_knee_.*", ".*_ankle_roll", ".*_elbow_.*"),
    phase_schedule_profile=PhaseScheduleProfile(names=("grounded", "supported", "upright")),
    episode_length_s=50.0,
)

CommandsCfg = build_commands_cfg(_HUMANOID_POSTURE_RECOVERY_PROFILE)
ObservationsCfg = build_observations_cfg(_HUMANOID_POSTURE_RECOVERY_PROFILE)
ActionsCfg = build_actions_cfg(_HUMANOID_POSTURE_RECOVERY_PROFILE)
RewardsCfg = build_rewards_cfg(_HUMANOID_POSTURE_RECOVERY_PROFILE)
TerminationsCfg = build_terminations_cfg(_HUMANOID_POSTURE_RECOVERY_PROFILE)
EventsCfg = build_events_cfg(_HUMANOID_POSTURE_RECOVERY_PROFILE)
CurriculumsCfg = build_curriculums_cfg(_HUMANOID_POSTURE_RECOVERY_PROFILE)
BerkeleyHumanoidLitePostureRecoveryEnvCfg = build_environment_cfg(
    class_name="BerkeleyHumanoidLitePostureRecoveryEnvCfg",
    profile=_HUMANOID_POSTURE_RECOVERY_PROFILE,
    commands_cfg_cls=CommandsCfg,
    observations_cfg_cls=ObservationsCfg,
    actions_cfg_cls=ActionsCfg,
    rewards_cfg_cls=RewardsCfg,
    terminations_cfg_cls=TerminationsCfg,
    events_cfg_cls=EventsCfg,
    curriculums_cfg_cls=CurriculumsCfg,
)
