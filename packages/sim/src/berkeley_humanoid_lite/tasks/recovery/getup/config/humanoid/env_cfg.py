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

_FULL_BODY_CONTACT_PATTERNS = ("base", ".*_hip_.*", ".*_knee_.*", ".*_shoulder_.*", ".*_elbow_.*", ".*_ankle_roll")

_HUMANOID_GETUP_PROFILE = RecoveryTaskProfile(
    class_name="BerkeleyHumanoidLiteGetup",
    joint_names=tuple(FULL_BODY_JOINT_NAMES),
    robot_cfg=FULL_BODY_ARTICULATION_CFG,
    command_range=CommandRange(
        linear_velocity_x=(0.0, 0.0),
        linear_velocity_y=(0.0, 0.0),
        angular_velocity_z=(0.0, 0.0),
    ),
    reward_profile=RecoveryRewardProfile(
        target_base_height=0.72,
        upright_weight=3.0,
        height_weight=-2.0,
        height_progress_weight=6.0,
        upright_progress_weight=5.0,
        success_hold_bonus_weight=8.0,
        support_transition_weight=0.25,
        action_rate_weight=-0.002,
        torque_weight=-3.0e-5,
        acceleration_weight=-2.0e-7,
        termination_penalty_weight=-25.0,
        dangerous_contact_weight=0.0,
        dangerous_contact_body_patterns=(".*_shoulder_.*", ".*_elbow_.*"),
        joint_deviation_penalties=(
            JointDeviationPenalty("joint_deviation_hip", (".*_hip_yaw_joint", ".*_hip_roll_joint"), -0.1),
            JointDeviationPenalty("joint_deviation_ankle_roll", (".*_ankle_roll_joint",), -0.1),
            JointDeviationPenalty(
                "joint_deviation_shoulder",
                (".*_shoulder_pitch_joint", ".*_shoulder_roll_joint", ".*_shoulder_yaw_joint"),
                -0.05,
            ),
            JointDeviationPenalty("joint_deviation_elbow", (".*_elbow_pitch_joint", ".*_elbow_roll_joint"), -0.05),
        ),
    ),
    pose_buckets=(
        PoseBucketProfile(
            name="supine",
            pose_range={"x": (-0.05, 0.05), "y": (-0.05, 0.05), "yaw": (-0.5, 0.5)},
            velocity_range={
                "x": (-0.1, 0.1),
                "y": (-0.1, 0.1),
                "z": (0.0, 0.0),
                "roll": (-0.2, 0.2),
                "pitch": (-0.2, 0.2),
                "yaw": (-0.2, 0.2),
            },
            joint_position_range=(0.35, 1.65),
            joint_velocity_range=(-0.5, 0.5),
            allowed_contact_patterns=("base", ".*_hip_.*", ".*_shoulder_.*", ".*_elbow_.*", ".*_ankle_roll"),
        ),
        PoseBucketProfile(
            name="prone",
            pose_range={"x": (-0.05, 0.05), "y": (-0.05, 0.05), "yaw": (-0.5, 0.5)},
            velocity_range={
                "x": (-0.1, 0.1),
                "y": (-0.1, 0.1),
                "z": (0.0, 0.0),
                "roll": (-0.2, 0.2),
                "pitch": (-0.2, 0.2),
                "yaw": (-0.2, 0.2),
            },
            joint_position_range=(0.3, 1.7),
            joint_velocity_range=(-0.5, 0.5),
            # prone 先通过更宽的允许接触集合，为后续 roll/reorient 留出生存空间
            allowed_contact_patterns=_FULL_BODY_CONTACT_PATTERNS,
        ),
        PoseBucketProfile(
            name="lateral_left",
            pose_range={"x": (-0.05, 0.05), "y": (-0.05, 0.05), "yaw": (-0.5, 0.5)},
            velocity_range={
                "x": (-0.1, 0.1),
                "y": (-0.1, 0.1),
                "z": (0.0, 0.0),
                "roll": (-0.2, 0.2),
                "pitch": (-0.2, 0.2),
                "yaw": (-0.2, 0.2),
            },
            joint_position_range=(0.35, 1.65),
            joint_velocity_range=(-0.5, 0.5),
            allowed_contact_patterns=_FULL_BODY_CONTACT_PATTERNS,
        ),
        PoseBucketProfile(
            name="lateral_right",
            pose_range={"x": (-0.05, 0.05), "y": (-0.05, 0.05), "yaw": (-0.5, 0.5)},
            velocity_range={
                "x": (-0.1, 0.1),
                "y": (-0.1, 0.1),
                "z": (0.0, 0.0),
                "roll": (-0.2, 0.2),
                "pitch": (-0.2, 0.2),
                "yaw": (-0.2, 0.2),
            },
            joint_position_range=(0.35, 1.65),
            joint_velocity_range=(-0.5, 0.5),
            allowed_contact_patterns=_FULL_BODY_CONTACT_PATTERNS,
        ),
    ),
    success_hold_profile=SuccessHoldProfile(
        duration_s=1.0,
        minimum_height=0.58,
        maximum_tilt_l2=0.22,
    ),
    termination_profile=RecoveryTerminationProfile(
        orientation_limit=2.8,
        minimum_base_height=0.18,
        contact_force_threshold=250.0,
        dangerous_contact_patterns=(".*_shoulder_.*", ".*_elbow_.*"),
    ),
    contact_observation_patterns=("base", ".*_knee_.*", ".*_elbow_.*", ".*_ankle_roll"),
    phase_schedule_profile=PhaseScheduleProfile(
        names=("ground_contact", "reorient", "rise", "stabilize"),
    ),
    episode_length_s=80.0,
)

CommandsCfg = build_commands_cfg(_HUMANOID_GETUP_PROFILE)
ObservationsCfg = build_observations_cfg(_HUMANOID_GETUP_PROFILE)
ActionsCfg = build_actions_cfg(_HUMANOID_GETUP_PROFILE)
RewardsCfg = build_rewards_cfg(_HUMANOID_GETUP_PROFILE)
TerminationsCfg = build_terminations_cfg(_HUMANOID_GETUP_PROFILE)
EventsCfg = build_events_cfg(_HUMANOID_GETUP_PROFILE)
CurriculumsCfg = build_curriculums_cfg(_HUMANOID_GETUP_PROFILE)
BerkeleyHumanoidLiteGetupEnvCfg = build_environment_cfg(
    class_name="BerkeleyHumanoidLiteGetupEnvCfg",
    profile=_HUMANOID_GETUP_PROFILE,
    commands_cfg_cls=CommandsCfg,
    observations_cfg_cls=ObservationsCfg,
    actions_cfg_cls=ActionsCfg,
    rewards_cfg_cls=RewardsCfg,
    terminations_cfg_cls=TerminationsCfg,
    events_cfg_cls=EventsCfg,
    curriculums_cfg_cls=CurriculumsCfg,
)
