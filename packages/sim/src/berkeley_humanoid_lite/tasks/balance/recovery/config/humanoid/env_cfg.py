from berkeley_humanoid_lite_assets.robots import FULL_BODY_ARTICULATION_CFG, FULL_BODY_JOINT_NAMES

from berkeley_humanoid_lite.tasks.balance.common import (
    BalanceRewardProfile,
    BalanceTaskProfile,
    DisturbanceProfile,
    ResetProfile,
    TerminationProfile,
    build_actions_cfg,
    build_commands_cfg,
    build_curriculums_cfg,
    build_environment_cfg,
    build_events_cfg,
    build_observations_cfg,
    build_rewards_cfg,
    build_terminations_cfg,
)
from berkeley_humanoid_lite.tasks.locomotion.velocity.config.common import CommandRange, JointDeviationPenalty

_HUMANOID_RECOVERY_PROFILE = BalanceTaskProfile(
    class_name="BerkeleyHumanoidLiteRecovery",
    joint_names=tuple(FULL_BODY_JOINT_NAMES),
    robot_cfg=FULL_BODY_ARTICULATION_CFG,
    command_range=CommandRange(
        linear_velocity_x=(0.0, 0.0),
        linear_velocity_y=(0.0, 0.0),
        angular_velocity_z=(0.0, 0.0),
    ),
    reward_profile=BalanceRewardProfile(
        tracking_standard_deviation=0.35,
        linear_velocity_tracking_weight=2.0,
        angular_velocity_tracking_weight=1.0,
        flat_orientation_weight=-1.0,
        action_rate_weight=-0.001,
        torque_weight=-2.0e-5,
        acceleration_weight=-1.0e-7,
        termination_penalty_weight=-20.0,
        undesired_contact_weight=-2.0,
        undesired_contact_body_patterns=("base", ".*_hip_.*", ".*_knee_.*", ".*_shoulder_.*", ".*_elbow_.*"),
        feet_slide_weight=-0.2,
        feet_air_time_penalty_weight=-0.1,
        joint_deviation_penalties=(
            JointDeviationPenalty("joint_deviation_hip", (".*_hip_yaw_joint", ".*_hip_roll_joint"), -0.5),
            JointDeviationPenalty("joint_deviation_ankle_roll", (".*_ankle_roll_joint",), -0.5),
            JointDeviationPenalty(
                "joint_deviation_shoulder",
                (".*_shoulder_pitch_joint", ".*_shoulder_roll_joint", ".*_shoulder_yaw_joint"),
                -0.25,
            ),
            JointDeviationPenalty("joint_deviation_elbow", (".*_elbow_pitch_joint", ".*_elbow_roll_joint"), -0.25),
        ),
    ),
    reset_profile=ResetProfile(
        pose_range={"x": (-0.1, 0.1), "y": (-0.1, 0.1), "yaw": (-0.5, 0.5)},
        velocity_range={
            "x": (-0.1, 0.1),
            "y": (-0.1, 0.1),
            "z": (0.0, 0.0),
            "roll": (-0.2, 0.2),
            "pitch": (-0.2, 0.2),
            "yaw": (-0.2, 0.2),
        },
        joint_position_range=(0.85, 1.15),
        joint_velocity_range=(-0.2, 0.2),
    ),
    termination_profile=TerminationProfile(
        orientation_limit=0.75,
        contact_force_threshold=1.0,
        body_contact_patterns=("base", ".*_hip_.*", ".*_knee_.*", ".*_shoulder_.*", ".*_elbow_.*"),
    ),
    disturbance_profile=DisturbanceProfile(
        interval_range_s=(4.0, 6.0),
        velocity_range={"x": (-0.6, 0.6), "y": (-0.4, 0.4)},
    ),
    episode_length_s=40.0,
)

CommandsCfg = build_commands_cfg(_HUMANOID_RECOVERY_PROFILE)
ObservationsCfg = build_observations_cfg(_HUMANOID_RECOVERY_PROFILE)
ActionsCfg = build_actions_cfg(_HUMANOID_RECOVERY_PROFILE)
RewardsCfg = build_rewards_cfg(_HUMANOID_RECOVERY_PROFILE)
TerminationsCfg = build_terminations_cfg(_HUMANOID_RECOVERY_PROFILE)
EventsCfg = build_events_cfg(_HUMANOID_RECOVERY_PROFILE)
CurriculumsCfg = build_curriculums_cfg(_HUMANOID_RECOVERY_PROFILE)
BerkeleyHumanoidLiteRecoveryEnvCfg = build_environment_cfg(
    class_name="BerkeleyHumanoidLiteRecoveryEnvCfg",
    profile=_HUMANOID_RECOVERY_PROFILE,
    commands_cfg_cls=CommandsCfg,
    observations_cfg_cls=ObservationsCfg,
    actions_cfg_cls=ActionsCfg,
    rewards_cfg_cls=RewardsCfg,
    terminations_cfg_cls=TerminationsCfg,
    events_cfg_cls=EventsCfg,
    curriculums_cfg_cls=CurriculumsCfg,
)
