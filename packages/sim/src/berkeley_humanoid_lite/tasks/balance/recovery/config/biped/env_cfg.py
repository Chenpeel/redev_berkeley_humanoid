from berkeley_humanoid_lite_assets.robots import BIPED_ARTICULATION_CFG, LEG_JOINT_NAMES

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

_BIPED_RECOVERY_PROFILE = BalanceTaskProfile(
    class_name="BerkeleyHumanoidLiteBipedRecovery",
    joint_names=tuple(LEG_JOINT_NAMES),
    robot_cfg=BIPED_ARTICULATION_CFG,
    command_range=CommandRange(
        linear_velocity_x=(0.0, 0.0),
        linear_velocity_y=(0.0, 0.0),
        angular_velocity_z=(0.0, 0.0),
    ),
    reward_profile=BalanceRewardProfile(
        tracking_standard_deviation=0.25,
        linear_velocity_tracking_weight=2.5,
        angular_velocity_tracking_weight=1.25,
        flat_orientation_weight=-1.5,
        action_rate_weight=-0.01,
        torque_weight=-2.0e-3,
        acceleration_weight=-1.0e-6,
        termination_penalty_weight=-20.0,
        undesired_contact_weight=-2.5,
        undesired_contact_body_patterns=("base", ".*_hip_.*", ".*_knee_.*"),
        feet_slide_weight=-0.25,
        feet_air_time_penalty_weight=-0.15,
        joint_deviation_penalties=(
            JointDeviationPenalty("joint_deviation_hip", (".*_hip_yaw_joint", ".*_hip_roll_joint"), -0.2),
            JointDeviationPenalty("joint_deviation_ankle_roll", (".*_ankle_roll_joint",), -0.2),
        ),
    ),
    reset_profile=ResetProfile(
        pose_range={"x": (-0.1, 0.1), "y": (-0.1, 0.1), "yaw": (-0.4, 0.4)},
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
        orientation_limit=0.7,
        contact_force_threshold=1.0,
        body_contact_patterns=("base", ".*_hip_.*", ".*_knee_.*"),
    ),
    disturbance_profile=DisturbanceProfile(
        interval_range_s=(4.0, 6.0),
        velocity_range={"x": (-0.5, 0.5), "y": (-0.35, 0.35)},
    ),
    episode_length_s=40.0,
)

CommandsCfg = build_commands_cfg(_BIPED_RECOVERY_PROFILE)
ObservationsCfg = build_observations_cfg(_BIPED_RECOVERY_PROFILE)
ActionsCfg = build_actions_cfg(_BIPED_RECOVERY_PROFILE)
RewardsCfg = build_rewards_cfg(_BIPED_RECOVERY_PROFILE)
TerminationsCfg = build_terminations_cfg(_BIPED_RECOVERY_PROFILE)
EventsCfg = build_events_cfg(_BIPED_RECOVERY_PROFILE)
CurriculumsCfg = build_curriculums_cfg(_BIPED_RECOVERY_PROFILE)
BerkeleyHumanoidLiteBipedRecoveryEnvCfg = build_environment_cfg(
    class_name="BerkeleyHumanoidLiteBipedRecoveryEnvCfg",
    profile=_BIPED_RECOVERY_PROFILE,
    commands_cfg_cls=CommandsCfg,
    observations_cfg_cls=ObservationsCfg,
    actions_cfg_cls=ActionsCfg,
    rewards_cfg_cls=RewardsCfg,
    terminations_cfg_cls=TerminationsCfg,
    events_cfg_cls=EventsCfg,
    curriculums_cfg_cls=CurriculumsCfg,
)
