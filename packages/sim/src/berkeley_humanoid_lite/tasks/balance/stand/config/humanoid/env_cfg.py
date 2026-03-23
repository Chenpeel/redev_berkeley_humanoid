from berkeley_humanoid_lite_assets.robots import FULL_BODY_ARTICULATION_CFG, FULL_BODY_JOINT_NAMES

from berkeley_humanoid_lite.tasks.balance.common import (
    BalanceRewardProfile,
    BalanceTaskProfile,
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

_HUMANOID_STAND_PROFILE = BalanceTaskProfile(
    class_name="BerkeleyHumanoidLiteStand",
    joint_names=tuple(FULL_BODY_JOINT_NAMES),
    robot_cfg=FULL_BODY_ARTICULATION_CFG,
    command_range=CommandRange(
        linear_velocity_x=(0.0, 0.0),
        linear_velocity_y=(0.0, 0.0),
        angular_velocity_z=(0.0, 0.0),
    ),
    reward_profile=BalanceRewardProfile(
        tracking_standard_deviation=0.2,
        linear_velocity_tracking_weight=2.5,
        angular_velocity_tracking_weight=1.5,
        flat_orientation_weight=-1.5,
        action_rate_weight=-0.001,
        torque_weight=-2.0e-5,
        acceleration_weight=-1.0e-7,
        termination_penalty_weight=-20.0,
        undesired_contact_weight=-2.0,
        undesired_contact_body_patterns=("base", ".*_hip_.*", ".*_knee_.*", ".*_shoulder_.*", ".*_elbow_.*"),
        joint_deviation_penalties=(
            JointDeviationPenalty("joint_deviation_hip", (".*_hip_yaw_joint", ".*_hip_roll_joint"), -1.0),
            JointDeviationPenalty("joint_deviation_ankle_roll", (".*_ankle_roll_joint",), -1.0),
            JointDeviationPenalty(
                "joint_deviation_shoulder",
                (".*_shoulder_pitch_joint", ".*_shoulder_roll_joint", ".*_shoulder_yaw_joint"),
                -0.5,
            ),
            JointDeviationPenalty("joint_deviation_elbow", (".*_elbow_pitch_joint", ".*_elbow_roll_joint"), -0.5),
        ),
    ),
    reset_profile=ResetProfile(
        pose_range={"x": (-0.05, 0.05), "y": (-0.05, 0.05), "yaw": (-0.25, 0.25)},
        velocity_range={
            "x": (-0.05, 0.05),
            "y": (-0.05, 0.05),
            "z": (0.0, 0.0),
            "roll": (-0.1, 0.1),
            "pitch": (-0.1, 0.1),
            "yaw": (-0.1, 0.1),
        },
        joint_position_range=(0.9, 1.1),
        joint_velocity_range=(-0.1, 0.1),
    ),
    termination_profile=TerminationProfile(
        orientation_limit=0.6,
        contact_force_threshold=1.0,
        body_contact_patterns=("base", ".*_hip_.*", ".*_knee_.*", ".*_shoulder_.*", ".*_elbow_.*"),
    ),
)

CommandsCfg = build_commands_cfg(_HUMANOID_STAND_PROFILE)
ObservationsCfg = build_observations_cfg(_HUMANOID_STAND_PROFILE)
ActionsCfg = build_actions_cfg(_HUMANOID_STAND_PROFILE)
RewardsCfg = build_rewards_cfg(_HUMANOID_STAND_PROFILE)
TerminationsCfg = build_terminations_cfg(_HUMANOID_STAND_PROFILE)
EventsCfg = build_events_cfg(_HUMANOID_STAND_PROFILE)
CurriculumsCfg = build_curriculums_cfg(_HUMANOID_STAND_PROFILE)
BerkeleyHumanoidLiteStandEnvCfg = build_environment_cfg(
    class_name="BerkeleyHumanoidLiteStandEnvCfg",
    profile=_HUMANOID_STAND_PROFILE,
    commands_cfg_cls=CommandsCfg,
    observations_cfg_cls=ObservationsCfg,
    actions_cfg_cls=ActionsCfg,
    rewards_cfg_cls=RewardsCfg,
    terminations_cfg_cls=TerminationsCfg,
    events_cfg_cls=EventsCfg,
    curriculums_cfg_cls=CurriculumsCfg,
)
