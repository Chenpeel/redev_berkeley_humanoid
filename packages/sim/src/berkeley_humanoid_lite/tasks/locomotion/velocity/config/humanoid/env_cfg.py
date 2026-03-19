from berkeley_humanoid_lite_assets.robots import FULL_BODY_JOINT_NAMES, FULL_BODY_ARTICULATION_CFG
from berkeley_humanoid_lite.tasks.locomotion.velocity.config.common import (
    CommandRange,
    JointDeviationPenalty,
    RewardProfile,
    VelocityTaskProfile,
    build_actions_cfg,
    build_commands_cfg,
    build_environment_cfg,
    build_observations_cfg,
    build_rewards_cfg,
)

_HUMANOID_PROFILE = VelocityTaskProfile(
    class_name="BerkeleyHumanoidLite",
    joint_names=tuple(FULL_BODY_JOINT_NAMES),
    robot_cfg=FULL_BODY_ARTICULATION_CFG,
    command_range=CommandRange(
        linear_velocity_x=(-1.0, 1.0),
        linear_velocity_y=(-0.5, 0.5),
        angular_velocity_z=(-1.5, 1.5),
    ),
    reward_profile=RewardProfile(
        tracking_standard_deviation=0.5,
        flat_orientation_weight=-1.0,
        action_rate_weight=-0.001,
        torque_weight=-2.0e-5,
        acceleration_weight=-1.0e-7,
        feet_air_time_threshold=0.5,
        feet_air_time_weight=2.0,
        undesired_contact_body_patterns=("base", ".*_hip_.*", ".*_knee_.*", ".*_shoulder_.*", ".*_elbow_.*"),
        joint_deviation_penalties=(
            JointDeviationPenalty("joint_deviation_hip", (".*_hip_yaw_joint", ".*_hip_roll_joint"), -1.0),
            JointDeviationPenalty("joint_deviation_ankle_roll", (".*_ankle_roll_joint",), -1.0),
            JointDeviationPenalty(
                "joint_deviation_shoulder",
                (".*_shoulder_pitch_joint", ".*_shoulder_roll_joint", ".*_shoulder_yaw_joint"),
                -1.0,
            ),
            JointDeviationPenalty("joint_deviation_elbow", (".*_elbow_pitch_joint", ".*_elbow_roll_joint"), -1.0),
        ),
    ),
)

CommandsCfg = build_commands_cfg(_HUMANOID_PROFILE)
ObservationsCfg = build_observations_cfg(_HUMANOID_PROFILE)
ActionsCfg = build_actions_cfg(_HUMANOID_PROFILE)
RewardsCfg = build_rewards_cfg(_HUMANOID_PROFILE)
BerkeleyHumanoidLiteEnvCfg = build_environment_cfg(
    class_name="BerkeleyHumanoidLiteEnvCfg",
    profile=_HUMANOID_PROFILE,
    commands_cfg_cls=CommandsCfg,
    observations_cfg_cls=ObservationsCfg,
    actions_cfg_cls=ActionsCfg,
    rewards_cfg_cls=RewardsCfg,
)
