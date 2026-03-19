from berkeley_humanoid_lite_assets.robots import LEG_JOINT_NAMES, BIPED_ARTICULATION_CFG
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

_BIPED_PROFILE = VelocityTaskProfile(
    class_name="BerkeleyHumanoidLiteBiped",
    joint_names=tuple(LEG_JOINT_NAMES),
    robot_cfg=BIPED_ARTICULATION_CFG,
    command_range=CommandRange(
        linear_velocity_x=(-0.5, 0.5),
        linear_velocity_y=(-0.25, 0.25),
        angular_velocity_z=(-1.0, 1.0),
    ),
    reward_profile=RewardProfile(
        tracking_standard_deviation=0.25,
        flat_orientation_weight=-2.0,
        action_rate_weight=-0.01,
        torque_weight=-2.0e-3,
        acceleration_weight=-1.0e-6,
        feet_air_time_threshold=0.4,
        feet_air_time_weight=1.0,
        undesired_contact_body_patterns=("base", ".*_hip_.*", ".*_knee_.*"),
        joint_deviation_penalties=(
            JointDeviationPenalty("joint_deviation_hip", (".*_hip_yaw_joint", ".*_hip_roll_joint"), -0.2),
            JointDeviationPenalty("joint_deviation_ankle_roll", (".*_ankle_roll_joint",), -0.2),
        ),
    ),
)

CommandsCfg = build_commands_cfg(_BIPED_PROFILE)
ObservationsCfg = build_observations_cfg(_BIPED_PROFILE)
ActionsCfg = build_actions_cfg(_BIPED_PROFILE)
RewardsCfg = build_rewards_cfg(_BIPED_PROFILE)
BerkeleyHumanoidLiteBipedEnvCfg = build_environment_cfg(
    class_name="BerkeleyHumanoidLiteBipedEnvCfg",
    profile=_BIPED_PROFILE,
    commands_cfg_cls=CommandsCfg,
    observations_cfg_cls=ObservationsCfg,
    actions_cfg_cls=ActionsCfg,
    rewards_cfg_cls=RewardsCfg,
)
