from isaaclab.utils import configclass

from berkeley_humanoid_lite.tasks.locomotion.velocity.config.humanoid.agents.rsl_rl_ppo_cfg import (
    BerkeleyHumanoidLitePPORunnerCfg,
)


@configclass
class BerkeleyHumanoidLitePostureRecoveryPPORunnerCfg(BerkeleyHumanoidLitePPORunnerCfg):
    experiment_name = "posture_recovery_humanoid"
    max_iterations = 6000
