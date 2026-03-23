from isaaclab.utils import configclass

from berkeley_humanoid_lite.tasks.locomotion.velocity.config.humanoid.agents.rsl_rl_ppo_cfg import (
    BerkeleyHumanoidLitePPORunnerCfg,
)


@configclass
class BerkeleyHumanoidLiteStandPPORunnerCfg(BerkeleyHumanoidLitePPORunnerCfg):
    experiment_name = "stand_humanoid"
    max_iterations = 4000

