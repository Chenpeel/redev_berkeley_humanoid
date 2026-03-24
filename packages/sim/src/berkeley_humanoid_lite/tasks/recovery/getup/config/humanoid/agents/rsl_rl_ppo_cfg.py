from isaaclab.utils import configclass

from berkeley_humanoid_lite.tasks.locomotion.velocity.config.humanoid.agents.rsl_rl_ppo_cfg import (
    BerkeleyHumanoidLitePPORunnerCfg,
)


@configclass
class BerkeleyHumanoidLiteGetupPPORunnerCfg(BerkeleyHumanoidLitePPORunnerCfg):
    experiment_name = "getup_humanoid"
    max_iterations = 8000
