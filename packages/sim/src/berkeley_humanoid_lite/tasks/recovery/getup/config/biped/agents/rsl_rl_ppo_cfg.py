from isaaclab.utils import configclass

from berkeley_humanoid_lite.tasks.locomotion.velocity.config.biped.agents.rsl_rl_ppo_cfg import (
    BerkeleyHumanoidLiteBipedPPORunnerCfg,
)


@configclass
class BerkeleyHumanoidLiteBipedGetupPPORunnerCfg(BerkeleyHumanoidLiteBipedPPORunnerCfg):
    experiment_name = "getup_biped"
    max_iterations = 8000
