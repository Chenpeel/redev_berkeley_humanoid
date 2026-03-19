from dataclasses import MISSING

import isaaclab.sim as sim_utils
from isaaclab.assets import ArticulationCfg, AssetBaseCfg
from isaaclab.envs import ManagerBasedRLEnvCfg
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.sensors import ContactSensorCfg
from isaaclab.terrains import TerrainImporterCfg
from isaaclab.utils import configclass

from berkeley_humanoid_lite_assets.scene_materials import resolve_scene_material_path


def _ground_visual_material_cfg():
    """优先使用项目内预下载场景材质，不存在时回退到本地预览材质。"""
    material_path = resolve_scene_material_path()
    if material_path is not None:
        return sim_utils.MdlFileCfg(mdl_path=str(material_path), project_uvw=True)
    return sim_utils.PreviewSurfaceCfg(diffuse_color=(0.12, 0.12, 0.12))


##
# Scene definition
##

@configclass
class FlatTerrainSceneCfg(InteractiveSceneCfg):
    """Configuration for the terrain scene with a legged robot."""

    # ground terrain
    terrain = TerrainImporterCfg(
        prim_path="/World/ground",
        terrain_type="plane",
        terrain_generator=None,
        max_init_terrain_level=5,
        collision_group=-1,
        physics_material=sim_utils.RigidBodyMaterialCfg(
            friction_combine_mode="multiply",
            restitution_combine_mode="multiply",
            static_friction=1.0,
            dynamic_friction=1.0,
        ),
        visual_material=_ground_visual_material_cfg(),
        debug_vis=False,
    )

    # robots
    robot: ArticulationCfg = MISSING

    # sensors
    contact_forces = ContactSensorCfg(prim_path="{ENV_REGEX_NS}/robot/.*", history_length=3, track_air_time=True)

    # lights
    light = AssetBaseCfg(
        prim_path="/World/light",
        spawn=sim_utils.DistantLightCfg(color=(0.75, 0.75, 0.75), intensity=3000.0),
    )
    sky_light = AssetBaseCfg(
        prim_path="/World/skyLight",
        spawn=sim_utils.DomeLightCfg(color=(0.13, 0.13, 0.13), intensity=1000.0),
    )


##
# Environment configuration
##

@configclass
class LocomotionVelocityEnvCfg(ManagerBasedRLEnvCfg):
    """Configuration for the locomotion velocity-tracking environment."""

    # Scene settings
    scene: FlatTerrainSceneCfg = FlatTerrainSceneCfg(num_envs=4096, env_spacing=2.5)

    def __post_init__(self):
        """Post initialization."""
        # general settings
        self.decimation = 4
        self.episode_length_s = 20.0

        # simulation settings
        self.sim.dt = 0.005
        self.sim.render_interval = self.decimation
        self.sim.physics_material = self.scene.terrain.physics_material
        self.sim.physx.gpu_max_rigid_patch_count = 10 * 2**15
        # update sensor update periods
        # we tick all the sensors based on the smallest update period (physics update period)
        if self.scene.contact_forces is not None:
            self.scene.contact_forces.update_period = self.sim.dt
