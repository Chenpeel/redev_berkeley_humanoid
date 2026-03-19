from __future__ import annotations

import contextlib
from dataclasses import dataclass
import os
from pathlib import Path


@dataclass(frozen=True)
class UrdfUsdConversionConfiguration:
    """描述一次 URDF 到 USD 的转换参数。"""

    input_path: Path
    fix_base: bool = False
    merge_fixed_joints: bool = False
    joint_stiffness: float = 100.0
    joint_damping: float = 1.0
    joint_target_type: str = "position"
    import_sites: bool = True
    make_instanceable: bool = True

    @property
    def output_path(self) -> Path:
        source_directory = self.input_path.parent.parent
        return source_directory / "usd" / self.input_path.with_suffix(".usd").name


def convert_urdf_to_usd(configuration: UrdfUsdConversionConfiguration) -> Path:
    """将 URDF 转换为 USD。

    调用前应先由入口脚本启动 Isaac Sim 应用。
    """
    import carb
    import isaacsim.core.utils.stage as stage_utils
    import omni.kit.app

    from isaaclab.sim.converters import UrdfConverter, UrdfConverterCfg
    from isaaclab.utils.assets import check_file_path
    from isaaclab.utils.dict import print_dict

    input_path = configuration.input_path.expanduser().resolve()
    if not check_file_path(input_path):
        raise ValueError(f"无效的 URDF 文件路径: {input_path}")

    output_path = configuration.output_path
    output_path.parent.mkdir(parents=True, exist_ok=True)

    converter_configuration = UrdfConverterCfg(
        asset_path=input_path,
        usd_dir=os.path.dirname(output_path),
        usd_file_name=os.path.basename(output_path),
        fix_base=configuration.fix_base,
        merge_fixed_joints=configuration.merge_fixed_joints,
        force_usd_conversion=True,
        joint_drive=UrdfConverterCfg.JointDriveCfg(
            gains=UrdfConverterCfg.JointDriveCfg.PDGainsCfg(
                stiffness=configuration.joint_stiffness,
                damping=configuration.joint_damping,
            ),
            target_type=configuration.joint_target_type,
        ),
        import_sites=configuration.import_sites,
        make_instanceable=configuration.make_instanceable,
        self_collision=True,
        replace_cylinders_with_capsules=True,
    )

    print("=" * 80)
    print(f"Input URDF file: {input_path.name}")
    print("URDF importer config:")
    print_dict(converter_configuration.to_dict(), nesting=0)
    print("=" * 80)

    urdf_converter = UrdfConverter(converter_configuration)
    print("URDF importer output:")
    print(f"Generated USD file: {urdf_converter.usd_path}")
    print("=" * 80)

    carb_settings = carb.settings.get_settings()
    local_gui_enabled = carb_settings.get("/app/window/enabled")
    livestream_gui_enabled = carb_settings.get("/app/livestream/enabled")
    if local_gui_enabled or livestream_gui_enabled:
        stage_utils.open_stage(urdf_converter.usd_path)
        application = omni.kit.app.get_app_interface()
        with contextlib.suppress(KeyboardInterrupt):
            while application.is_running():
                application.update()

    return Path(urdf_converter.usd_path)
