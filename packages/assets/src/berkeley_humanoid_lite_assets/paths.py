from __future__ import annotations

from pathlib import Path


_PACKAGE_ROOT = Path(__file__).resolve().parent
_DEFAULT_VENDOR = "berkeley_humanoid"
_DEFAULT_ROBOT = "berkeley_humanoid_lite"
_DEFAULT_SCENE = "default"


def get_package_root() -> Path:
    """返回资产包根目录。"""
    return _PACKAGE_ROOT


def get_data_dir() -> Path:
    """返回资产数据目录。"""
    data_dir = _PACKAGE_ROOT / "data"
    if not data_dir.exists():
        raise FileNotFoundError(f"资产目录不存在: {data_dir}")
    return data_dir


def get_robot_dir(robot_name: str = _DEFAULT_ROBOT, vendor_name: str = _DEFAULT_VENDOR) -> Path:
    """返回某个机器人资产目录。"""
    robot_dir = get_data_dir() / "robots" / vendor_name / robot_name
    if not robot_dir.exists():
        raise FileNotFoundError(f"机器人资产目录不存在: {robot_dir}")
    return robot_dir


def get_mesh_dir(robot_name: str = _DEFAULT_ROBOT, vendor_name: str = _DEFAULT_VENDOR) -> Path:
    """返回 mesh 目录。"""
    return get_robot_dir(robot_name=robot_name, vendor_name=vendor_name) / "meshes"


def get_scad_dir(robot_name: str = _DEFAULT_ROBOT, vendor_name: str = _DEFAULT_VENDOR) -> Path:
    """返回 SCAD 目录。"""
    return get_robot_dir(robot_name=robot_name, vendor_name=vendor_name) / "scad"


def get_format_dir(
    asset_format: str,
    *,
    robot_name: str = _DEFAULT_ROBOT,
    vendor_name: str = _DEFAULT_VENDOR,
) -> Path:
    """返回指定资源格式目录。"""
    return get_robot_dir(robot_name=robot_name, vendor_name=vendor_name) / asset_format


def get_mjcf_path(file_name: str, robot_name: str = _DEFAULT_ROBOT, vendor_name: str = _DEFAULT_VENDOR) -> Path:
    """返回 MJCF 文件路径。"""
    return get_robot_dir(robot_name=robot_name, vendor_name=vendor_name) / "mjcf" / file_name


def get_mjcf_export_config_path(
    robot_name: str = _DEFAULT_ROBOT,
    vendor_name: str = _DEFAULT_VENDOR,
) -> Path:
    """返回 MJCF 导出配置文件路径。"""
    return get_mjcf_path("config.json", robot_name=robot_name, vendor_name=vendor_name)


def get_urdf_path(
    file_name: str = "berkeley_humanoid_lite.urdf",
    robot_name: str = _DEFAULT_ROBOT,
    vendor_name: str = _DEFAULT_VENDOR,
) -> Path:
    """返回 URDF 文件路径。"""
    return get_robot_dir(robot_name=robot_name, vendor_name=vendor_name) / "urdf" / file_name


def get_urdf_export_config_path(
    robot_name: str = _DEFAULT_ROBOT,
    vendor_name: str = _DEFAULT_VENDOR,
) -> Path:
    """返回 URDF 导出配置文件路径。"""
    return get_urdf_path("config.json", robot_name=robot_name, vendor_name=vendor_name)


def get_format_config_path(
    asset_format: str,
    *,
    robot_name: str = _DEFAULT_ROBOT,
    vendor_name: str = _DEFAULT_VENDOR,
) -> Path:
    """返回指定资源格式的导出配置文件路径。"""
    return get_format_dir(asset_format, robot_name=robot_name, vendor_name=vendor_name) / "config.json"


def get_usd_path(
    file_name: str = "berkeley_humanoid_lite.usd",
    robot_name: str = _DEFAULT_ROBOT,
    vendor_name: str = _DEFAULT_VENDOR,
) -> Path:
    """返回 USD 文件路径。"""
    return get_robot_dir(robot_name=robot_name, vendor_name=vendor_name) / "usd" / file_name


def get_scene_dir(scene_name: str = _DEFAULT_SCENE) -> Path:
    """返回项目内场景资产目录。

    该目录是可选资源目录，首次使用前可能不存在。
    """
    return get_data_dir() / "scenes" / scene_name


def get_scene_materials_dir(scene_name: str = _DEFAULT_SCENE) -> Path:
    """返回项目内场景材质目录。"""
    return get_scene_dir(scene_name=scene_name) / "materials"


def ensure_scene_materials_dir(scene_name: str = _DEFAULT_SCENE) -> Path:
    """确保项目内场景材质目录存在。"""
    materials_dir = get_scene_materials_dir(scene_name=scene_name)
    materials_dir.mkdir(parents=True, exist_ok=True)
    return materials_dir


def get_scene_material_path(
    file_name: str = "ground_surface.mdl",
    *,
    scene_name: str = _DEFAULT_SCENE,
) -> Path:
    """返回项目内场景材质文件路径。"""
    return get_scene_materials_dir(scene_name=scene_name) / file_name
