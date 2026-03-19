from __future__ import annotations

import shutil
from dataclasses import dataclass
from pathlib import Path
from urllib.request import urlopen

from .paths import ensure_scene_materials_dir, get_scene_materials_dir


SCENE_MATERIAL_CANDIDATE_FILE_NAMES = ("ground_surface.mdl", "Shingles_01.mdl")
DEFAULT_ISAAC_CONTENT_VERSION = "5.1"
SCENE_MATERIAL_PRESET_NAMES = ("isaac-shingles-01",)


@dataclass(frozen=True)
class SceneMaterialDownload:
    relative_path: str
    source_url: str


@dataclass(frozen=True)
class SceneMaterialPreset:
    name: str
    version: str
    description: str
    downloads: tuple[SceneMaterialDownload, ...]


def list_scene_material_presets() -> tuple[str, ...]:
    """返回可用的场景材质预设名称。"""
    return SCENE_MATERIAL_PRESET_NAMES


def get_scene_material_preset(
    preset_name: str,
    *,
    preset_version: str = DEFAULT_ISAAC_CONTENT_VERSION,
) -> SceneMaterialPreset:
    """根据名称和内容版本返回场景材质预设定义。"""
    if preset_name != "isaac-shingles-01":
        available_presets = ", ".join(sorted(SCENE_MATERIAL_PRESET_NAMES))
        raise ValueError(f"Unknown scene material preset: {preset_name}. Available presets: {available_presets}")

    base_url = (
        "https://omniverse-content-production.s3-us-west-2.amazonaws.com/"
        f"Assets/Isaac/{preset_version}/NVIDIA/Materials/Base/Architecture/"
    )
    return SceneMaterialPreset(
        name="isaac-shingles-01",
        version=preset_version,
        description="NVIDIA Isaac base architecture shingles material bundle.",
        downloads=(
            SceneMaterialDownload(
                relative_path="Shingles_01.mdl",
                source_url=f"{base_url}Shingles_01.mdl",
            ),
            SceneMaterialDownload(
                relative_path="Shingles_01/Shingles_01_BaseColor.png",
                source_url=f"{base_url}Shingles_01/Shingles_01_BaseColor.png",
            ),
            SceneMaterialDownload(
                relative_path="Shingles_01/Shingles_01_ORM.png",
                source_url=f"{base_url}Shingles_01/Shingles_01_ORM.png",
            ),
            SceneMaterialDownload(
                relative_path="Shingles_01/Shingles_01_Normal.png",
                source_url=f"{base_url}Shingles_01/Shingles_01_Normal.png",
            ),
        ),
    )


def resolve_scene_material_path(scene_name: str = "default") -> Path | None:
    """返回项目内当前可用的场景材质文件路径。"""
    materials_dir = get_scene_materials_dir(scene_name=scene_name)
    for file_name in SCENE_MATERIAL_CANDIDATE_FILE_NAMES:
        candidate_path = materials_dir / file_name
        if candidate_path.is_file():
            return candidate_path

    mdl_files = sorted(materials_dir.glob("*.mdl"))
    if mdl_files:
        return mdl_files[0]
    return None


def copy_scene_material_from_file(
    source_path: Path,
    *,
    scene_name: str = "default",
    output_file_name: str = "ground_surface.mdl",
    overwrite: bool = False,
) -> Path:
    """复制本地材质文件到项目内场景材质目录。"""
    if not source_path.is_file():
        raise FileNotFoundError(f"Scene material source file does not exist: {source_path}")

    output_path = ensure_scene_materials_dir(scene_name=scene_name) / output_file_name
    if output_path.exists() and not overwrite:
        raise FileExistsError(f"Scene material already exists: {output_path}")

    shutil.copy2(source_path, output_path)
    return output_path


def download_scene_material_from_url(
    source_url: str,
    *,
    scene_name: str = "default",
    output_file_name: str = "ground_surface.mdl",
    overwrite: bool = False,
) -> Path:
    """从 URL 下载单个材质文件到项目内场景材质目录。"""
    output_path = ensure_scene_materials_dir(scene_name=scene_name) / output_file_name
    if output_path.exists() and not overwrite:
        raise FileExistsError(f"Scene material already exists: {output_path}")

    with urlopen(source_url) as response, output_path.open("wb") as output_file:
        shutil.copyfileobj(response, output_file)
    return output_path


def download_scene_material_preset(
    preset_name: str,
    *,
    scene_name: str = "default",
    preset_version: str = DEFAULT_ISAAC_CONTENT_VERSION,
    overwrite: bool = False,
) -> tuple[Path, ...]:
    """下载一个场景材质预设及其依赖文件到项目内。"""
    preset = get_scene_material_preset(preset_name, preset_version=preset_version)
    materials_dir = ensure_scene_materials_dir(scene_name=scene_name)
    prepared_paths: list[Path] = []
    for download in preset.downloads:
        output_path = materials_dir / download.relative_path
        if output_path.exists() and not overwrite:
            raise FileExistsError(f"Scene material already exists: {output_path}")
        output_path.parent.mkdir(parents=True, exist_ok=True)
        with urlopen(download.source_url) as response, output_path.open("wb") as output_file:
            shutil.copyfileobj(response, output_file)
        prepared_paths.append(output_path)

    return tuple(prepared_paths)
