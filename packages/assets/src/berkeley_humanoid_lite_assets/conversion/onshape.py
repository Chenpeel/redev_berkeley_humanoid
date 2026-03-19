from __future__ import annotations

import json
from pathlib import Path
import shutil
import subprocess
from contextlib import contextmanager
from dataclasses import dataclass
from typing import Iterator

from .postprocess import inject_mjcf_sensors, rewrite_mesh_references


_DEFAULT_ONSHAPE_COMMAND = "onshape-to-robot"


@dataclass(frozen=True)
class OnshapeExportLayout:
    """描述一次 onshape-to-robot 导出的目录布局。"""

    config_path: Path
    output_stem: str
    export_directory: Path
    robot_directory: Path
    mesh_directory: Path
    staging_directory: Path

    def get_output_path(self, suffix: str) -> Path:
        """返回导出结果文件路径。"""
        return self.export_directory / f"{self.output_stem}.{suffix}"


def load_onshape_export_layout(config_path: str | Path) -> OnshapeExportLayout:
    """从配置文件解析导出目录布局。"""
    resolved_config_path = Path(config_path).expanduser().resolve()
    if not resolved_config_path.exists():
        raise FileNotFoundError(f"配置文件不存在: {resolved_config_path}")

    with resolved_config_path.open("r", encoding="utf-8") as file:
        configuration = json.load(file)

    try:
        output_stem = str(configuration["output_filename"])
    except KeyError as error:
        raise KeyError(f"导出配置缺少 output_filename: {resolved_config_path}") from error

    export_directory = resolved_config_path.parent
    robot_directory = export_directory.parent
    return OnshapeExportLayout(
        config_path=resolved_config_path,
        output_stem=output_stem,
        export_directory=export_directory,
        robot_directory=robot_directory,
        mesh_directory=robot_directory / "meshes",
        staging_directory=export_directory / "assets",
    )


def export_onshape_to_urdf(config_path: str | Path, *, command: str = _DEFAULT_ONSHAPE_COMMAND) -> Path:
    """执行 Onshape 到 URDF 的标准导出流程。"""
    layout = load_onshape_export_layout(config_path)
    _run_onshape_export(layout, command=command)
    output_path = layout.get_output_path("urdf")
    _rewrite_exported_file(output_path)
    return output_path


def export_onshape_to_mjcf(config_path: str | Path, *, command: str = _DEFAULT_ONSHAPE_COMMAND) -> Path:
    """执行 Onshape 到 MJCF 的标准导出流程。"""
    layout = load_onshape_export_layout(config_path)
    _run_onshape_export(layout, command=command)
    output_path = layout.get_output_path("xml")
    _rewrite_exported_file(output_path)
    inject_mjcf_sensors(output_path, output_stem=layout.output_stem)
    return output_path


def _run_onshape_export(layout: OnshapeExportLayout, *, command: str) -> None:
    with _stage_scad_assets(layout):
        subprocess.run(
            [command, str(layout.export_directory)],
            check=True,
        )
        _sync_merged_meshes(layout)


@contextmanager
def _stage_scad_assets(layout: OnshapeExportLayout) -> Iterator[None]:
    scad_directory = layout.robot_directory / "scad"
    if scad_directory.exists():
        layout.staging_directory.mkdir(exist_ok=True)
        for source_path in scad_directory.iterdir():
            if source_path.is_file():
                shutil.copy2(source_path, layout.staging_directory / source_path.name)

    try:
        yield
    finally:
        if layout.staging_directory.exists():
            shutil.rmtree(layout.staging_directory)


def _sync_merged_meshes(layout: OnshapeExportLayout) -> None:
    merged_directory = layout.staging_directory / "merged"
    if merged_directory.exists():
        layout.mesh_directory.mkdir(parents=True, exist_ok=True)
        shutil.copytree(merged_directory, layout.mesh_directory, dirs_exist_ok=True)


def _rewrite_exported_file(output_path: Path) -> None:
    if not output_path.exists():
        raise FileNotFoundError(f"导出结果不存在: {output_path}")

    original_content = output_path.read_text(encoding="utf-8")
    updated_content = rewrite_mesh_references(original_content)
    if updated_content != original_content:
        output_path.write_text(updated_content, encoding="utf-8")
