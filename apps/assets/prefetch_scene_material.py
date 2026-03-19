from __future__ import annotations

import argparse
from pathlib import Path

from berkeley_humanoid_lite_assets.scene_materials import (
    DEFAULT_ISAAC_CONTENT_VERSION,
    copy_scene_material_from_file,
    download_scene_material_from_url,
    download_scene_material_preset,
    list_scene_material_presets,
)


def build_argument_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Stage a project-local scene material for simulation rendering.")
    source_group = parser.add_mutually_exclusive_group(required=True)
    source_group.add_argument("--source", type=Path, help="Local material file to copy into the workspace asset store.")
    source_group.add_argument("--url", type=str, help="Remote material URL to download into the workspace asset store.")
    source_group.add_argument(
        "--preset",
        type=str,
        choices=list_scene_material_presets(),
        help="Named scene material preset to download into the workspace asset store.",
    )
    parser.add_argument("--scene-name", type=str, default="default", help="Logical scene asset namespace.")
    parser.add_argument("--name", type=str, default="ground_surface.mdl", help="Target material file name.")
    parser.add_argument(
        "--preset-version",
        type=str,
        default=DEFAULT_ISAAC_CONTENT_VERSION,
        help="Isaac content version used by --preset.",
    )
    parser.add_argument(
        "--force",
        action=argparse.BooleanOptionalAction,
        default=False,
        help="Overwrite the target file if it already exists.",
    )
    return parser


def main() -> None:
    parser = build_argument_parser()
    args = parser.parse_args()

    if args.source is not None:
        prepared_path = copy_scene_material_from_file(
            args.source,
            scene_name=args.scene_name,
            output_file_name=args.name,
            overwrite=args.force,
        )
        print(f"Prepared scene material: {prepared_path}")
        return

    if args.url is not None:
        prepared_path = download_scene_material_from_url(
            args.url,
            scene_name=args.scene_name,
            output_file_name=args.name,
            overwrite=args.force,
        )
        print(f"Prepared scene material: {prepared_path}")
        return

    prepared_paths = download_scene_material_preset(
        args.preset,
        scene_name=args.scene_name,
        preset_version=args.preset_version,
        overwrite=args.force,
    )
    print("Prepared scene material preset:")
    for prepared_path in prepared_paths:
        print(prepared_path)


if __name__ == "__main__":
    main()
