# Copyright (c) 2022-2025, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Launch Isaac Sim Simulator first."""

import argparse
from pathlib import Path

from isaaclab.app import AppLauncher

from berkeley_humanoid_lite_assets import UrdfUsdConversionConfiguration, convert_urdf_to_usd


def build_argument_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Convert a URDF asset into USD format.")
    parser.add_argument("input", type=Path, help="The path to the input URDF file.")
    parser.add_argument(
        "--fix-base",
        action=argparse.BooleanOptionalAction,
        default=False,
        help="Fix the imported base link in the stage.",
    )
    parser.add_argument(
        "--merge-joints",
        action=argparse.BooleanOptionalAction,
        default=False,
        help="Consolidate links that are connected by fixed joints.",
    )
    parser.add_argument(
        "--joint-stiffness",
        type=float,
        default=100.0,
        help="The stiffness of the joint drive.",
    )
    parser.add_argument(
        "--joint-damping",
        type=float,
        default=1.0,
        help="The damping of the joint drive.",
    )
    parser.add_argument(
        "--joint-target-type",
        type=str,
        default="position",
        choices=["position", "velocity", "none"],
        help="The target type of the joint drive.",
    )
    parser.add_argument(
        "--import-sites",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="Import sites defined by the source asset.",
    )
    parser.add_argument(
        "--make-instanceable",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="Make the generated USD instanceable for efficient cloning.",
    )
    return parser


parser = build_argument_parser()
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

"""Rest everything follows."""


def main() -> None:
    configuration = UrdfUsdConversionConfiguration(
        input_path=args_cli.input,
        fix_base=args_cli.fix_base,
        merge_fixed_joints=args_cli.merge_joints,
        joint_stiffness=args_cli.joint_stiffness,
        joint_damping=args_cli.joint_damping,
        joint_target_type=args_cli.joint_target_type,
        import_sites=args_cli.import_sites,
        make_instanceable=args_cli.make_instanceable,
    )
    output_path = convert_urdf_to_usd(configuration)
    print(f"Generated USD file: {output_path}")


if __name__ == "__main__":
    main()
    simulation_app.close()
